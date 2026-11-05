#include "maze_runner_core/nmpc_cbf.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace maze_runner
{
namespace
{

// Range given to unused obstacle slots: far enough that the constraint never binds.
constexpr double kFarAway = 1e3;

double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

NmpcCbf::NmpcCbf(const NmpcCbfConfig & config)
: cfg_(config)
{
  const int N = cfg_.horizon;
  const int M = cfg_.max_obstacles;
  if (N < 1 || M < 0 || cfg_.scan_downsample < 1) {
    throw std::invalid_argument("NmpcCbf: horizon >= 1, max_obstacles >= 0, scan_downsample >= 1");
  }
  using casadi::MX;

  X_ = opti_.variable(3, N + 1);
  U_ = opti_.variable(2, N);
  x0_ = opti_.parameter(3);
  xref_ = opti_.parameter(3, N + 1);
  ranges_ = opti_.parameter(M);
  bearings_ = opti_.parameter(M);

  opti_.subject_to(X_(casadi::Slice(), 0) == x0_);
  for (int k = 0; k < N; ++k) {
    const MX th = X_(2, k);
    const MX v = U_(0, k);
    const MX w = U_(1, k);
    opti_.subject_to(X_(casadi::Slice(), k + 1) == X_(casadi::Slice(), k) + cfg_.dt * MX::vertcat({
      v * cos(th), v * sin(th), w}));
  }
  opti_.subject_to(opti_.bounded(cfg_.v_min, U_(0, casadi::Slice()), cfg_.v_max));
  opti_.subject_to(opti_.bounded(cfg_.omega_min, U_(1, casadi::Slice()), cfg_.omega_max));

  const auto & qw = cfg_.state_weights;
  const auto & rw = cfg_.control_weights;
  const MX Q = MX::diag(MX::vertcat({qw(0), qw(1), qw(2)}));
  const MX R = MX::diag(MX::vertcat({rw(0), rw(1)}));

  MX cost = 0;
  for (int k = 0; k <= N; ++k) {
    const MX e = X_(casadi::Slice(), k) - xref_(casadi::Slice(), k);
    cost += MX::mtimes({e.T(), Q, e});
    if (k < N) {
      const MX u = U_(casadi::Slice(), k);
      cost += MX::mtimes({u.T(), R, u});
    }
  }

  // HOCBF with h = r - d_safe for each point, evaluated on each control in the horizon:
  // h_ddot + 2 alpha h_dot + alpha^2 h >= -s, with the robot-relative point held fixed.
  if (M > 0) {
    const double a = cfg_.alpha_cbf;
    MX S;
    if (cfg_.use_slack) {
      S = opti_.variable(M, N);
      opti_.subject_to(MX::vec(S) >= 0);  // vec: element-wise, not a matrix inequality
      cost += cfg_.slack_penalty * MX::sumsqr(S);
    }
    for (int k = 0; k < N; ++k) {
      const MX v = U_(0, k);
      const MX w = U_(1, k);
      for (int i = 0; i < M; ++i) {
        const MX r = ranges_(i);
        const MX phi = bearings_(i);
        const MX h = r - cfg_.safety_radius;
        const MX h_dot = -v * cos(phi);
        const MX h_ddot = v * v * sin(phi) * sin(phi) / r - v * w * sin(phi);
        MX cbf = h_ddot + 2 * a * h_dot + a * a * h;
        if (cfg_.use_slack) {
          cbf += S(i, k);
        }
        opti_.subject_to(cbf >= 0);
      }
    }
  }

  opti_.minimize(cost);

  casadi::Dict opts;
  opts["print_time"] = false;
  opts["expand"] = true;  // MX -> SX: much faster function and derivative evaluation
  opts["ipopt.print_level"] = 0;
  opts["ipopt.sb"] = "yes";
  opts["ipopt.max_iter"] = cfg_.max_iterations;
  opts["ipopt.acceptable_tol"] = 1e-6;
  opts["ipopt.acceptable_obj_change_tol"] = 1e-6;
  opts["ipopt.warm_start_init_point"] = "yes";
  opti_.solver("ipopt", opts);
  // Compiled once: calling this avoids Opti::solve's per-call setup.
  solver_ = opti_.to_function("nmpc_cbf", {x0_, xref_, ranges_, bearings_, U_}, {U_});

  reset();
}

void NmpcCbf::reset()
{
  last_u_ = casadi::DM::zeros(2, cfg_.horizon);
  succeeded_ = true;
}

std::vector<ScanPoint> NmpcCbf::select_obstacles(const std::vector<ScanPoint> & scan) const
{
  std::vector<ScanPoint> candidates;
  for (const auto & p : scan) {
    if (std::isfinite(p.range) && p.range < cfg_.constraint_range &&
        std::abs(wrap_angle(p.bearing)) <= cfg_.fov)
    {
      candidates.push_back(p);
    }
  }
  std::vector<ScanPoint> selected;
  for (std::size_t i = 0; i < candidates.size(); i += cfg_.scan_downsample) {
    selected.push_back(candidates[i]);
  }
  if (static_cast<int>(selected.size()) > cfg_.max_obstacles) {
    std::partial_sort(
      selected.begin(), selected.begin() + cfg_.max_obstacles, selected.end(),
      [](const ScanPoint & a, const ScanPoint & b) { return a.range < b.range; });
    selected.resize(cfg_.max_obstacles);
  }
  return selected;
}

Eigen::Vector2d NmpcCbf::compute(
  const Eigen::Vector3d & x, const std::vector<Eigen::Vector3d> & xref,
  const std::vector<ScanPoint> & scan)
{
  if (xref.empty()) {
    throw std::invalid_argument("NmpcCbf::compute: empty reference");
  }
  const int N = cfg_.horizon;
  const int M = cfg_.max_obstacles;

  // Reference, padded with its last pose; headings unwrapped to be continuous from x(2)
  // so the cost never sees a 2*pi jump.
  casadi::DM ref = casadi::DM::zeros(3, N + 1);
  double prev_heading = x(2);
  for (int k = 0; k <= N; ++k) {
    const auto & r = xref[std::min<std::size_t>(k, xref.size() - 1)];
    prev_heading += wrap_angle(r(2) - prev_heading);
    ref(0, k) = r(0);
    ref(1, k) = r(1);
    ref(2, k) = prev_heading;
  }

  const auto obstacles = select_obstacles(scan);
  obstacle_count_ = static_cast<int>(obstacles.size());
  std::vector<double> ranges(M, kFarAway);
  std::vector<double> bearings(M, 0.0);
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    ranges[i] = obstacles[i].range;
    bearings[i] = obstacles[i].bearing;
  }

  const std::vector<casadi::DM> out = solver_(std::vector<casadi::DM>{
    casadi::DM(std::vector<double>{x(0), x(1), x(2)}), ref, casadi::DM(ranges),
    casadi::DM(bearings), last_u_});
  succeeded_ = solver_.stats().at("success").as_bool();
  if (succeeded_) {
    last_u_ = out[0];
  }  // else keep last_u_, as the MATLAB version did
  return {static_cast<double>(last_u_(0, 0)), static_cast<double>(last_u_(1, 0))};
}

}  // namespace maze_runner
