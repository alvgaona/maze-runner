#include "maze_runner_core/control/nmpc_cbf.hpp"

#include "maze_runner_core/angles.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace maze_runner::control
{
namespace
{

// Offset given to unused obstacle slots: far enough that the constraint never binds.
constexpr double kFarAway = 100.0;
// Consecutive failed solves to ride out on the previous plan before stopping.
constexpr int kMaxFailures = 5;


}  // namespace

NmpcCbf::NmpcCbf(const NmpcCbfConfig & config)
: cfg_(config)
{
  const int N = cfg_.horizon;
  const int M = cfg_.max_obstacles;
  if (N < 1 || M < 0 || cfg_.cbf_gamma <= 0.0 || cfg_.cbf_gamma > 1.0) {
    throw std::invalid_argument("NmpcCbf: horizon >= 1, max_obstacles >= 0, cbf_gamma in (0, 1]");
  }
  using casadi::MX;

  X_ = opti_.variable(3, N + 1);
  U_ = opti_.variable(2, N);
  x0_ = opti_.parameter(3);
  xref_ = opti_.parameter(3, N + 1);
  obstacles_ = opti_.parameter(2, M);
  u_prev_ = opti_.parameter(2);

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

  const auto & rd = cfg_.control_rate_weights;
  const MX Rd = MX::diag(MX::vertcat({rd(0), rd(1)}));

  MX cost = 0;
  for (int k = 0; k <= N; ++k) {
    const MX e = X_(casadi::Slice(), k) - xref_(casadi::Slice(), k);
    cost += MX::mtimes({e.T(), Q, e});
    if (k < N) {
      const MX u = U_(casadi::Slice(), k);
      cost += MX::mtimes({u.T(), R, u});
      const MX du = u - (k == 0 ? u_prev_ : U_(casadi::Slice(), k - 1));
      cost += MX::mtimes({du.T(), Rd, du});
    }
  }

  if (M > 0) {
    const double d2 = cfg_.safety_radius * cfg_.safety_radius;
    auto barrier = [&](int k, int i) {
      const MX dx = X_(0, k) - obstacles_(0, i);
      const MX dy = X_(1, k) - obstacles_(1, i);
      return dx * dx + dy * dy - d2;
    };
    MX S;
    if (cfg_.use_slack) {
      S = opti_.variable(M, N);
      opti_.subject_to(MX::vec(S) >= 0);  // vec: element-wise, not a matrix inequality
      cost += cfg_.slack_penalty * MX::sum1(MX::vec(S));  // L1: exact penalty, well scaled
    }
    for (int k = 0; k < N; ++k) {
      for (int i = 0; i < M; ++i) {
        MX cbf = barrier(k + 1, i) - (1.0 - cfg_.cbf_gamma) * barrier(k, i);
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
  // A late command is worse than a failed solve: the fallback follows the last plan.
  opts["ipopt.max_cpu_time"] = 0.8 * cfg_.dt;
  opts["ipopt.acceptable_tol"] = 1e-6;
  opts["ipopt.acceptable_obj_change_tol"] = 1e-6;
  opts["ipopt.warm_start_init_point"] = "yes";
  opti_.solver("ipopt", opts);
  // Compiled once: calling this avoids Opti::solve's per-call setup.
  solver_ = opti_.to_function("nmpc_cbf", {x0_, xref_, obstacles_, u_prev_, U_, X_}, {U_});

  reset();
}

void NmpcCbf::reset()
{
  plan_ = casadi::DM::zeros(2, cfg_.horizon);
  last_sent_ = casadi::DM::zeros(2, 1);
  succeeded_ = true;
  failures_ = 0;
}

std::vector<Eigen::Vector2d> NmpcCbf::select_obstacles(
  const Eigen::Vector3d & x, const std::vector<Eigen::Vector2d> & points) const
{
  const int M = cfg_.max_obstacles;
  std::vector<double> best_range(M, std::numeric_limits<double>::infinity());
  std::vector<Eigen::Vector2d> nearest(M);
  for (const auto & p : points) {
    const Eigen::Vector2d rel = p - x.head<2>();
    const double range = rel.norm();
    const double bearing = wrap_angle(std::atan2(rel.y(), rel.x()) - x(2));
    if (!std::isfinite(range) || range >= cfg_.constraint_range || std::abs(bearing) > cfg_.fov) {
      continue;
    }
    const int sector =
      std::clamp(static_cast<int>((bearing + cfg_.fov) / (2.0 * cfg_.fov) * M), 0, M - 1);
    if (range < best_range[sector]) {
      best_range[sector] = range;
      nearest[sector] = p;
    }
  }
  std::vector<Eigen::Vector2d> selected;
  for (int i = 0; i < M; ++i) {
    if (std::isfinite(best_range[i])) {
      selected.push_back(nearest[i]);
    }
  }
  return selected;
}

Eigen::Vector2d NmpcCbf::compute(
  const Eigen::Vector3d & x, const std::vector<Eigen::Vector3d> & xref,
  const std::vector<Eigen::Vector2d> & obstacles_world)
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

  const auto obstacles = select_obstacles(x, obstacles_world);
  obstacle_count_ = static_cast<int>(obstacles.size());
  casadi::DM points = casadi::DM::zeros(2, M);
  for (int i = 0; i < M; ++i) {
    const bool used = i < obstacle_count_;
    points(0, i) = used ? obstacles[i].x() : x(0) + kFarAway;
    points(1, i) = used ? obstacles[i].y() : x(1);
  }

  // Initial guess: last plan shifted one step, and the states it produces from x. The
  // barrier depends on the states, so a guess far from the robot makes IPOPT fail.
  casadi::DM u_guess = casadi::DM::zeros(2, N);
  for (int k = 0; k < N; ++k) {
    const int from = std::min(k + 1, N - 1);
    u_guess(0, k) = plan_(0, from);
    u_guess(1, k) = plan_(1, from);
  }
  casadi::DM x_guess = casadi::DM::zeros(3, N + 1);
  Eigen::Vector3d s = x;
  for (int k = 0; k <= N; ++k) {
    x_guess(0, k) = s(0);
    x_guess(1, k) = s(1);
    x_guess(2, k) = s(2);
    if (k < N) {
      const double v = static_cast<double>(u_guess(0, k));
      const double w = static_cast<double>(u_guess(1, k));
      s += cfg_.dt * Eigen::Vector3d(v * std::cos(s(2)), v * std::sin(s(2)), w);
    }
  }

  const std::vector<casadi::DM> out = solver_(std::vector<casadi::DM>{
    casadi::DM(std::vector<double>{x(0), x(1), x(2)}), ref, points, last_sent_, u_guess, x_guess});
  succeeded_ = solver_.stats().at("success").as_bool();
  if (succeeded_) {
    plan_ = out[0];
    failures_ = 0;
  } else if (++failures_ <= kMaxFailures) {
    plan_ = u_guess;  // keep executing the last good plan
  } else {
    plan_ = casadi::DM::zeros(2, N);  // no good plan for too long: stop
  }
  last_sent_ = plan_(casadi::Slice(), 0);
  return {static_cast<double>(plan_(0, 0)), static_cast<double>(plan_(1, 0))};
}

}  // namespace maze_runner::control
