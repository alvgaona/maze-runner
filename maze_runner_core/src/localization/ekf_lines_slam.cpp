#include "maze_runner_core/localization/ekf_lines_slam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "maze_runner_core/angles.hpp"

namespace maze_runner::localization
{
namespace
{

// How close a new observation must be to a candidate (world frame) to count as a re-sighting.
constexpr double kCandidateAngle = 5.0 * std::numbers::pi / 180.0;
constexpr double kCandidateDist = 0.1;  // [m]

Eigen::Matrix2d obs_noise(const ObservedLine & obs)
{
  return Eigen::Vector2d(obs.sigma_alpha * obs.sigma_alpha, obs.sigma_d * obs.sigma_d).asDiagonal();
}

}  // namespace

LineMeasurement measure_line(const Eigen::Vector3d & pose, const Line & m)
{
  const double c = std::cos(m.alpha);
  const double s = std::sin(m.alpha);
  LineMeasurement out;
  out.z << wrap_angle(m.alpha - pose(2)), m.d - c * pose(0) - s * pose(1);
  out.H_pose << 0, 0, -1,
                -c, -s, 0;
  out.H_line << 1, 0,
                s * pose(0) - c * pose(1), 1;
  return out;
}

LineInit init_line(const Eigen::Vector3d & pose, const ObservedLine & obs)
{
  const double alpha = wrap_angle(obs.alpha + pose(2));
  const double c = std::cos(alpha);
  const double s = std::sin(alpha);
  const double dd_dalpha = -s * pose(0) + c * pose(1);
  LineInit out;
  out.line = {alpha, obs.d + c * pose(0) + s * pose(1)};
  out.G_pose << 0, 0, 1,
                c, s, dd_dalpha;
  out.G_obs << 1, 0,
               dd_dalpha, 1;
  return out;
}

EkfLinesSlam::EkfLinesSlam(const Eigen::Vector3d & x0, const Eigen::Matrix3d & P0, const EkfLinesSlamConfig & config)
: x_(x0), P_(P0), cfg_(config)
{
}

void EkfLinesSlam::predict(double delta_d, double delta_theta)
{
  Eigen::Vector3d pose = x_.head<3>();
  const auto step = move_pose(pose, delta_d, delta_theta, cfg_.filter);
  x_.head<3>() = pose;

  const Eigen::Index n = x_.size();
  P_.topLeftCorner<3, 3>() = step.A * P_.topLeftCorner<3, 3>() * step.A.transpose() + step.Q;
  if (n > 3) {
    P_.topRightCorner(3, n - 3) = step.A * P_.topRightCorner(3, n - 3);
    P_.bottomLeftCorner(n - 3, 3) = P_.topRightCorner(3, n - 3).transpose();
  }
}

int EkfLinesSlam::update(const std::vector<ObservedLine> & observed)
{
  ++updates_;
  int applied = 0;

  for (const auto & obs : observed) {
    const Eigen::Vector3d pose = x_.head<3>();
    const Eigen::Matrix2d R = obs_noise(obs);

    int best = -1;
    double best_m2 = std::numeric_limits<double>::infinity();
    LineMeasurement best_meas;
    Eigen::Vector2d best_nu;
    Eigen::Matrix2d best_S;

    for (int i = 0; i < size(); ++i) {
      const auto meas = measure_line(pose, line(i));
      // A wall is only seen from its own side.
      if (meas.z(1) <= 0.0 || !overlaps(world_extent(obs, line(i)), extents_[i].first, extents_[i].second)) {
        continue;
      }
      const Eigen::Vector2d nu(wrap_angle(obs.alpha - meas.z(0)), obs.d - meas.z(1));

      // S only needs the pose and this line's blocks of P.
      const int j = 3 + 2 * i;
      const Eigen::Matrix2d S = meas.H_pose * P_.topLeftCorner<3, 3>() * meas.H_pose.transpose() +
        meas.H_pose * P_.block<3, 2>(0, j) * meas.H_line.transpose() +
        meas.H_line * P_.block<2, 3>(j, 0) * meas.H_pose.transpose() +
        meas.H_line * P_.block<2, 2>(j, j) * meas.H_line.transpose() + R;
      const double m2 = nu.dot(S.inverse() * nu);
      if (m2 < best_m2) {
        best = i;
        best_m2 = m2;
        best_meas = meas;
        best_nu = nu;
        best_S = S;
      }
    }

    if (best >= 0 && best_m2 < cfg_.filter.chi2_threshold) {
      const int j = 3 + 2 * best;
      const Eigen::MatrixXd PHt =
        P_.leftCols<3>() * best_meas.H_pose.transpose() + P_.middleCols<2>(j) * best_meas.H_line.transpose();
      const Eigen::MatrixXd K = PHt * best_S.inverse();
      x_ += K * best_nu;
      x_(2) = wrap_angle(x_(2));
      x_(j) = wrap_angle(x_(j));
      P_ -= K * PHt.transpose();
      P_ = 0.5 * (P_ + P_.transpose());

      const auto e = world_extent(obs, line(best));
      extents_[best].first = std::min(extents_[best].first, e.first);
      extents_[best].second = std::max(extents_[best].second, e.second);
      ++applied;
    } else if (best < 0 || best_m2 >= cfg_.new_line_chi2) {
      track_candidate(obs);
    }
    // Otherwise ambiguous: close to a mapped line but not close enough. Dropped.
  }

  std::erase_if(candidates_, [this](const Candidate & c) {
    return updates_ - c.last_seen > cfg_.candidate_timeout;
  });
  return applied;
}

std::vector<LineLandmark> EkfLinesSlam::landmarks() const
{
  std::vector<LineLandmark> out;
  out.reserve(extents_.size());
  for (int i = 0; i < size(); ++i) {
    out.push_back({line(i), extents_[i].first, extents_[i].second});
  }
  return out;
}

std::pair<double, double> EkfLinesSlam::world_extent(const ObservedLine & obs, const Line & m) const
{
  const double c = std::cos(x_(2));
  const double s = std::sin(x_(2));
  const Eigen::Vector2d n(std::cos(obs.alpha), std::sin(obs.alpha));
  const Eigen::Vector2d t(-n.y(), n.x());
  const Eigen::Vector2d tm(-std::sin(m.alpha), std::cos(m.alpha));

  auto project = [&](double s_obs) {
    const Eigen::Vector2d e = obs.d * n + s_obs * t;
    const Eigen::Vector2d w(x_(0) + c * e.x() - s * e.y(), x_(1) + s * e.x() + c * e.y());
    return tm.dot(w);
  };
  const double a = project(obs.s_min);
  const double b = project(obs.s_max);
  return {std::min(a, b), std::max(a, b)};
}

bool EkfLinesSlam::overlaps(const std::pair<double, double> & e, double s_min, double s_max) const
{
  return e.second >= s_min - cfg_.overlap_margin && e.first <= s_max + cfg_.overlap_margin;
}

void EkfLinesSlam::add_landmark(const ObservedLine & obs)
{
  const auto init = init_line(x_.head<3>(), obs);
  const Eigen::Index n = x_.size();

  x_.conservativeResize(n + 2);
  x_.tail<2>() << init.line.alpha, init.line.d;

  const Eigen::MatrixXd cross = init.G_pose * P_.topRows<3>();  // 2 x n
  P_.conservativeResize(n + 2, n + 2);
  P_.bottomLeftCorner(2, n) = cross;
  P_.topRightCorner(n, 2) = cross.transpose();
  P_.bottomRightCorner<2, 2>() = init.G_pose * P_.topLeftCorner<3, 3>() * init.G_pose.transpose() +
    init.G_obs * obs_noise(obs) * init.G_obs.transpose();

  extents_.push_back(world_extent(obs, init.line));
}

void EkfLinesSlam::track_candidate(const ObservedLine & obs)
{
  const Line w = init_line(x_.head<3>(), obs).line;
  const auto e = world_extent(obs, w);

  auto it = std::find_if(candidates_.begin(), candidates_.end(), [&](const Candidate & c) {
    return std::abs(wrap_angle(c.lm.line.alpha - w.alpha)) < kCandidateAngle &&
      std::abs(c.lm.line.d - w.d) < kCandidateDist && overlaps(e, c.lm.s_min, c.lm.s_max);
  });
  if (it == candidates_.end()) {
    candidates_.push_back({{w, e.first, e.second}, 0, updates_});
    it = std::prev(candidates_.end());
  }
  it->lm.line = w;
  it->lm.s_min = std::min(it->lm.s_min, e.first);
  it->lm.s_max = std::max(it->lm.s_max, e.second);
  it->last_seen = updates_;

  if (++it->hits >= cfg_.min_observations) {
    add_landmark(obs);
    extents_.back() = {std::min(extents_.back().first, it->lm.s_min), std::max(extents_.back().second, it->lm.s_max)};
    candidates_.erase(it);
  }
}

}  // namespace maze_runner::localization
