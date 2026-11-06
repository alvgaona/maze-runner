#include "maze_runner_core/localization/ekf_lines.hpp"

#include "maze_runner_core/angles.hpp"

#include <cmath>
#include <numbers>
#include <utility>

namespace maze_runner::localization
{
namespace
{

// Association gates and weights, as tuned in the MATLAB version.
constexpr double kAngleWeight = 2.0;
constexpr double kDistWeight = 1.0;
constexpr double kMaxAngleDiff = 30.0 * std::numbers::pi / 180.0;
constexpr double kMaxDistDiff = 0.5;

}  // namespace

EkfLines::EkfLines(
  const Eigen::Vector3d & x0, const Eigen::Matrix3d & P0, std::vector<Line> map,
  const EkfLinesConfig & config)
: x_(x0), P_(P0), map_(std::move(map)), cfg_(config)
{
}

MotionStep move_pose(Eigen::Vector3d & x, double delta_d, double delta_theta, const EkfLinesConfig & cfg)
{
  const double theta_mid = x(2) + delta_theta / 2.0;
  const double c = std::cos(theta_mid);
  const double s = std::sin(theta_mid);

  x(0) += delta_d * c;
  x(1) += delta_d * s;
  x(2) = wrap_angle(x(2) + delta_theta);

  MotionStep step;
  step.A << 1, 0, -delta_d * s,
            0, 1, delta_d * c,
            0, 0, 1;

  Eigen::Matrix<double, 3, 2> W;
  W << c, -0.5 * delta_d * s,
       s, 0.5 * delta_d * c,
       0, 1;

  step.Q = W * cfg.process_noise * W.transpose();

  const Eigen::Vector2d lateral(-s, c);
  step.Q.topLeftCorner<2, 2>() += cfg.lateral_variance * std::abs(delta_d) * lateral * lateral.transpose();
  step.Q(2, 2) += cfg.heading_variance * (std::abs(delta_d) + std::abs(delta_theta));
  return step;
}

void EkfLines::predict(double delta_d, double delta_theta)
{
  const auto step = move_pose(x_, delta_d, delta_theta, cfg_);
  P_ = step.A * P_ * step.A.transpose() + step.Q;
}

int EkfLines::update(const std::vector<ObservedLine> & observed)
{
  int applied = 0;

  for (const auto & obs : observed) {
    const int idx = associate(obs);
    if (idx < 0) {
      continue;
    }
    const Line m = oriented(idx);
    const double c = std::cos(m.alpha);
    const double s = std::sin(m.alpha);

    // h(x) = [alpha_map - theta; d_map - n . (x, y)]
    const Eigen::Vector2d innovation(
      wrap_angle(obs.alpha - (m.alpha - x_(2))),
      obs.d - (m.d - c * x_(0) - s * x_(1)));

    Eigen::Matrix<double, 2, 3> H;
    H << 0, 0, -1,
         -c, -s, 0;

    const Eigen::Matrix2d R =
      Eigen::Vector2d(obs.sigma_alpha * obs.sigma_alpha, obs.sigma_d * obs.sigma_d).asDiagonal();
    const Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix2d S_inv = S.inverse();

    if (innovation.dot(S_inv * innovation) > cfg_.chi2_threshold) {
      continue;
    }

    const Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S_inv;
    x_ += K * innovation;
    x_(2) = wrap_angle(x_(2));
    P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
    ++applied;
  }
  return applied;
}

Line EkfLines::oriented(int i) const
{
  const Line & m = map_[i];
  const double d_robot = m.d - std::cos(m.alpha) * x_(0) - std::sin(m.alpha) * x_(1);
  if (d_robot >= 0.0) {
    return m;
  }
  return {wrap_angle(m.alpha + std::numbers::pi), -m.d};
}

int EkfLines::associate(const ObservedLine & obs) const
{
  const double alpha_world = obs.alpha + x_(2);

  int best = -1;
  double best_score = 0.0;
  double best_angle_diff = 0.0;
  double best_dist_diff = 0.0;

  for (int i = 0; i < static_cast<int>(map_.size()); ++i) {
    const Line m = oriented(i);
    const double angle_diff = std::abs(wrap_angle(m.alpha - alpha_world));
    const double d_predicted = m.d - std::cos(m.alpha) * x_(0) - std::sin(m.alpha) * x_(1);
    const double dist_diff = std::abs(obs.d - d_predicted);
    const double score = kAngleWeight * angle_diff + kDistWeight * dist_diff;

    if (best < 0 || score < best_score) {
      best = i;
      best_score = score;
      best_angle_diff = angle_diff;
      best_dist_diff = dist_diff;
    }
  }

  if (best < 0 || best_angle_diff >= kMaxAngleDiff || best_dist_diff >= kMaxDistDiff) {
    return -1;
  }
  return best;
}

}  // namespace maze_runner::localization
