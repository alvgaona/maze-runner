#pragma once

#include <vector>

#include <Eigen/Dense>

#include "maze_runner_core/lines.hpp"

namespace maze_runner
{

// EKF localization against a known map of lines. State is [x, y, theta].
class EkfLines
{
public:
  // process_noise: covariance of the odometry increments [delta_d, delta_theta].
  // chi2_threshold: Mahalanobis gate on the 2D innovation.
  EkfLines(
    const Eigen::Vector3d & x0, const Eigen::Matrix3d & P0, std::vector<Line> map,
    const Eigen::Matrix2d & process_noise, double chi2_threshold);

  // Midpoint odometry model.
  void predict(double delta_d, double delta_theta);

  // Sequential update, one observed line at a time. Returns how many were applied.
  int update(const std::vector<ObservedLine> & observed);

  const Eigen::Vector3d & state() const { return x_; }
  const Eigen::Matrix3d & covariance() const { return P_; }

private:
  // Map line i with its normal flipped, if needed, to point from the robot toward it,
  // matching the canonical form of observed lines.
  Line oriented(int i) const;

  // Index of the map line matching the observation, or -1.
  int associate(const ObservedLine & obs) const;

  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;
  std::vector<Line> map_;
  Eigen::Matrix2d Q_;
  double chi2_threshold_;
};

}  // namespace maze_runner
