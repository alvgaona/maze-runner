#pragma once

#include <vector>

#include <Eigen/Dense>

#include "maze_runner_core/localization/lines.hpp"

namespace maze_runner::localization
{

struct EkfLinesConfig
{
  // Covariance of the odometry increments [delta_d, delta_theta].
  Eigen::Matrix2d process_noise = Eigen::Vector2d(4e-6, 4e-6).asDiagonal();
  // Mahalanobis gate on the 2D innovation (9.21: 99% for 2 DOF).
  double chi2_threshold = 9.21;
  // Sideways slip variance per meter traveled [m^2/m]; models motion the unicycle model
  // can't explain (wheel slip, scraping a wall). 0 disables it.
  double lateral_variance = 0.0;
  // Heading slip variance per unit of motion [rad^2 per (m + rad)]; wheel slip shows up
  // as a wrong delta_theta. 0 disables it.
  double heading_variance = 0.0;
};

// EKF localization against a known map of lines. State is [x, y, theta].
class EkfLines
{
public:
  EkfLines(
    const Eigen::Vector3d & x0, const Eigen::Matrix3d & P0, std::vector<Line> map,
    const EkfLinesConfig & config = {});

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
  EkfLinesConfig cfg_;
};

}  // namespace maze_runner::localization
