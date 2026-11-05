#pragma once

#include <vector>

#include <Eigen/Dense>

namespace maze_runner
{

// Infinite line in Hesse normal form: points p with n(alpha) . p = d.
struct Line
{
  double alpha;  // angle of the line normal [rad]
  double d;      // distance from origin along the normal [m]
};

// A line extracted from a scan, in the robot frame, with its 1-sigma uncertainty.
struct ObservedLine
{
  double alpha;
  double d;
  double sigma_alpha;
  double sigma_d;
};

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
  // Index of the map line matching the observation, or -1.
  int associate(const ObservedLine & obs) const;

  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;
  std::vector<Line> map_;
  Eigen::Matrix2d Q_;
  double chi2_threshold_;
};

}  // namespace maze_runner
