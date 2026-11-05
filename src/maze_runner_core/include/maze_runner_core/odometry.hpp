#pragma once

#include <Eigen/Core>

namespace maze_runner
{

struct OdometryDelta
{
  double d;      // signed arc length traveled [m], negative when reversing
  double theta;  // heading change [rad]
};

// Increment between two odometry poses [x, y, theta], as the EKF predict step expects.
// Uses the arc length when turning, so it is consistent with the midpoint motion model.
OdometryDelta odometry_delta(const Eigen::Vector3d & prev, const Eigen::Vector3d & curr);

}  // namespace maze_runner
