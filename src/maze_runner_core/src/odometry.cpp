#include "maze_runner_core/odometry.hpp"

#include <cmath>

namespace maze_runner
{

OdometryDelta odometry_delta(const Eigen::Vector3d & prev, const Eigen::Vector3d & curr)
{
  constexpr double kStraightThreshold = 1e-4;  // [rad] below this, chord == arc

  const double dtheta = std::atan2(std::sin(curr(2) - prev(2)), std::cos(curr(2) - prev(2)));
  const double dx = curr(0) - prev(0);
  const double dy = curr(1) - prev(1);
  const double chord = std::hypot(dx, dy);

  double d = chord;
  if (std::abs(dtheta) > kStraightThreshold) {
    d = chord * std::abs(dtheta) / (2.0 * std::sin(std::abs(dtheta) / 2.0));
  }

  const double theta_mid = prev(2) + dtheta / 2.0;
  if (dx * std::cos(theta_mid) + dy * std::sin(theta_mid) < 0.0) {
    d = -d;
  }
  return {d, dtheta};
}

}  // namespace maze_runner
