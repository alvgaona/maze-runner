#pragma once

#include <cmath>

namespace maze_runner
{

// Angle wrapped to [-pi, pi].
inline double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// Infinite line in Hesse normal form: points p with n(alpha) . p = d.
struct Line
{
  double alpha;  // angle of the line normal [rad]
  double d;      // distance from origin along the normal [m]
};

// A line extracted from a scan, in the robot frame, with its 1-sigma uncertainty.
// Canonical form: d >= 0, i.e. the normal points from the robot toward the line.
struct ObservedLine
{
  double alpha;
  double d;
  double sigma_alpha;
  double sigma_d;
};

}  // namespace maze_runner
