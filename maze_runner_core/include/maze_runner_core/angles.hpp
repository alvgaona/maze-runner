#pragma once

#include <cmath>

namespace maze_runner
{

// Angle wrapped to [-pi, pi].
inline double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace maze_runner
