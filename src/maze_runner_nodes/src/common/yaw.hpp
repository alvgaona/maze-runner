#pragma once

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>

namespace maze_runner
{

// Heading of a (planar) orientation.
inline double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace maze_runner
