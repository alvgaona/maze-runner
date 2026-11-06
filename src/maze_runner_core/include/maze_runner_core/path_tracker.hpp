#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace maze_runner
{

// Dense path [x, y, theta] through the waypoints, one point every `spacing` meters.
// Heading is the direction of the segment each point lies on.
std::vector<Eigen::Vector3d> densify(const std::vector<Eigen::Vector2d> & waypoints, double spacing);

// Forward-gated progression along a dense path: the tracked index only moves forward,
// searching a bounded window so the robot can't skip to a later part of a looping path.
class PathTracker
{
public:
  explicit PathTracker(std::vector<Eigen::Vector3d> path, std::size_t search_window = 200);

  // Advance to the closest path point (never backward) and return its index. The NMPC tracks
  // the path from there: points are one control step apart, so any lead makes it outrun
  // reference_speed.
  std::size_t update(const Eigen::Vector2d & position);

  // True once the closest point is the end of the path and the robot is within tolerance.
  bool reached_goal(const Eigen::Vector2d & position, double tolerance) const;

  const std::vector<Eigen::Vector3d> & path() const { return path_; }

private:
  std::vector<Eigen::Vector3d> path_;
  std::size_t search_window_;
  std::size_t ref_idx_ = 0;
  bool initialized_ = false;
};

}  // namespace maze_runner
