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
  explicit PathTracker(
    std::vector<Eigen::Vector3d> path, double waypoint_threshold = 0.1,
    std::size_t search_window = 200, double lookahead = 0.5);

  // Advance with the current position; returns the index the controller should track from.
  std::size_t update(const Eigen::Vector2d & position);

  // True once the closest point is the end of the path and the robot is within tolerance.
  bool reached_goal(const Eigen::Vector2d & position, double tolerance) const;

  const std::vector<Eigen::Vector3d> & path() const { return path_; }

private:
  std::vector<Eigen::Vector3d> path_;
  double waypoint_threshold_;
  std::size_t search_window_;
  double lookahead_;
  std::size_t ref_idx_ = 0;
  bool initialized_ = false;
};

}  // namespace maze_runner
