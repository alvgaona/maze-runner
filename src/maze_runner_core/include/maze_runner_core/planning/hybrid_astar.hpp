#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "maze_runner_core/planning/occupancy_grid.hpp"

namespace maze_runner::planning
{

struct HybridAStarConfig
{
  double step = 0.3;             // arc length of one motion [m]
  double min_turn_radius = 1.0;  // tightest arc [m]
  int steering_samples = 5;      // curvatures evenly over [-1/r, 1/r]; odd keeps straight
  int heading_bins = 72;         // states closer than one bin in heading are merged
  double goal_tolerance = 0.3;   // reached when this close to the goal [m]
  double curve_penalty = 0.1;    // extra cost per meter on the tightest arc, scaled by |k| r
  // Heuristic: max of straight-line distance and the grid path length to the goal
  // around walls. Off: straight-line only, which expands far more in a maze.
  bool obstacle_heuristic = true;
  std::size_t max_expansions = 2'000'000;
};

struct PosePath
{
  std::vector<Eigen::Vector3d> poses;  // [x, y, theta], start to goal, `step` apart
  std::size_t expanded = 0;            // states taken off the open list
};

// Hybrid A*: searches over continuous poses with forward arcs (the NMPC can't reverse),
// merging states that share a grid cell and heading bin. The robot center must stay in
// free cells along each arc. Empty if the start is blocked or no path is found.
std::optional<PosePath> hybrid_astar(
  const OccupancyGrid & grid, const Eigen::Vector3d & start, const Eigen::Vector2d & goal,
  const HybridAStarConfig & config = {});

}  // namespace maze_runner::planning
