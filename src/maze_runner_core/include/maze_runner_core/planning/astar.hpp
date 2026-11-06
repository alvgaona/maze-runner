#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "maze_runner_core/planning/occupancy_grid.hpp"

namespace maze_runner::planning
{

struct GridPath
{
  std::vector<Eigen::Vector2d> points;  // cell centers, start to goal
  std::size_t expanded = 0;             // cells taken off the open list
};

// 8-connected A* over free cells, octile heuristic. Diagonal moves can't cut a blocked
// corner. Empty if start or goal is blocked or no path exists.
std::optional<GridPath> astar(
  const OccupancyGrid & grid, const Eigen::Vector2d & start, const Eigen::Vector2d & goal);

// The same path with every point dropped whose neighbors see each other in a straight
// line through free cells. Keeps the first and last point.
std::vector<Eigen::Vector2d> shortcut(
  const OccupancyGrid & grid, const std::vector<Eigen::Vector2d> & points);

// Path length from every cell to `goal` over free cells, 8-connected (Dijkstra).
// Infinity where the goal can't be reached. Indexed by OccupancyGrid::index.
std::vector<double> distance_to(const OccupancyGrid & grid, const Eigen::Vector2d & goal);

}  // namespace maze_runner::planning
