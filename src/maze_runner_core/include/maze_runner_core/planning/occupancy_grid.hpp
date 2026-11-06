#pragma once

#include <vector>

#include <Eigen/Core>

#include "maze_runner_core/walls.hpp"

namespace maze_runner::planning
{

struct Cell
{
  int x;
  int y;
  bool operator==(const Cell &) const = default;
};

// Grid over the walls' bounding box. Cells closer than `inflation` to a wall are blocked,
// so any free cell is a safe spot for the robot center. Outside the grid counts as blocked.
class OccupancyGrid
{
public:
  // inflation must be at least resolution / sqrt(2), so no wall slips between cell centers.
  OccupancyGrid(const std::vector<Segment> & walls, double resolution, double inflation);

  bool in_bounds(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_; }
  bool free(Cell c) const { return in_bounds(c) && !blocked_[index(c)]; }
  bool free(const Eigen::Vector2d & p) const { return free(cell_of(p)); }

  Cell cell_of(const Eigen::Vector2d & p) const;
  Eigen::Vector2d center(Cell c) const;
  int index(Cell c) const { return c.y * width_ + c.x; }

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }

private:
  double resolution_;
  Eigen::Vector2d origin_;  // corner of cell (0, 0)
  int width_;
  int height_;
  std::vector<char> blocked_;
};

}  // namespace maze_runner::planning
