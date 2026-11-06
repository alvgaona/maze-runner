#include "maze_runner_core/planning/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace maze_runner::planning
{
namespace
{

double distance_to_segment(const Eigen::Vector2d & p, const Segment & s)
{
  const Eigen::Vector2d ab = s.b - s.a;
  const double len2 = ab.squaredNorm();
  const double t = len2 == 0.0 ? 0.0 : std::clamp((p - s.a).dot(ab) / len2, 0.0, 1.0);
  return (s.a + t * ab - p).norm();
}

}  // namespace

OccupancyGrid::OccupancyGrid(const std::vector<Segment> & walls, double resolution, double inflation)
: resolution_(resolution)
{
  if (resolution <= 0.0 || inflation < resolution / std::sqrt(2.0)) {
    throw std::invalid_argument("OccupancyGrid needs resolution > 0 and inflation >= resolution / sqrt(2)");
  }
  if (walls.empty()) {
    throw std::invalid_argument("OccupancyGrid needs at least one wall");
  }

  Eigen::Vector2d lo = walls.front().a;
  Eigen::Vector2d hi = lo;
  for (const auto & s : walls) {
    lo = lo.cwiseMin(s.a).cwiseMin(s.b);
    hi = hi.cwiseMax(s.a).cwiseMax(s.b);
  }
  origin_ = lo.array() - inflation;
  width_ = static_cast<int>(std::ceil((hi.x() - lo.x() + 2 * inflation) / resolution));
  height_ = static_cast<int>(std::ceil((hi.y() - lo.y() + 2 * inflation) / resolution));
  blocked_.assign(static_cast<std::size_t>(width_) * height_, 0);

  for (const auto & s : walls) {
    const Cell c0 = cell_of(s.a.cwiseMin(s.b).array() - inflation);
    const Cell c1 = cell_of(s.a.cwiseMax(s.b).array() + inflation);
    for (int y = std::max(c0.y, 0); y <= std::min(c1.y, height_ - 1); ++y) {
      for (int x = std::max(c0.x, 0); x <= std::min(c1.x, width_ - 1); ++x) {
        const Cell c{x, y};
        if (!blocked_[index(c)] && distance_to_segment(center(c), s) < inflation) {
          blocked_[index(c)] = 1;
        }
      }
    }
  }
}

Cell OccupancyGrid::cell_of(const Eigen::Vector2d & p) const
{
  return {
    static_cast<int>(std::floor((p.x() - origin_.x()) / resolution_)),
    static_cast<int>(std::floor((p.y() - origin_.y()) / resolution_))};
}

Eigen::Vector2d OccupancyGrid::center(Cell c) const
{
  return origin_ + resolution_ * Eigen::Vector2d(c.x + 0.5, c.y + 0.5);
}

}  // namespace maze_runner::planning
