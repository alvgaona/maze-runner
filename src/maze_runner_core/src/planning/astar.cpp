#include "maze_runner_core/planning/astar.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace maze_runner::planning
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kSqrt2 = 1.4142135623730951;

struct Move
{
  int dx;
  int dy;
  double cost;  // in cells
};
constexpr Move kMoves[] = {
  {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
  {1, 1, kSqrt2}, {1, -1, kSqrt2}, {-1, 1, kSqrt2}, {-1, -1, kSqrt2}};

struct Search
{
  std::vector<double> g;   // cost from the start [m]
  std::vector<int> parent;
  std::size_t expanded = 0;
};

// Best-first search from `start`. Stops when `stop` is expanded (-1: never, explores all).
// heuristic(cell) must not overestimate the remaining cost.
template<typename Heuristic>
Search search(const OccupancyGrid & grid, Cell start, int stop, Heuristic heuristic)
{
  const std::size_t n = static_cast<std::size_t>(grid.width()) * grid.height();
  Search s{std::vector<double>(n, kInf), std::vector<int>(n, -1)};
  std::vector<char> closed(n, 0);
  using Entry = std::pair<double, int>;  // (f, index)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;

  s.g[grid.index(start)] = 0.0;
  open.emplace(heuristic(start), grid.index(start));
  while (!open.empty()) {
    const int i = open.top().second;
    open.pop();
    if (closed[i]) {
      continue;
    }
    closed[i] = 1;
    ++s.expanded;
    if (i == stop) {
      break;
    }
    const Cell c{i % grid.width(), i / grid.width()};
    for (const auto & m : kMoves) {
      const Cell next{c.x + m.dx, c.y + m.dy};
      const bool diagonal = m.dx != 0 && m.dy != 0;
      if (!grid.free(next) ||
        (diagonal && (!grid.free(Cell{next.x, c.y}) || !grid.free(Cell{c.x, next.y}))))
      {
        continue;
      }
      const int j = grid.index(next);
      const double g = s.g[i] + m.cost * grid.resolution();
      if (g < s.g[j]) {
        s.g[j] = g;
        s.parent[j] = i;
        open.emplace(g + heuristic(next), j);
      }
    }
  }
  return s;
}

}  // namespace

std::optional<GridPath> astar(
  const OccupancyGrid & grid, const Eigen::Vector2d & start, const Eigen::Vector2d & goal)
{
  const Cell s = grid.cell_of(start);
  const Cell t = grid.cell_of(goal);
  if (!grid.free(s) || !grid.free(t)) {
    return std::nullopt;
  }
  const auto octile = [&](Cell c) {
      const double dx = std::abs(c.x - t.x);
      const double dy = std::abs(c.y - t.y);
      return grid.resolution() * (std::max(dx, dy) + (kSqrt2 - 1.0) * std::min(dx, dy));
    };
  const Search result = search(grid, s, grid.index(t), octile);
  if (result.g[grid.index(t)] == kInf) {
    return std::nullopt;
  }

  GridPath path{{}, result.expanded};
  for (int i = grid.index(t); i != -1; i = result.parent[i]) {
    path.points.push_back(grid.center({i % grid.width(), i / grid.width()}));
  }
  std::reverse(path.points.begin(), path.points.end());
  return path;
}

std::vector<Eigen::Vector2d> shortcut(
  const OccupancyGrid & grid, const std::vector<Eigen::Vector2d> & points)
{
  const auto visible = [&](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
      const int steps = static_cast<int>(std::ceil((b - a).norm() / (0.5 * grid.resolution())));
      for (int k = 1; k < steps; ++k) {
        if (!grid.free(Eigen::Vector2d(a + (b - a) * (static_cast<double>(k) / steps)))) {
          return false;
        }
      }
      return true;
    };

  if (points.size() < 3) {
    return points;
  }
  std::vector<Eigen::Vector2d> out{points.front()};
  std::size_t from = 0;
  while (from + 1 < points.size()) {
    // Walk forward while the next point is still in sight; the neighbor always is.
    std::size_t to = from + 1;
    while (to + 1 < points.size() && visible(points[from], points[to + 1])) {
      ++to;
    }
    out.push_back(points[to]);
    from = to;
  }
  return out;
}

std::vector<double> distance_to(const OccupancyGrid & grid, const Eigen::Vector2d & goal)
{
  const Cell t = grid.cell_of(goal);
  if (!grid.free(t)) {
    return std::vector<double>(static_cast<std::size_t>(grid.width()) * grid.height(), kInf);
  }
  return search(grid, t, -1, [](Cell) {return 0.0;}).g;
}

}  // namespace maze_runner::planning
