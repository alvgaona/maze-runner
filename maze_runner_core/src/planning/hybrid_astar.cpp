#include "maze_runner_core/planning/hybrid_astar.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "maze_runner_core/angles.hpp"
#include "maze_runner_core/planning/astar.hpp"

namespace maze_runner::planning
{
namespace
{

constexpr double kTwoPi = 2.0 * std::numbers::pi;

// Pose after driving arc length s on curvature k.
Eigen::Vector3d drive(const Eigen::Vector3d & p, double k, double s)
{
  if (std::abs(k) < 1e-9) {
    return {p(0) + s * std::cos(p(2)), p(1) + s * std::sin(p(2)), p(2)};
  }
  const double th = p(2) + k * s;
  return {
    p(0) + (std::sin(th) - std::sin(p(2))) / k,
    p(1) - (std::cos(th) - std::cos(p(2))) / k,
    wrap_angle(th)};
}

struct Node
{
  Eigen::Vector3d pose;
  double g;
  int parent;
};

}  // namespace

std::optional<PosePath> hybrid_astar(
  const OccupancyGrid & grid, const Eigen::Vector3d & start, const Eigen::Vector2d & goal,
  const HybridAStarConfig & cfg)
{
  if (cfg.steering_samples < 1 || cfg.heading_bins < 1 || cfg.step <= 0.0 ||
    cfg.min_turn_radius <= 0.0)
  {
    throw std::invalid_argument("hybrid_astar: bad config");
  }
  if (!grid.free(Eigen::Vector2d(start.head<2>()))) {
    return std::nullopt;
  }

  std::vector<double> curvatures;
  for (int i = 0; i < cfg.steering_samples; ++i) {
    curvatures.push_back(
      cfg.steering_samples == 1 ? 0.0 :
      (2.0 * i / (cfg.steering_samples - 1) - 1.0) / cfg.min_turn_radius);
  }

  std::vector<double> grid_distance;
  if (cfg.obstacle_heuristic) {
    grid_distance = distance_to(grid, goal);
  }
  const auto heuristic = [&](const Eigen::Vector3d & p) {
      const double euclid = (p.head<2>() - goal).norm();
      return cfg.obstacle_heuristic ?
             std::max(euclid, grid_distance[grid.index(grid.cell_of(p.head<2>()))]) : euclid;
    };
  const auto key = [&](const Eigen::Vector3d & p) {
      const double turn = std::fmod(p(2) + kTwoPi, kTwoPi) / kTwoPi;
      const int bin = std::min(static_cast<int>(turn * cfg.heading_bins), cfg.heading_bins - 1);
      return static_cast<long long>(grid.index(grid.cell_of(p.head<2>()))) * cfg.heading_bins + bin;
    };
  const int samples = std::max(1, static_cast<int>(std::ceil(cfg.step / (0.5 * grid.resolution()))));
  const auto arc_free = [&](const Eigen::Vector3d & p, double k) {
      for (int i = 1; i <= samples; ++i) {
        if (!grid.free(Eigen::Vector2d(drive(p, k, cfg.step * i / samples).head<2>()))) {
          return false;
        }
      }
      return true;
    };

  std::vector<Node> nodes{{start, 0.0, -1}};
  std::unordered_map<long long, double> best_g{{key(start), 0.0}};
  std::unordered_map<long long, char> closed;
  using Entry = std::pair<double, int>;  // (f, node)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
  open.emplace(heuristic(start), 0);

  PosePath path;
  while (!open.empty() && path.expanded < cfg.max_expansions) {
    const int i = open.top().second;
    open.pop();
    const Node node = nodes[i];
    if (!closed.emplace(key(node.pose), 1).second) {
      continue;
    }
    ++path.expanded;

    if ((node.pose.head<2>() - goal).norm() <= cfg.goal_tolerance) {
      for (int j = i; j != -1; j = nodes[j].parent) {
        path.poses.push_back(nodes[j].pose);
      }
      std::reverse(path.poses.begin(), path.poses.end());
      return path;
    }

    for (const double k : curvatures) {
      if (!arc_free(node.pose, k)) {
        continue;
      }
      const Eigen::Vector3d next = drive(node.pose, k, cfg.step);
      const double h = heuristic(next);
      if (std::isinf(h)) {
        continue;
      }
      const long long next_key = key(next);
      if (closed.count(next_key)) {
        continue;
      }
      const double g =
        node.g + cfg.step * (1.0 + cfg.curve_penalty * std::abs(k) * cfg.min_turn_radius);
      const auto it = best_g.find(next_key);
      if (it != best_g.end() && it->second <= g) {
        continue;
      }
      best_g[next_key] = g;
      nodes.push_back({next, g, i});
      open.emplace(g + h, static_cast<int>(nodes.size()) - 1);
    }
  }
  return std::nullopt;
}

}  // namespace maze_runner::planning
