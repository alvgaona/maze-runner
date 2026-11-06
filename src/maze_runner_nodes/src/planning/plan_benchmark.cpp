// Times A* and Hybrid A* on a world's walls and prints path quality next to it.
// Usage: plan_benchmark WORLD_SDF MODEL START_X START_Y START_YAW GOAL_X GOAL_Y
//                       [RESOLUTION=0.1] [INFLATION=0.6] [RUNS=20]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "common/world_map_loader.hpp"
#include "maze_runner_core/angles.hpp"
#include "maze_runner_core/control/path_tracker.hpp"
#include "maze_runner_core/planning/astar.hpp"
#include "maze_runner_core/planning/hybrid_astar.hpp"

namespace
{

using maze_runner::Segment;
using namespace maze_runner::planning;

double median_ms(int runs, const std::function<void()> & work)
{
  std::vector<double> ms;
  for (int i = 0; i < runs; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    work();
    ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  }
  std::sort(ms.begin(), ms.end());
  return ms[ms.size() / 2];
}

double distance_to_segment(const Eigen::Vector2d & p, const Segment & s)
{
  const Eigen::Vector2d ab = s.b - s.a;
  const double len2 = ab.squaredNorm();
  const double t = len2 == 0.0 ? 0.0 : std::clamp((p - s.a).dot(ab) / len2, 0.0, 1.0);
  return (s.a + t * ab - p).norm();
}

// Length, total turning and wall clearance of a path, measured every 5 cm.
void report(
  const char * name, double ms, std::size_t expanded, const std::vector<Eigen::Vector2d> & points,
  const std::vector<Segment> & walls)
{
  const auto dense = maze_runner::control::densify(points, 0.05);
  double length = 0.0;
  double turning = 0.0;
  double clearance = 1e9;
  for (std::size_t i = 0; i < dense.size(); ++i) {
    for (const auto & s : walls) {
      clearance = std::min(clearance, distance_to_segment(dense[i].head<2>(), s));
    }
    if (i > 0) {
      length += (dense[i].head<2>() - dense[i - 1].head<2>()).norm();
      turning += std::abs(maze_runner::wrap_angle(dense[i](2) - dense[i - 1](2)));
    }
  }
  std::printf(
    "%-34s %9.1f ms %10zu %8.1f m %8.0f deg %6.2f m %6zu\n", name, ms, expanded, length,
    turning * 180.0 / M_PI, clearance, points.size());
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 8) {
    std::fprintf(
      stderr, "usage: %s WORLD_SDF MODEL START_X START_Y START_YAW GOAL_X GOAL_Y "
      "[RESOLUTION=0.1] [INFLATION=0.6] [RUNS=20]\n", argv[0]);
    return 2;
  }
  const Eigen::Vector3d start(std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]));
  const Eigen::Vector2d goal(std::atof(argv[6]), std::atof(argv[7]));
  const double resolution = argc > 8 ? std::atof(argv[8]) : 0.1;
  const double inflation = argc > 9 ? std::atof(argv[9]) : 0.6;
  const int runs = argc > 10 ? std::atoi(argv[10]) : 20;

  const auto walls = wall_segments(maze_runner::WorldMapLoader(argv[1]).load(argv[2]));
  std::optional<OccupancyGrid> grid;
  const double grid_ms = median_ms(runs, [&] { grid.emplace(walls, resolution, inflation); });
  std::printf(
    "%zu wall segments, grid %d x %d at %.2f m, inflation %.2f m, built in %.1f ms (median of %d)\n\n",
    walls.size(), grid->width(), grid->height(), resolution, inflation, grid_ms, runs);
  std::printf(
    "%-34s %12s %10s %10s %12s %8s %6s\n", "planner", "time", "expanded", "length", "turning",
    "clear", "points");

  std::optional<GridPath> raw;
  std::vector<Eigen::Vector2d> smooth;
  const double astar_ms = median_ms(runs, [&] { raw = astar(*grid, start.head<2>(), goal); });
  if (!raw) {
    std::printf("A*: no path\n");
    return 1;
  }
  const double shortcut_ms = median_ms(runs, [&] { smooth = shortcut(*grid, raw->points); });
  report("A* (grid path)", astar_ms, raw->expanded, raw->points, walls);
  report("A* + shortcut", astar_ms + shortcut_ms, raw->expanded, smooth, walls);

  const auto hybrid = [&](const char * name, HybridAStarConfig cfg, int n) {
      std::optional<PosePath> path;
      const double ms = median_ms(n, [&] { path = hybrid_astar(*grid, start, goal, cfg); });
      if (!path) {
        std::printf("%-34s no path within %zu expansions (%.0f ms)\n", name, cfg.max_expansions, ms);
        return;
      }
      std::vector<Eigen::Vector2d> pts;
      for (const auto & p : path->poses) {
        pts.push_back(p.head<2>());
      }
      report(name, ms, path->expanded, pts, walls);
    };

  HybridAStarConfig cfg;
  hybrid("Hybrid A* r=1.0 (grid heuristic)", cfg, runs);
  cfg.min_turn_radius = 0.5;
  hybrid("Hybrid A* r=0.5 (grid heuristic)", cfg, runs);
  cfg.min_turn_radius = 1.0;
  cfg.obstacle_heuristic = false;
  hybrid("Hybrid A* r=1.0 (euclidean only)", cfg, 1);
  return 0;
}
