#include <cmath>
#include <numbers>
#include <stdexcept>

#include <gtest/gtest.h>

#include "maze_runner_core/angles.hpp"
#include "maze_runner_core/planning/astar.hpp"
#include "maze_runner_core/planning/hybrid_astar.hpp"
#include "maze_runner_core/planning/occupancy_grid.hpp"

using maze_runner::Segment;
using maze_runner::planning::astar;
using maze_runner::planning::distance_to;
using maze_runner::planning::hybrid_astar;
using maze_runner::planning::HybridAStarConfig;
using maze_runner::planning::OccupancyGrid;
using maze_runner::planning::shortcut;
using V = Eigen::Vector2d;

namespace
{

constexpr double kPi = std::numbers::pi;

// 10 x 6 m room split by a wall at x = 5 from the floor up to `wall_top`.
// The only way across is over the top of that wall.
OccupancyGrid room(double wall_top = 4.0)
{
  return OccupancyGrid(
    {{{0, 0}, {10, 0}}, {{10, 0}, {10, 6}}, {{10, 6}, {0, 6}}, {{0, 6}, {0, 0}},
      {{5, 0}, {5, wall_top}}},
    0.1, 0.5);
}

double length(const std::vector<V> & pts)
{
  double l = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    l += (pts[i] - pts[i - 1]).norm();
  }
  return l;
}

// Every point of every straight piece is in a free cell.
bool all_free(const OccupancyGrid & grid, const std::vector<V> & pts)
{
  for (std::size_t i = 1; i < pts.size(); ++i) {
    for (double t = 0.0; t <= 1.0; t += 0.01) {
      if (!grid.free(V(pts[i - 1] + t * (pts[i] - pts[i - 1])))) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

TEST(OccupancyGrid, BlocksInflatedWallsAndOutside)
{
  const auto grid = room();
  EXPECT_FALSE(grid.free(V(5.0, 2.0)));    // on the wall
  EXPECT_FALSE(grid.free(V(5.45, 2.0)));   // inside the inflation
  EXPECT_TRUE(grid.free(V(5.6, 2.0)));
  EXPECT_TRUE(grid.free(V(5.0, 4.6)));     // above the wall top
  EXPECT_FALSE(grid.free(V(-2.0, 3.0)));   // off the grid
}

TEST(OccupancyGrid, RejectsInflationThatLetsWallsSlipThrough)
{
  EXPECT_THROW(OccupancyGrid({{{0, 0}, {1, 0}}}, 0.1, 0.05), std::invalid_argument);
}

TEST(AStar, GoesOverTheWallAndShortcutKeepsItFree)
{
  const auto grid = room();
  const auto path = astar(grid, {2.0, 1.0}, {8.0, 1.0});
  ASSERT_TRUE(path);
  EXPECT_TRUE(all_free(grid, path->points));
  double top = 0.0;
  for (const auto & p : path->points) {
    top = std::max(top, p.y());
  }
  EXPECT_GT(top, 4.5);

  const auto short_path = shortcut(grid, path->points);
  EXPECT_LE(short_path.size(), 6u);  // from ~100 cells to a few corners
  EXPECT_TRUE(short_path.front().isApprox(path->points.front()));
  EXPECT_TRUE(short_path.back().isApprox(path->points.back()));
  EXPECT_TRUE(all_free(grid, short_path));
  EXPECT_LE(length(short_path), length(path->points));
}

TEST(AStar, NoPathWhenBlockedOrUnreachable)
{
  EXPECT_FALSE(astar(room(), {2.0, 1.0}, {5.0, 1.0}));  // goal on the wall
  EXPECT_FALSE(astar(room(6.0), {2.0, 1.0}, {8.0, 1.0}));  // wall closes the room
}

TEST(AStar, DistanceFieldMatchesPathLength)
{
  const auto grid = room();
  const auto path = astar(grid, {2.0, 1.0}, {8.0, 1.0});
  ASSERT_TRUE(path);
  const auto field = distance_to(grid, {8.0, 1.0});
  EXPECT_NEAR(field[grid.index(grid.cell_of({2.0, 1.0}))], length(path->points), 1e-9);
  EXPECT_TRUE(std::isinf(distance_to(room(6.0), {8.0, 1.0})[grid.index(grid.cell_of({2.0, 1.0}))]));
}

TEST(HybridAStar, ReachesGoalWithBoundedCurvature)
{
  const auto grid = room();
  const HybridAStarConfig cfg;
  const auto path = hybrid_astar(grid, {2.0, 1.0, kPi / 2}, {8.0, 1.0}, cfg);
  ASSERT_TRUE(path);
  EXPECT_LE((path->poses.back().head<2>() - V(8.0, 1.0)).norm(), cfg.goal_tolerance);

  std::vector<V> pts;
  for (std::size_t i = 0; i < path->poses.size(); ++i) {
    pts.push_back(path->poses[i].head<2>());
    if (i > 0) {
      const double turn = maze_runner::wrap_angle(path->poses[i](2) - path->poses[i - 1](2));
      EXPECT_LE(std::abs(turn), cfg.step / cfg.min_turn_radius + 1e-9);
      EXPECT_NEAR((pts[i] - pts[i - 1]).norm(), cfg.step, 0.01);  // chord of a short arc
    }
  }
  EXPECT_TRUE(all_free(grid, pts));
}

TEST(HybridAStar, ObstacleHeuristicExpandsLess)
{
  const auto grid = room();
  HybridAStarConfig cfg;
  const auto with = hybrid_astar(grid, {2.0, 1.0, kPi / 2}, {8.0, 1.0}, cfg);
  cfg.obstacle_heuristic = false;
  const auto without = hybrid_astar(grid, {2.0, 1.0, kPi / 2}, {8.0, 1.0}, cfg);
  ASSERT_TRUE(with);
  ASSERT_TRUE(without);
  EXPECT_LT(with->expanded, without->expanded);
}

TEST(HybridAStar, NoPathWhenUnreachable)
{
  EXPECT_FALSE(hybrid_astar(room(6.0), {2.0, 1.0, 0.0}, {8.0, 1.0}));
  EXPECT_FALSE(hybrid_astar(room(), {5.0, 1.0, 0.0}, {8.0, 1.0}));  // start on the wall
}
