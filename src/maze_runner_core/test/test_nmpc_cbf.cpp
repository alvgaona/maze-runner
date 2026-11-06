#include <chrono>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <optional>

#include <gtest/gtest.h>

#include "maze_runner_core/nmpc_cbf.hpp"
#include "maze_runner_core/path_tracker.hpp"

using maze_runner::NmpcCbf;
using maze_runner::NmpcCbfConfig;

namespace
{

constexpr double kPi = std::numbers::pi;

// Straight reference along +x from the origin at speed v.
std::vector<Eigen::Vector3d> straight_ref(double v, double dt, int n)
{
  std::vector<Eigen::Vector3d> ref;
  for (int k = 0; k <= n; ++k) {
    ref.emplace_back(v * dt * k, 0.0, 0.0);
  }
  return ref;
}

}  // namespace

TEST(NmpcCbf, TracksStraightReference)
{
  NmpcCbf mpc;
  const auto u = mpc.compute({0.0, 0.0, 0.0}, straight_ref(0.5, 0.1, 20), {});
  EXPECT_TRUE(mpc.last_solve_succeeded());
  EXPECT_NEAR(u(0), 0.5, 0.15);
  EXPECT_NEAR(u(1), 0.0, 0.05);
}

TEST(NmpcCbf, RespectsVelocityLimits)
{
  NmpcCbf mpc;
  // Reference running away at 5 m/s, and to the left.
  auto ref = straight_ref(5.0, 0.1, 20);
  for (auto & r : ref) {
    r(1) = 3.0;
  }
  const auto u = mpc.compute({0.0, 0.0, 0.0}, ref, {});
  EXPECT_TRUE(mpc.last_solve_succeeded());
  EXPECT_LE(u(0), 1.0 + 1e-6);
  EXPECT_GE(u(0), 0.0 - 1e-6);
  EXPECT_LE(std::abs(u(1)), 2.0 + 1e-6);
}

TEST(NmpcCbf, HeadingReferenceAcrossPiDoesNotSpin)
{
  // Robot facing ~+pi, reference facing ~-pi: same direction, so no big turn.
  NmpcCbf mpc;
  std::vector<Eigen::Vector3d> ref;
  for (int k = 0; k <= 20; ++k) {
    ref.emplace_back(-0.05 * k, 0.0, -kPi + 0.01);
  }
  const auto u = mpc.compute({0.0, 0.0, kPi - 0.01}, ref, {});
  EXPECT_TRUE(mpc.last_solve_succeeded());
  EXPECT_LT(std::abs(u(1)), 0.5);
}

TEST(NmpcCbf, StopsBeforeWallAcrossPath)
{
  // Path runs into a wall at x = 2 (no slack): the robot must stop safety_radius short.
  const double dt = 0.1;
  NmpcCbfConfig cfg;
  cfg.use_slack = false;
  NmpcCbf mpc(cfg);
  Eigen::Vector3d x(0.0, 0.0, 0.0);
  for (int step = 0; step < 100; ++step) {
    std::vector<Eigen::Vector2d> scan;
    for (double y = -2.0; y <= 2.0; y += 0.02) {
      scan.emplace_back(2.0, y);
    }
    std::vector<Eigen::Vector3d> ref;
    for (int k = 0; k <= 20; ++k) {
      ref.emplace_back(x(0) + 0.05 * k, 0.0, 0.0);
    }
    const auto u = mpc.compute(x, ref, scan);
    ASSERT_TRUE(mpc.last_solve_succeeded());
    x(0) += dt * u(0) * std::cos(x(2));
    x(1) += dt * u(0) * std::sin(x(2));
    x(2) += dt * u(1);
  }
  EXPECT_LE(x(0), 2.0 - cfg.safety_radius + 1e-3);
  EXPECT_GT(x(0), 2.0 - cfg.safety_radius - 0.2);  // and it does get there, not stuck far back
}

TEST(NmpcCbf, IgnoresPointsOutsideRangeAndFov)
{
  NmpcCbf mpc;
  mpc.compute(
    {0.0, 0.0, 0.0}, straight_ref(0.5, 0.1, 20),
    // 5 m ahead (out of range), 1 m at 135 deg (outside the fov), and a bad point.
    {{5.0, 0.0}, {std::cos(0.75 * kPi), std::sin(0.75 * kPi)},
     {std::numeric_limits<double>::infinity(), 0.0}});
  EXPECT_EQ(mpc.last_obstacle_count(), 0);
}

TEST(NmpcCbf, ClosedLoopFollowsLPath)
{
  // Unicycle simulation following an L path with the forward-gated tracker.
  const double dt = 0.1;
  const auto path = maze_runner::densify({{0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}}, 0.1);
  maze_runner::PathTracker tracker(path);
  NmpcCbf mpc;

  Eigen::Vector3d x(0.0, 0.0, 0.0);
  double max_cross_track = 0.0;
  double total_ms = 0.0;
  double worst_ms = 0.0;
  int steps = 0;
  for (; steps < 300 && !tracker.reached_goal(x.head<2>(), 0.15); ++steps) {
    const std::size_t idx = tracker.update(x.head<2>());
    const std::vector<Eigen::Vector3d> ref(path.begin() + idx, path.end());

    const auto t0 = std::chrono::steady_clock::now();
    const auto u = mpc.compute(x, ref, {});
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    total_ms += ms;
    worst_ms = std::max(worst_ms, ms);

    x(0) += dt * u(0) * std::cos(x(2));
    x(1) += dt * u(0) * std::sin(x(2));
    x(2) += dt * u(1);

    // Distance to the L (either leg).
    const double d1 = (x(0) <= 4.0) ? std::abs(x(1)) : std::hypot(x(0) - 4.0, x(1));
    const double d2 = (x(1) >= 0.0) ? std::abs(x(0) - 4.0) : std::hypot(x(0) - 4.0, x(1));
    max_cross_track = std::max(max_cross_track, std::min(d1, d2));
  }

  std::printf(
    "closed loop: %d steps, solve avg %.1f ms, worst %.1f ms, max cross-track %.3f m\n", steps,
    total_ms / steps, worst_ms, max_cross_track);
  EXPECT_TRUE(tracker.reached_goal(x.head<2>(), 0.15));
  EXPECT_LT(max_cross_track, 0.3);
}

namespace
{

// Lidar points on the visible side of a round pillar.
std::vector<Eigen::Vector2d> pillar_scan(
  const Eigen::Vector3d & x, const Eigen::Vector2d & c, double radius)
{
  std::vector<Eigen::Vector2d> scan;
  for (int i = 0; i < 360; ++i) {
    const double a = 2.0 * kPi * i / 360.0;
    const Eigen::Vector2d n(std::cos(a), std::sin(a));
    const Eigen::Vector2d p = c + radius * n;
    const Eigen::Vector2d rel = p - x.head<2>();
    if (n.dot(rel) >= 0.0) {
      continue;  // back side, hidden
    }
    scan.push_back(p);
  }
  return scan;
}


struct PillarRun
{
  double min_clearance = 1e9;
  bool reached_goal = false;
  int failed_solves = 0;
  int turn_reversals = 0;  // sign flips of a sizeable turn command
  Eigen::Vector3d end;
};

// Straight path along +x from origin to origin + (6, 0) past a round pillar at
// origin + (3, pillar_y).
PillarRun run_past_pillar(
  double pillar_y, const NmpcCbfConfig & cfg, const Eigen::Vector2d & origin = {0.0, 0.0},
  std::optional<double> wall_y = std::nullopt)
{
  const double dt = 0.1;
  const Eigen::Vector2d pillar = origin + Eigen::Vector2d(3.0, pillar_y);
  const double pillar_radius = 0.15;
  const auto path = maze_runner::densify({origin, origin + Eigen::Vector2d(6.0, 0.0)}, 0.05);
  maze_runner::PathTracker tracker(path);
  NmpcCbf mpc(cfg);

  PillarRun run;
  double last_turn = 0.0;
  Eigen::Vector3d x(origin.x(), origin.y(), 0.0);
  for (int steps = 0; steps < 400 && !tracker.reached_goal(x.head<2>(), 0.15); ++steps) {
    const std::size_t idx = tracker.update(x.head<2>());
    const std::vector<Eigen::Vector3d> ref(path.begin() + idx, path.end());
    auto scan = pillar_scan(x, pillar, pillar_radius);
    if (wall_y) {  // straight wall along x, as lidar points every 2 cm
      for (double wx = origin.x() - 2.0; wx < origin.x() + 8.0; wx += 0.02) {
        scan.emplace_back(wx, origin.y() + *wall_y);
      }
    }
    const auto u = mpc.compute(x, ref, scan);
    run.failed_solves += !mpc.last_solve_succeeded();
    if (std::abs(u(1)) > 0.5) {
      run.turn_reversals += (last_turn * u(1) < 0.0);
      last_turn = u(1);
    }
    x(0) += dt * u(0) * std::cos(x(2));
    x(1) += dt * u(0) * std::sin(x(2));
    x(2) += dt * u(1);
    run.min_clearance = std::min(run.min_clearance, (x.head<2>() - pillar).norm() - pillar_radius);
  }
  run.reached_goal = tracker.reached_goal(x.head<2>(), 0.15);
  run.end = x;
  std::printf(
    "pillar at y=%.2f: min clearance %.3f m, end (%.2f, %.2f), %d failed solves, %d turn "
    "reversals\n", pillar_y, run.min_clearance, x(0), x(1), run.failed_solves, run.turn_reversals);
  return run;
}

}  // namespace

TEST(NmpcCbf, ClosedLoopSwervesAroundPillarNearPath)
{
  // The path passes 15 cm from the pillar surface: the robot must swerve, keep
  // ~safety_radius (soft constraint), and get back to the goal.
  const NmpcCbfConfig cfg;
  const auto run = run_past_pillar(0.3, cfg);
  EXPECT_GT(run.min_clearance, 0.9 * cfg.safety_radius);
  EXPECT_TRUE(run.reached_goal);
}

TEST(NmpcCbf, ClosedLoopStopsSafelyBeforePillarOnPath)
{
  // Pillar dead on the path: a local controller can't find the way around (that needs a
  // planner), but it must stop short instead of hitting it.
  const NmpcCbfConfig cfg;
  const auto run = run_past_pillar(0.0, cfg);
  EXPECT_GT(run.min_clearance, 0.9 * cfg.safety_radius);
  EXPECT_LT(run.end(0), 3.0);
}

TEST(NmpcCbf, ClosedLoopSwervesFarFromOrigin)
{
  // Same as near the origin, but where the maze is: the solver must not depend on the
  // robot being near (0, 0) (it did when the state guess defaulted to zeros).
  const NmpcCbfConfig cfg;
  const auto run = run_past_pillar(0.3, cfg, {20.0, 40.0});
  EXPECT_GT(run.min_clearance, 0.9 * cfg.safety_radius);
  EXPECT_TRUE(run.reached_goal);
  EXPECT_EQ(run.failed_solves, 0);
}

TEST(NmpcCbf, ClosedLoopPassesBetweenPillarAndWallSmoothly)
{
  // Maze scene: pillar 0.3 m left of the path, wall 1.42 m right. The gap the center may
  // use is ~0.57 m wide; the robot must get through without flip-flopping its turn.
  const NmpcCbfConfig cfg;
  const auto run = run_past_pillar(0.3, cfg, {0.0, 0.0}, -1.42);
  EXPECT_GT(run.min_clearance, 0.9 * cfg.safety_radius);
  EXPECT_TRUE(run.reached_goal);
  EXPECT_LE(run.turn_reversals, 4);
}
