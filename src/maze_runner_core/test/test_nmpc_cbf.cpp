#include <chrono>
#include <cmath>
#include <cstdio>
#include <numbers>

#include <gtest/gtest.h>

#include "maze_runner_core/nmpc_cbf.hpp"
#include "maze_runner_core/path_tracker.hpp"

using maze_runner::NmpcCbf;
using maze_runner::NmpcCbfConfig;
using maze_runner::ScanPoint;

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

TEST(NmpcCbf, HardBarrierStopsBeforeObstacleAhead)
{
  // Point dead ahead at 0.6 m, d_safe 0.5: with phi = 0 the CBF reduces to
  // v <= alpha (r - d_safe) / 2.
  NmpcCbfConfig cfg;
  cfg.use_slack = false;
  NmpcCbf mpc(cfg);
  const auto u = mpc.compute({0.0, 0.0, 0.0}, straight_ref(0.5, 0.1, 20), {{0.6, 0.0}});
  EXPECT_TRUE(mpc.last_solve_succeeded());
  EXPECT_EQ(mpc.last_obstacle_count(), 1);
  EXPECT_LE(u(0), cfg.alpha_cbf * (0.6 - cfg.safety_radius) / 2 + 1e-6);
}

TEST(NmpcCbf, SoftBarrierSlowsDown)
{
  // With the MATLAB default penalty (1000) the slack is cheaper than the tracking cost
  // and the barrier barely acts; a strong penalty makes it bind.
  NmpcCbfConfig cfg;
  cfg.slack_penalty = 1e6;
  NmpcCbf free(cfg);
  NmpcCbf blocked(cfg);
  const auto ref = straight_ref(0.5, 0.1, 20);
  const double v_free = free.compute({0.0, 0.0, 0.0}, ref, {})(0);
  const double v_blocked = blocked.compute({0.0, 0.0, 0.0}, ref, {{0.6, 0.0}})(0);
  EXPECT_LT(v_blocked, 0.5 * v_free);
}

TEST(NmpcCbf, IgnoresPointsOutsideRangeAndFov)
{
  NmpcCbf mpc;
  mpc.compute(
    {0.0, 0.0, 0.0}, straight_ref(0.5, 0.1, 20),
    {{5.0, 0.0}, {1.0, kPi / 2}, {std::numeric_limits<double>::infinity(), 0.0}});
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
