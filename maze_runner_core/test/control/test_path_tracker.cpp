#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "maze_runner_core/control/path_tracker.hpp"

using maze_runner::control::densify;
using maze_runner::control::PathTracker;

TEST(Densify, SpacingAndHeadings)
{
  const auto path = densify({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, 0.25);
  ASSERT_EQ(path.size(), 9u);  // 4 + 4 + final point
  EXPECT_TRUE(path.front().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_NEAR(path[4](2), std::numbers::pi / 2, 1e-12);  // first point of 2nd leg
  EXPECT_TRUE(path.back().isApprox(Eigen::Vector3d(1.0, 1.0, std::numbers::pi / 2)));
}

TEST(Densify, RejectsBadInput)
{
  EXPECT_THROW(densify({{0.0, 0.0}}, 0.1), std::invalid_argument);
  EXPECT_THROW(densify({{0.0, 0.0}, {0.0, 0.0}}, 0.1), std::invalid_argument);
}

TEST(PathTracker, StartsAtClosestPoint)
{
  // No lead: the NMPC reference must start where the robot is, or it outruns reference_speed.
  PathTracker tracker(densify({{0.0, 0.0}, {10.0, 0.0}}, 0.1));
  EXPECT_EQ(tracker.update({3.0, 0.05}), 30u);
}

TEST(PathTracker, NeverMovesBackward)
{
  PathTracker tracker(densify({{0.0, 0.0}, {10.0, 0.0}}, 0.1));
  const auto a = tracker.update({5.0, 0.0});
  const auto b = tracker.update({1.0, 0.0});  // robot estimate jumps back
  EXPECT_GE(b, a);
}

TEST(PathTracker, WindowPreventsSkippingAhead)
{
  // Out and back along the same line: from the start, the return leg is equally close,
  // but the window keeps the tracker on the outbound leg.
  PathTracker tracker(densify({{0.0, 0.0}, {30.0, 0.0}, {0.0, 0.1}}, 0.1), 50);
  tracker.update({0.0, 0.0});
  EXPECT_LT(tracker.update({0.5, 0.08}), 60u);
}

TEST(PathTracker, ReachesGoal)
{
  PathTracker tracker(densify({{0.0, 0.0}, {1.0, 0.0}}, 0.1));
  tracker.update({0.5, 0.0});
  EXPECT_FALSE(tracker.reached_goal({0.5, 0.0}, 0.1));
  tracker.update({1.0, 0.0});
  EXPECT_TRUE(tracker.reached_goal({1.0, 0.0}, 0.1));
}
