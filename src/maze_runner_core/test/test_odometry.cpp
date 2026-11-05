#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "maze_runner_core/odometry.hpp"

using maze_runner::odometry_delta;

constexpr double kPi = std::numbers::pi;

TEST(Odometry, Straight)
{
  const auto delta = odometry_delta({1.0, 1.0, kPi / 2}, {1.0, 1.3, kPi / 2});
  EXPECT_NEAR(delta.d, 0.3, 1e-12);
  EXPECT_NEAR(delta.theta, 0.0, 1e-12);
}

TEST(Odometry, Reverse)
{
  const auto delta = odometry_delta({0.0, 0.0, 0.0}, {-0.2, 0.0, 0.0});
  EXPECT_NEAR(delta.d, -0.2, 1e-12);
}

TEST(Odometry, ArcLengthOnTurn)
{
  // Quarter circle of radius 1 from (0,0) heading +x to (1,1) heading +y.
  const auto delta = odometry_delta({0.0, 0.0, 0.0}, {1.0, 1.0, kPi / 2});
  EXPECT_NEAR(delta.d, kPi / 2, 1e-12);
  EXPECT_NEAR(delta.theta, kPi / 2, 1e-12);
}

TEST(Odometry, HeadingWraps)
{
  const auto delta = odometry_delta({0.0, 0.0, kPi - 0.05}, {0.0, 0.0, -kPi + 0.05});
  EXPECT_NEAR(delta.theta, 0.1, 1e-12);
  EXPECT_NEAR(delta.d, 0.0, 1e-12);
}
