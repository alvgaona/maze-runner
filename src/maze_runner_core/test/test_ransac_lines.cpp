#include <cmath>
#include <numbers>
#include <random>

#include <gtest/gtest.h>

#include "maze_runner_core/ransac_lines.hpp"

using maze_runner::ObservedLine;
using maze_runner::ransac_lines;

namespace
{

constexpr double kPi = std::numbers::pi;

// Points along the segment a -> b, every `step` meters, with optional gaussian noise.
void add_segment(
  std::vector<Eigen::Vector2d> & pts, Eigen::Vector2d a, Eigen::Vector2d b, double step,
  double noise = 0.0, unsigned seed = 1)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, noise > 0 ? noise : 1.0);
  const int count = static_cast<int>((b - a).norm() / step);
  for (int i = 0; i <= count; ++i) {
    Eigen::Vector2d p = a + (b - a) * (static_cast<double>(i) / count);
    if (noise > 0) {
      p += Eigen::Vector2d(n(rng), n(rng));
    }
    pts.push_back(p);
  }
}

bool has_line(const std::vector<ObservedLine> & lines, double alpha, double d, double tol_a, double tol_d)
{
  for (const auto & l : lines) {
    const double da = std::atan2(std::sin(l.alpha - alpha), std::cos(l.alpha - alpha));
    if (std::abs(da) < tol_a && std::abs(l.d - d) < tol_d) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(RansacLines, FindsTwoPerpendicularWalls)
{
  std::vector<Eigen::Vector2d> pts;
  add_segment(pts, {2.0, -1.0}, {2.0, 1.0}, 0.05);    // wall ahead: alpha 0, d 2
  add_segment(pts, {-1.0, -1.5}, {1.5, -1.5}, 0.05);  // wall right: alpha -pi/2, d 1.5

  std::mt19937 rng(42);
  const auto lines = ransac_lines(pts, 0.015, 3, rng);

  ASSERT_EQ(lines.size(), 2u);
  EXPECT_TRUE(has_line(lines, 0.0, 2.0, 1e-9, 1e-9));
  EXPECT_TRUE(has_line(lines, -kPi / 2, 1.5, 1e-9, 1e-9));
  // Noise-free fit hits the uncertainty floors.
  for (const auto & l : lines) {
    EXPECT_DOUBLE_EQ(l.sigma_d, 0.02);
    EXPECT_DOUBLE_EQ(l.sigma_alpha, kPi / 180.0);
  }
}

TEST(RansacLines, CanonicalFormHasNonNegativeDistance)
{
  std::vector<Eigen::Vector2d> pts;
  add_segment(pts, {-3.0, -1.0}, {-3.0, 1.0}, 0.05);  // wall behind
  add_segment(pts, {-1.0, 2.5}, {1.0, 2.5}, 0.05);    // wall left

  std::mt19937 rng(7);
  const auto lines = ransac_lines(pts, 0.015, 3, rng);

  ASSERT_EQ(lines.size(), 2u);
  EXPECT_TRUE(has_line(lines, kPi, 3.0, 1e-9, 1e-9));
  EXPECT_TRUE(has_line(lines, kPi / 2, 2.5, 1e-9, 1e-9));
  for (const auto & l : lines) {
    EXPECT_GE(l.d, 0.0);
  }
}

TEST(RansacLines, NoisyWallWithinTolerance)
{
  std::vector<Eigen::Vector2d> pts;
  add_segment(pts, {3.0, -2.0}, {3.0, 2.0}, 0.02, 0.005);

  std::mt19937 rng(3);
  const auto lines = ransac_lines(pts, 0.015, 3, rng);

  ASSERT_GE(lines.size(), 1u);
  EXPECT_NEAR(lines[0].alpha, 0.0, 0.01);
  EXPECT_NEAR(lines[0].d, 3.0, 0.01);
}

TEST(RansacLines, TooFewPointsGivesNothing)
{
  std::mt19937 rng(1);
  EXPECT_TRUE(ransac_lines({{1.0, 0.0}, {1.0, 1.0}, {1.0, 2.0}}, 0.015, 3, rng).empty());
}

TEST(RansacLines, ScatteredPointsGiveNoLongLines)
{
  // Points on a circle: any 3 can't be collinear within 1.5 cm at this spacing.
  std::vector<Eigen::Vector2d> pts;
  for (int i = 0; i < 36; ++i) {
    const double a = 2 * kPi * i / 36;
    pts.emplace_back(2.0 * std::cos(a), 2.0 * std::sin(a));
  }
  std::mt19937 rng(5);
  EXPECT_TRUE(ransac_lines(pts, 0.015, 3, rng).empty());
}
