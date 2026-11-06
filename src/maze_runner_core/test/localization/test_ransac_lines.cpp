#include <cmath>
#include <numbers>
#include <random>

#include <gtest/gtest.h>

#include "maze_runner_core/localization/ransac_lines.hpp"

using maze_runner::localization::ObservedLine;
using maze_runner::localization::ransac_lines;
using maze_runner::localization::RansacConfig;

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

  const auto lines = ransac_lines(pts, RansacConfig{.distance_threshold = 0.015, .min_points = 3, .seed = 42});

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

  const auto lines = ransac_lines(pts, RansacConfig{.distance_threshold = 0.015, .min_points = 3, .seed = 7});

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

  const auto lines = ransac_lines(pts, RansacConfig{.distance_threshold = 0.015, .min_points = 3, .seed = 3});

  ASSERT_GE(lines.size(), 1u);
  EXPECT_NEAR(lines[0].alpha, 0.0, 0.01);
  EXPECT_NEAR(lines[0].d, 3.0, 0.01);
}

TEST(RansacLines, TooFewPointsGivesNothing)
{
  EXPECT_TRUE(ransac_lines({{1.0, 0.0}, {1.0, 1.0}, {1.0, 2.0}}, RansacConfig{.distance_threshold = 0.015, .min_points = 3, .seed = 1}).empty());
}

TEST(RansacLines, ScatteredPointsGiveNoLongLines)
{
  // Points on a circle: any 3 can't be collinear within 1.5 cm at this spacing.
  std::vector<Eigen::Vector2d> pts;
  for (int i = 0; i < 36; ++i) {
    const double a = 2 * kPi * i / 36;
    pts.emplace_back(2.0 * std::cos(a), 2.0 * std::sin(a));
  }
  EXPECT_TRUE(ransac_lines(pts, RansacConfig{.distance_threshold = 0.015, .min_points = 3, .seed = 5}).empty());
}

TEST(RansacLines, SameInputSameLines)
{
  // Pure: no RNG state carried between calls.
  std::vector<Eigen::Vector2d> pts;
  add_segment(pts, {2.0, -1.0}, {2.0, 1.0}, 0.05, 0.01);
  add_segment(pts, {-1.0, -1.5}, {1.5, -1.5}, 0.05, 0.01, 2);
  const RansacConfig cfg{.distance_threshold = 0.03, .min_points = 3};
  const auto a = ransac_lines(pts, cfg);
  const auto b = ransac_lines(pts, cfg);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].alpha, b[i].alpha);
    EXPECT_EQ(a[i].d, b[i].d);
  }
}
