#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "maze_runner_core/ekf_lines.hpp"

using maze_runner::EkfLines;
using maze_runner::Line;
using maze_runner::ObservedLine;

namespace
{

constexpr double kPi = std::numbers::pi;

// 10 x 10 m box: walls x=0, x=10, y=0, y=10 (normals point away from the origin side).
std::vector<Line> box_map()
{
  return {{0.0, 0.0}, {0.0, 10.0}, {kPi / 2, 0.0}, {kPi / 2, 10.0}};
}

EkfLines make_ekf(const Eigen::Vector3d & x0, double p0 = 0.1)
{
  return EkfLines(
    x0, Eigen::Matrix3d::Identity() * p0, box_map(),
    Eigen::Vector2d(0.01, 0.001).asDiagonal(), 9.21);
}

// What the robot at `pose` would observe for map line `m`, in canonical form (d >= 0).
ObservedLine observe(const Line & m, const Eigen::Vector3d & pose, double sigma = 0.02)
{
  double alpha = m.alpha - pose(2);
  double d = m.d - std::cos(m.alpha) * pose(0) - std::sin(m.alpha) * pose(1);
  if (d < 0) {
    alpha += kPi;
    d = -d;
  }
  return {std::atan2(std::sin(alpha), std::cos(alpha)), d, sigma, sigma};
}

}  // namespace

TEST(EkfLines, PredictStraight)
{
  auto ekf = make_ekf({1.0, 2.0, 0.0});
  ekf.predict(0.5, 0.0);
  EXPECT_NEAR(ekf.state()(0), 1.5, 1e-12);
  EXPECT_NEAR(ekf.state()(1), 2.0, 1e-12);
  EXPECT_NEAR(ekf.state()(2), 0.0, 1e-12);
}

TEST(EkfLines, PredictUsesMidpointHeading)
{
  auto ekf = make_ekf({0.0, 0.0, 0.0});
  ekf.predict(1.0, kPi / 2);
  // Moves along the mid heading pi/4, ends facing pi/2.
  EXPECT_NEAR(ekf.state()(0), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(ekf.state()(1), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(ekf.state()(2), kPi / 2, 1e-12);
}

TEST(EkfLines, PredictWrapsHeading)
{
  auto ekf = make_ekf({0.0, 0.0, kPi - 0.1});
  ekf.predict(0.0, 0.3);
  EXPECT_NEAR(ekf.state()(2), -kPi + 0.2, 1e-12);
}

TEST(EkfLines, PredictCovariance)
{
  // theta = 0, delta_theta = 0: A = [1 0 0; 0 1 d; 0 0 1], W = [1 0; 0 d/2; 0 1].
  auto ekf = make_ekf({0.0, 0.0, 0.0}, 0.1);
  ekf.predict(2.0, 0.0);
  const auto & P = ekf.covariance();
  EXPECT_NEAR(P(0, 0), 0.1 + 0.01, 1e-12);
  EXPECT_NEAR(P(1, 1), 0.1 + 4.0 * 0.1 + 1.0 * 0.001, 1e-12);
  EXPECT_NEAR(P(1, 2), 2.0 * 0.1 + 1.0 * 0.001, 1e-12);
  EXPECT_NEAR(P(2, 2), 0.1 + 0.001, 1e-12);
  EXPECT_TRUE(P.isApprox(P.transpose()));
}

TEST(EkfLines, LateralNoiseGrowsSidewaysOnly)
{
  // Heading +y: sideways is x. 2 m traveled with 0.01 m^2/m adds 0.02 to P(0,0) only.
  const Eigen::Matrix2d Q = Eigen::Vector2d(0.01, 0.001).asDiagonal();
  EkfLines plain({0.0, 0.0, kPi / 2}, Eigen::Matrix3d::Identity() * 0.1, box_map(), Q, 9.21);
  EkfLines slip({0.0, 0.0, kPi / 2}, Eigen::Matrix3d::Identity() * 0.1, box_map(), Q, 9.21, 0.01);
  plain.predict(-2.0, 0.0);  // reversing counts too
  slip.predict(-2.0, 0.0);

  Eigen::Matrix3d diff = slip.covariance() - plain.covariance();
  EXPECT_NEAR(diff(0, 0), 0.02, 1e-12);
  diff(0, 0) = 0.0;
  EXPECT_NEAR(diff.cwiseAbs().maxCoeff(), 0.0, 1e-12);
}

TEST(EkfLines, UpdateCorrectsTowardTruth)
{
  const Eigen::Vector3d truth(3.0, 4.0, 0.1);
  auto ekf = make_ekf({3.2, 3.85, 0.15});

  std::vector<ObservedLine> obs;
  for (const auto & m : box_map()) {
    obs.push_back(observe(m, truth));
  }
  const double trace_before = ekf.covariance().trace();

  EXPECT_EQ(ekf.update(obs), 4);
  EXPECT_NEAR(ekf.state()(0), truth(0), 0.02);
  EXPECT_NEAR(ekf.state()(1), truth(1), 0.02);
  EXPECT_NEAR(ekf.state()(2), truth(2), 0.01);
  EXPECT_LT(ekf.covariance().trace(), trace_before);
}

TEST(EkfLines, NoMatchWhenTooFar)
{
  auto ekf = make_ekf({5.0, 5.0, 0.0});
  // A wall at x=8 is not in the map; nearest x-wall predicts d=5 vs observed 3.
  EXPECT_EQ(ekf.update({{0.0, 3.0, 0.02, 0.02}}), 0);
  EXPECT_EQ(ekf.state(), Eigen::Vector3d(5.0, 5.0, 0.0));
}

TEST(EkfLines, MatchesWallBehindMapNormal)
{
  // Wall x=0 is stored as (0, 0); from x=3 the robot sees it at alpha=pi, d=3.
  auto ekf = make_ekf({3.2, 5.0, 0.0});
  EXPECT_EQ(ekf.update({{kPi, 3.0, 0.02, 0.02}}), 1);
  EXPECT_NEAR(ekf.state()(0), 3.0, 0.01);
}

TEST(EkfLines, MahalanobisGateRejects)
{
  // Tiny covariance + tiny noise: a 0.3 m error passes association but fails the gate.
  EkfLines ekf(
    {5.0, 5.0, 0.0}, Eigen::Matrix3d::Identity() * 1e-6, box_map(),
    Eigen::Matrix2d::Identity() * 1e-6, 9.21);
  EXPECT_EQ(ekf.update({{0.0, 5.3, 0.001, 0.001}}), 0);
  EXPECT_EQ(ekf.state(), Eigen::Vector3d(5.0, 5.0, 0.0));
}

TEST(EkfLines, ConvergesOverTrajectory)
{
  // Drive a square-ish loop with biased odometry; line updates keep the error bounded.
  Eigen::Vector3d truth(2.0, 2.0, 0.0);
  auto ekf = make_ekf(truth, 0.01);

  for (int k = 0; k < 200; ++k) {
    const double dd = 0.05;
    const double dth = (k % 50 == 49) ? kPi / 2 : 0.0;
    const double mid = truth(2) + dth / 2;
    truth(0) += dd * std::cos(mid);
    truth(1) += dd * std::sin(mid);
    truth(2) = std::atan2(std::sin(truth(2) + dth), std::cos(truth(2) + dth));

    ekf.predict(dd * 1.05, dth * 1.02);  // 5% / 2% odometry bias

    std::vector<ObservedLine> obs;
    for (const auto & m : box_map()) {
      obs.push_back(observe(m, truth));
    }
    ekf.update(obs);
  }
  EXPECT_NEAR(ekf.state()(0), truth(0), 0.05);
  EXPECT_NEAR(ekf.state()(1), truth(1), 0.05);
  EXPECT_NEAR(ekf.state()(2), truth(2), 0.02);
}
