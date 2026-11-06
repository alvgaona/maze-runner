#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "maze_runner_core/angles.hpp"
#include "maze_runner_core/localization/ekf_lines_slam.hpp"

using namespace maze_runner::localization;

namespace
{

constexpr double kPi = std::numbers::pi;

struct Wall
{
  Eigen::Vector2d a;
  Eigen::Vector2d b;
};

// What the robot at `pose` sees of `wall`: the line in canonical form plus the segment extent.
ObservedLine observe(const Eigen::Vector3d & pose, const Wall & wall, double sigma = 0.01)
{
  const double c = std::cos(pose(2));
  const double s = std::sin(pose(2));
  auto to_robot = [&](const Eigen::Vector2d & p) {
    const Eigen::Vector2d q = p - pose.head<2>();
    return Eigen::Vector2d(c * q.x() + s * q.y(), -s * q.x() + c * q.y());
  };
  const Eigen::Vector2d a = to_robot(wall.a);
  const Eigen::Vector2d b = to_robot(wall.b);
  const Eigen::Vector2d u = (b - a).normalized();
  Eigen::Vector2d n(-u.y(), u.x());
  if (n.dot(a) < 0) {
    n = -n;
  }
  const Eigen::Vector2d t(-n.y(), n.x());
  return {
    std::atan2(n.y(), n.x()), n.dot(a), sigma, sigma, std::min(t.dot(a), t.dot(b)), std::max(t.dot(a), t.dot(b))};
}

std::vector<ObservedLine> observe_all(const Eigen::Vector3d & pose, const std::vector<Wall> & walls)
{
  std::vector<ObservedLine> out;
  for (const auto & w : walls) {
    out.push_back(observe(pose, w));
  }
  return out;
}

EkfLinesSlamConfig config(int min_observations = 1)
{
  return {
    .filter = {.process_noise = Eigen::Vector2d(1e-4, 1e-4).asDiagonal()},
    .min_observations = min_observations};
}

}  // namespace

TEST(EkfLinesSlam, MeasureLineJacobians)
{
  const Eigen::Vector3d pose(1.2, -0.7, 0.4);
  const Line m{2.1, 3.5};
  const auto meas = measure_line(pose, m);
  const double h = 1e-6;

  for (int k = 0; k < 3; ++k) {
    Eigen::Vector3d p = pose;
    p(k) += h;
    const Eigen::Vector2d num = (measure_line(p, m).z - meas.z) / h;
    EXPECT_TRUE(num.isApprox(meas.H_pose.col(k), 1e-5)) << "pose column " << k;
  }
  const Eigen::Vector2d num_alpha = (measure_line(pose, {m.alpha + h, m.d}).z - meas.z) / h;
  const Eigen::Vector2d num_d = (measure_line(pose, {m.alpha, m.d + h}).z - meas.z) / h;
  EXPECT_TRUE(num_alpha.isApprox(meas.H_line.col(0), 1e-5));
  EXPECT_TRUE(num_d.isApprox(meas.H_line.col(1), 1e-5));
}

TEST(EkfLinesSlam, InitLineInvertsMeasureAndJacobians)
{
  const Eigen::Vector3d pose(1.2, -0.7, 0.4);
  const ObservedLine obs{0.3, 2.0, 0.01, 0.01};
  const auto init = init_line(pose, obs);

  const auto back = measure_line(pose, init.line);
  EXPECT_NEAR(back.z(0), obs.alpha, 1e-12);
  EXPECT_NEAR(back.z(1), obs.d, 1e-12);

  const double h = 1e-6;
  auto as_vec = [](const Line & l) { return Eigen::Vector2d(l.alpha, l.d); };
  for (int k = 0; k < 3; ++k) {
    Eigen::Vector3d p = pose;
    p(k) += h;
    const Eigen::Vector2d num = (as_vec(init_line(p, obs).line) - as_vec(init.line)) / h;
    EXPECT_TRUE(num.isApprox(init.G_pose.col(k), 1e-5)) << "pose column " << k;
  }
  ObservedLine oa = obs;
  oa.alpha += h;
  ObservedLine od = obs;
  od.d += h;
  EXPECT_TRUE(((as_vec(init_line(pose, oa).line) - as_vec(init.line)) / h).isApprox(init.G_obs.col(0), 1e-5));
  EXPECT_TRUE(((as_vec(init_line(pose, od).line) - as_vec(init.line)) / h).isApprox(init.G_obs.col(1), 1e-5));
}

TEST(EkfLinesSlam, NewLineNeedsMinObservations)
{
  EkfLinesSlam slam(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), config(3));
  const std::vector<Wall> walls{{{2.0, -1.0}, {2.0, 1.0}}};

  slam.update(observe_all(Eigen::Vector3d::Zero(), walls));
  slam.update(observe_all(Eigen::Vector3d::Zero(), walls));
  EXPECT_TRUE(slam.landmarks().empty());

  slam.update(observe_all(Eigen::Vector3d::Zero(), walls));
  ASSERT_EQ(slam.landmarks().size(), 1u);
  const auto lm = slam.landmarks()[0];
  EXPECT_NEAR(lm.line.alpha, 0.0, 1e-12);
  EXPECT_NEAR(lm.line.d, 2.0, 1e-12);
  EXPECT_NEAR(lm.s_min, -1.0, 1e-12);
  EXPECT_NEAR(lm.s_max, 1.0, 1e-12);
}

TEST(EkfLinesSlam, UnconfirmedCandidateExpires)
{
  auto cfg = config(2);
  cfg.candidate_timeout = 2;
  EkfLinesSlam slam(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), cfg);
  const std::vector<Wall> walls{{{2.0, -1.0}, {2.0, 1.0}}};

  slam.update(observe_all(Eigen::Vector3d::Zero(), walls));
  for (int i = 0; i < 3; ++i) {
    slam.update({});
  }
  slam.update(observe_all(Eigen::Vector3d::Zero(), walls));
  EXPECT_TRUE(slam.landmarks().empty());  // the first sighting was forgotten
}

TEST(EkfLinesSlam, FirstLandmarkCovarianceIsObservationNoise)
{
  EkfLinesSlam slam(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), config());
  slam.update({{0.0, 2.0, 0.02, 0.03, -1.0, 1.0}});

  ASSERT_EQ(slam.state().size(), 5);
  const Eigen::MatrixXd & P = slam.covariance();
  // Exact pose at the origin: G_obs = I, no pose terms.
  EXPECT_NEAR(P(3, 3), 0.02 * 0.02, 1e-15);
  EXPECT_NEAR(P(4, 4), 0.03 * 0.03, 1e-15);
  EXPECT_NEAR(P(3, 4), 0.0, 1e-15);
  EXPECT_TRUE(P.topRightCorner(3, 2).isZero());
}

TEST(EkfLinesSlam, ReobservingMappedWallsCorrectsPose)
{
  const std::vector<Wall> walls{
    {{0, 0}, {10, 0}}, {{10, 0}, {10, 10}}, {{10, 10}, {0, 10}}, {{0, 10}, {0, 0}}};
  const Eigen::Vector3d start(5.0, 5.0, 0.0);
  auto cfg = config();
  cfg.filter.process_noise = Eigen::Vector2d(0.05 * 0.05, 1e-4).asDiagonal();  // the 0.1 m error is 2 sigma
  EkfLinesSlam slam(start, Eigen::Matrix3d::Zero(), cfg);
  slam.update(observe_all(start, walls));
  ASSERT_EQ(slam.landmarks().size(), 4u);

  // Odometry says 1.1 m, the robot moved 1 m.
  slam.predict(1.1, 0.0);
  const Eigen::Vector3d truth(6.0, 5.0, 0.0);
  const int applied = slam.update(observe_all(truth, walls));

  EXPECT_EQ(applied, 4);
  EXPECT_EQ(slam.landmarks().size(), 4u);
  EXPECT_LT((slam.pose() - truth).norm(), 0.02);
}

TEST(EkfLinesSlam, TwoFacesOfAThinWallStaySeparate)
{
  // A 10 cm thick wall: face y = 0 seen from below, face y = 0.1 seen from above.
  const Wall below{{-1.0, 0.0}, {1.0, 0.0}};
  const Wall above{{-1.0, 0.1}, {1.0, 0.1}};
  auto cfg = config();
  cfg.filter.process_noise = Eigen::Vector2d(0.01, 0.01).asDiagonal();  // loose: any match would pass the gate
  EkfLinesSlam slam({0.0, -1.0, kPi / 2}, Eigen::Matrix3d::Zero(), cfg);

  slam.update({observe({0.0, -1.0, kPi / 2}, below)});
  slam.predict(2.1, 0.0);
  slam.update({observe({0.0, 1.1, kPi / 2}, above)});

  ASSERT_EQ(slam.landmarks().size(), 2u);
  EXPECT_NEAR(std::abs(maze_runner::wrap_angle(slam.landmarks()[0].line.alpha - slam.landmarks()[1].line.alpha)), kPi, 1e-9);
}

TEST(EkfLinesSlam, DistantCollinearWallIsNewLandmark)
{
  // Same infinite line, 3 m apart along it: the segment extents keep them apart.
  const Wall near_wall{{2.0, -1.0}, {2.0, 1.0}};
  const Wall far_wall{{2.0, 3.0}, {2.0, 5.0}};
  EkfLinesSlam slam(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), config());
  slam.update({observe(Eigen::Vector3d::Zero(), near_wall)});
  slam.update({observe(Eigen::Vector3d::Zero(), far_wall)});
  EXPECT_EQ(slam.landmarks().size(), 2u);
}

TEST(EkfLinesSlam, LoopWithBiasedOdometryStaysOnTruth)
{
  const std::vector<Wall> walls{
    {{0, 0}, {10, 0}}, {{10, 0}, {10, 10}}, {{10, 10}, {0, 10}}, {{0, 10}, {0, 0}}};
  const EkfLinesConfig motion{};
  Eigen::Vector3d truth(2.0, 2.0, 0.0);
  Eigen::Vector3d dead_reckoning = truth;
  EkfLinesSlam slam(truth, Eigen::Matrix3d::Zero(), config(3));

  auto step = [&](double dd, double dth) {
    move_pose(truth, dd, dth, motion);
    move_pose(dead_reckoning, 1.05 * dd, 1.03 * dth, motion);
    slam.predict(1.05 * dd, 1.03 * dth);
    slam.update(observe_all(truth, walls));
  };
  for (int leg = 0; leg < 4; ++leg) {
    for (int i = 0; i < 60; ++i) {
      step(0.1, 0.0);
    }
    for (int i = 0; i < 16; ++i) {
      step(0.0, kPi / 32);
    }
  }

  EXPECT_EQ(slam.landmarks().size(), 4u);
  EXPECT_GT((dead_reckoning.head<2>() - truth.head<2>()).norm(), 0.5);
  EXPECT_LT((slam.pose().head<2>() - truth.head<2>()).norm(), 0.05);
  EXPECT_LT(std::abs(maze_runner::wrap_angle(slam.pose()(2) - truth(2))), 0.01);
  EXPECT_TRUE(slam.covariance().isApprox(slam.covariance().transpose()));
}
