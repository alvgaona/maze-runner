#pragma once

#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "maze_runner_core/localization/ekf_lines.hpp"
#include "maze_runner_core/localization/lines.hpp"

namespace maze_runner::localization
{

struct EkfLinesSlamConfig
{
  // Motion noise and the match gate, as in localization.
  EkfLinesConfig filter{};
  // An observation whose best Mahalanobis distance is between filter.chi2_threshold and
  // this is ambiguous and dropped. Only one farther than this from every line starts a new one.
  double new_line_chi2 = 50.0;
  // Largest gap [m] along a line between an observed segment and a landmark's seen extent
  // that still counts as the same wall.
  double overlap_margin = 0.5;
  // Times a new line must be seen before it enters the map (one bad fit never does).
  int min_observations = 3;
  // Updates (scans) a candidate may go unseen before it's dropped.
  int candidate_timeout = 10;
};

// A mapped wall: a directed line in the world frame (its normal points from the side the
// wall was seen from toward the wall), with the extent seen so far along the tangent
// t = (-sin alpha, cos alpha).
struct LineLandmark
{
  Line line;
  double s_min;
  double s_max;
};

// Predicted observation z = [alpha, d] of world line m from pose, with its Jacobians.
struct LineMeasurement
{
  Eigen::Vector2d z;
  Eigen::Matrix<double, 2, 3> H_pose;
  Eigen::Matrix2d H_line;
};
LineMeasurement measure_line(const Eigen::Vector3d & pose, const Line & m);

// World line from an observation made at pose, with its Jacobians (inverse of measure_line).
struct LineInit
{
  Line line;
  Eigen::Matrix<double, 2, 3> G_pose;
  Eigen::Matrix2d G_obs;
};
LineInit init_line(const Eigen::Vector3d & pose, const ObservedLine & obs);

// EKF SLAM with line landmarks. State is [x, y, theta, alpha_1, d_1, ..., alpha_n, d_n].
// The map frame is the start pose, so x0/P0 anchor the map.
class EkfLinesSlam
{
public:
  EkfLinesSlam(const Eigen::Vector3d & x0, const Eigen::Matrix3d & P0, const EkfLinesSlamConfig & config = {});

  // Midpoint odometry model; the map is static.
  void predict(double delta_d, double delta_theta);

  // Sequential update, one observed line at a time: matched lines correct the state, new
  // ones become candidates and enter the map once seen min_observations times.
  // Returns how many observations were applied as corrections.
  int update(const std::vector<ObservedLine> & observed);

  Eigen::Vector3d pose() const { return x_.head<3>(); }
  Eigen::Matrix3d pose_covariance() const { return P_.topLeftCorner<3, 3>(); }
  const Eigen::VectorXd & state() const { return x_; }
  const Eigen::MatrixXd & covariance() const { return P_; }

  // Mapped lines, current estimates (state order).
  std::vector<LineLandmark> landmarks() const;

private:
  struct Candidate
  {
    LineLandmark lm;
    int hits;
    int last_seen;
  };

  Line line(int i) const { return {x_(3 + 2 * i), x_(4 + 2 * i)}; }
  int size() const { return static_cast<int>(extents_.size()); }

  // Observed segment endpoints projected onto the tangent of world line m, in world frame.
  std::pair<double, double> world_extent(const ObservedLine & obs, const Line & m) const;
  bool overlaps(const std::pair<double, double> & e, double s_min, double s_max) const;

  void add_landmark(const ObservedLine & obs);
  void track_candidate(const ObservedLine & obs);

  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  std::vector<std::pair<double, double>> extents_;  // [s_min, s_max] per landmark
  std::vector<Candidate> candidates_;
  int updates_ = 0;
  EkfLinesSlamConfig cfg_;
};

}  // namespace maze_runner::localization
