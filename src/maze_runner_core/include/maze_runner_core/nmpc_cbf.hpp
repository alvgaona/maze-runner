#pragma once

#include <numbers>
#include <vector>

#include <casadi/casadi.hpp>
#include <Eigen/Core>

namespace maze_runner
{

struct NmpcCbfConfig
{
  int horizon = 20;
  double dt = 0.1;
  Eigen::Vector3d state_weights{1000.0, 1000.0, 100.0};
  Eigen::Vector2d control_weights{1.0, 1.0};
  double v_min = 0.0;
  double v_max = 1.0;
  double omega_min = -2.0;
  double omega_max = 2.0;

  // HOCBF on lidar points: h = r - safety_radius.
  double safety_radius = 0.5;
  double alpha_cbf = 0.05;
  double constraint_range = 3.0;                        // ignore points farther than this [m]
  double fov = 10.0 * std::numbers::pi / 180.0;         // half-angle ahead [rad]
  int scan_downsample = 5;                              // keep every n-th candidate point
  int max_obstacles = 16;                               // fixed constraint slots
  bool use_slack = true;
  double slack_penalty = 1000.0;

  int max_iterations = 100;
};

// Lidar point in the robot frame, polar.
struct ScanPoint
{
  double range;
  double bearing;
};

// Nonlinear MPC on the unicycle model with high-order CBF constraints from lidar points.
// The NLP is built once; obstacles fill a fixed number of parameter slots.
class NmpcCbf
{
public:
  explicit NmpcCbf(const NmpcCbfConfig & config = {});

  // x: [x, y, theta]. xref: reference poses, one per step starting now; padded with
  // its last pose up to horizon + 1. Returns [v, omega].
  Eigen::Vector2d compute(
    const Eigen::Vector3d & x, const std::vector<Eigen::Vector3d> & xref,
    const std::vector<ScanPoint> & scan);

  // Whether the last compute() converged; if not, it returned the previous command.
  bool last_solve_succeeded() const { return succeeded_; }
  // Number of scan points used as constraints in the last compute().
  int last_obstacle_count() const { return obstacle_count_; }

  void reset();

private:
  // Points within range and FoV, downsampled, nearest max_obstacles.
  std::vector<ScanPoint> select_obstacles(const std::vector<ScanPoint> & scan) const;

  NmpcCbfConfig cfg_;
  casadi::Opti opti_;
  casadi::MX X_, U_, x0_, xref_, ranges_, bearings_;
  casadi::Function solver_;  // (x0, xref, ranges, bearings, U guess) -> U
  casadi::DM last_u_;
  bool succeeded_ = true;
  int obstacle_count_ = 0;
};

}  // namespace maze_runner
