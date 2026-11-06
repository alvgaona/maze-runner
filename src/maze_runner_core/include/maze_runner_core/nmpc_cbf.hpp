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
  // Cost on command changes [v, omega], including against the command sent last step.
  // Without it the solver flips between near-equal plans (e.g. squeezing past an obstacle).
  Eigen::Vector2d control_rate_weights{10.0, 10.0};
  double v_min = 0.0;
  double v_max = 1.0;
  double omega_min = -2.0;
  double omega_max = 2.0;

  // Discrete-time CBF on lidar points held fixed in the world, along the predicted path:
  // h = |p - o|^2 - safety_radius^2 and h(k+1) >= (1 - cbf_gamma) h(k).
  double safety_radius = 0.5;                           // robot center to point [m]
  double cbf_gamma = 0.1;                               // in (0, 1]; smaller acts earlier
  double constraint_range = 3.0;                        // ignore points farther than this [m]
  double fov = 2.0 * std::numbers::pi / 3.0;            // half-angle around the heading [rad]
  int max_obstacles = 16;                               // slots; nearest point per fov sector
  bool use_slack = true;                                // keeps the NLP feasible if boxed in
  double slack_penalty = 1e4;                          // per meter^2 of violation (L1)

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

  // Whether the last compute() converged; if not, it followed the previous plan (and stops
  // after a few failures in a row).
  bool last_solve_succeeded() const { return succeeded_; }
  // Number of scan points used as constraints in the last compute().
  int last_obstacle_count() const { return obstacle_count_; }

  void reset();

private:
  // Nearest point in each of max_obstacles equal sectors of the fov, within range, so a
  // long wall can't take every slot and hide a smaller obstacle.
  std::vector<ScanPoint> select_obstacles(const std::vector<ScanPoint> & scan) const;

  NmpcCbfConfig cfg_;
  casadi::Opti opti_;
  casadi::MX X_, U_, x0_, xref_, obstacles_, u_prev_;
  casadi::Function solver_;  // (x0, xref, obstacles 2xM, u_prev, U guess, X guess) -> U
  casadi::DM plan_;  // controls of the last good solve, shifted while solves fail
  casadi::DM last_sent_;  // command returned by the last compute()
  int failures_ = 0;
  bool succeeded_ = true;
  int obstacle_count_ = 0;
};

}  // namespace maze_runner
