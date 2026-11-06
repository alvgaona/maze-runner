#pragma once

#include <vector>

#include <Eigen/Core>

#include "maze_runner_core/localization/lines.hpp"

namespace maze_runner::localization
{

struct RansacConfig
{
  double distance_threshold = 0.03;  // inlier band half-width [m]
  int min_points = 10;               // fewest inliers that make a line (>= 2)
  int iterations = 30;               // samples per line
  unsigned seed = 0;                 // same points + config give the same lines
};

// Extract lines from 2D points (robot frame, no NaN/inf) by repeated RANSAC + total
// least squares refit. Each found line's inliers are removed before searching again.
// Lines come out in canonical form (d >= 0) with residual-based 1-sigma uncertainties.
std::vector<ObservedLine> ransac_lines(std::vector<Eigen::Vector2d> points, const RansacConfig & config);

}  // namespace maze_runner::localization
