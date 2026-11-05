#pragma once

#include <random>
#include <vector>

#include <Eigen/Core>

#include "maze_runner_core/lines.hpp"

namespace maze_runner
{

// Extract lines from 2D points (robot frame, no NaN/inf) by repeated RANSAC + total
// least squares refit. Each found line's inliers are removed before searching again.
// Lines come out in canonical form (d >= 0) with residual-based 1-sigma uncertainties.
std::vector<ObservedLine> ransac_lines(
  std::vector<Eigen::Vector2d> points, double distance_threshold, int min_points,
  std::mt19937 & rng, int iterations = 30);

}  // namespace maze_runner
