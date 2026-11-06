#include "maze_runner_core/localization/ransac_lines.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

#include <Eigen/Eigenvalues>

namespace maze_runner::localization
{
namespace
{

constexpr double kMinSampleSpacing = 0.05;  // [m] reject near-duplicate sample pairs
constexpr double kMinExtent = 0.1;          // [m] floor for the half-length in sigma_alpha
constexpr double kMinSigmaAlpha = std::numbers::pi / 180.0;  // 1 deg
constexpr double kMinSigmaD = 0.02;                           // 2 cm

// Total least squares fit of a line to the points, with uncertainty estimates.
ObservedLine fit_line(const std::vector<Eigen::Vector2d> & pts)
{
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto & p : pts) {
    mean += p;
  }
  mean /= static_cast<double>(pts.size());

  Eigen::Matrix2d scatter = Eigen::Matrix2d::Zero();
  for (const auto & p : pts) {
    scatter += (p - mean) * (p - mean).transpose();
  }
  // Normal = direction of least spread (eigenvalues are sorted ascending).
  const Eigen::Vector2d n = Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(scatter).eigenvectors().col(0);
  const Eigen::Vector2d t(-n.y(), n.x());
  const double d = n.dot(mean);

  // Sample std of the residuals (N-1, as MATLAB's std) and half-length along the line.
  double res_mean = 0.0;
  for (const auto & p : pts) {
    res_mean += n.dot(p) - d;
  }
  res_mean /= static_cast<double>(pts.size());
  double ss = 0.0;
  double extent = 0.0;
  for (const auto & p : pts) {
    const double r = n.dot(p) - d - res_mean;
    ss += r * r;
    extent = std::max(extent, std::abs(t.dot(p - mean)));
  }
  const double sigma_d = std::sqrt(ss / static_cast<double>(pts.size() - 1));

  ObservedLine line{
    std::atan2(n.y(), n.x()), d,
    std::max(sigma_d / std::max(extent, kMinExtent), kMinSigmaAlpha),
    std::max(sigma_d, kMinSigmaD)};

  if (line.d < 0.0) {
    line.d = -line.d;
    line.alpha = std::atan2(-n.y(), -n.x());
  }
  return line;
}

}  // namespace

std::vector<ObservedLine> ransac_lines(std::vector<Eigen::Vector2d> points, const RansacConfig & config)
{
  std::mt19937 rng(config.seed);
  std::vector<ObservedLine> lines;
  std::vector<char> inliers;
  std::vector<char> best;

  while (static_cast<int>(points.size()) > config.min_points) {
    std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);
    int best_count = 0;

    for (int it = 0; it < config.iterations; ++it) {
      const Eigen::Vector2d & p1 = points[pick(rng)];
      const Eigen::Vector2d & p2 = points[pick(rng)];
      const Eigen::Vector2d dp = p2 - p1;
      if (dp.norm() < kMinSampleSpacing) {
        continue;
      }
      const Eigen::Vector2d n = Eigen::Vector2d(-dp.y(), dp.x()).normalized();
      const double d = n.dot(p1);

      inliers.assign(points.size(), 0);
      int count = 0;
      for (std::size_t i = 0; i < points.size(); ++i) {
        if (std::abs(n.dot(points[i]) - d) < config.distance_threshold) {
          inliers[i] = 1;
          ++count;
        }
      }
      if (count > best_count) {
        best_count = count;
        best.swap(inliers);
      }
    }

    if (best_count < config.min_points) {
      break;
    }

    std::vector<Eigen::Vector2d> on_line;
    std::vector<Eigen::Vector2d> rest;
    for (std::size_t i = 0; i < points.size(); ++i) {
      (best[i] ? on_line : rest).push_back(points[i]);
    }
    lines.push_back(fit_line(on_line));
    points = std::move(rest);
  }
  return lines;
}

}  // namespace maze_runner::localization
