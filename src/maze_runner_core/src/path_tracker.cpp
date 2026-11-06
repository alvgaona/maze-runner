#include "maze_runner_core/path_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace maze_runner
{

std::vector<Eigen::Vector3d> densify(const std::vector<Eigen::Vector2d> & waypoints, double spacing)
{
  if (waypoints.size() < 2 || spacing <= 0.0) {
    throw std::invalid_argument("densify: need >= 2 waypoints and spacing > 0");
  }
  std::vector<Eigen::Vector3d> path;
  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const Eigen::Vector2d a = waypoints[i];
    const Eigen::Vector2d d = waypoints[i + 1] - a;
    const double length = d.norm();
    if (length == 0.0) {
      continue;
    }
    const double heading = std::atan2(d.y(), d.x());
    const int steps = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int s = 0; s < steps; ++s) {
      const Eigen::Vector2d p = a + d * (static_cast<double>(s) / steps);
      path.emplace_back(p.x(), p.y(), heading);
    }
  }
  if (path.empty()) {
    throw std::invalid_argument("densify: all waypoints coincide");
  }
  path.emplace_back(waypoints.back().x(), waypoints.back().y(), path.back()(2));
  return path;
}

PathTracker::PathTracker(std::vector<Eigen::Vector3d> path, std::size_t search_window)
: path_(std::move(path)), search_window_(search_window)
{
  if (path_.empty()) {
    throw std::invalid_argument("PathTracker: empty path");
  }
}

std::size_t PathTracker::update(const Eigen::Vector2d & position)
{
  const std::size_t last = path_.size() - 1;
  auto dist = [&](std::size_t i) { return (path_[i].head<2>() - position).norm(); };

  // Closest point: anywhere on the first call, then only within the forward window.
  const std::size_t end = initialized_ ? std::min(ref_idx_ + search_window_, last) : last;
  std::size_t best = ref_idx_;
  for (std::size_t i = ref_idx_; i <= end; ++i) {
    if (dist(i) < dist(best)) {
      best = i;
    }
  }
  ref_idx_ = best;
  initialized_ = true;
  return ref_idx_;
}

bool PathTracker::reached_goal(const Eigen::Vector2d & position, double tolerance) const
{
  return ref_idx_ == path_.size() - 1 &&
         (path_.back().head<2>() - position).norm() < tolerance;
}

}  // namespace maze_runner
