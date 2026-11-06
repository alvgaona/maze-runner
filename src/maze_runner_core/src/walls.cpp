#include "maze_runner_core/walls.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace maze_runner
{

std::vector<Segment> wall_segments(const Walls & walls)
{
  constexpr double kMaxVerticalTilt = 0.1;  // |n_z| / |n| above this: not a wall face

  std::vector<Segment> segments;
  for (const auto & w : walls.boxes) {
    const Eigen::Vector2d ux(std::cos(w.yaw), std::sin(w.yaw));
    const Eigen::Vector2d uy(-ux.y(), ux.x());
    const Eigen::Vector2d c(w.x, w.y);
    const Eigen::Vector2d hx = ux * (w.length / 2);
    const Eigen::Vector2d hy = uy * (w.width / 2);
    const std::array<Eigen::Vector2d, 4> p{c + hx + hy, c - hx + hy, c - hx - hy, c + hx - hy};
    for (int i = 0; i < 4; ++i) {
      segments.push_back({p[i], p[(i + 1) % 4]});
    }
  }

  for (const auto & t : walls.triangles) {
    const Eigen::Vector3d n = (t[1] - t[0]).cross(t[2] - t[0]);
    const double norm = n.norm();
    if (norm == 0.0 || std::abs(n.z()) / norm > kMaxVerticalTilt) {
      continue;
    }
    // A vertical triangle projects onto a segment: its two extreme points along the face.
    const Eigen::Vector2d tangent = Eigen::Vector2d(-n.y(), n.x()).normalized();
    const auto along = [&](const Eigen::Vector3d & p) { return tangent.dot(p.head<2>()); };
    const auto [lo, hi] = std::minmax_element(
      t.begin(), t.end(), [&](const auto & p, const auto & q) { return along(p) < along(q); });
    segments.push_back({lo->head<2>(), hi->head<2>()});
  }
  return segments;
}

}  // namespace maze_runner
