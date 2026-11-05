#include "maze_runner_core/map_lines.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <Eigen/Geometry>

namespace maze_runner
{
namespace
{

constexpr double kPi = std::numbers::pi;
constexpr double kMergeAngleTol = 1e-6;  // [rad]
constexpr double kMergeDistTol = 1e-3;   // [m]

double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// Unique representation: d >= 0, and alpha in [0, pi) when the line passes through the origin.
Line canonical(double alpha, double d)
{
  if (d < 0.0) {
    alpha += kPi;
    d = -d;
  }
  alpha = wrap_angle(alpha);
  if (d < kMergeDistTol && (alpha < 0.0 || alpha >= kPi)) {
    alpha = wrap_angle(alpha + kPi);
  }
  return {alpha, d};
}

bool same_line(const Line & a, const Line & b)
{
  return std::abs(wrap_angle(a.alpha - b.alpha)) < kMergeAngleTol &&
         std::abs(a.d - b.d) < kMergeDistTol;
}

void add_unique(std::vector<Line> & lines, double alpha, double d)
{
  const Line line = canonical(alpha, d);
  if (std::none_of(lines.begin(), lines.end(), [&](const Line & l) { return same_line(l, line); })) {
    lines.push_back(line);
  }
}

}  // namespace

std::vector<Line> wall_lines(const std::vector<WallBox> & walls, double min_face_length)
{
  std::vector<Line> lines;

  for (const auto & w : walls) {
    // Faces as (normal angle in box frame, offset from center, face length).
    const struct { double angle, offset, length; } faces[] = {
      {0.0, w.length / 2, w.width},
      {kPi, w.length / 2, w.width},
      {kPi / 2, w.width / 2, w.length},
      {-kPi / 2, w.width / 2, w.length},
    };

    for (const auto & f : faces) {
      if (f.length < min_face_length) {
        continue;
      }
      const double alpha = w.yaw + f.angle;
      // d = n . (center + offset * n)
      const double d = std::cos(alpha) * w.x + std::sin(alpha) * w.y + f.offset;
      add_unique(lines, alpha, d);
    }
  }
  return lines;
}

std::vector<Line> mesh_lines(const std::vector<Triangle> & triangles, double min_face_length)
{
  constexpr double kMaxVerticalTilt = 0.1;  // |n_z| / |n| above this: not a wall face

  std::vector<Line> lines;
  for (const auto & t : triangles) {
    const Eigen::Vector3d n = (t[1] - t[0]).cross(t[2] - t[0]);
    const double norm = n.norm();
    if (norm == 0.0 || std::abs(n.z()) / norm > kMaxVerticalTilt) {
      continue;
    }
    const Eigen::Vector2d n2 = n.head<2>().normalized();
    const Eigen::Vector2d tangent(-n2.y(), n2.x());

    double lo = tangent.dot(t[0].head<2>());
    double hi = lo;
    for (const auto & p : t) {
      lo = std::min(lo, tangent.dot(p.head<2>()));
      hi = std::max(hi, tangent.dot(p.head<2>()));
    }
    if (hi - lo < min_face_length) {
      continue;
    }
    add_unique(lines, std::atan2(n2.y(), n2.x()), n2.dot(t[0].head<2>()));
  }
  return lines;
}

}  // namespace maze_runner
