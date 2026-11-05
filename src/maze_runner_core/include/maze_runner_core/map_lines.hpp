#pragma once

#include <array>
#include <vector>

#include <Eigen/Core>

#include "maze_runner_core/lines.hpp"

namespace maze_runner
{

// A wall as an axis-aligned box in its own frame, placed at (x, y, yaw) in the world.
struct WallBox
{
  double x;
  double y;
  double yaw;
  double length;  // along the box's local x [m]
  double width;   // along the box's local y [m]
};

// One map line per box face at least `min_face_length` long (short end caps are noise
// for line matching). Collinear faces from different boxes are merged. Lines are in
// world frame with d >= 0.
std::vector<Line> wall_lines(const std::vector<WallBox> & walls, double min_face_length = 0.5);

using Triangle = std::array<Eigen::Vector3d, 3>;

// Map lines from the vertical faces of a wall mesh (world frame). Same rules as
// wall_lines: faces narrower than min_face_length in plan view are skipped, collinear
// faces merged, d >= 0. Non-vertical triangles (floors, wall tops) are ignored.
std::vector<Line> mesh_lines(const std::vector<Triangle> & triangles, double min_face_length = 0.5);

}  // namespace maze_runner
