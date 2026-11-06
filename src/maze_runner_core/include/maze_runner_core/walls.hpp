#pragma once

#include <array>
#include <vector>

#include <Eigen/Core>

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

// A mesh triangle in the world frame.
using Triangle = std::array<Eigen::Vector3d, 3>;

// The walls of a world: boxes and mesh triangles, both in the world frame.
struct Walls
{
  std::vector<WallBox> boxes;
  std::vector<Triangle> triangles;
};

// A wall face seen from above.
struct Segment
{
  Eigen::Vector2d a;
  Eigen::Vector2d b;
};

// Plan view of the walls: the four sides of each box and the vertical faces of the mesh.
// Non-vertical triangles (floors, wall tops) give nothing.
std::vector<Segment> wall_segments(const Walls & walls);

}  // namespace maze_runner
