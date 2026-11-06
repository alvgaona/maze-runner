#pragma once

#include <vector>

#include "maze_runner_core/localization/lines.hpp"
#include "maze_runner_core/walls.hpp"

namespace maze_runner::localization
{

// One map line per box face at least `min_face_length` long (short end caps are noise
// for line matching). Collinear faces from different boxes are merged. Lines are in
// world frame with d >= 0.
std::vector<Line> wall_lines(const std::vector<WallBox> & walls, double min_face_length = 0.5);

// Map lines from the vertical faces of a wall mesh (world frame). Same rules as
// wall_lines: faces narrower than min_face_length in plan view are skipped, collinear
// faces merged, d >= 0. Non-vertical triangles (floors, wall tops) are ignored.
std::vector<Line> mesh_lines(const std::vector<Triangle> & triangles, double min_face_length = 0.5);

}  // namespace maze_runner::localization
