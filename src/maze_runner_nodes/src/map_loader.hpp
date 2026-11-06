#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "maze_runner_core/lines.hpp"

namespace maze_runner
{

struct LoadedMap
{
  std::vector<Line> lines;
  std::size_t boxes = 0;
  std::size_t triangles = 0;
};

// Map lines from the wall collisions (boxes and meshes) of one model in a world SDF.
// Throws std::runtime_error if the world, model or a mesh can't be loaded.
// shortcut: assumes default relative_to (link in model, collision in link) and upright
// geometry; switch to SemanticPose::Resolve if worlds start using frames.
LoadedMap load_map_lines(
  const std::string & world_file, const std::string & model_name, double min_face_length);

}  // namespace maze_runner
