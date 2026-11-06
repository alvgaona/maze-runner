#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "maze_runner_core/localization/lines.hpp"

namespace maze_runner::localization
{

struct LoadedMap
{
  std::vector<Line> lines;
  std::size_t boxes = 0;
  std::size_t triangles = 0;
};

// Reads the map lines of one model's wall collisions (boxes and meshes) from a world SDF.
// Does file I/O, and meshes go through Gazebo's global MeshManager cache.
// shortcut: assumes default relative_to (link in model, collision in link) and upright
// geometry; switch to SemanticPose::Resolve if worlds start using frames.
class WorldMapLoader
{
public:
  // Faces shorter than min_face_length [m] (wall ends, small parts) give no line.
  explicit WorldMapLoader(double min_face_length) : min_face_length_(min_face_length) {}

  // Throws std::runtime_error if the world, model or a mesh can't be loaded.
  LoadedMap load(const std::string & world_file, const std::string & model_name) const;

private:
  double min_face_length_;
};

}  // namespace maze_runner::localization
