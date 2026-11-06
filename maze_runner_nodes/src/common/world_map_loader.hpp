#pragma once

#include <string>

#include "maze_runner_core/walls.hpp"

namespace maze_runner
{

// Reads the wall collisions (boxes and meshes) of models in a world SDF, in the world frame.
// Does file I/O, and meshes go through Gazebo's global MeshManager cache.
// shortcut: assumes default relative_to (link in model, collision in link) and upright
// geometry; switch to SemanticPose::Resolve if worlds start using frames.
class WorldMapLoader
{
public:
  explicit WorldMapLoader(std::string world_file) : world_file_(std::move(world_file)) {}

  // Throws std::runtime_error if the world, model or a mesh can't be loaded.
  Walls load(const std::string & model_name) const;

private:
  std::string world_file_;
};

}  // namespace maze_runner
