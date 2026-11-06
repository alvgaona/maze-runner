#include "common/world_map_loader.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <Eigen/Core>

namespace maze_runner
{
namespace
{

// Planar pose composition a * (x, y, yaw of b).
Eigen::Vector3d compose(const Eigen::Vector3d & a, const gz::math::Pose3d & b)
{
  const double c = std::cos(a(2));
  const double s = std::sin(a(2));
  return {
    a(0) + c * b.Pos().X() - s * b.Pos().Y(),
    a(1) + s * b.Pos().X() + c * b.Pos().Y(),
    a(2) + b.Rot().Yaw()};
}

}  // namespace

Walls WorldMapLoader::load(const std::string & model_name) const
{
  const std::string & path = world_file_;
  sdf::Root root;
  const auto errors = root.Load(path);
  const sdf::World * world = root.WorldByIndex(0);
  if (!world) {
    throw std::runtime_error(
      "cannot load world '" + path + "'" + (errors.empty() ? "" : ": " + errors.front().Message()));
  }
  const sdf::Model * model = world->ModelByName(model_name);
  if (!model) {
    throw std::runtime_error("model '" + model_name + "' not found in " + path);
  }

  Walls walls;
  const Eigen::Vector3d model_pose = compose(Eigen::Vector3d::Zero(), model->RawPose());
  for (uint64_t i = 0; i < model->LinkCount(); ++i) {
    const sdf::Link * link = model->LinkByIndex(i);
    const Eigen::Vector3d link_pose = compose(model_pose, link->RawPose());
    for (uint64_t j = 0; j < link->CollisionCount(); ++j) {
      const sdf::Collision * col = link->CollisionByIndex(j);
      const Eigen::Vector3d p = compose(link_pose, col->RawPose());

      if (const sdf::Box * box = col->Geom()->BoxShape()) {
        walls.boxes.push_back({p(0), p(1), p(2), box->Size().X(), box->Size().Y()});
      } else if (const sdf::Mesh * mesh_sdf = col->Geom()->MeshShape()) {
        // Relative URIs are relative to the SDF file, as Gazebo resolves them.
        std::filesystem::path file = mesh_sdf->Uri();
        if (file.is_relative()) {
          file = std::filesystem::path(mesh_sdf->FilePath()).parent_path() / file;
        }
        const gz::common::Mesh * mesh = gz::common::MeshManager::Instance()->Load(file.string());
        if (!mesh) {
          throw std::runtime_error("cannot load mesh " + file.string());
        }
        const auto & scale = mesh_sdf->Scale();
        const double c = std::cos(p(2));
        const double s = std::sin(p(2));
        for (unsigned int m = 0; m < mesh->SubMeshCount(); ++m) {
          const auto sub = mesh->SubMeshByIndex(m).lock();
          for (unsigned int k = 0; sub && k + 2 < sub->IndexCount(); k += 3) {
            Triangle tri;
            for (int v = 0; v < 3; ++v) {
              const auto q = sub->Vertex(sub->Index(k + v)) * scale;
              tri[v] = {p(0) + c * q.X() - s * q.Y(), p(1) + s * q.X() + c * q.Y(), q.Z()};
            }
            walls.triangles.push_back(tri);
          }
        }
      }
    }
  }

  return walls;
}

}  // namespace maze_runner
