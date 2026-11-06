#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <stdexcept>

#include <gtest/gtest.h>

#include "map_loader.hpp"

namespace fs = std::filesystem;
using maze_runner::Line;

namespace
{

constexpr double kPi = std::numbers::pi;

bool has_line(const std::vector<Line> & lines, double alpha, double d)
{
  for (const auto & l : lines) {
    if (std::abs(maze_runner::wrap_angle(l.alpha - alpha)) < 1e-6 && std::abs(l.d - d) < 1e-6) {
      return true;
    }
  }
  return false;
}

// Binary STL of the unit cube [0, 1]^3.
void write_unit_cube_stl(const fs::path & file)
{
  using V = std::array<float, 3>;
  const V p[8] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
  const int quads[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
  std::ofstream out(file, std::ios::binary);
  const char header[80] = {};
  out.write(header, 80);
  const std::uint32_t count = 12;
  out.write(reinterpret_cast<const char *>(&count), 4);
  for (const auto & q : quads) {
    for (const auto & tri : {std::array{q[0], q[1], q[2]}, std::array{q[0], q[2], q[3]}}) {
      const float normal[3] = {0, 0, 0};  // loaders recompute it
      out.write(reinterpret_cast<const char *>(normal), 12);
      for (int i : tri) {
        out.write(reinterpret_cast<const char *>(p[i].data()), 12);
      }
      const std::uint16_t attr = 0;
      out.write(reinterpret_cast<const char *>(&attr), 2);
    }
  }
}

// Model "maze" at (1, 0) rotated 90 deg, with a 4 x 0.2 box wall and the cube mesh scaled x2,
// referenced relative to the world file.
fs::path write_world()
{
  const fs::path dir = fs::temp_directory_path() / "maze_runner_test_map_loader";
  fs::create_directories(dir / "meshes");
  write_unit_cube_stl(dir / "meshes" / "cube.stl");
  std::ofstream(dir / "world.sdf") << R"(<?xml version="1.0"?>
<sdf version="1.9">
  <world name="test">
    <model name="maze">
      <static>true</static>
      <pose>1 0 0 0 0 1.5707963267948966</pose>
      <link name="walls">
        <collision name="box">
          <pose>2 0 0.5 0 0 0</pose>
          <geometry><box><size>4 0.2 1</size></box></geometry>
        </collision>
        <collision name="mesh">
          <geometry><mesh><uri>meshes/cube.stl</uri><scale>2 2 2</scale></mesh></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
)";
  return dir / "world.sdf";
}

}  // namespace

TEST(MapLoader, BoxAndRelativeScaledMeshInRotatedModel)
{
  const auto map = maze_runner::load_map_lines(write_world().string(), "maze", 0.5);
  EXPECT_EQ(map.boxes, 1u);
  EXPECT_EQ(map.triangles, 12u);
  ASSERT_EQ(map.lines.size(), 6u);
  // Box ends up centered at (1, 2) along world y: long faces x = 0.9 and x = 1.1.
  EXPECT_TRUE(has_line(map.lines, 0.0, 0.9));
  EXPECT_TRUE(has_line(map.lines, 0.0, 1.1));
  // Cube becomes x in [-1, 1], y in [0, 2].
  EXPECT_TRUE(has_line(map.lines, 0.0, 1.0));
  EXPECT_TRUE(has_line(map.lines, kPi, 1.0));
  EXPECT_TRUE(has_line(map.lines, kPi / 2, 2.0));
  EXPECT_TRUE(has_line(map.lines, kPi / 2, 0.0));
}

TEST(MapLoader, MissingModelThrows)
{
  EXPECT_THROW(maze_runner::load_map_lines(write_world().string(), "nope", 0.5), std::runtime_error);
}
