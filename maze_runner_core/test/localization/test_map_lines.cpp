#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "maze_runner_core/localization/map_lines.hpp"

using maze_runner::localization::Line;
using maze_runner::localization::wall_lines;
using maze_runner::WallBox;

namespace
{

constexpr double kPi = std::numbers::pi;

bool has_line(const std::vector<Line> & lines, double alpha, double d)
{
  for (const auto & l : lines) {
    if (std::abs(std::atan2(std::sin(l.alpha - alpha), std::cos(l.alpha - alpha))) < 1e-9 &&
        std::abs(l.d - d) < 1e-9)
    {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(MapLines, SingleWallGivesTwoLongFaces)
{
  // 4 m x 0.1 m wall centered at (2, 3): faces y = 2.95 and y = 3.05; end caps skipped.
  const auto lines = wall_lines({{2.0, 3.0, 0.0, 4.0, 0.1}});
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_TRUE(has_line(lines, kPi / 2, 3.05));
  EXPECT_TRUE(has_line(lines, kPi / 2, 2.95));
}

TEST(MapLines, RotatedWall)
{
  // Same wall rotated 90 deg at (5, 0): faces x = 4.95 and x = 5.05.
  const auto lines = wall_lines({{5.0, 0.0, kPi / 2, 4.0, 0.1}});
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_TRUE(has_line(lines, 0.0, 5.05));
  EXPECT_TRUE(has_line(lines, 0.0, 4.95));
}

TEST(MapLines, ShortFacesKeptWhenAllowed)
{
  EXPECT_EQ(wall_lines({{0.0, 0.0, 0.0, 1.0, 1.0}}, 0.5).size(), 4u);
  EXPECT_EQ(wall_lines({{0.0, 0.0, 0.0, 1.0, 0.1}}, 0.5).size(), 2u);
}

TEST(MapLines, CollinearWallsMerge)
{
  // Two walls on y = 3 with a gap: still only two distinct infinite lines.
  const auto lines = wall_lines({{1.0, 3.0, 0.0, 2.0, 0.1}, {6.0, 3.0, 0.0, 2.0, 0.1}});
  EXPECT_EQ(lines.size(), 2u);
}

TEST(MapLines, NonNegativeDistance)
{
  // Wall on the negative side of the origin.
  const auto lines = wall_lines({{0.0, -4.0, 0.0, 4.0, 0.2}});
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_TRUE(has_line(lines, -kPi / 2, 4.1));
  EXPECT_TRUE(has_line(lines, -kPi / 2, 3.9));
  for (const auto & l : lines) {
    EXPECT_GE(l.d, 0.0);
  }
}

TEST(MapLines, TestMazeWorld)
{
  // Same boxes as maze_runner_gazebo/worlds/test_maze.sdf.
  const auto lines = wall_lines({
    {5.0, 0.0, 0.0, 10.0, 0.1},
    {5.0, 10.0, 0.0, 10.0, 0.1},
    {0.0, 5.0, kPi / 2, 10.0, 0.1},
    {10.0, 5.0, kPi / 2, 10.0, 0.1},
    {3.5, 3.0, 0.0, 7.0, 0.1},
    {6.5, 7.0, 0.0, 7.0, 0.1},
  });
  // 4 outer walls x 2 faces + 2 inner walls x 2 faces.
  EXPECT_EQ(lines.size(), 12u);
  // Inner faces of the enclosure, as seen by the robot.
  EXPECT_TRUE(has_line(lines, kPi / 2, 0.05));
  EXPECT_TRUE(has_line(lines, kPi / 2, 9.95));
  EXPECT_TRUE(has_line(lines, 0.0, 0.05));
  EXPECT_TRUE(has_line(lines, 0.0, 9.95));
}

namespace
{

// The 4 side faces of an axis-aligned box as triangles, plus its top.
std::vector<maze_runner::Triangle> box_mesh(double x0, double y0, double x1, double y1, double h)
{
  const Eigen::Vector3d a(x0, y0, 0), b(x1, y0, 0), c(x1, y1, 0), d(x0, y1, 0);
  const Eigen::Vector3d up(0, 0, h);
  std::vector<maze_runner::Triangle> tris;
  for (const auto & [p, q] : {std::pair{a, b}, {b, c}, {c, d}, {d, a}}) {
    tris.push_back({p, q, q + up});
    tris.push_back({p, q + up, p + up});
  }
  tris.push_back({a + up, b + up, c + up});  // top: ignored
  return tris;
}

}  // namespace

TEST(MapLines, MeshMatchesEquivalentBoxes)
{
  // Same wall as SingleWallGivesTwoLongFaces, as a mesh.
  const auto lines = maze_runner::localization::mesh_lines(box_mesh(0.0, 2.95, 4.0, 3.05, 1.0));
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_TRUE(has_line(lines, kPi / 2, 3.05));
  EXPECT_TRUE(has_line(lines, kPi / 2, 2.95));
}

TEST(MapLines, MeshKeepsThickEndsAndMergesCollinear)
{
  // 1 m thick hedges: end faces are 1 m, so they count. Two hedges share the y=0 and y=1
  // faces -> 2 shared lines + 4 end lines (x = 0, 3, 5, 8).
  auto tris = box_mesh(0.0, 0.0, 3.0, 1.0, 2.0);
  const auto more = box_mesh(5.0, 0.0, 8.0, 1.0, 2.0);
  tris.insert(tris.end(), more.begin(), more.end());

  const auto lines = maze_runner::localization::mesh_lines(tris);
  EXPECT_EQ(lines.size(), 6u);
  EXPECT_TRUE(has_line(lines, kPi / 2, 1.0));
  EXPECT_TRUE(has_line(lines, 0.0, 8.0));
}
