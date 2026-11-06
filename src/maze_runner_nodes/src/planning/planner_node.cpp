// Plans a path from the localized pose to a goal on the world's walls and publishes it
// on `plan`. The goal comes from the `goal` parameter at startup or from RViz (`goal_pose`).
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "maze_runner_core/planning/astar.hpp"
#include "maze_runner_core/planning/hybrid_astar.hpp"
#include "common/world_map_loader.hpp"
#include "common/yaw.hpp"

namespace
{

using maze_runner::yaw_of;
using namespace maze_runner::planning;

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("planner")
  {
    const auto world_file = declare_parameter<std::string>("world_file");
    const auto wall_model = declare_parameter<std::string>("wall_model", "maze");
    // astar: grid path. astar_shortcut: grid path cut down to straight pieces.
    // hybrid_astar: forward arcs no tighter than min_turn_radius.
    planner_ = declare_parameter<std::string>("planner", "astar_shortcut");
    const double resolution = declare_parameter<double>("resolution", 0.1);
    // Robot center keeps this far from walls; above the NMPC safety radius so the CBF
    // stays quiet along the plan.
    const double inflation = declare_parameter<double>("inflation", 0.6);
    hybrid_.step = declare_parameter<double>("hybrid.step", hybrid_.step);
    hybrid_.min_turn_radius = declare_parameter<double>("hybrid.min_turn_radius", hybrid_.min_turn_radius);
    hybrid_.steering_samples =
      static_cast<int>(declare_parameter<int>("hybrid.steering_samples", hybrid_.steering_samples));
    hybrid_.heading_bins = static_cast<int>(declare_parameter<int>("hybrid.heading_bins", hybrid_.heading_bins));
    hybrid_.goal_tolerance = declare_parameter<double>("hybrid.goal_tolerance", hybrid_.goal_tolerance);
    hybrid_.obstacle_heuristic =
      declare_parameter<bool>("hybrid.obstacle_heuristic", hybrid_.obstacle_heuristic);
    const auto goal = declare_parameter<std::vector<double>>("goal", std::vector<double>{});
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    if (planner_ != "astar" && planner_ != "astar_shortcut" && planner_ != "hybrid_astar") {
      throw std::invalid_argument("planner must be astar, astar_shortcut or hybrid_astar");
    }
    if (!goal.empty() && goal.size() != 2) {
      throw std::invalid_argument("goal must be [x, y] or empty");
    }

    const auto walls = wall_segments(maze_runner::WorldMapLoader(world_file).load(wall_model));
    grid_.emplace(walls, resolution, inflation);
    RCLCPP_INFO(
      get_logger(), "%s on a %d x %d grid (%.2f m cells, %.2f m inflation) from %zu wall segments",
      planner_.c_str(), grid_->width(), grid_->height(), resolution, inflation, walls.size());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", rclcpp::QoS(1).transient_local());
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 1, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id != map_frame_) {
          RCLCPP_WARN(get_logger(), "Goal in frame '%s', expected '%s'", msg->header.frame_id.c_str(), map_frame_.c_str());
          return;
        }
        goal_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
      });
    if (!goal.empty()) {
      goal_ = Eigen::Vector2d(goal[0], goal[1]);
    }
    // Plans once the pose is known; retries until then.
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { try_plan(); });
  }

private:
  void try_plan()
  {
    if (!goal_) {
      return;
    }
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for %s -> %s", map_frame_.c_str(), base_frame_.c_str());
      return;
    }
    const Eigen::Vector3d start(
      t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation));
    const Eigen::Vector2d goal = *goal_;
    goal_.reset();

    const auto t0 = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector3d> poses;
    std::size_t expanded = 0;
    if (planner_ == "hybrid_astar") {
      if (const auto path = hybrid_astar(*grid_, start, goal, hybrid_)) {
        poses = path->poses;
        expanded = path->expanded;
      }
    } else if (const auto path = astar(*grid_, start.head<2>(), goal)) {
      expanded = path->expanded;
      const auto points = planner_ == "astar_shortcut" ? shortcut(*grid_, path->points) : path->points;
      // Each point faces the next one; the last keeps the heading of the final piece.
      double heading = start(2);
      for (std::size_t i = 0; i < points.size(); ++i) {
        if (i + 1 < points.size()) {
          heading = std::atan2(points[i + 1].y() - points[i].y(), points[i + 1].x() - points[i].x());
        }
        poses.emplace_back(points[i].x(), points[i].y(), heading);
      }
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    if (poses.empty()) {
      RCLCPP_ERROR(
        get_logger(), "%s found no path from (%.2f, %.2f) to (%.2f, %.2f) in %.1f ms. Is the "
        "start or goal closer to a wall than the inflation?", planner_.c_str(), start(0), start(1),
        goal.x(), goal.y(), ms);
      return;
    }
    RCLCPP_INFO(
      get_logger(), "%s: %zu poses to (%.2f, %.2f) in %.1f ms, %zu expanded", planner_.c_str(),
      poses.size(), goal.x(), goal.y(), ms, expanded);

    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    for (const auto & p : poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p(0);
      ps.pose.position.y = p(1);
      ps.pose.orientation.z = std::sin(p(2) / 2);
      ps.pose.orientation.w = std::cos(p(2) / 2);
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
  }

  std::string planner_;
  HybridAStarConfig hybrid_;
  std::optional<OccupancyGrid> grid_;
  std::optional<Eigen::Vector2d> goal_;
  std::string map_frame_;
  std::string base_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
