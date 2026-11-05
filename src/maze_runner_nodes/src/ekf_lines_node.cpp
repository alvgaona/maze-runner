// Line-based EKF localization: predicts from wheel odometry, corrects with lines
// extracted from the lidar scan, against map lines built from the world SDF walls.

#include <cmath>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "maze_runner_core/ekf_lines.hpp"
#include "maze_runner_core/map_lines.hpp"
#include "maze_runner_core/odometry.hpp"
#include "maze_runner_core/ransac_lines.hpp"

namespace
{

using maze_runner::WallBox;

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

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

// Box collisions of one model in the world SDF, as planar wall boxes.
// shortcut: assumes default relative_to (link in model, collision in link) and upright
// boxes; switch to SemanticPose::Resolve if worlds start using frames.
std::vector<WallBox> load_walls(const std::string & path, const std::string & model_name)
{
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

  const Eigen::Vector3d model_pose = compose(Eigen::Vector3d::Zero(), model->RawPose());
  std::vector<WallBox> walls;
  for (uint64_t i = 0; i < model->LinkCount(); ++i) {
    const sdf::Link * link = model->LinkByIndex(i);
    const Eigen::Vector3d link_pose = compose(model_pose, link->RawPose());
    for (uint64_t j = 0; j < link->CollisionCount(); ++j) {
      const sdf::Collision * col = link->CollisionByIndex(j);
      const sdf::Box * box = col->Geom()->BoxShape();
      if (!box) {
        continue;
      }
      const Eigen::Vector3d p = compose(link_pose, col->RawPose());
      walls.push_back({p(0), p(1), p(2), box->Size().X(), box->Size().Y()});
    }
  }
  return walls;
}

class EkfLinesNode : public rclcpp::Node
{
public:
  EkfLinesNode()
  : Node("ekf_lines"), rng_(0)
  {
    const auto world_file = declare_parameter<std::string>("world_file");
    const auto wall_model = declare_parameter<std::string>("wall_model", "maze");
    const auto x0 = declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0});
    const auto s0 = declare_parameter<std::vector<double>>("initial_sigma", {0.1, 0.1, 0.01});
    const auto sq = declare_parameter<std::vector<double>>("odom_sigma", {0.002, 0.002});
    const auto chi2 = declare_parameter<double>("chi2_threshold", 9.21);
    // Sideways slip std per sqrt(meter) traveled; covers wheel slip and wall contact.
    const auto lateral_sigma = declare_parameter<double>("lateral_sigma", 0.1);
    const auto min_face = declare_parameter<double>("min_face_length", 0.5);
    ransac_threshold_ = declare_parameter<double>("ransac_threshold", 0.03);
    ransac_min_points_ = static_cast<int>(declare_parameter<int>("ransac_min_points", 10));
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    if (x0.size() != 3 || s0.size() != 3 || sq.size() != 2) {
      throw std::invalid_argument("initial_pose/initial_sigma need 3 values, odom_sigma needs 2");
    }

    const auto walls = load_walls(world_file, wall_model);
    auto map = maze_runner::wall_lines(walls, min_face);
    RCLCPP_INFO(
      get_logger(), "Loaded %zu walls -> %zu map lines from %s", walls.size(), map.size(),
      world_file.c_str());

    ekf_.emplace(
      Eigen::Vector3d(x0[0], x0[1], x0[2]),
      Eigen::Vector3d(s0[0] * s0[0], s0[1] * s0[1], s0[2] * s0[2]).asDiagonal().toDenseMatrix(),
      std::move(map),
      Eigen::Vector2d(sq[0] * sq[0], sq[1] * sq[1]).asDiagonal().toDenseMatrix(), chi2,
      lateral_sigma * lateral_sigma);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf/pose", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "diff_drive_controller/odom", 50,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(*msg); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(*msg); });
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose;
    const Eigen::Vector3d odom(p.position.x, p.position.y, yaw_of(p.orientation));
    if (last_odom_) {
      const auto delta = maze_runner::odometry_delta(*last_odom_, odom);
      ekf_->predict(delta.d, delta.theta);
    }
    last_odom_ = odom;
    publish(msg.header.stamp);
  }

  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    if (!lidar_pose_) {
      try {
        const auto t = tf_buffer_->lookupTransform(base_frame_, msg.header.frame_id, tf2::TimePointZero);
        lidar_pose_ = Eigen::Vector3d(
          t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation));
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for lidar TF: %s", e.what());
        return;
      }
    }

    // Scan points in the robot base frame.
    const auto & l = *lidar_pose_;
    std::vector<Eigen::Vector2d> points;
    points.reserve(msg.ranges.size());
    for (std::size_t i = 0; i < msg.ranges.size(); ++i) {
      const double r = msg.ranges[i];
      if (!std::isfinite(r) || r < msg.range_min || r > msg.range_max) {
        continue;
      }
      const double a = l(2) + msg.angle_min + static_cast<double>(i) * msg.angle_increment;
      points.emplace_back(l(0) + r * std::cos(a), l(1) + r * std::sin(a));
    }

    const auto lines = maze_runner::ransac_lines(
      std::move(points), ransac_threshold_, ransac_min_points_, rng_);
    const int applied = ekf_->update(lines);
    RCLCPP_DEBUG(get_logger(), "scan: %zu lines, %d applied", lines.size(), applied);
  }

  void publish(const builtin_interfaces::msg::Time & stamp)
  {
    const auto & x = ekf_->state();
    const auto & P = ekf_->covariance();

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = map_frame_;
    out.pose.pose.position.x = x(0);
    out.pose.pose.position.y = x(1);
    out.pose.pose.orientation.z = std::sin(x(2) / 2.0);
    out.pose.pose.orientation.w = std::cos(x(2) / 2.0);
    // Row-major 6x6 over (x, y, z, roll, pitch, yaw); fill the planar entries.
    const int idx[3] = {0, 1, 5};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        out.pose.covariance[idx[r] * 6 + idx[c]] = P(r, c);
      }
    }
    pose_pub_->publish(out);
  }

  std::optional<maze_runner::EkfLines> ekf_;
  std::optional<Eigen::Vector3d> last_odom_;
  std::optional<Eigen::Vector3d> lidar_pose_;
  std::mt19937 rng_;
  double ransac_threshold_;
  int ransac_min_points_;
  std::string map_frame_;
  std::string base_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfLinesNode>());
  rclcpp::shutdown();
  return 0;
}
