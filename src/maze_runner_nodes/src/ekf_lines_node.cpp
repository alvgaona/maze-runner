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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "maze_runner_core/ekf_lines.hpp"
#include "maze_runner_core/odometry.hpp"
#include "maze_runner_core/ransac_lines.hpp"
#include "map_loader.hpp"
#include "yaw.hpp"

namespace
{

using maze_runner::yaw_of;

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
    // Heading slip std per sqrt(m + rad) of motion; startup wheel slip skews odom heading.
    const auto heading_sigma = declare_parameter<double>("heading_sigma", 0.1);
    const auto min_face = declare_parameter<double>("min_face_length", 0.5);
    ransac_threshold_ = declare_parameter<double>("ransac_threshold", 0.03);
    ransac_min_points_ = static_cast<int>(declare_parameter<int>("ransac_min_points", 10));
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    if (x0.size() != 3 || s0.size() != 3 || sq.size() != 2) {
      throw std::invalid_argument("initial_pose/initial_sigma need 3 values, odom_sigma needs 2");
    }

    if (ransac_min_points_ < 2) {
      throw std::invalid_argument("ransac_min_points must be >= 2 (a line needs two points)");
    }

    auto map = maze_runner::load_map_lines(world_file, wall_model, min_face);
    RCLCPP_INFO(
      get_logger(), "Map from %s: %zu boxes, %zu mesh triangles -> %zu lines", world_file.c_str(),
      map.boxes, map.triangles, map.lines.size());

    ekf_.emplace(
      Eigen::Vector3d(x0[0], x0[1], x0[2]),
      Eigen::Vector3d(s0[0] * s0[0], s0[1] * s0[1], s0[2] * s0[2]).asDiagonal().toDenseMatrix(),
      std::move(map.lines),
      Eigen::Vector2d(sq[0] * sq[0], sq[1] * sq[1]).asDiagonal().toDenseMatrix(), chi2,
      lateral_sigma * lateral_sigma, heading_sigma * heading_sigma);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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
    last_odom_stamp_ = msg.header.stamp;
    odom_frame_ = msg.header.frame_id;
    publish(msg.header.stamp);
    publish_map_to_odom(msg, odom);
  }

  // map->odom such that map->odom->base equals the EKF pose (REP 105).
  void publish_map_to_odom(const nav_msgs::msg::Odometry & msg, const Eigen::Vector3d & odom)
  {
    const auto & x = ekf_->state();
    const double yaw = x(2) - odom(2);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg.header.stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = msg.header.frame_id;
    t.transform.translation.x = x(0) - (c * odom(0) - s * odom(1));
    t.transform.translation.y = x(1) - (s * odom(0) + c * odom(1));
    t.transform.rotation.z = std::sin(yaw / 2.0);
    t.transform.rotation.w = std::cos(yaw / 2.0);
    tf_broadcaster_->sendTransform(t);
  }

  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    if (!last_odom_stamp_) {
      return;
    }
    // Lidar at scan time, in the base frame at the EKF state's time (last odom), through
    // odom: undoes the motion since the scan, which rotates the lines in fast turns.
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(
        base_frame_, *last_odom_stamp_, msg.header.frame_id, msg.header.stamp, odom_frame_,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Skipping scan, no TF: %s", e.what());
      return;
    }
    const Eigen::Vector3d l(
      t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation));

    // Scan points in the robot base frame.
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
  std::optional<rclcpp::Time> last_odom_stamp_;
  std::string odom_frame_;
  std::mt19937 rng_;
  double ransac_threshold_;
  int ransac_min_points_;
  std::string map_frame_;
  std::string base_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
