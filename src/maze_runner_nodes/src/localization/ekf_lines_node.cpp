// Line-based EKF: predicts from wheel odometry, corrects with lines extracted from the
// lidar scan. Localization matches them to map lines built from the world SDF walls;
// with slam:=true it builds the line map itself, starting from the initial pose.

#include <cmath>
#include <optional>
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
#include <visualization_msgs/msg/marker.hpp>

#include "maze_runner_core/localization/ekf_lines.hpp"
#include "maze_runner_core/localization/ekf_lines_slam.hpp"
#include "maze_runner_core/localization/map_lines.hpp"
#include "maze_runner_core/localization/odometry.hpp"
#include "maze_runner_core/localization/ransac_lines.hpp"
#include "common/world_map_loader.hpp"
#include "common/yaw.hpp"

namespace
{

using maze_runner::yaw_of;
using namespace maze_runner::localization;

class EkfLinesNode : public rclcpp::Node
{
public:
  EkfLinesNode()
  : Node("ekf_lines")
  {
    const bool slam = declare_parameter<bool>("slam", false);
    const auto x0 = declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0});
    const auto s0 = declare_parameter<std::vector<double>>("initial_sigma", {0.1, 0.1, 0.01});
    const auto sq = declare_parameter<std::vector<double>>("odom_sigma", {0.002, 0.002});
    const auto chi2 = declare_parameter<double>("chi2_threshold", 9.21);
    // Sideways slip std per sqrt(meter) traveled; covers wheel slip and wall contact.
    const auto lateral_sigma = declare_parameter<double>("lateral_sigma", 0.1);
    // Heading slip std per sqrt(m + rad) of motion; startup wheel slip skews odom heading.
    const auto heading_sigma = declare_parameter<double>("heading_sigma", 0.1);
    const auto min_face = declare_parameter<double>("min_face_length", 0.5);
    ransac_.distance_threshold = declare_parameter<double>("ransac_threshold", 0.03);
    ransac_.min_points = static_cast<int>(declare_parameter<int>("ransac_min_points", 10));
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    if (x0.size() != 3 || s0.size() != 3 || sq.size() != 2) {
      throw std::invalid_argument("initial_pose/initial_sigma need 3 values, odom_sigma needs 2");
    }

    if (ransac_.min_points < 2) {
      throw std::invalid_argument("ransac_min_points must be >= 2 (a line needs two points)");
    }

    const Eigen::Vector3d pose0(x0[0], x0[1], x0[2]);
    const EkfLinesConfig filter{
      .process_noise = Eigen::Vector2d(sq[0] * sq[0], sq[1] * sq[1]).asDiagonal(),
      .chi2_threshold = chi2,
      .lateral_variance = lateral_sigma * lateral_sigma,
      .heading_variance = heading_sigma * heading_sigma};

    if (slam) {
      EkfLinesSlamConfig cfg{
        .filter = filter,
        .new_line_chi2 = declare_parameter<double>("new_line_chi2", 50.0),
        .overlap_margin = declare_parameter<double>("overlap_margin", 0.5),
        .min_observations = static_cast<int>(declare_parameter<int>("min_observations", 3)),
        .candidate_timeout = static_cast<int>(declare_parameter<int>("candidate_timeout", 10))};
      if (cfg.min_observations < 1 || cfg.new_line_chi2 < chi2) {
        throw std::invalid_argument("min_observations must be >= 1 and new_line_chi2 >= chi2_threshold");
      }
      // The map frame is the start pose, so it's exact; initial_sigma doesn't apply.
      slam_.emplace(pose0, Eigen::Matrix3d::Zero(), cfg);
      landmarks_pub_ = create_publisher<visualization_msgs::msg::Marker>("ekf/landmarks", 1);
      RCLCPP_INFO(get_logger(), "SLAM mode: building the line map from (%.2f, %.2f, %.2f)", x0[0], x0[1], x0[2]);
    } else {
      const auto world_file = declare_parameter<std::string>("world_file");
      const auto wall_model = declare_parameter<std::string>("wall_model", "maze");
      const auto walls = maze_runner::WorldMapLoader(world_file).load(wall_model);
      auto lines = wall_lines(walls.boxes, min_face);
      for (const auto & l : mesh_lines(walls.triangles, min_face)) {
        lines.push_back(l);
      }
      RCLCPP_INFO(
        get_logger(), "Map from %s: %zu boxes, %zu mesh triangles -> %zu lines", world_file.c_str(),
        walls.boxes.size(), walls.triangles.size(), lines.size());

      ekf_.emplace(
        pose0, Eigen::Vector3d(s0[0] * s0[0], s0[1] * s0[1], s0[2] * s0[2]).asDiagonal().toDenseMatrix(),
        std::move(lines), filter);
    }

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
      const auto delta = odometry_delta(*last_odom_, odom);
      if (ekf_) {
        ekf_->predict(delta.d, delta.theta);
      } else {
        slam_->predict(delta.d, delta.theta);
      }
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
    const Eigen::Vector3d x = pose();
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

    const auto lines = ransac_lines(std::move(points), ransac_);
    const int applied = ekf_ ? ekf_->update(lines) : slam_->update(lines);
    RCLCPP_DEBUG(get_logger(), "scan: %zu lines, %d applied", lines.size(), applied);
    if (slam_) {
      publish_landmarks(msg.header.stamp);
    }
  }

  Eigen::Vector3d pose() const { return ekf_ ? ekf_->state() : slam_->pose(); }
  Eigen::Matrix3d pose_covariance() const { return ekf_ ? ekf_->covariance() : slam_->pose_covariance(); }

  // Mapped walls as segments over the extent seen so far.
  void publish_landmarks(const builtin_interfaces::msg::Time & stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.ns = "lines";
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.05;
    m.color.r = 1.0;
    m.color.g = 0.5;
    m.color.a = 1.0;
    for (const auto & lm : slam_->landmarks()) {
      const double c = std::cos(lm.line.alpha);
      const double s = std::sin(lm.line.alpha);
      for (const double t : {lm.s_min, lm.s_max}) {
        geometry_msgs::msg::Point p;
        p.x = lm.line.d * c - t * s;
        p.y = lm.line.d * s + t * c;
        m.points.push_back(p);
      }
    }
    landmarks_pub_->publish(m);
  }

  void publish(const builtin_interfaces::msg::Time & stamp)
  {
    const Eigen::Vector3d x = pose();
    const Eigen::Matrix3d P = pose_covariance();

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

  // Exactly one is set: localization against a known map, or SLAM.
  std::optional<EkfLines> ekf_;
  std::optional<EkfLinesSlam> slam_;
  std::optional<Eigen::Vector3d> last_odom_;
  std::optional<rclcpp::Time> last_odom_stamp_;
  std::string odom_frame_;
  RansacConfig ransac_;
  std::string map_frame_;
  std::string base_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmarks_pub_;
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
