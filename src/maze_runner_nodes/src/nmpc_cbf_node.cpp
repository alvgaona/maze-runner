// Path following with NMPC + lidar CBF: tracks a waypoint path using the localized pose
// (map -> base TF) and publishes velocity commands to the diff drive controller.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "maze_runner_core/nmpc_cbf.hpp"
#include "maze_runner_core/path_tracker.hpp"
#include "yaw.hpp"

namespace
{

using maze_runner::yaw_of;

// Older inputs than these stop the robot: driving on them is driving blind.
constexpr double kMaxPoseAge = 0.5;  // [s] map -> base (localization alive)
constexpr double kMaxScanAge = 0.3;  // [s]

class NmpcCbfNode : public rclcpp::Node
{
public:
  NmpcCbfNode()
  : Node("nmpc_cbf")
  {
    maze_runner::NmpcCbfConfig cfg;
    cfg.horizon = static_cast<int>(declare_parameter<int>("horizon", cfg.horizon));
    cfg.dt = declare_parameter<double>("dt", cfg.dt);
    const auto qw = declare_parameter<std::vector<double>>("state_weights", {1000.0, 1000.0, 100.0});
    const auto rw = declare_parameter<std::vector<double>>("control_weights", {1.0, 1.0});
    const auto rdw = declare_parameter<std::vector<double>>(
      "control_rate_weights", {cfg.control_rate_weights(0), cfg.control_rate_weights(1)});
    cfg.v_min = declare_parameter<double>("v_min", cfg.v_min);
    cfg.v_max = declare_parameter<double>("v_max", cfg.v_max);
    cfg.omega_min = declare_parameter<double>("omega_min", cfg.omega_min);
    cfg.omega_max = declare_parameter<double>("omega_max", cfg.omega_max);
    cfg.safety_radius = declare_parameter<double>("safety_radius", cfg.safety_radius);
    cfg.cbf_gamma = declare_parameter<double>("cbf_gamma", cfg.cbf_gamma);
    cfg.constraint_range = declare_parameter<double>("constraint_range", cfg.constraint_range);
    cfg.fov = declare_parameter<double>("fov", cfg.fov);
    cfg.max_obstacles = static_cast<int>(declare_parameter<int>("max_obstacles", cfg.max_obstacles));
    cfg.use_slack = declare_parameter<bool>("use_slack", cfg.use_slack);
    cfg.slack_penalty = declare_parameter<double>("slack_penalty", cfg.slack_penalty);
    cfg.max_iterations = static_cast<int>(declare_parameter<int>("max_iterations", cfg.max_iterations));
    if (qw.size() != 3 || rw.size() != 2 || rdw.size() != 2) {
      throw std::invalid_argument(
        "state_weights needs 3 values, control_weights and control_rate_weights need 2");
    }
    cfg.state_weights = {qw[0], qw[1], qw[2]};
    cfg.control_weights = {rw[0], rw[1]};
    cfg.control_rate_weights = {rdw[0], rdw[1]};

    // Path: flat [x0, y0, x1, y1, ...] in the map frame, sampled at reference_speed * dt so
    // consecutive reference points are one control step apart.
    const auto flat = declare_parameter<std::vector<double>>("waypoints");
    const double reference_speed = declare_parameter<double>("reference_speed", 0.5);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.15);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    if (flat.size() < 4 || flat.size() % 2 != 0) {
      throw std::invalid_argument("waypoints must be [x0, y0, x1, y1, ...] with >= 2 points");
    }
    std::vector<Eigen::Vector2d> waypoints;
    for (std::size_t i = 0; i < flat.size(); i += 2) {
      waypoints.emplace_back(flat[i], flat[i + 1]);
    }
    tracker_.emplace(maze_runner::densify(waypoints, reference_speed * cfg.dt));
    mpc_.emplace(cfg);
    RCLCPP_INFO(
      get_logger(), "Tracking %zu waypoints (%zu reference points), horizon %d, dt %.2f s",
      waypoints.size(), tracker_->path().size(), cfg.horizon, cfg.dt);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("diff_drive_controller/cmd_vel", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "nmpc/reference", rclcpp::QoS(1).transient_local());
    // Wall-clock solve time per step [ms], for offline evaluation.
    solve_time_pub_ = create_publisher<std_msgs::msg::Float64>("nmpc/solve_time_ms", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(*msg); });
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(cfg.dt), [this]() { on_timer(); });

    publish_reference();
  }

private:
  // Scan points in the map frame, using the pose at scan time: they stay put however much
  // the robot moves before the next solve.
  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(
        map_frame_, msg.header.frame_id, msg.header.stamp, rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Skipping scan, no TF: %s", e.what());
      return;
    }
    const double lx = t.transform.translation.x;
    const double ly = t.transform.translation.y;
    const double lyaw = yaw_of(t.transform.rotation);

    scan_.clear();
    for (std::size_t i = 0; i < msg.ranges.size(); ++i) {
      const double r = msg.ranges[i];
      if (!std::isfinite(r) || r < msg.range_min || r > msg.range_max) {
        continue;
      }
      const double a = lyaw + msg.angle_min + static_cast<double>(i) * msg.angle_increment;
      scan_.emplace_back(lx + r * std::cos(a), ly + r * std::sin(a));
    }
    scan_stamp_ = msg.header.stamp;
  }

  void on_timer()
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for pose TF: %s", e.what());
      return;
    }
    const Eigen::Vector3d x(
      t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation));

    if (const double age = (now() - rclcpp::Time(t.header.stamp)).seconds(); age > kMaxPoseAge) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Pose is %.1f s old (localization down?), stopping", age);
      stop();
      return;
    }
    if (!scan_stamp_ || (now() - *scan_stamp_).seconds() > kMaxScanAge) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No recent scan, stopping");
      stop();
      return;
    }

    if (done_ || tracker_->reached_goal(x.head<2>(), goal_tolerance_)) {
      if (!done_) {
        RCLCPP_INFO(get_logger(), "Goal reached at (%.2f, %.2f)", x(0), x(1));
        done_ = true;
      }
      publish_cmd(0.0, 0.0);
      return;
    }

    const std::size_t idx = tracker_->update(x.head<2>());
    const auto & path = tracker_->path();
    const std::vector<Eigen::Vector3d> ref(path.begin() + idx, path.end());

    const auto t0 = std::chrono::steady_clock::now();
    const Eigen::Vector2d u = mpc_->compute(x, ref, scan_);
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    publish_cmd(u(0), u(1));
    solve_time_pub_->publish(std_msgs::msg::Float64().set__data(ms));

    if (!mpc_->last_solve_succeeded()) {
      RCLCPP_WARN(get_logger(), "NMPC did not converge, following last plan (stops after 5 in a row)");
    }
    solve_ms_sum_ += ms;
    solve_ms_max_ = std::max(solve_ms_max_, ms);
    if (++solves_ % 50 == 0) {
      RCLCPP_INFO(
        get_logger(), "solve avg %.1f ms, max %.1f ms | obstacles %d | progress %zu/%zu",
        solve_ms_sum_ / 50.0, solve_ms_max_, mpc_->last_obstacle_count(), idx, path.size() - 1);
      solve_ms_sum_ = 0.0;
      solve_ms_max_ = 0.0;
    }
  }

  void stop()
  {
    publish_cmd(0.0, 0.0);
    mpc_->reset();  // the old plan no longer matches what was sent
  }

  void publish_cmd(double v, double w)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = base_frame_;
    cmd.twist.linear.x = v;
    cmd.twist.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  void publish_reference()
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    for (const auto & p : tracker_->path()) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p(0);
      ps.pose.position.y = p(1);
      ps.pose.orientation.z = std::sin(p(2) / 2.0);
      ps.pose.orientation.w = std::cos(p(2) / 2.0);
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
  }

  std::optional<maze_runner::PathTracker> tracker_;
  std::optional<maze_runner::NmpcCbf> mpc_;
  std::vector<Eigen::Vector2d> scan_;  // map frame
  std::optional<rclcpp::Time> scan_stamp_;
  double goal_tolerance_;
  std::string map_frame_;
  std::string base_frame_;
  bool done_ = false;
  int solves_ = 0;
  double solve_ms_sum_ = 0.0;
  double solve_ms_max_ = 0.0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr solve_time_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NmpcCbfNode>());
  rclcpp::shutdown();
  return 0;
}
