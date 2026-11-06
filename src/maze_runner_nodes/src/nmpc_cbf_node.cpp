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

namespace
{

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

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
    cfg.v_min = declare_parameter<double>("v_min", cfg.v_min);
    cfg.v_max = declare_parameter<double>("v_max", cfg.v_max);
    cfg.omega_min = declare_parameter<double>("omega_min", cfg.omega_min);
    cfg.omega_max = declare_parameter<double>("omega_max", cfg.omega_max);
    cfg.safety_radius = declare_parameter<double>("safety_radius", cfg.safety_radius);
    cfg.alpha_cbf = declare_parameter<double>("alpha_cbf", cfg.alpha_cbf);
    cfg.constraint_range = declare_parameter<double>("constraint_range", cfg.constraint_range);
    cfg.fov = declare_parameter<double>("fov", cfg.fov);
    cfg.scan_downsample = static_cast<int>(declare_parameter<int>("scan_downsample", cfg.scan_downsample));
    cfg.max_obstacles = static_cast<int>(declare_parameter<int>("max_obstacles", cfg.max_obstacles));
    cfg.use_slack = declare_parameter<bool>("use_slack", cfg.use_slack);
    cfg.slack_penalty = declare_parameter<double>("slack_penalty", cfg.slack_penalty);
    cfg.max_iterations = static_cast<int>(declare_parameter<int>("max_iterations", cfg.max_iterations));
    if (qw.size() != 3 || rw.size() != 2) {
      throw std::invalid_argument("state_weights needs 3 values, control_weights needs 2");
    }
    cfg.state_weights = {qw[0], qw[1], qw[2]};
    cfg.control_weights = {rw[0], rw[1]};

    // Path: flat [x0, y0, x1, y1, ...] in the map frame, sampled at reference_speed * dt so
    // consecutive reference points are one control step apart.
    const auto flat = declare_parameter<std::vector<double>>("waypoints");
    const double reference_speed = declare_parameter<double>("reference_speed", 0.5);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.15);
    // Reference starts this far ahead of the closest path point. Any lead makes the NMPC
    // chase a gap that never closes, so it runs at v_max instead of reference_speed.
    const double lookahead = declare_parameter<double>("lookahead", 0.0);
    const double waypoint_threshold = declare_parameter<double>("waypoint_threshold", 0.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    if (flat.size() < 4 || flat.size() % 2 != 0) {
      throw std::invalid_argument("waypoints must be [x0, y0, x1, y1, ...] with >= 2 points");
    }
    std::vector<Eigen::Vector2d> waypoints;
    for (std::size_t i = 0; i < flat.size(); i += 2) {
      waypoints.emplace_back(flat[i], flat[i + 1]);
    }
    tracker_.emplace(
      maze_runner::densify(waypoints, reference_speed * cfg.dt), waypoint_threshold, 200, lookahead);
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
  // Scan points relative to the robot center, as the CBF expects.
  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(base_frame_, msg.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for lidar TF: %s", e.what());
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
      const double px = lx + r * std::cos(a);
      const double py = ly + r * std::sin(a);
      scan_.push_back({std::hypot(px, py), std::atan2(py, px)});
    }
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
      RCLCPP_WARN(get_logger(), "NMPC did not converge, reusing last command");
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
  std::vector<maze_runner::ScanPoint> scan_;
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
