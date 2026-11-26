#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rpp_ros_interfaces/msg/control_layout.hpp>
#include <rpp_ros_interfaces/msg/numeric_goal.hpp>
#include <rpp_ros_interfaces/msg/pose_reference.hpp>
#include <rpp_ros_interfaces/msg/twist_reference.hpp>
#include <rpp_ros_interfaces/msg/wrench_reference.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <rpp_cpp/list_definitions.hpp>
#include <std_msgs/msg/header.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rpp_control
{
class ControlScriptNode;
// Optional ROS facade. Its callbacks share the owning node's mutually exclusive
// callback group; the numeric worker never accesses these buffers.
class NavigationInterface
{
public:
  explicit NavigationInterface(ControlScriptNode & node);
  void accept_layout(const rpp_ros_interfaces::msg::ControlLayout & layout);

private:
  using Layout = rpp_ros_interfaces::msg::ControlLayout;
  using Field = rpp_ros_interfaces::msg::NumericField;
  using Goal = rpp_ros_interfaces::msg::NumericGoal;
  using Pose = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::TwistStamped;
  using Wrench = geometry_msgs::msg::WrenchStamped;
  using PoseReference = rpp_ros_interfaces::msg::PoseReference;
  using TwistReference = rpp_ros_interfaces::msg::TwistReference;
  using WrenchReference = rpp_ros_interfaces::msg::WrenchReference;
  using FollowPath = nav2_msgs::action::FollowPath;
  using PathHandle = rclcpp_action::ServerGoalHandle<FollowPath>;
  enum class Kind : std::size_t {Pose, Twist, Wrench};
  enum class GoalAccess {ReferenceToken, VelocityOverride};

  struct Binding
  {
    Goal goal;
    Field linear, angular;
    std::string frame;
    std::array<std::vector<std::pair<std::size_t, double>>, 6> selections;
  };
  struct Routes
  {
    std::map<std::string, Binding> interfaces;
    std::optional<std::string> navigation_interface;
  };

  struct PathStateRoute
  {
    Field position;
    std::optional<Field> orientation, velocity;
  };
  struct PathSnapshot
  {
    geometry_msgs::msg::PoseStamped pose;
    double speed = 0.0;
    std::uint64_t time_ns = 0, reset_revision = 0;
  };

  void initialize_follow_path();
  void bind_path_state();
  bool supports_path() const;
  bool valid_path(const nav_msgs::msg::Path & path) const;
  std::optional<PathSnapshot> path_snapshot();
  rclcpp_action::GoalResponse accept_path(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal> goal);
  void start_path(std::shared_ptr<PathHandle> goal);
  void update_path();
  bool submit_path(const nav_msgs::msg::Path & path, bool new_mission);
  void finish_path(std::uint16_t code, const std::string & message, bool canceled = false);
  void publish_path_feedback(const PathSnapshot & snapshot);

  void receive_pose(const Pose & message);
  void receive_twist(const Twist & message);
  void receive_wrench(const Wrench & message);
  void receive_external_pose(const PoseReference & message);
  void receive_external_twist(const TwistReference & message);
  void receive_external_wrench(const WrenchReference & message);
  void submit(
    Kind kind, GoalAccess access, const std_msgs::msg::Header & header,
    const std::array<double, 3> & linear, const std::array<double, 4> & angular,
    const std::array<bool, 6> * mask = nullptr, const std::string & token = "");

  static bool same_field(const Field & left, const Field & right);
  static const Field * find_field(
    const rpp_ros_interfaces::msg::NumericLayout & layout, const std::string & name);
  static bool consumed(const Layout & layout, const Field & linear, const Field & angular);
  static bool compatible(Kind kind, const Field & linear, const Field & angular);
  static Binding prepare_binding(
    Kind kind,
    const rpp_ros_interfaces::msg::NumericInterface & interface,
    const Field & linear, const Field & angular);

  ControlScriptNode & node_;
  const std::string resource_, generator_, frame_prefix_, goal_monitor_;
  const std::chrono::nanoseconds command_timeout_;
  std::array<Routes, 3> bindings_;
  rclcpp::Subscription<Pose>::SharedPtr pose_subscription_;
  rclcpp::Subscription<Twist>::SharedPtr twist_subscription_;
  rclcpp::Subscription<Wrench>::SharedPtr wrench_subscription_;
  rclcpp::Subscription<PoseReference>::SharedPtr external_pose_subscription_;
  rclcpp::Subscription<TwistReference>::SharedPtr external_twist_subscription_;
  rclcpp::Subscription<WrenchReference>::SharedPtr external_wrench_subscription_;
  rclcpp_action::Server<FollowPath>::SharedPtr path_server_;
  rclcpp::TimerBase::SharedPtr path_timer_;
  std::shared_ptr<PathHandle> path_goal_;
  std::optional<Binding> path_binding_;
  std::optional<PathStateRoute> path_state_route_;
  std::string path_interface_, path_token_;
  std::string path_goal_monitor_;
  std::vector<bool> path_update_mask_;
  rpp::List<bool> path_native_mask_;
  // Executor-only bookkeeping; the script receives ordinary point goals.
  rpp::List<bool> path_velocity_mask_;
  rpp::List<double> path_resume_values_;
  std::optional<std::uint64_t> path_velocity_deadline_ns_;
  std::uint64_t path_route_revision_ = 0;
  std::uint64_t path_reset_revision_ = 0, path_submitted_ns_ = 0;
};
}  // namespace rpp_control
