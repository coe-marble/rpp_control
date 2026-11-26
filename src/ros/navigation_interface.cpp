#include <rpp_control/ros/navigation_interface.hpp>
#include <rpp_control/ros/control_script_node.hpp>
#include <rpp_control/ros/control_script_node_helpers.hpp>
#include <rpp_control/ros/control_script_time.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace rpp_control
{
NavigationInterface::NavigationInterface(ControlScriptNode & node)
: node_(node),
  resource_(node_helpers::declare_startup_parameter<std::string>(node,
    "navigation_resource", "", "Target resource; empty requires one unambiguous match.")),
  generator_(node_helpers::declare_startup_parameter<std::string>(node,
    "navigation_generator", "", "Goal interface name; empty requires one unambiguous match.")),
  frame_prefix_(node_helpers::declare_startup_parameter<std::string>(node,
    "navigation_frame_prefix", "", "Explicit ROS frame namespace, including its trailing slash; no TF transform.")),
  goal_monitor_(node_helpers::declare_startup_parameter<std::string>(node,
    "navigation_goal_monitor", "", "Default goal-deciding mission monitor; empty requires one eligible active instance. FollowPath goal_checker_id overrides it.")),
  command_timeout_(node_helpers::declare_duration(node, "navigation_reference_timeout_sec", 0.5))
{
  const bool enable_nav2_cmd_vel = node_helpers::declare_startup_parameter<bool>(node,
    "enable_nav2_cmd_vel", true,
    "Expose TwistStamped cmd_vel outside development mode. Commands override reference-token ownership; "
    "frame, timestamp, value, and expiry checks remain enabled.");
  if (!frame_prefix_.empty() && !frame_prefix_.ends_with("/")) {
    throw std::invalid_argument("navigation_frame_prefix must be empty or end with '/'");
  }
  if (!node_.layout_ || !node_.routes_ready_) {
    throw std::invalid_argument("NavigationInterface requires configured control routes");
  }
  accept_layout(node_.layout_->description);
  if (!path_binding_) {throw std::invalid_argument("NavigationInterface requires one unambiguous position-target interface; set navigation_generator/navigation_resource");}
  if (!supports_path()) {
    throw std::invalid_argument("Reference generator '" + path_binding_->goal.interface_name + "' does not advertise setPath");
  }
  if (!node_.script_->has_goal_monitor(goal_monitor_)) {
    throw std::invalid_argument("NavigationInterface requires an unambiguous active goal-capable MissionMonitor; set navigation_goal_monitor");
  }
  initialize_follow_path();
  if (node_.is_dev_) {
    pose_subscription_ = node_.create_subscription<Pose>("cmd_pos", rclcpp::QoS(10),
      [this](Pose::ConstSharedPtr message) { receive_pose(*message); });
    wrench_subscription_ = node_.create_subscription<Wrench>("cmd_wrench", rclcpp::QoS(10),
      [this](Wrench::ConstSharedPtr message) { receive_wrench(*message); });
  }
  if (node_.is_dev_ || enable_nav2_cmd_vel) {
    twist_subscription_ = node_.create_subscription<Twist>("cmd_vel", rclcpp::QoS(10),
      [this](Twist::ConstSharedPtr message) { receive_twist(*message); });
  }
  external_pose_subscription_ = node_.create_subscription<PoseReference>("pose_ext", rclcpp::QoS(10),
    [this](PoseReference::ConstSharedPtr message) { receive_external_pose(*message); });
  external_twist_subscription_ = node_.create_subscription<TwistReference>("twist_ext", rclcpp::QoS(10),
    [this](TwistReference::ConstSharedPtr message) { receive_external_twist(*message); });
  external_wrench_subscription_ = node_.create_subscription<WrenchReference>("wrench_ext", rclcpp::QoS(10),
    [this](WrenchReference::ConstSharedPtr message) { receive_external_wrench(*message); });
}

bool NavigationInterface::same_field(const Field & left, const Field & right)
{
  return left.name == right.name && left.coordinate_kind == right.coordinate_kind &&
    left.quantity == right.quantity && left.width == right.width &&
    left.frame_id == right.frame_id && left.child_frame_id == right.child_frame_id &&
    left.resource == right.resource;
}

const NavigationInterface::Field * NavigationInterface::find_field(
  const rpp_ros_interfaces::msg::NumericLayout & layout, const std::string & name)
{
  const auto found = std::find_if(layout.fields.begin(), layout.fields.end(),
    [&](const auto & field) { return field.name == name; });
  return found == layout.fields.end() ? nullptr : &*found;
}

bool NavigationInterface::consumed(const Layout & layout, const Field & linear, const Field & angular)
{
  return std::any_of(layout.controllers.begin(), layout.controllers.end(), [&](const auto & controller) {
    const auto * input_linear = find_field(controller.reference, linear.name);
    const auto * input_angular = find_field(controller.reference, angular.name);
    return input_linear && input_angular && same_field(linear, *input_linear) &&
      same_field(angular, *input_angular);
  });
}

bool NavigationInterface::compatible(Kind kind, const Field & linear, const Field & angular)
{
  if (linear.coordinate_kind != Field::COORDINATE_EUCLIDEAN ||
    (linear.width != 2 && linear.width != 3) || linear.resource != angular.resource ||
    linear.frame_id.empty() || linear.frame_id != angular.frame_id ||
    linear.child_frame_id != angular.child_frame_id) {
    return false;
  }
  if (kind == Kind::Pose) {
    return linear.quantity == Field::QUANTITY_LINEAR_POSITION &&
      ((linear.width == 2 && angular.width == 1 &&
        angular.coordinate_kind == Field::COORDINATE_CONTINUOUS_ANGLE &&
        angular.quantity == Field::QUANTITY_ANGULAR_POSITION) ||
       (linear.width == 3 && angular.width == 4 &&
        angular.coordinate_kind == Field::COORDINATE_QUATERNION &&
        angular.quantity == Field::QUANTITY_ORIENTATION));
  }
  const auto linear_quantity = kind == Kind::Twist ? Field::QUANTITY_LINEAR_VELOCITY : Field::QUANTITY_FORCE;
  const auto angular_quantity = kind == Kind::Twist ? Field::QUANTITY_ANGULAR_VELOCITY : Field::QUANTITY_MOMENT;
  return linear.quantity == linear_quantity && angular.quantity == angular_quantity &&
    angular.coordinate_kind == Field::COORDINATE_EUCLIDEAN &&
    angular.width == (linear.width == 2 ? 1u : 3u);
}

NavigationInterface::Binding NavigationInterface::prepare_binding(Kind kind,
  const rpp_ros_interfaces::msg::NumericInterface & interface,
  const Field & linear, const Field & angular)
{
  Binding binding;
  binding.linear = linear;
  binding.angular = angular;
  binding.frame = linear.frame_id;
  binding.goal.interface_name = interface.name;
  binding.goal.values.resize(interface.layout.value_count, 0.0);
  binding.goal.update_mask.resize(interface.layout.value_count, false);
  for (const auto & field : interface.layout.fields) {
    if (field.resource != linear.resource || field.coordinate_kind != Field::COORDINATE_EUCLIDEAN ||
      field.quantity != Field::QUANTITY_NORMALIZED || !field.name.ends_with(".selection")) {
      continue;
    }
    const auto prefix = "." + linear.resource + ".";
    const bool velocity_linear = field.name.ends_with(prefix + "linearVelocity.selection");
    const bool velocity_angular = field.name.ends_with(prefix + "angularVelocity.selection");
    const bool force = field.name.ends_with(prefix + "force.selection");
    const bool moment = field.name.ends_with(prefix + "moment.selection");
    const bool angular_selection = velocity_angular || moment;
    if (!(velocity_linear || velocity_angular || force || moment) ||
      field.width != (angular_selection ? (linear.width == 2 ? 1u : 3u) : linear.width)) {continue;}
    const bool external = (kind == Kind::Twist && (velocity_linear || velocity_angular)) ||
      (kind == Kind::Wrench && (force || moment));
    for (std::size_t axis = 0; axis < field.width; ++axis) {
      const auto dof = angular_selection ? (field.width == 1 ? 5 : 3 + axis) : axis;
      binding.selections[dof].emplace_back(field.offset + axis,
        external ? rpp_interfaces::contract::REFERENCE_EXTERNAL : rpp_interfaces::contract::REFERENCE_INTERNAL);
    }
  }
  return binding;
}

void NavigationInterface::accept_layout(const Layout & layout)
{
  std::array<Routes, 3> prepared;
  const std::array<std::string, 3> linear_names{"position", "linearVelocity", "force"};
  const std::array<std::string, 3> angular_names{"orientation", "angularVelocity", "moment"};
  for (std::size_t index = 0; index < prepared.size(); ++index) {
    std::size_t matches = 0;
    const auto kind = static_cast<Kind>(index);
    for (const auto & interface : layout.goals) {
      // Validate bounds before preparing any index used by the command callbacks.
      const bool valid = std::all_of(interface.layout.fields.begin(), interface.layout.fields.end(),
        [&](const auto & field) {
          return field.width > 0 && static_cast<std::size_t>(field.offset) + field.width <= interface.layout.value_count;
        });
      if (!valid) { continue; }
      std::optional<Binding> target;
      std::size_t interface_matches = 0;
      for (const auto & linear : interface.layout.fields) {
        if (linear.resource.empty() || (!resource_.empty() && linear.resource != resource_) ||
          linear.name != linear.resource + "." + linear_names[index]) { continue; }
        const auto * angular = find_field(interface.layout, linear.resource + "." + angular_names[index]);
        if (!angular || !compatible(kind, linear, *angular) || !consumed(layout, linear, *angular)) { continue; }
        ++interface_matches;
        target = prepare_binding(kind, interface, linear, *angular);
        target->frame = frame_prefix_ + target->frame;
      }
      if (interface_matches == 1) {
        prepared[index].interfaces.emplace(interface.name, std::move(*target));
        if (generator_.empty() || interface.name == generator_) {
          ++matches;
          prepared[index].navigation_interface = interface.name;
        }
      }
    }
    if (matches != 1) {
      prepared[index].navigation_interface.reset();
      if (matches > 1) {
        RCLCPP_WARN(node_.get_logger(), "Navigation command has multiple targets; set navigation_resource and navigation_generator.");
      }
    }
  }
  bindings_ = std::move(prepared);
  bind_path_state();
}

void NavigationInterface::receive_pose(const Pose & message)
{
  const auto & position = message.pose.position;
  const auto & orientation = message.pose.orientation;
  submit(Kind::Pose, GoalAccess::ReferenceToken, message.header, {position.x, position.y, position.z},
    {orientation.x, orientation.y, orientation.z, orientation.w});
}

void NavigationInterface::receive_twist(const Twist & message)
{
  const auto & linear = message.twist.linear;
  const auto & angular = message.twist.angular;
  submit(Kind::Twist, GoalAccess::VelocityOverride, message.header, {linear.x, linear.y, linear.z},
    {angular.x, angular.y, angular.z, 0.0});
}

void NavigationInterface::receive_wrench(const Wrench & message)
{
  const auto & force = message.wrench.force;
  const auto & torque = message.wrench.torque;
  submit(Kind::Wrench, GoalAccess::ReferenceToken, message.header, {force.x, force.y, force.z},
    {torque.x, torque.y, torque.z, 0.0});
}

void NavigationInterface::receive_external_pose(const PoseReference & message)
{
  const auto & position = message.reference.position;
  const auto & orientation = message.reference.orientation;
  submit(Kind::Pose, GoalAccess::ReferenceToken, message.header, {position.x, position.y, position.z},
    {orientation.x, orientation.y, orientation.z, orientation.w}, &message.mask, message.identity_token);
}

void NavigationInterface::receive_external_twist(const TwistReference & message)
{
  const auto & linear = message.reference.linear;
  const auto & angular = message.reference.angular;
  submit(Kind::Twist, GoalAccess::ReferenceToken, message.header, {linear.x, linear.y, linear.z},
    {angular.x, angular.y, angular.z, 0.0}, &message.mask, message.identity_token);
}

void NavigationInterface::receive_external_wrench(const WrenchReference & message)
{
  const auto & force = message.reference.force;
  const auto & torque = message.reference.torque;
  submit(Kind::Wrench, GoalAccess::ReferenceToken, message.header, {force.x, force.y, force.z},
    {torque.x, torque.y, torque.z, 0.0}, &message.mask, message.identity_token);
}

void NavigationInterface::submit(Kind kind, GoalAccess access, const std_msgs::msg::Header & header,
  const std::array<double, 3> & linear, const std::array<double, 4> & angular,
  const std::array<bool, 6> * mask, const std::string & token)
{
  const auto reject = [&](const char * reason) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "Navigation reference rejected: %s", reason);
  };
  auto & routes = bindings_[static_cast<std::size_t>(kind)];
  const bool override_reference_token = access == GoalAccess::VelocityOverride;
  const auto interface = node_.is_dev_ || override_reference_token
    ? routes.navigation_interface : node_.reference_claims_.interface_for(token);
  if (!interface) {reject("reference token is invalid or no unique navigation target is configured"); return;}
  const auto route = routes.interfaces.find(*interface);
  if (route == routes.interfaces.end()) {reject("the claimed interface has no compatible navigation fields"); return;}
  auto & binding = route->second;
  const bool spatial = binding.linear.width == 3;
  const auto axes = mask ? *mask : std::array<bool, 6>{true, true, spatial, spatial, spatial, true};
  if (std::none_of(axes.begin(), axes.end(), [](bool selected) {return selected;})) {
    reject("mask selects no axes"); return;
  }
  if (!spatial && (axes[2] || axes[3] || axes[4])) {
    reject("mask selects axes absent from the planar reference"); return;
  }
  if (kind == Kind::Pose && spatial && (axes[3] != axes[4] || axes[4] != axes[5])) {
    reject("a quaternion's roll, pitch, and yaw mask must update together"); return;
  }
  const auto received_at = node_.now();
  const auto stamp = node_.is_dev_ ?
    static_cast<builtin_interfaces::msg::Time>(received_at) : header.stamp;
  const auto deadline = time_helpers::navigation_deadline(stamp, received_at, command_timeout_);
  if (!deadline) { reject("invalid, future, or stale timestamp"); return; }
  if (header.frame_id != binding.frame) { reject("frame_id does not match the declared reference frame"); return; }
  for (std::size_t axis = 0; axis < binding.linear.width; ++axis) {
    if (axes[axis] && !std::isfinite(linear[axis])) {reject("non-finite linear coordinate"); return;}
  }
  if (kind != Kind::Pose) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (axes[3 + axis] && !std::isfinite(angular[axis])) {reject("non-finite angular coordinate"); return;}
    }
  }
  double norm = 1.0;
  const bool orientation_selected = axes[3] || axes[4] || axes[5];
  if (kind == Kind::Pose && orientation_selected) {
    norm = 0.0;
    for (const auto value : angular) { norm += value * value; }
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
      reject("orientation must be a unit quaternion"); return;
    }
  }
  const bool complete_pose = kind == Kind::Pose && axes[0] && axes[1] &&
    (binding.linear.width == 2 || axes[2]) && orientation_selected;
  // A complete pose switches the whole cascade back to internal references.
  // Partial pose updates retain their existing per-axis semantics.
  std::fill(binding.goal.update_mask.begin(), binding.goal.update_mask.end(), complete_pose);
  if (complete_pose) {std::fill(binding.goal.values.begin(), binding.goal.values.end(), 0.0);}
  for (std::size_t axis = 0; axis < binding.linear.width; ++axis) {
    if (axes[axis]) {
      binding.goal.values[binding.linear.offset + axis] = linear[axis];
      binding.goal.update_mask[binding.linear.offset + axis] = true;
    }
  }
  if (kind == Kind::Pose && binding.angular.width == 1 && orientation_selected) {
    const auto & q = angular;
    binding.goal.values[binding.angular.offset] =
      std::atan2(2.0 * (q[3] * q[2] + q[0] * q[1]), norm - 2.0 * (q[1] * q[1] + q[2] * q[2]));
    binding.goal.update_mask[binding.angular.offset] = true;
  } else {
    for (std::size_t axis = 0; axis < binding.angular.width; ++axis) {
      const bool selected = kind == Kind::Pose ? orientation_selected : axes[binding.angular.width == 1 ? 5 : 3 + axis];
      if (selected) {
        binding.goal.values[binding.angular.offset + axis] = angular[binding.angular.width == 1 ? 2 : axis];
        binding.goal.update_mask[binding.angular.offset + axis] = true;
      }
    }
  }
  for (std::size_t axis = 0; axis < axes.size(); ++axis) {
    if (!axes[axis]) {continue;}
    for (const auto & [index, value] : binding.selections[axis]) {
      binding.goal.values[index] = value;
      binding.goal.update_mask[index] = true;
    }
  }
  binding.goal.stamp = stamp;
  binding.goal.identity_token = token;
  binding.goal.deadline_ns = *deadline;
  if (!node_.receive_goal(binding.goal, override_reference_token)) {return;}
  if (override_reference_token && path_goal_ && *interface == path_interface_ &&
    path_route_revision_ == node_.worker_bindings_.route_revision) {
    const auto & native_route = node_.layout_->goal_routes.at(path_interface_);
    for (std::size_t axis = 0; axis < binding.goal.update_mask.size(); ++axis) {
      const auto native = native_route[axis];
      path_velocity_mask_[native] = binding.goal.update_mask[axis];
      // The path heartbeat refreshes only its own fields during a velocity command.
      path_native_mask_[native] = path_update_mask_[axis] && !binding.goal.update_mask[axis];
    }
    path_velocity_deadline_ns_ = *deadline;
  }
}
}  // namespace rpp_control
