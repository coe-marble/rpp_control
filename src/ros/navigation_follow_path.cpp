#include <rpp_control/ros/navigation_interface.hpp>
#include <rpp_control/ros/control_script_node.hpp>
#include <rpp_control/ros/control_script_time.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rpp_control
{
namespace timing = time_helpers;

void NavigationInterface::initialize_follow_path()
{
  path_server_ = rclcpp_action::create_server<FollowPath>(&node_, "follow_path",
    [this](const auto & id, auto goal) {return accept_path(id, std::move(goal));},
    [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](auto goal) {start_path(std::move(goal));});
  path_timer_ = node_.create_wall_timer(node_.tick_interval_, [this] {update_path();});
}

bool NavigationInterface::supports_path() const
{
  return path_binding_ && node_.script_ &&
    (node_.script_->goal_features(path_binding_->goal.interface_name) &
    rpp_interfaces::contract::GENERATOR_SET_PATH);
}

void NavigationInterface::bind_path_state()
{
  path_binding_.reset();
  path_state_route_.reset();
  if (!node_.layout_) {return;}
  std::size_t matches = 0;
  for (const auto & interface : node_.layout_->description.goals) {
    if (!generator_.empty() && interface.name != generator_) {continue;}
    for (const auto & position : interface.layout.fields) {
      if (position.resource.empty() || (!resource_.empty() && position.resource != resource_) ||
        position.name != position.resource + ".position" ||
        position.quantity != Field::QUANTITY_LINEAR_POSITION || position.frame_id.empty() ||
        position.coordinate_kind != Field::COORDINATE_EUCLIDEAN ||
        (position.width != 2 && position.width != 3)) {continue;}
      const auto * orientation = find_field(interface.layout, position.resource + ".orientation");
      if (orientation && !compatible(Kind::Pose, position, *orientation)) {continue;}
      ++matches;
      path_binding_ = prepare_binding(Kind::Pose, interface, position, orientation ? *orientation : Field{});
      path_binding_->frame = frame_prefix_ + position.frame_id;
    }
  }
  if (matches != 1) {path_binding_.reset(); return;}
  const auto measured = node_helpers::describe_layout(node_.layout_->measured_fields);
  const auto & target = path_binding_->linear;
  const auto * position = find_field(measured, target.name);
  if (position && same_field(target, *position)) {
    PathStateRoute route;
    route.position = *position;
    const auto * orientation = find_field(measured, target.resource + ".orientation");
    if (orientation && compatible(Kind::Pose, *position, *orientation)) {route.orientation = *orientation;}
    const auto * velocity = find_field(measured, target.resource + ".linearVelocity");
    if (velocity && velocity->quantity == Field::QUANTITY_LINEAR_VELOCITY &&
      velocity->coordinate_kind == Field::COORDINATE_EUCLIDEAN && velocity->width == target.width &&
      velocity->frame_id == target.child_frame_id) {route.velocity = *velocity;}
    path_state_route_ = std::move(route);
  }
  if (path_goal_ && path_binding_->goal.interface_name == path_interface_) {
    const auto & route = node_.layout_->goal_routes.at(path_interface_);
    path_native_mask_.resize(route.size());
    for (std::size_t axis = 0; axis < route.size(); ++axis) {path_native_mask_[route[axis]] = path_update_mask_[axis];}
  }
}

bool NavigationInterface::valid_path(const nav_msgs::msg::Path & path) const
{
  if (!path_binding_ || path.poses.empty() || path.header.frame_id != path_binding_->frame ||
    path.poses.size() > rpp_interfaces::ReferenceHorizon::max_values / path_binding_->goal.values.size()) {return false;}
  for (const auto & point : path.poses) {
    if (!point.header.frame_id.empty() && point.header.frame_id != path.header.frame_id) {return false;}
    const auto & p = point.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {return false;}
    if (path_binding_->angular.width) {
      const auto & q = point.pose.orientation;
      const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {return false;}
    }
  }
  return true;
}

std::optional<NavigationInterface::PathSnapshot> NavigationInterface::path_snapshot()
{
  std::lock_guard<std::mutex> tick_lock(node_.control_tick_mutex_);
  std::lock_guard<std::mutex> lock(node_.mailbox_mutex_);
  const auto now = node_.now();
  const auto time = timing::timestamp_ns(now);
  if (!time || !node_.inputs_.enabled || !node_.routes_ready_ || node_.slot_operation_pending_ ||
    node_.stop_requested_ || !node_.inputs_.state_time_ns ||
    !timing::measurement_fresh(*node_.inputs_.state_time_ns, *time, node_.state_timeout_)) {return std::nullopt;}
  PathSnapshot snapshot;
  snapshot.time_ns = *time;
  snapshot.reset_revision = node_.inputs_.reset_revision;
  snapshot.pose.header.stamp = now;
  snapshot.pose.header.frame_id = path_binding_->frame;
  snapshot.pose.pose.orientation.w = 1.0;
  if (path_state_route_) {
    const auto & state = node_.incoming_state_;
    const auto & route = *path_state_route_;
    auto & p = snapshot.pose.pose.position;
    p.x = state[route.position.offset];
    p.y = state[route.position.offset + 1];
    if (route.position.width == 3) {p.z = state[route.position.offset + 2];}
    if (route.orientation) {
      const auto & field = *route.orientation;
      auto & q = snapshot.pose.pose.orientation;
      if (field.width == 1) {
        const double yaw = state[field.offset];
        q.z = std::sin(yaw * 0.5); q.w = std::cos(yaw * 0.5);
      } else {
        q.x = state[field.offset]; q.y = state[field.offset + 1];
        q.z = state[field.offset + 2]; q.w = state[field.offset + 3];
      }
    }
    if (route.velocity) {
      for (std::size_t axis = 0; axis < route.velocity->width; ++axis) {
        snapshot.speed = std::hypot(snapshot.speed, static_cast<double>(state[route.velocity->offset + axis]));
      }
    }
  }
  return snapshot;
}

rclcpp_action::GoalResponse NavigationInterface::accept_path(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal> goal)
{
  // Check the current generator, including after runtime replacement.
  if (!supports_path() || !valid_path(goal->path) || !path_snapshot() ||
    (!goal->controller_id.empty() && goal->controller_id != "rpp" && goal->controller_id != "FollowPath") ||
    !goal->progress_checker_id.empty() || !goal->path_handler_id.empty()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "FollowPath rejected: check setPath support, path/frame, enable/state, and plugin IDs (controller_id is 'rpp' or 'FollowPath').");
    return rclcpp_action::GoalResponse::REJECT;
  }
  try {
    std::lock_guard<std::mutex> tick_lock(node_.control_tick_mutex_);
    const auto & selected = goal->goal_checker_id.empty() ? goal_monitor_ : goal->goal_checker_id;
    if (!node_.script_->has_goal_monitor(selected)) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
        "FollowPath rejected: goal_checker_id must select an active goal-capable MissionMonitor.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  } catch (const std::exception & error) {
    node_.mark_worker_failed();
    RCLCPP_ERROR(node_.get_logger(), "Mission monitor description threw; control worker stopped: %s", error.what());
    return rclcpp_action::GoalResponse::REJECT;
  } catch (...) {
    node_.mark_worker_failed();
    RCLCPP_ERROR(node_.get_logger(), "Mission monitor description threw; control worker stopped.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto & name = path_binding_->goal.interface_name;
  if (node_.reference_claims_.claimed(name) && !node_.reference_claims_.authorizes(name, path_token_)) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "FollowPath rejected: reference interface is already claimed.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void NavigationInterface::start_path(std::shared_ptr<PathHandle> goal)
{
  if (!supports_path() || !valid_path(goal->get_goal()->path)) {
    auto result = std::make_shared<FollowPath::Result>();
    result->error_code = FollowPath::Result::INVALID_PATH;
    result->error_msg = "Reference generator no longer supports setPath, or path binding changed.";
    goal->abort(result);
    return;
  }
  const bool new_mission = !path_goal_ || path_goal_->is_canceling();
  if (path_goal_) {
    auto result = std::make_shared<FollowPath::Result>();
    result->error_msg = "Replaced by a newer path.";
    if (path_goal_->is_canceling()) {path_goal_->canceled(result);} else {path_goal_->abort(result);}
  }
  path_goal_ = std::move(goal);
  path_goal_monitor_ = path_goal_->get_goal()->goal_checker_id.empty()
    ? goal_monitor_ : path_goal_->get_goal()->goal_checker_id;
  path_interface_ = path_binding_->goal.interface_name;
  if (!node_.reference_claims_.authorizes(path_interface_, path_token_)) {
    ControlScriptNode::RequestReference::Request request;
    ControlScriptNode::RequestReference::Response response;
    request.interface_name = path_interface_;
    request.identity = "NavigationInterface/FollowPath";
    node_.request_reference(request, response);
    if (!response.success) {finish_path(FollowPath::Result::NO_VALID_CONTROL, response.message); return;}
    path_token_ = response.identity_token;
    path_velocity_deadline_ns_.reset();
  }
  if (!submit_path(path_goal_->get_goal()->path, new_mission)) {
    finish_path(FollowPath::Result::NO_VALID_CONTROL, "Path submission or mission supervision preparation failed.");
  }
}

bool NavigationInterface::submit_path(const nav_msgs::msg::Path & path, bool new_mission)
{
  if (!supports_path() || !valid_path(path)) {return false;}
  auto & binding = *path_binding_;
  const auto width = binding.goal.values.size();
  rpp::List<double> values;
  values.resize(path.poses.size() * width);
  // A path owns the complete reference interface. Initialize unused inner goals
  // to zero and keep them fresh too, so all derived outputs have valid inputs.
  path_update_mask_.assign(width, true);
  std::fill(binding.goal.values.begin(), binding.goal.values.end(), 0.0);
  for (const auto & axis : binding.selections) {
    for (const auto & [index, mode] : axis) {binding.goal.values[index] = mode; path_update_mask_[index] = true;}
  }
  const auto & route = node_.layout_->goal_routes.at(path_interface_);
  path_native_mask_.resize(width);
  const auto current_time = timing::timestamp_ns(node_.now());
  const bool keep_velocity = path_velocity_deadline_ns_ && current_time &&
    *current_time < *path_velocity_deadline_ns_ &&
    path_route_revision_ == node_.worker_bindings_.route_revision && path_velocity_mask_.size() == width;
  if (!keep_velocity) {
    path_velocity_deadline_ns_.reset();
    path_velocity_mask_.resize(width);
  }
  for (std::size_t axis = 0; axis < width; ++axis) {
    const auto native = route[axis];
    path_native_mask_[native] = path_update_mask_[axis] && !(keep_velocity && path_velocity_mask_[native]);
  }
  for (std::size_t row = 0; row < path.poses.size(); ++row) {
    const auto & pose = path.poses[row].pose;
    auto & point = binding.goal.values;
    point[binding.linear.offset] = pose.position.x;
    point[binding.linear.offset + 1] = pose.position.y;
    if (binding.linear.width == 3) {point[binding.linear.offset + 2] = pose.position.z;}
    const auto & q = pose.orientation;
    if (binding.angular.width == 1) {
      point[binding.angular.offset] = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        q.x * q.x + q.w * q.w - q.y * q.y - q.z * q.z);
    } else if (binding.angular.width == 4) {
      point[binding.angular.offset] = q.x; point[binding.angular.offset + 1] = q.y;
      point[binding.angular.offset + 2] = q.z; point[binding.angular.offset + 3] = q.w;
    }
    for (std::size_t axis = 0; axis < width; ++axis) {values[row * width + route[axis]] = point[axis];}
  }
  rpp::List<double> final_target;
  final_target.resize(width);
  for (std::size_t axis = 0; axis < width; ++axis) {
    final_target[axis] = values[(path.poses.size() - 1) * width + axis];
  }
  std::lock_guard<std::mutex> tick_lock(node_.control_tick_mutex_);
  {
    std::lock_guard<std::mutex> lock(node_.mailbox_mutex_);
    if (!node_.inputs_.enabled || !node_.routes_ready_ || node_.slot_operation_pending_ || node_.stop_requested_) {return false;}
    path_reset_revision_ = node_.inputs_.reset_revision;
  }
  if (!supports_path() || !node_.reference_claims_.authorizes(path_interface_, path_token_)) {return false;}
  const auto now = node_.now();
  const auto deadline = timing::navigation_deadline(now, now, command_timeout_);
  if (!deadline) {return false;}
  try {
    node_.apply_pending_reset(*node_.script_, node_.worker_bindings_, path_reset_revision_);
    const auto prepared = node_.script_->prepare_mission(
      path_goal_monitor_, path_interface_, final_target, new_mission);
    if (prepared != rpp_interfaces::contract::CODE_OK) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
        "Mission monitor preparation rejected FollowPath (code %u).", static_cast<unsigned>(prepared));
      return false;
    }
    const auto status = node_.script_->set_path(path_interface_, values, path_native_mask_, *deadline);
    if (control_success(status)) {
      path_submitted_ns_ = *timing::timestamp_ns(now);
      path_route_revision_ = node_.worker_bindings_.route_revision;
      path_resume_values_ = std::move(final_target);
    }
    return control_success(status);
  } catch (const std::exception & error) {
    node_.mark_worker_failed();
    RCLCPP_ERROR(node_.get_logger(), "Path submission plugin threw; control worker stopped: %s", error.what());
    return false;
  } catch (...) {
    node_.mark_worker_failed();
    RCLCPP_ERROR(node_.get_logger(), "Path submission plugin threw; control worker stopped.");
    return false;
  }
}

void NavigationInterface::update_path()
{
  if (!path_goal_) {return;}
  if (path_goal_->is_canceling()) {finish_path(FollowPath::Result::NONE, "Path canceled.", true); return;}
  if (!supports_path() || path_binding_->goal.interface_name != path_interface_ ||
    !node_.reference_claims_.authorizes(path_interface_, path_token_) ||
    path_route_revision_ != node_.worker_bindings_.route_revision) {
    finish_path(FollowPath::Result::NO_VALID_CONTROL, "Reference ownership or setPath support changed."); return;
  }
  const auto snapshot = path_snapshot();
  if (!snapshot || snapshot->reset_revision != path_reset_revision_) {
    finish_path(FollowPath::Result::CONTROLLER_TIMED_OUT, "Control disabled, state stale, or execution reset."); return;
  }
  bool reached = false;
  bool monitor_stopped = false;
  std::string monitor_failure;
  bool valid = false;
  try {
    std::lock_guard<std::mutex> tick_lock(node_.control_tick_mutex_);
    const auto deadline = timing::navigation_deadline(snapshot->pose.header.stamp, node_.now(), command_timeout_);
    valid = supports_path() && deadline;
    if (valid && path_velocity_deadline_ns_ && snapshot->time_ns >= *path_velocity_deadline_ns_) {
      // Restore only the path's velocity values and selectors. Leaving pose
      // fields unselected preserves the generator's pruned path and progress.
      valid = control_success(node_.script_->set_point(
        path_interface_, path_resume_values_, path_velocity_mask_, *deadline));
      if (valid) {
        for (std::size_t axis = 0; axis < path_native_mask_.size(); ++axis) {path_native_mask_[axis] = true;}
        path_velocity_deadline_ns_.reset();
      }
    }
    valid = valid && control_success(node_.script_->refresh_goal(
      path_interface_, path_native_mask_, snapshot->time_ns, *deadline));
    const auto & output = node_.worker_bindings_.output;
    if (output.generated_at && *output.generated_at >= path_submitted_ns_) {
      valid = valid && output.status.state != ControlScriptNode::Status::FAULT;
      if (valid && output.status.state == ControlScriptNode::Status::RUNNING) {
        const auto evaluation = node_.script_->supervise_mission(snapshot->time_ns);
        monitor_stopped = !evaluation.ok;
        reached = evaluation.ok && evaluation.reached;
        if (monitor_stopped) {
          monitor_failure = evaluation.source.empty() ? "Mission supervision routes became unavailable."
            : "Mission monitor '" + evaluation.source + "' stopped execution (code " +
              std::to_string(evaluation.status) + ").";
          if (!evaluation.message.empty()) {monitor_failure += " " + evaluation.message;}
        }
      }
    }
  } catch (const std::exception & error) {
    node_.mark_worker_failed();
    RCLCPP_ERROR(node_.get_logger(), "FollowPath plugin call threw; control worker stopped: %s", error.what());
    finish_path(FollowPath::Result::NO_VALID_CONTROL, "FollowPath plugin call threw; control worker stopped."); return;
  } catch (...) {
    node_.mark_worker_failed();
    RCLCPP_ERROR(node_.get_logger(), "FollowPath plugin call threw; control worker stopped.");
    finish_path(FollowPath::Result::NO_VALID_CONTROL, "FollowPath plugin call threw; control worker stopped."); return;
  }
  if (!valid) {finish_path(FollowPath::Result::NO_VALID_CONTROL, "Path expired or control failed."); return;}
  if (monitor_stopped) {
    finish_path(FollowPath::Result::NO_VALID_CONTROL, monitor_failure); return;
  }
  publish_path_feedback(*snapshot);
  if (reached) {finish_path(FollowPath::Result::NONE, "Selected mission monitor reports the goal reached.");}
}

void NavigationInterface::finish_path(std::uint16_t code, const std::string & message, bool canceled)
{
  auto goal = std::exchange(path_goal_, {});
  if (!goal) {return;}
  path_velocity_deadline_ns_.reset();
  std::string result_message = message;
  bool released = true;
  if (node_.reference_claims_.interface_for(path_token_)) {
    ControlScriptNode::ReleaseReference::Request request;
    ControlScriptNode::ReleaseReference::Response response;
    request.identity_token = path_token_;
    node_.release_reference(request, response);
    if (!response.success) {
      code = FollowPath::Result::NO_VALID_CONTROL;
      result_message += " Reference release failed: " + response.message;
      released = false;
    }
  }
  // Keep a failed release's token so a subsequent request can retry cleanup.
  if (released) {path_token_.clear(); path_interface_.clear();}
  auto result = std::make_shared<FollowPath::Result>();
  result->error_code = code;
  result->error_msg = result_message;
  if (canceled && goal->is_canceling()) {goal->canceled(result);}
  else if (code == FollowPath::Result::NONE) {goal->succeed(result);}
  else {goal->abort(result);}
}

void NavigationInterface::publish_path_feedback(const PathSnapshot & snapshot)
{
  if (!path_state_route_) {return;}
  auto feedback = std::make_shared<FollowPath::Feedback>();
  auto & tracking = feedback->tracking_feedback;
  tracking.header = snapshot.pose.header;
  tracking.robot_pose = snapshot.pose;
  tracking.speed = static_cast<float>(snapshot.speed);
  const auto & points = path_goal_->get_goal()->path.poses;
  const auto & robot = snapshot.pose.pose.position;
  double nearest = std::numeric_limits<double>::infinity();
  std::size_t closest = 0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto & p = points[index].pose.position;
    const double distance = std::hypot(p.x - robot.x, p.y - robot.y, p.z - robot.z);
    if (distance < nearest) {nearest = distance; closest = index;}
  }
  tracking.current_path_index = static_cast<std::uint32_t>(closest);
  if (points.size() > 1) {
    const auto segment = closest + 1 < points.size() ? closest : closest - 1;
    const auto & a = points[segment].pose.position;
    const auto & b = points[segment + 1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length > 1e-9) {
      tracking.position_tracking_error = static_cast<float>((dx * (robot.y - a.y) - dy * (robot.x - a.x)) / length);
      const auto & q = snapshot.pose.pose.orientation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), q.x * q.x + q.w * q.w - q.y * q.y - q.z * q.z);
      tracking.heading_tracking_error = static_cast<float>(std::remainder(std::atan2(dy, dx) - yaw, 6.283185307179586));
    }
  }
  const auto & end = points.back().pose.position;
  tracking.distance_to_goal = static_cast<float>(std::hypot(end.x - robot.x, end.y - robot.y, end.z - robot.z));
  double remaining = nearest;
  for (std::size_t index = closest + 1; index < points.size(); ++index) {
    const auto & a = points[index - 1].pose.position;
    const auto & b = points[index].pose.position;
    remaining += std::hypot(b.x - a.x, b.y - a.y, b.z - a.z);
  }
  tracking.remaining_path_length = static_cast<float>(remaining);
  // This is display feedback only. Pruning and lookahead remain in the plugins.
  path_goal_->publish_feedback(feedback);
}
}  // namespace rpp_control
