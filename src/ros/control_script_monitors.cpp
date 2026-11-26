#include <rpp_control/ros/control_script_node.hpp>

namespace rpp_control
{
void ControlScriptNode::publish_mission_monitors()
{
  auto & message = mission_monitor_message_;
  std::size_t count = 0;
  {
    // Publishing observes cached decisions; never call plugins or wait for a tick.
    std::unique_lock<std::mutex> tick_lock(control_tick_mutex_, std::try_to_lock);
    if (!tick_lock.owns_lock()) {return;}
    message.stamp.sec = 0;
    message.stamp.nanosec = 0;
    if (script_) {
      const auto evaluated_at = script_->visit_mission_monitor_status(
        [&](const auto & name, const auto & instance, const auto & status, bool selected) {
          // Reuse message/string storage while the active list remains stable.
          if (count == message.monitors.size()) {message.monitors.resize(count + 1);}
          auto & monitor = message.monitors[count++];
          monitor.name = name;
          monitor.instance_id = instance;
          monitor.evaluated = status.evaluated;
          monitor.ok = status.ok;
          monitor.rpp_status_code = status.status;
          monitor.message = status.message;
          monitor.selected_for_goal = selected;
          monitor.goal_evaluated = status.goal_evaluated;
          monitor.goal_reached = status.goal_reached;
          monitor.goal_status_code = status.goal_status;
        });
      if (evaluated_at) {
        const auto stamp = time_helpers::stamp_from_ns(*evaluated_at);
        if (!stamp) {return;}
        message.stamp = *stamp;
      }
    }
    message.monitors.resize(count);
  }
  mission_monitor_publisher_->publish(message);
}
}  // namespace rpp_control
