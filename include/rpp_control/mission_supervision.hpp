#pragma once

#include <rpp_cpp/runtime.hpp>
#include <rpp_plugin_types/rpp_interfaces/MissionMonitor.hpp>
#include <rpp_schema/rpp_interfaces/MissionMonitorDesc.hpp>
#include <rpp_interfaces/field_routing.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace rpp_control
{
// The execution owner serializes supervision with ticks and lifecycle operations.
// Routes and buffers are prepared once; evaluation borrows a coherent control snapshot.
template<bool Concurrent>
class MissionSupervision
{
public:
  struct Evaluation
  {
    bool ok = false;
    bool reached = false;
    std::uint8_t status = rpp_interfaces::contract::CODE_NOT_READY;
    std::string source;
    std::string message;
  };

  struct MonitorStatus
  {
    bool evaluated = false, ok = false;
    std::uint8_t status = rpp_interfaces::contract::CODE_OK;
    std::string message;
    bool goal_evaluated = false, goal_reached = false;
    std::uint8_t goal_status = rpp_interfaces::contract::CODE_OK;
  };

  explicit MissionSupervision(rpp::BasicScriptRuntime<Concurrent> & runtime)
  : runtime_(runtime), context_(runtime.get_context()) {}

  std::vector<std::string> monitor_names() const
  {
    std::vector<std::string> names;
    for (const auto & component : runtime_.get_subcomponent_statuses(slot_)) {
      if (component.state == rpp::LifecycleState::ACTIVE) {names.push_back(component.name);}
    }
    return names;
  }

  // Read cached decisions only. Visiting status must not advance monitor history.
  template<class Visitor>
  std::optional<std::uint64_t> visit_status(const Visitor & visitor) const
  {
    const MonitorStatus unknown;
    for (const auto & component : runtime_.get_subcomponent_statuses(slot_)) {
      if (component.state != rpp::LifecycleState::ACTIVE) {continue;}
      const auto cached = std::find_if(monitors_.begin(), monitors_.end(),
          [&](const auto & monitor) {return monitor.instance == component.instance_id;});
      const bool selected = cached != monitors_.end() &&
        static_cast<std::size_t>(cached - monitors_.begin()) == goal_monitor_;
      visitor(component.name, component.instance_id,
        cached == monitors_.end() ? unknown : cached->last_status, selected);
    }
    return evaluated_at_;
  }

  bool has_goal_monitor(const std::string & name) const
  {
    std::size_t matches = 0;
    for (const auto & component : runtime_.get_subcomponent_statuses(slot_)) {
      if (component.state != rpp::LifecycleState::ACTIVE ||
        (!name.empty() && component.name != name)) {continue;}
      const auto plugin = context_.get_subcomponent_context(slot_, component.name)
        .template get_instance<rpp_interfaces::MissionMonitor>();
      if (!plugin->getDesc().as_struct().goal.fields.empty()) {++matches;}
    }
    return matches == 1;
  }

  std::uint8_t prepare(
    const std::string & goal_monitor,
    const std::vector<std::string> & monitor_names,
    const rpp_interfaces::FieldLayout & state_layout,
    const rpp_interfaces::FieldLayout & target_layout,
    const rpp_interfaces::FieldLayout & reference_layout,
    rpp::ListConst<double> target, std::uint64_t route_revision, bool new_mission)
  {
    prepared_ = false;
    if (target.size() != target_layout.size() || !target_layout.valid_row(target)) {
      return rpp_interfaces::contract::CODE_INVALID_INPUT;
    }

    std::vector<Monitor> monitors;
    std::size_t selected_goal = missing_monitor;
    const auto components = runtime_.get_subcomponent_statuses(slot_);
    for (const auto & name : monitor_names) {
      if (std::any_of(monitors.begin(), monitors.end(),
        [&](const auto & monitor) {return monitor.name == name;}))
      {
        return rpp_interfaces::contract::CODE_INVALID_INPUT;
      }
      const auto component = std::find_if(components.begin(), components.end(),
          [&](const auto & candidate) {
            return candidate.name == name && candidate.state == rpp::LifecycleState::ACTIVE;
        });
      if (component == components.end()) {return rpp_interfaces::contract::CODE_NOT_READY;}
      Monitor monitor;
      monitor.name = name;
      monitor.instance = component->instance_id;
      const auto plugin = context_.get_subcomponent_context(slot_, name)
        .template get_instance<rpp_interfaces::MissionMonitor>();
      const auto description = plugin->getDesc().as_struct();
      const rpp_interfaces::FieldLayout state(description.state.fields),
      reference(description.reference.fields), goal(description.goal.fields);
      if (!rpp_interfaces::validate_required_fields(state_layout, state) ||
        !rpp_interfaces::validate_required_fields(reference_layout, reference))
      {
        return rpp_interfaces::contract::CODE_INCOMPATIBLE;
      }
      monitor.state_route = rpp_interfaces::field_route(state_layout, state);
      monitor.reference_route = rpp_interfaces::field_route(reference_layout, reference);
      monitor.state.resize(state.size());
      monitor.reference.resize(reference.size());

      if ((!goal_monitor.empty() && name == goal_monitor) ||
        (goal_monitor.empty() && goal.size() != 0))
      {
        if (selected_goal != missing_monitor || goal.size() == 0) {
          return rpp_interfaces::contract::CODE_INVALID_INPUT;
        }
        if (!rpp_interfaces::validate_required_fields(target_layout, goal)) {
          return rpp_interfaces::contract::CODE_INCOMPATIBLE;
        }
        selected_goal = monitors.size();
        monitor.goal.resize(goal.size());
        rpp_interfaces::gather_fields(target, rpp_interfaces::field_route(target_layout, goal),
            monitor.goal);
      }
      monitors.push_back(std::move(monitor));
    }
    // The goal authority is always one of the mandatory isOk participants.
    if (selected_goal == missing_monitor) {return rpp_interfaces::contract::CODE_NOT_READY;}

    for (const auto & monitor : monitors) {
      const bool existing = std::any_of(monitors_.begin(), monitors_.end(),
          [&](const auto & previous) {return previous.instance == monitor.instance;});
      if (!new_mission && existing) {continue;}
      const auto reset = context_.template with_active_component<rpp_interfaces::MissionMonitor>(
        slot_, monitor.name, [](auto & plugin) {return plugin.resetHistory();}, monitor.instance);
      const auto code = reset.second.value_or(rpp_interfaces::contract::CODE_NOT_READY);
      if (code != rpp_interfaces::contract::CODE_OK) {return code;}
    }
    monitors_ = std::move(monitors);
    goal_monitor_ = selected_goal;
    route_revision_ = route_revision;
    evaluated_at_.reset();
    prepared_ = true;
    return rpp_interfaces::contract::CODE_OK;
  }

  Evaluation evaluate(
    rpp::ListConst<double> state, rpp::ListConst<double> reference,
    std::uint64_t time_ns, std::uint64_t route_revision)
  {
    // A lost guard or changed layout cannot silently authorize continuation.
    if (!prepared_ || route_revision != route_revision_) {return {};}
    evaluated_at_ = time_ns;
    // Clear the entire frame first, including goal results skipped after a veto.
    // A plugin exception leaves unevaluated entries unknown instead of stale/OK.
    for (auto & monitor : monitors_) {monitor.last_status = {};}
    Evaluation permission{true, false, rpp_interfaces::contract::CODE_OK, {}, {}};
    for (auto & monitor : monitors_) {
      rpp_interfaces::gather_fields(state, monitor.state_route, monitor.state);
      rpp_interfaces::gather_fields(reference, monitor.reference_route, monitor.reference);
      auto result = context_.template with_active_component<rpp_interfaces::MissionMonitor>(
        slot_, monitor.name,
        [&](auto & plugin) {
          return plugin.isOk(monitor.state, monitor.reference, time_ns);
          }, monitor.instance);
      auto & status = monitor.last_status;
      if (result.second) {
        auto & [ok, code, message] = *result.second;
        status.evaluated = true;
        status.ok = ok && code == rpp_interfaces::contract::CODE_OK;
        status.status = code;
        status.message = std::move(message);
      } else {
        status.status = rpp_interfaces::contract::CODE_NOT_READY;
        status.message = "Mission monitor is inactive or unavailable.";
      }
      if (permission.ok && !status.ok) {
        permission = {false, false, status.status, monitor.name, status.message};
      }
    }
    if (!permission.ok) {return permission;}
    // Only the selected instance is asked for arrival, after every guard approves.
    auto & goal = monitors_[goal_monitor_];
    const auto result = context_.template with_active_component<rpp_interfaces::MissionMonitor>(
      slot_, goal.name,
      [&](auto & plugin) {
        return plugin.isGoalReached(goal.state, goal.goal, time_ns);
        }, goal.instance);
    if (!result.second) {
      goal.last_status.goal_status = rpp_interfaces::contract::CODE_NOT_READY;
      return {false, false, rpp_interfaces::contract::CODE_NOT_READY, goal.name, {}};
    }
    const auto [reached, code] = *result.second;
    goal.last_status.goal_evaluated = true;
    goal.last_status.goal_reached = reached && code == rpp_interfaces::contract::CODE_OK;
    goal.last_status.goal_status = code;
    return {code == rpp_interfaces::contract::CODE_OK, reached, code,
      code == rpp_interfaces::contract::CODE_OK ? std::string{} : goal.name, {}};
  }

private:
  struct Monitor
  {
    std::string name, instance;
    std::vector<std::size_t> state_route, reference_route;
    rpp::List<double> state, reference, goal;
    MonitorStatus last_status;
  };
  static constexpr std::size_t missing_monitor = static_cast<std::size_t>(-1);
  rpp::BasicScriptRuntime<Concurrent> & runtime_;
  const rpp::ComponentContext & context_;
  const std::string slot_ = "mission_monitor";
  std::vector<Monitor> monitors_;
  std::size_t goal_monitor_ = missing_monitor;
  std::uint64_t route_revision_ = 0;
  std::optional<std::uint64_t> evaluated_at_;
  bool prepared_ = false;
};
}  // namespace rpp_control
