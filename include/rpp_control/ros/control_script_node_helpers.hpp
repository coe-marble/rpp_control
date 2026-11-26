#pragma once

#include <rpp_control/ros/control_script_time.hpp>

#include <rpp_ros_interfaces/msg/control_script_status.hpp>
#include <rpp_ros_interfaces/msg/numeric_field.hpp>
#include <rpp_ros_interfaces/msg/numeric_layout.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node.hpp>
#include <rpp_control/auxiliary_state.hpp>
#include <rpp_control/control_script.hpp>
#include <rpp_interfaces/field_routing.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// ROS parameter declarations, message and handover conversion, resource mapping,
// and component status snapshots.
namespace rpp_control::node_helpers
{
using Status = rpp_ros_interfaces::msg::ControlScriptStatus;
using NumericLayout = rpp_ros_interfaces::msg::NumericLayout;
using NumericField = rpp_ros_interfaces::msg::NumericField;
using ControlScript = BasicControlScript<true>;
namespace contract = rpp_interfaces::contract;

template<class Value>
Value declare_startup_parameter(rclcpp::Node & node, const std::string & name,
  const Value & default_value, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  // These values size buffers or set worker timing and are applied only at startup.
  descriptor.read_only = true;
  return node.declare_parameter<Value>(name, default_value, descriptor);
}

inline time_helpers::Duration declare_duration(
  rclcpp::Node & node, const std::string & name, double default_seconds)
{
  return time_helpers::duration_from_seconds(name,
    declare_startup_parameter<double>(
      node, name, default_seconds, "Positive duration in seconds."));
}

inline NumericLayout describe_layout(const rpp_interfaces::FieldLayout & layout)
{
  NumericLayout message;
  message.value_count = static_cast<std::uint32_t>(layout.size());
  for (std::size_t index = 0; index < layout.fields().size(); ++index) {
    const auto & field = layout.fields()[index];
    const auto & range = layout.ranges()[index];
    NumericField description;
    description.name = field.name;
    description.coordinate_kind = field.kind;
    description.quantity = field.quantity;
    description.offset = static_cast<std::uint32_t>(range.offset);
    description.width = static_cast<std::uint32_t>(range.size);
    description.frame_id = field.frameId;
    description.child_frame_id = field.childFrameId;
    description.resource = field.resource;
    message.fields.push_back(std::move(description));
  }
  return message;
}

struct NumericInterfaceRoute
{
  std::string name;
  rpp_interfaces::FieldLayout fields;
  std::vector<std::size_t> indices;
};

inline std::vector<NumericInterfaceRoute> prepare_resource_interfaces(
  const rpp_interfaces::FieldLayout & full_layout,
  std::map<std::string, rpp_interfaces::FieldLayout> & definitions)
{
  std::map<std::string, std::vector<rpp_interfaces::Field>> resources;
  for (const auto & field : full_layout.fields()) {
    // Hardware-facing streams follow physical resources, independently of which
    // controller interfaces and allocators contribute their values.
    const auto & name = field.resource.empty() ? field.name : field.resource;
    resources[name].push_back(field);
  }
  std::vector<NumericInterfaceRoute> routes;
  for (auto & [name, fields] : resources) {
    rpp_interfaces::FieldLayout available(std::move(fields));
    auto previous = definitions.find(name);
    if (previous == definitions.end()) {
      previous = definitions.emplace(name, available).first;
    } else if (available.size() != previous->second.size() ||
      !rpp_interfaces::validate_required_fields(available, previous->second)) {
      throw std::invalid_argument("Cannot redefine numeric interface '" + name + "'");
    }
    // Preserve the published order when the script's internal field order changes.
    routes.push_back(
      {name, previous->second, rpp_interfaces::field_route(full_layout, previous->second)});
  }
  return routes;
}

inline Status make_status(std::uint8_t state, std::uint8_t failed_stage = Status::STAGE_NONE,
  std::uint8_t code = contract::CODE_OK)
{
  Status status;
  status.state = state;
  status.failed_stage = failed_stage;
  status.rpp_status_code = code;
  return status;
}

inline std::uint8_t describe_step_stage(ControlScript::StepStage stage)
{
  switch (stage) {
    case ControlScript::StepStage::None:
      return Status::STAGE_NONE;
    case ControlScript::StepStage::Sample:
      return Status::STAGE_SAMPLE;
    case ControlScript::StepStage::Compute:
      return Status::STAGE_COMPUTE;
    case ControlScript::StepStage::Map:
      return Status::STAGE_MAP;
  }
  return Status::STAGE_NONE;
}

template<class State>
rpp::HandoverState copy_application_handover(ControlScript & script,
  bool include_state,
  bool include_previous_control, const AuxiliaryState & auxiliary)
{
  State handover;
  auto measured = handover.state();
  auto state_description = measured.desc();
  script.state_layout().write(state_description);
  if (include_state) {
    const auto & state = script.state();
    auto values = measured.state();
    values.resize(state.size());
    for (std::size_t index = 0; index < state.size(); ++index) {
      values[index] = state[index];
    }
  }

  auto previous_control = handover.previousControl();
  auto control_description = previous_control.desc();
  script.command_layout().write(control_description);
  if (include_previous_control) {
    const auto & realized = script.realized();
    auto values = previous_control.state();
    values.resize(realized.size());
    for (std::size_t index = 0; index < realized.size(); ++index) {
      values[index] = realized[index];
    }
  }
  auto auxiliary_state = handover.auxiliaryState();
  auxiliary.write_to(auxiliary_state);
  return rpp::HandoverState(typename State::Const(std::move(handover)));
}

inline std::vector<std::string> active_instance_ids(const rpp::ConcurrentScriptRuntime & runtime)
{
  std::vector<std::string> instances;
  for (const auto & slot : runtime.get_context().list_subcomponents()) {
    for (const auto & component : runtime.get_subcomponent_statuses(slot)) {
      if (component.state == rpp::LifecycleState::ACTIVE) {
        instances.push_back(component.instance_id);
      }
    }
  }
  return instances;
}
} // namespace rpp_control::node_helpers
