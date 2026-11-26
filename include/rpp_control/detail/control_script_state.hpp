#pragma once

// Layout and buffer access, instance lookups, auxiliary snapshots, and state preservation across reconfiguration.
namespace rpp_control
{
template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::set_state(
  rpp::ListConst<double> values, std::uint64_t time_ns)
{
  if (!configured_ || !state_source_active()) {
    return rpp_interfaces::contract::CODE_NOT_READY;
  }
  return state_provider_->setState(values, time_ns);
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::read_state()
{
  if (!configured_ || !state_source_active()) {
    return rpp_interfaces::contract::CODE_NOT_READY;
  }
  const auto [values, time_ns, status] = state_provider_->getState();
  if (!control_success(status)) {return status;}
  if (values.size() != state_layout().size()) {
    return rpp_interfaces::contract::CODE_INVALID_INPUT;
  }
  // The ingress owner checks freshness; step validates the complete numeric row.
  auto & destination = state();
  for (std::size_t index = 0; index < values.size(); ++index) {
    destination[index] = values[index];
  }
  return status;
}

template<bool Concurrent>
bool BasicControlScript<Concurrent>::ready() const
{
  return configured_;
}

template<bool Concurrent>
const rpp_interfaces::FieldLayout & BasicControlScript<Concurrent>::state_layout() const
{
  return signal_router_.layout(state_signal_);
}

template<bool Concurrent>
const rpp_interfaces::FieldLayout &
BasicControlScript<Concurrent>::generated_reference_layout() const
{
  return signal_router_.layout(reference_signal_);
}

template<bool Concurrent>
const rpp_interfaces::FieldLayout &
BasicControlScript<Concurrent>::controller_reference_input_layout(const std::string & name) const
{
  return controllers_.at(controller_index(name)).layout.reference();
}

template<bool Concurrent>
const rpp_interfaces::ControlLawLayout & BasicControlScript<Concurrent>::controller_layout(
  const std::string & name) const
{
  return controllers_.at(controller_index(name)).layout;
}

template<bool Concurrent>
const rpp_interfaces::FieldLayout & BasicControlScript<Concurrent>::command_layout() const
{
  return signal_router_.layout(command_signal_);
}

template<bool Concurrent>
const rpp_interfaces::FieldLayout & BasicControlScript<Concurrent>::actuator_layout() const
{
  return signal_router_.layout(actuator_signal_);
}

template<bool Concurrent>
std::vector<std::string> BasicControlScript<Concurrent>::controller_names() const
{
  std::vector<std::string> names;
  for (const auto & controller : controllers_) {
    names.push_back(controller.name);
  }
  return names;
}

template<bool Concurrent>
std::vector<std::string> BasicControlScript<Concurrent>::goal_names() const
{
  std::vector<std::string> names;
  for (const auto & generator : reference_generators_) {
    if (generator.targets.size() != 0) {
      names.push_back(generator.name);
    }
  }
  return names;
}

template<bool Concurrent>
const rpp_interfaces::FieldLayout & BasicControlScript<Concurrent>::goal_layout(
  const std::string & name) const
{
  return reference_generators_.at(generator_index(name)).targets;
}

template<bool Concurrent>
const std::string & BasicControlScript<Concurrent>::goal_instance_id(const std::string & name) const
{
  return reference_generators_.at(generator_index(name)).instance_id;
}

template<bool Concurrent>
rpp::List<double> & BasicControlScript<Concurrent>::state()
{
  return signal_router_.signal_values(state_signal_);
}

template<bool Concurrent>
rpp::List<double> & BasicControlScript<Concurrent>::references()
{
  return signal_router_.signal_values(reference_signal_);
}

template<bool Concurrent>
rpp::List<double> & BasicControlScript<Concurrent>::commands()
{
  return signal_router_.signal_values(command_signal_);
}

template<bool Concurrent>
rpp::List<bool> & BasicControlScript<Concurrent>::enabled_axes()
{
  return enabled_axes_;
}

template<bool Concurrent>
const rpp::List<double> & BasicControlScript<Concurrent>::actuators() const
{
  return signal_router_.signal_values(actuator_signal_);
}

template<bool Concurrent>
const rpp::List<double> & BasicControlScript<Concurrent>::realized() const
{
  return signal_router_.signal_values(realized_signal_);
}

template<bool Concurrent>
bool BasicControlScript<Concurrent>::has_previous_control() const
{
  return !feedback_valid_.empty() &&
         std::all_of(feedback_valid_.begin(), feedback_valid_.end(), [](bool valid) {
             return valid;
      });
}

template<bool Concurrent>
ControlAuxiliaryState BasicControlScript<Concurrent>::controller_auxiliary_state() const
{
  ControlAuxiliaryState auxiliary;
  append_auxiliary_states<rpp_interfaces::ControlLaw>(controller_slot_, auxiliary);
  return auxiliary;
}

template<bool Concurrent>
AuxiliaryState BasicControlScript<Concurrent>::allocator_auxiliary_state() const
{
  AuxiliaryState auxiliary;
  append_auxiliary_states<rpp_interfaces::ActuationMapper>(allocator_slot_, auxiliary);
  return auxiliary;
}

template<bool Concurrent>
rpp::List<double> & BasicControlScript<Concurrent>::actuator_values()
{
  return signal_router_.signal_values(actuator_signal_);
}

template<bool Concurrent>
rpp::List<double> & BasicControlScript<Concurrent>::realized_values()
{
  return signal_router_.signal_values(realized_signal_);
}

template<bool Concurrent>
std::size_t BasicControlScript<Concurrent>::generator_index(const std::string & name) const
{
  for (std::size_t index = 0; index < reference_generators_.size(); ++index) {
    if (reference_generators_[index].name == name) {
      return index;
    }
  }
  throw std::invalid_argument("Unknown reference generator: " + name);
}

template<bool Concurrent>
std::size_t BasicControlScript<Concurrent>::controller_index(const std::string & name) const
{
  if (name.empty() && controllers_.size() == 1) {
    return 0;
  }
  for (std::size_t index = 0; index < controllers_.size(); ++index) {
    if (controllers_[index].name == name) {
      return index;
    }
  }
  throw std::invalid_argument("Unknown or ambiguous controller: " + name);
}

template<bool Concurrent>
template<class Plugin>
void BasicControlScript<Concurrent>::append_auxiliary_states(
  const std::string & slot, AuxiliaryState & auxiliary) const
{
  // The owner pauses ticks before collecting. Guards keep foreign and native
  // instances available throughout each auxiliary-state read.
  for (const auto & component : runtime_.get_subcomponent_statuses(slot)) {
    if (component.state != rpp::LifecycleState::ACTIVE) {
      continue;
    }
    auto [read, snapshot] = context_.template with_active_component<Plugin>(
      slot, component.name, [](auto & plugin) {return plugin.getAuxiliaryState();},
      component.instance_id);
    if (!read) {
      throw std::runtime_error("Cannot read auxiliary state from '" + component.name + "'");
    }
    auxiliary.merge(*snapshot);
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::restore_generator_goals(
  const std::vector<ReferenceGeneratorBinding> & previous_generators)
{
  // Goals survive only for the same generator instance and compatible target fields.
  for (auto & generator : reference_generators_) {
    const auto previous = std::find_if(previous_generators.begin(), previous_generators.end(),
        [&](const auto & candidate) {return candidate.instance_id == generator.instance_id;});
    if (previous == previous_generators.end() ||
      !rpp_interfaces::validate_required_fields(previous->targets, generator.targets))
    {
      continue;
    }
    const auto route = rpp_interfaces::field_route(previous->targets, generator.targets);
    for (std::size_t scalar = 0; scalar < route.size(); ++scalar) {
      generator.deadlines[scalar] = previous->deadlines[route[scalar]];
      generator.trajectory_timing[scalar] = previous->trajectory_timing[route[scalar]];
    }
    generator.last_sample_ns = previous->last_sample_ns;
    generator.timing_pending = previous->timing_pending;
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::restore_enabled_axes(
  const rpp_interfaces::FieldLayout & previous_layout,
  const std::vector<bool> & previous_enabled_axes)
{
  // Switching components must not silently re-enable command axes that still exist.
  for (std::size_t field = 0; field < command_layout().fields().size(); ++field) {
    const auto previous =
      rpp_interfaces::find_field(previous_layout, command_layout().fields()[field].name);
    if (!previous ||
      !rpp_interfaces::same_field(previous_layout, *previous, command_layout(), field))
    {
      continue;
    }
    const auto & source = previous_layout.ranges()[*previous];
    const auto & destination = command_layout().ranges()[field];
    if (source.offset + source.size > previous_enabled_axes.size()) {
      continue;
    }
    for (std::size_t axis = 0; axis < source.size; ++axis) {
      enabled_axes_[destination.offset + axis] = previous_enabled_axes[source.offset + axis];
    }
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::restore_application_limits(
  const rpp_interfaces::FieldLayout & previous_layout, const std::vector<double> & lower,
  const std::vector<double> & upper)
{
  // A route rebuild must preserve application caps by field meaning, even if
  // allocator order changes. New actuator fields start without additional caps.
  for (std::size_t field = 0; field < actuator_layout().fields().size(); ++field) {
    const auto previous =
      rpp_interfaces::find_field(previous_layout, actuator_layout().fields()[field].name);
    if (!previous ||
      !rpp_interfaces::same_field(previous_layout, *previous, actuator_layout(), field))
    {
      continue;
    }
    const auto & source = previous_layout.ranges()[*previous];
    const auto & destination = actuator_layout().ranges()[field];
    if (source.offset + source.size > lower.size() || source.offset + source.size > upper.size()) {
      continue;
    }
    for (std::size_t offset = 0; offset < source.size; ++offset) {
      apply_actuator_limit(
        destination.offset + offset, lower[source.offset + offset], upper[source.offset + offset]);
    }
  }
}

template<bool Concurrent>
auto BasicControlScript<Concurrent>::remember_actuator_commands() const
-> ActuatorCommandsByInstance
{
  ActuatorCommandsByInstance commands;
  for (const auto & allocator : allocators_) {
    if (allocator.output_route == missing_source) {
      continue;
    }
    auto & previous = commands[allocator.instance_id];
    for (const auto index : signal_router_.indices(allocator.output_route)) {
      if (index >= last_actuator_values_.size()) {
        break;
      }
      previous.push_back(last_actuator_values_[index]);
    }
  }
  return commands;
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::restore_actuator_commands(
  const ActuatorCommandsByInstance & commands)
{
  // Surviving allocators keep their rate-limit history. Removed channels
  // keep the safe output written by clear_outputs().
  for (const auto & allocator : allocators_) {
    if (allocator.output_route == missing_source) {
      continue;
    }
    const auto previous = commands.find(allocator.instance_id);
    if (previous == commands.end() ||
      previous->second.size() != signal_router_.indices(allocator.output_route).size())
    {
      continue;
    }
    for (std::size_t axis = 0; axis < signal_router_.indices(allocator.output_route).size();
      ++axis)
    {
      const auto index = signal_router_.indices(allocator.output_route)[axis];
      const auto command = previous->second[axis];
      if (command >= allocator_lower_bounds_[index] && command <= allocator_upper_bounds_[index]) {
        last_actuator_values_[index] = command;
      }
    }
  }
}
} // namespace rpp_control
