#pragma once

// Component configuration, horizon binding, buffer allocation, and actuator constraints. Run with ticks paused.
namespace rpp_control
{
template<bool Concurrent>
BasicControlScript<Concurrent>::BasicControlScript(rpp::BasicScriptRuntime<Concurrent> & runtime)
: runtime_(runtime),
  context_(runtime.get_context()),
  supervision_(runtime),
  state_signal_(signal_router_.add_signal("state")),
  reference_signal_(signal_router_.add_signal(reference_slot_)),
  command_signal_(signal_router_.add_signal("control")),
  actuator_signal_(signal_router_.add_signal("actuators")),
  realized_signal_(signal_router_.add_signal("realized"))
{}

template<bool Concurrent>
void BasicControlScript<Concurrent>::configure(
  const std::vector<double> & safe_actuator_values,
  std::function<void(const std::string &)> report_binding_error,
  std::function<void(const std::string &)> report_warning)
{
  report_binding_error_ = std::move(report_binding_error);
  report_warning_ = std::move(report_warning);

  // Plugins must finish configuration before their descriptions can be routed.
  for (const auto & slot : context_.list_subcomponents()) {
    for (const auto & component : runtime_.get_subcomponent_statuses(slot)) {
      if (component.state != rpp::LifecycleState::UNCONFIGURED) {
        continue;
      }

      const auto result = runtime_.configure_component(slot, component.name);
      if (!result.success) {
        const auto operation = "Configure " + slot + " '" + component.name + "'";
        if (report_binding_error_) {
          report_binding_error_(operation + ": " + result.message);
        } else {
          result.require_success(operation);
        }
      }
    }
  }

  const auto configured_components = components_matching([](const auto & component) {
        return component.state == rpp::LifecycleState::INACTIVE ||
               component.state == rpp::LifecycleState::ACTIVE;
  });
  prepare_execution(configured_components, safe_actuator_values);
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::prepare_execution(
  const ComponentStatuses & components, const std::vector<double> & safe_values)
{
  configured_ = false;
  const auto previous_generators = std::move(reference_generators_);
  const auto previous_actuators = actuator_layout();
  const auto previous_controls = command_layout();
  const auto previous_lower = application_lower_bounds_;
  const auto previous_upper = application_upper_bounds_;
  const auto previous_commands = remember_actuator_commands();

  std::vector<bool> previous_enabled;
  for (std::size_t axis = 0; axis < enabled_axes_.size(); ++axis) {
    previous_enabled.push_back(enabled_axes_[axis]);
  }
  build_routes(components);

  restore_generator_goals(previous_generators);

  bind_reference_generators();

  // Allocate numeric buffers before resuming ticks, then restore compatible settings.
  signal_router_.prepare_buffers();
  enabled_axes_.resize(command_layout().size());
  for (std::size_t axis = 0; axis < enabled_axes_.size(); ++axis) {
    enabled_axes_[axis] = true;
  }
  last_actuator_values_.resize(actuator_layout().size());
  active_actuators_.assign(actuator_layout().size(), false);
  feedback_valid_.assign(command_layout().size(), false);

  configure_actuator_limits(safe_values);
  restore_application_limits(previous_actuators, previous_lower, previous_upper);
  restore_enabled_axes(previous_controls, previous_enabled);

  clear_outputs();
  restore_actuator_commands(previous_commands);
  configured_ = state_layout().size() != 0 && actuator_layout().size() != 0;
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::rebuild_active_routes(
  const std::vector<double> & safe_actuator_values)
{
  prepare_execution(active_components(), safe_actuator_values);
}

template<bool Concurrent>
rpp::CallbackResult BasicControlScript<Concurrent>::validate_switch_routes(
  const std::string & slot,
  const rpp::SwitchRequest & request, const std::vector<double> & safe_actuator_values) const
{
  if (request.activate.empty()) {
    return {};
  }

  // Preview the requested selection without binding plugins or changing lifecycle states.
  auto components = active_components();
  auto & selected = components.at(slot);
  selected.erase(std::remove_if(selected.begin(), selected.end(),
    [&](const auto & component) {
      return std::find(request.deactivate.begin(), request.deactivate.end(),
                              component.name) != request.deactivate.end();
                   }),
    selected.end());
  for (const auto & component : runtime_.get_subcomponent_statuses(slot)) {
    if (component.state == rpp::LifecycleState::INACTIVE &&
      std::find(request.activate.begin(), request.activate.end(), component.name) !=
      request.activate.end())
    {
      selected.push_back(component);
    }
  }

  std::string error;
  BasicControlScript preview(runtime_);
  preview.preview_only_ = true;
  preview.signal_router_.set_layout(preview.actuator_signal_, actuator_layout());
  preview.signal_router_.set_layout(preview.state_signal_, state_layout());
  preview.report_binding_error_ = [&](const std::string & message) {
      if (error.empty()) {
        error = message;
      }
    };
  try {
    preview.build_routes(components);
    if (!error.empty()) {
      return rpp::CallbackResult::failure(error);
    }
    preview.configure_actuator_limits(safe_actuator_values);
    return {};
  } catch (const std::exception & failure) {
    return rpp::CallbackResult::failure(failure.what());
  }
}

template<bool Concurrent>
rpp::CallbackResult BasicControlScript<Concurrent>::bind_reference_before_activation(
  const std::string & name)
{
  try {
    auto & generator = reference_generators_.at(generator_index(name));
    bind_reference_generator(generator);
    return {};
  } catch (const std::exception & error) {
    return rpp::CallbackResult::failure(error.what());
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::bind_reference_generators()
{
  if (preview_only_) {
    return;
  }

  // Rebinding is serialized with ticks and preserves active goals and lifecycle states.
  for (auto & generator : reference_generators_) {
    try {
      bind_reference_generator(generator);
    } catch (const std::exception & error) {
      generator.routed = false;
      if (report_binding_error_) {
        report_binding_error_(error.what());
      } else {
        RPP_LOG_WARN(*context_.get_logger(), error.what());
      }
    }
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::bind_reference_generator(ReferenceGeneratorBinding & generator)
{
  const auto length = generator.horizon.horizon_length();
  const auto previous = bound_generator_horizons_.find(generator.instance_id);
  if (previous != bound_generator_horizons_.end() && previous->second.length == length &&
    previous->second.offsets == generator.offsets)
  {
    return;
  }

  rpp::List<std::uint64_t> offsets;
  offsets.resize(generator.offsets.size());
  for (std::size_t index = 0; index < generator.offsets.size(); ++index) {
    offsets[index] = generator.offsets[index];
  }

  const auto code = generator.plugin->bind(offsets, length);
  if (!control_success(code)) {
    throw std::invalid_argument("Reference generator '" + generator.name +
      "' rejected the requested horizon (code " + std::to_string(code) + ")");
  }

  bound_generator_horizons_[generator.instance_id] = {generator.offsets, length};
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::set_actuator_limit(
  std::size_t index, double lower, double upper)
{
  if (!configured_ || index >= safe_actuator_values_.size()) {
    throw std::invalid_argument("Invalid application actuator limit");
  }
  apply_actuator_limit(index, lower, upper);
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::configure_actuator_limits(
  const std::vector<double> & safe_actuator_values)
{
  const auto actuator_count = actuator_layout().size();
  const auto unbounded = std::numeric_limits<double>::infinity();
  allocator_lower_bounds_.assign(actuator_count, -unbounded);
  allocator_upper_bounds_.assign(actuator_count, unbounded);
  actuator_rate_limits_.assign(actuator_count, unbounded);
  for (const auto & allocator : allocators_) {
    // Admitted outputs retain physical limits even while their control inputs are unavailable.
    if (allocator.output_route == missing_source) {
      continue;
    }
    for (const auto & limit : allocator.limits) {
      add_limit(limit, actuator_layout(), allocator_lower_bounds_, allocator_upper_bounds_,
        actuator_rate_limits_);
    }
  }

  if (!safe_actuator_values.empty() && safe_actuator_values.size() != actuator_count) {
    throw std::invalid_argument("Safe actuator vector width mismatch");
  }
  safe_actuator_values_.assign(actuator_count, 0.0);
  if (!safe_actuator_values.empty()) {
    safe_actuator_values_ = safe_actuator_values;
  }
  if (!actuator_layout().valid_row(safe_actuator_values_)) {
    throw std::invalid_argument("Safe actuator command is invalid");
  }

  // Additional application limits start unbounded. Even with no additional
  // limits, stop output must fit the allocator's bounds.
  application_lower_bounds_.assign(actuator_count, -unbounded);
  application_upper_bounds_.assign(actuator_count, unbounded);
  for (std::size_t index = 0; index < actuator_count; ++index) {
    const double safe_command = safe_actuator_values_[index];
    if (!std::isfinite(safe_command) || safe_command < allocator_lower_bounds_[index] ||
      safe_command > allocator_upper_bounds_[index])
    {
      throw std::invalid_argument("Safe actuator command is outside the declared limits");
    }
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::apply_actuator_limit(
  std::size_t index, double lower, double upper)
{
  if (std::isnan(lower) || std::isnan(upper) || lower > upper ||
    safe_actuator_values_[index] < lower || safe_actuator_values_[index] > upper ||
    std::max(lower, allocator_lower_bounds_[index]) >
    std::min(upper, allocator_upper_bounds_[index]))
  {
    throw std::invalid_argument("Invalid application actuator limit");
  }
  application_lower_bounds_[index] = lower;
  application_upper_bounds_[index] = upper;
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::add_limit(
  const ActuatorLimit & limit,
  const rpp_interfaces::FieldLayout & full, std::vector<double> & lower,
  std::vector<double> & upper, std::vector<double> & rates)
{
  const rpp_interfaces::FieldLayout fields(limit.fields.fields);
  const auto route = rpp_interfaces::field_route(full, fields);
  if ((!limit.lower.empty() && limit.lower.size() != route.size()) ||
    (!limit.upper.empty() && limit.upper.size() != route.size()) ||
    (!limit.maxRate.empty() && limit.maxRate.size() != route.size()))
  {
    throw std::invalid_argument("Actuator limit width mismatch");
  }
  for (std::size_t axis = 0; axis < route.size(); ++axis) {
    const auto index = route[axis];
    const double lower_bound =
      limit.lower.empty() ? -std::numeric_limits<double>::infinity() : limit.lower[axis];
    const double upper_bound =
      limit.upper.empty() ? std::numeric_limits<double>::infinity() : limit.upper[axis];
    const double rate =
      limit.maxRate.empty() ? std::numeric_limits<double>::infinity() : limit.maxRate[axis];
    if (std::isnan(lower_bound) || std::isnan(upper_bound) || std::isnan(rate) || rate < 0.0) {
      throw std::invalid_argument("Invalid actuator limits");
    }
    lower[index] = std::max(lower[index], lower_bound);
    upper[index] = std::min(upper[index], upper_bound);
    rates[index] = std::min(rates[index], rate);
    if (lower[index] > upper[index]) {
      throw std::invalid_argument("Contradictory actuator limits");
    }
  }
}

template<bool Concurrent>
template<class Bind>
void BasicControlScript<Concurrent>::bind_component(
  const std::string & slot, const std::string & name, const Bind & bind)
{
  try {
    bind();
  } catch (const std::exception & error) {
    reject_binding(slot, name, error.what());
  }
}

template<bool Concurrent>
bool BasicControlScript<Concurrent>::binding_rejected(
  const std::string & slot, const std::string & name) const
{
  return rejected_bindings_.find({slot, name}) != rejected_bindings_.end();
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::reject_binding(
  const std::string & slot, const std::string & name, const std::string & why)
{
  if (!report_binding_error_) {
    throw std::invalid_argument(why);
  }
  if (rejected_bindings_.emplace(slot, name).second) {
    report_binding_error_(slot + " '" + name + "': " + why);
  }
}
} // namespace rpp_control
