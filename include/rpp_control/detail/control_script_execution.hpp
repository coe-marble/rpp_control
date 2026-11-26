#pragma once

// Numeric sampling, controller execution, actuator mapping, runtime limits, feedback, and resets.
namespace rpp_control
{
template<bool Concurrent>
auto BasicControlScript<Concurrent>::step(std::uint64_t time_ns, std::uint64_t dt_ns) -> StepResult
{
  if (!configured_ || !state_source_active()) {
    reset();
    return {StepStage::Sample, rpp_interfaces::contract::CODE_NOT_READY};
  }
  if (!state_layout().valid_row(state())) {
    reset();
    return {StepStage::Sample, rpp_interfaces::contract::CODE_INVALID_INPUT};
  }
  if (dt_ns == 0 ||
    (!enabled_axes_.empty() && enabled_axes_.size() != command_layout().size())) {
    reset();
    return {StepStage::Compute, rpp_interfaces::contract::CODE_INVALID_INPUT};
  }

  // Run every stage so a failed branch does not suppress independent actuator outputs.
  const auto reference_status = sample_unchecked(time_ns);
  const auto control_status = compute_unchecked(time_ns, dt_ns);
  const auto mapping_status = command_map_unchecked(dt_ns);

  if (!control_success(reference_status)) {
    return {StepStage::Sample, reference_status};
  }
  if (!control_success(control_status)) {
    return {StepStage::Compute, control_status};
  }
  if (!control_success(mapping_status)) {
    return {StepStage::Map, mapping_status};
  }
  return {
    StepStage::None, merge_status(merge_status(reference_status, control_status), mapping_status)};
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::sample(std::uint64_t time_ns)
{
  if (!configured_ || !state_source_active()) {
    return rpp_interfaces::contract::CODE_NOT_READY;
  }
  if (!state_layout().valid_row(state())) {
    return rpp_interfaces::contract::CODE_INVALID_INPUT;
  }

  return sample_unchecked(time_ns);
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::sample_unchecked(std::uint64_t time_ns)
{
  goals_sampled_ = true;
  auto status = static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_OK);
  for (auto & generator : reference_generators_) {
    // Expiry changes numeric validity, never the generator's lifecycle state.
    for (auto & deadline : generator.deadlines) {
      if (time_ns >= deadline) {
        deadline = 0;
      }
    }

    for (std::size_t value = 0; value < generator.valid.size(); ++value) {
      generator.valid[value] = false;
    }
    generator.status = rpp_interfaces::contract::CODE_NOT_READY;
    if (!generator.selected) {
      continue;
    }

    const auto shared_deadline = generator.deadlines.empty()
      ? std::numeric_limits<std::uint64_t>::max()
      : *std::min_element(generator.deadlines.begin(), generator.deadlines.end());
    if (!generator.routed ||
      generator.horizon.maximum_offset() > std::numeric_limits<std::uint64_t>::max() - time_ns) {
      generator.status = generator.routed ? rpp_interfaces::contract::CODE_INVALID_INPUT
                                          : rpp_interfaces::contract::CODE_NOT_READY;
      status = merge_status(status, generator.status);
      continue;
    }

    const auto result = context_.template with_active_component<rpp_interfaces::ReferenceGenerator>(
      reference_slot_, generator.name,
      [&](auto & plugin) {
        auto [values, code] =
          plugin.sample(signal_router_.read(generator.state_route), time_ns);
        if (!control_success(code)) {
          return code;
        }
        if (!generator.horizon.valid_values(generator.outputs, values)) {
          return static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_INVALID_INPUT);
        }

        // Record activation once; coverage checks and warnings run on the executor.
        if (generator.timing_pending) {
          for (auto & timing : generator.trajectory_timing) {
            if (timing.pending) {timing.origin_ns = time_ns; timing.pending = false;}
          }
          generator.timing_pending = false;
        }
        generator.last_sample_ns = time_ns;

        const auto samples = generator.horizon.sample_count();
        for (std::size_t scalar = 0; scalar < generator.outputs.size(); ++scalar) {
          // Derived fields conservatively depend on every submitted target field.
          const auto target = generator.output_target_route[scalar];
          const auto deadline =
            target == missing_source ? shared_deadline : generator.deadlines[target];
          for (std::size_t sample = 0; sample < samples; ++sample) {
            const auto value = scalar * samples + sample;
            generator.values[value] = values[value];
            // A zero deadline already fails the unsigned expiry comparison.
            generator.valid[value] = generator.targets.size() == 0 ||
              time_ns + generator.horizon.offset(scalar, sample) < deadline;
          }
          generator.current_values[scalar] = values[scalar * samples];
        }

        if (!signal_router_.write(generator.output_route, generator.current_values)) {
          return static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_INVALID_INPUT);
        }
        return code;
      },
      generator.instance_id);
    generator.status = result.second.value_or(rpp_interfaces::contract::CODE_NOT_READY);
    status = merge_status(status, generator.status);
  }
  return status;
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::compute(std::uint64_t time_ns, std::uint64_t dt_ns)
{
  if (!configured_ || !state_source_active()) {
    return rpp_interfaces::contract::CODE_NOT_READY;
  }

  if (dt_ns == 0 || !state_layout().valid_row(state()) ||
    (!enabled_axes_.empty() && enabled_axes_.size() != command_layout().size())) {
    reset();
    control_computed_ = true;
    for (auto & controller : controllers_) {
      controller.status = rpp_interfaces::contract::CODE_INVALID_INPUT;
    }
    return rpp_interfaces::contract::CODE_INVALID_INPUT;
  }

  return compute_unchecked(time_ns, dt_ns);
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::compute_unchecked(
  std::uint64_t time_ns, std::uint64_t dt_ns)
{
  control_computed_ = true;
  auto status = static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_OK);
  for (auto & controller : controllers_) {
    controller.status = rpp_interfaces::contract::CODE_NOT_READY;
    bool available = controller.routed;
    bool goal_available = controller.layout.reference().size() == 0 || !goals_sampled_;

    // Direct callers may supply current references without running the generator stage.
    if (!goals_sampled_ && available) {
      available = controller.layout.horizon().sample_count() == 1;
      const auto values = signal_router_.read(controller.current_reference_route);
      for (std::size_t index = 0; index < values.size(); ++index) {
        controller.input_values[index] = values[index];
        controller.input_valid[index] = true;
      }
    } else if (available) {
      for (std::size_t index = 0; index < controller.reference_route.size(); ++index) {
        const auto & route = controller.reference_route[index];
        const auto & generator = reference_generators_[route.generator];
        if (!control_success(generator.status)) {
          available = false;
          break;
        }
        controller.input_values[index] = generator.values[route.value];
        controller.input_valid[index] = generator.valid[route.value];
        goal_available = goal_available || controller.input_valid[index];
      }
    }

    if (!available) {
      clear_controller_output(controller);
      status = merge_status(status, controller.status);
      continue;
    }
    if (!goal_available) {
      clear_controller_output(controller);
      controller.status = rpp_interfaces::contract::CODE_OK;
      continue;
    }

    // Previous control is valid only when all of this controller's channels have feedback.
    const auto & indices = signal_router_.indices(controller.output_route);
    bool feedback_valid = controller.commands_active;
    for (std::size_t axis = 0; axis < indices.size(); ++axis) {
      controller.enabled_axes[axis] = enabled_axes_.empty() || enabled_axes_[indices[axis]];
      feedback_valid = feedback_valid && feedback_valid_[indices[axis]];
    }

    const auto result = context_.template with_active_component<rpp_interfaces::ControlLaw>(
      controller_slot_, controller.name,
      [&](auto & plugin) {
        if (!controller.layout.horizon().valid_values(
              controller.layout.reference(), controller.input_values)) {
          return static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_INVALID_INPUT);
        }
        auto [values, code] = plugin.compute(signal_router_.read(controller.state_route),
          controller.input_values, controller.enabled_axes, controller.input_valid,
          signal_router_.read_from(controller.output_route, realized_values()), feedback_valid,
          time_ns, dt_ns);
        if (!control_success(code)) {
          return code;
        }
        if (!signal_router_.write(controller.output_route, values)) {
          return static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_INVALID_INPUT);
        }
        return code;
      },
      controller.instance_id);

    controller.status = result.second.value_or(rpp_interfaces::contract::CODE_NOT_READY);
    if (!control_success(controller.status)) {
      clear_controller_output(controller);
    } else {
      controller.commands_active = true;
    }
    status = merge_status(status, controller.status);
  }
  return status;
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::commandMap(std::uint64_t dt_ns)
{
  if (!configured_ || !state_source_active()) {
    return rpp_interfaces::contract::CODE_NOT_READY;
  }
  if (dt_ns == 0 || !state_layout().valid_row(state())) {
    clear_outputs();
    return rpp_interfaces::contract::CODE_INVALID_INPUT;
  }

  return command_map_unchecked(dt_ns);
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::command_map_unchecked(std::uint64_t dt_ns)
{
  const auto mapping_status = compute_actuator_commands(dt_ns);
  bool limited = false;
  if (!apply_actuator_limits(dt_ns, limited) || !actuator_layout().valid_row(actuator_values())) {
    clear_outputs();
    return rpp_interfaces::contract::CODE_INFEASIBLE;
  }

  // Feedback reflects the limited actuator command that will actually be applied.
  const auto feedback_status = compute_realized_commands();
  for (std::size_t index = 0; index < actuator_layout().size(); ++index) {
    last_actuator_values_[index] = actuator_values()[index];
  }
  return merge_status(merge_status(mapping_status, feedback_status),
    limited ? rpp_interfaces::contract::CODE_LIMITED : rpp_interfaces::contract::CODE_OK);
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::reset()
{
  clear_outputs();
  reset_controllers();
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::clear_outputs()
{
  control_computed_ = false;
  zero(commands());
  zero(realized_values());
  std::fill(feedback_valid_.begin(), feedback_valid_.end(), false);
  for (auto & controller : controllers_) {
    controller.commands_active = false;
  }

  // Reset slew history to the safe command as well as clearing the published output.
  for (auto * values : {&actuator_values(), &last_actuator_values_}) {
    for (std::size_t index = 0; index < values->size(); ++index) {
      (*values)[index] = index < safe_actuator_values_.size() ? safe_actuator_values_[index] : 0.0;
    }
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::reset_controllers()
{
  for (const auto & controller : controllers_) {
    (void)context_.template with_active_component<rpp_interfaces::ControlLaw>(
      controller_slot_, controller.name, [](auto & plugin) { plugin.reset(); },
      controller.instance_id);
  }
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::compute_actuator_commands(std::uint64_t dt_ns)
{
  auto status = static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_OK);
  // Direct commandMap callers may fill commands() without running sample() or compute().
  const bool manual_control = !goals_sampled_ && !control_computed_;
  std::fill(active_actuators_.begin(), active_actuators_.end(), false);
  std::fill(feedback_valid_.begin(), feedback_valid_.end(), false);
  zero(realized_values());
  for (std::size_t index = 0; index < actuator_layout().size(); ++index) {
    actuator_values()[index] = safe_actuator_values_[index];
  }

  for (auto & allocator : allocators_) {
    allocator.commands_active = false;
    allocator.status = rpp_interfaces::contract::CODE_NOT_READY;
    if (!allocator.routed) {
      status = merge_status(status, allocator.status);
      continue;
    }

    allocator.commands_active = true;
    allocator.status = rpp_interfaces::contract::CODE_OK;
    for (const auto source : allocator.command_sources) {
      if (manual_control) {
        const auto & controller = controllers_[source];
        const bool active =
          component_active<rpp_interfaces::ControlLaw>(controller_slot_, controller);
        allocator.commands_active = allocator.commands_active && active;
        if (!active) {
          allocator.status = rpp_interfaces::contract::CODE_NOT_READY;
        }
        continue;
      }
      allocator.status = merge_status(allocator.status, controllers_[source].status);
      allocator.commands_active = allocator.commands_active && controllers_[source].commands_active;
    }

    if (!allocator.commands_active) {
      status = merge_status(status, allocator.status);
      continue;
    }

    const auto result = context_.template with_active_component<rpp_interfaces::ActuationMapper>(
      allocator_slot_, allocator.name,
      [&](auto & plugin) {
        const auto input = signal_router_.read(allocator.input_route);
        if (!allocator.inputs.valid_row(input)) {
          return static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_INVALID_INPUT);
        }
        auto [values, estimate, code] =
          plugin.commandMap(signal_router_.read(allocator.state_route), input, dt_ns);
        if (!control_success(code)) {
          return code;
        }
        if (estimate.size() != allocator.inputs.size() || !detail::all_finite(estimate) ||
          !signal_router_.write(allocator.output_route, values)) {
          return static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_INVALID_INPUT);
        }
        return code;
      },
      allocator.instance_id);
    allocator.status = result.second.value_or(rpp_interfaces::contract::CODE_NOT_READY);
    allocator.commands_active = control_success(allocator.status);
    if (allocator.commands_active) {
      for (const auto index : signal_router_.indices(allocator.output_route)) {
        active_actuators_[index] = true;
      }
    }
    status = merge_status(status, allocator.status);
  }
  return status;
}

template<bool Concurrent>
bool BasicControlScript<Concurrent>::apply_actuator_limits(std::uint64_t dt_ns, bool & limited)
{
  const auto dt_seconds = static_cast<double>(dt_ns) * 1e-9;
  for (std::size_t index = 0; index < actuator_layout().size(); ++index) {
    // Expired or missing references bypass normal slew limits, like clear_outputs().
    if (!active_actuators_[index]) {
      continue;
    }
    const double previous_command = last_actuator_values_[index];
    const double max_change = actuator_rate_limits_[index] * dt_seconds;

    // Intersect the allocator, application, and per-tick rate limits.
    const double lower_bound = std::max({allocator_lower_bounds_[index],
      previous_command - max_change, application_lower_bounds_[index]});
    const double upper_bound = std::min({allocator_upper_bounds_[index],
      previous_command + max_change, application_upper_bounds_[index]});
    if (lower_bound > upper_bound) {
      return false;
    }
    const double requested_command = actuator_values()[index];
    const double limited_command = std::clamp(requested_command, lower_bound, upper_bound);
    limited = limited || limited_command != requested_command;
    actuator_values()[index] = limited_command;
  }
  return true;
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::compute_realized_commands()
{
  auto status = static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_OK);
  for (auto & allocator : allocators_) {
    if (!allocator.commands_active) {
      continue;
    }
    const auto result = context_.template with_active_component<rpp_interfaces::ActuationMapper>(
      allocator_slot_, allocator.name,
      [&](auto & plugin) {
        auto [values, code] = plugin.feedbackMap(signal_router_.route_values(allocator.state_route),
          signal_router_.read(allocator.output_route));
        if (!control_success(code)) {
          return code;
        }
        if (!signal_router_.write(allocator.feedback_route, values)) {
          return static_cast<std::uint8_t>(rpp_interfaces::contract::CODE_INVALID_INPUT);
        }
        return code;
      },
      allocator.instance_id);
    allocator.status = result.second.value_or(rpp_interfaces::contract::CODE_NOT_READY);
    if (control_success(allocator.status)) {
      for (const auto index : signal_router_.indices(allocator.feedback_route)) {
        feedback_valid_[index] = true;
      }
    } else {
      allocator.commands_active = false;
      // A feedback failure makes only this allocator's channels fall back to safe output.
      for (const auto index : signal_router_.indices(allocator.output_route)) {
        actuator_values()[index] = safe_actuator_values_[index];
        active_actuators_[index] = false;
      }
    }
    status = merge_status(status, allocator.status);
  }
  return status;
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::clear_controller_output(ControllerBinding & controller)
{
  // Reset history once when an output becomes unavailable, rather than on every failed tick.
  if (controller.commands_active) {
    (void)context_.template with_active_component<rpp_interfaces::ControlLaw>(
      controller_slot_, controller.name, [](auto & plugin) { plugin.reset(); },
      controller.instance_id);
  }
  if (controller.output_route != missing_source) {
    for (const auto index : signal_router_.indices(controller.output_route)) {
      commands()[index] = 0.0;
      feedback_valid_[index] = false;
    }
  }
  controller.commands_active = false;
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::merge_status(std::uint8_t previous, std::uint8_t next)
{
  // Preserve the first failure; successful limited output still propagates CODE_LIMITED.
  if (!control_success(previous)) {
    return previous;
  }
  if (!control_success(next)) {
    return next;
  }
  return previous == rpp_interfaces::contract::CODE_LIMITED ||
      next == rpp_interfaces::contract::CODE_LIMITED
    ? rpp_interfaces::contract::CODE_LIMITED
    : rpp_interfaces::contract::CODE_OK;
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::zero(rpp::List<double> & values)
{
  for (std::size_t index = 0; index < values.size(); ++index) {
    values[index] = 0.0;
  }
}
} // namespace rpp_control
