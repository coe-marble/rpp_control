#pragma once

// Read plugin contracts and prepare field, producer, and horizon routes. Run with ticks paused.
namespace rpp_control
{
template<bool Concurrent>
void BasicControlScript<Concurrent>::build_routes(const ComponentStatuses & components)
{
  ++route_revision_;
  const auto previous_actuators = actuator_layout();
  const auto previous_state = state_layout();
  goals_sampled_ = false;
  control_computed_ = false;
  controllers_.clear();
  allocators_.clear();
  reference_generators_.clear();
  rejected_bindings_.clear();

  // Preserve known state and actuator fields while their producers are unavailable.
  signal_router_.reset();
  signal_router_.set_layout(actuator_signal_, previous_actuators);
  signal_router_.set_layout(state_signal_, previous_state);
  state_provider_name_.clear();
  state_provider_.reset();
  state_provider_instance_.clear();

  // An inactive provider still describes the shared state, but cannot supply a valid tick.
  auto providers = components.at(state_slot_);
  if (providers.empty()) {
    providers = runtime_.get_subcomponent_statuses(state_slot_);
  }
  std::stable_sort(providers.begin(), providers.end(), [](const auto & left, const auto & right) {
    return left.state == rpp::LifecycleState::ACTIVE && right.state != rpp::LifecycleState::ACTIVE;
  });
  for (const auto & provider : providers) {
    if (provider.state != rpp::LifecycleState::ACTIVE &&
      provider.state != rpp::LifecycleState::INACTIVE) {
      continue;
    }
    bind_component(state_slot_, provider.name, [&] {
      const auto plugin =
        component_instance<rpp_interfaces::StateProvider>(state_slot_, provider.name);
      rpp_interfaces::FieldLayout layout(plugin->getDesc().as_struct().fields);
      if (layout.size() == 0) {
        throw std::invalid_argument("The shared state is empty");
      }
      signal_router_.set_layout(state_signal_, std::move(layout));
      state_provider_name_ = provider.name;
      state_provider_instance_ = provider.instance_id;
      state_provider_ = plugin;
    });
    if (!state_provider_name_.empty()) {
      break;
    }
  }

  collect_controller_layouts(components);
  collect_allocators(components);

  rpp_interfaces::FieldLayout controls;
  rpp_interfaces::FieldLayout references;
  for (auto & controller : controllers_) {
    bind_component(controller_slot_, controller.name, [&] {
      const auto prepared_controls =
        rpp_interfaces::merge_fields(controls, controller.layout.control());
      const auto prepared_references =
        rpp_interfaces::merge_fields(references, controller.layout.reference());
      controls = prepared_controls;
      references = prepared_references;
    });
    if (binding_rejected(controller_slot_, controller.name)) {
      controller.routed = false;
    }
  }

  // Include allocator inputs so a missing controller keeps its known control channels.
  for (auto & allocator : allocators_) {
    bind_component(allocator_slot_, allocator.name,
      [&] { controls = rpp_interfaces::merge_fields(controls, allocator.inputs); });
    if (binding_rejected(allocator_slot_, allocator.name)) {
      allocator.routed = false;
    }
  }
  signal_router_.set_layout(command_signal_, controls);
  signal_router_.set_layout(realized_signal_, controls);
  signal_router_.set_layout(reference_signal_, references);

  // Control outputs have exclusive writers. Shared reference inputs are handled separately.
  std::map<std::string, std::size_t> control_writers;
  for (std::size_t index = 0; index < controllers_.size(); ++index) {
    auto & controller = controllers_[index];
    if (!controller.routed) {
      continue;
    }
    bind_component(controller_slot_, controller.name, [&] {
      for (const auto & field : controller.layout.control().fields()) {
        const auto previous = control_writers.find(field.name);
        if (previous != control_writers.end()) {
          auto & peer = controllers_[previous->second];
          peer.routed = false;
          throw std::invalid_argument("Control field '" + field.name +
            "' has conflicting writers '" + peer.name + "' and '" + controller.name + "'");
        }
      }
      signal_router_.add_source(command_signal_, controller.name, controller.layout.control());
      for (const auto & field : controller.layout.control().fields()) {
        control_writers.emplace(field.name, index);
      }
      connect_state_inputs(controller);
    });
    if (binding_rejected(controller_slot_, controller.name)) {
      controller.routed = false;
    }
  }

  connect_reference_generators(components);
  prepare_controller_references();

  // Each control field has one allocator, and each actuator field has one writer.
  std::map<std::string, std::size_t> allocator_by_control;
  std::map<std::string, std::size_t> actuator_writers;
  for (std::size_t index = 0; index < allocators_.size(); ++index) {
    auto & allocator = allocators_[index];
    if (!allocator.routed) {
      continue;
    }
    bind_component(allocator_slot_, allocator.name, [&] {
      for (const auto & field : allocator.inputs.fields()) {
        const auto previous = allocator_by_control.find(field.name);
        if (previous != allocator_by_control.end()) {
          allocators_[previous->second].routed = false;
          throw std::invalid_argument("Control field '" + field.name + "' has multiple allocators");
        }
      }
      for (const auto & field : allocator.outputs.fields()) {
        const auto previous = actuator_writers.find(field.name);
        if (previous != actuator_writers.end()) {
          allocators_[previous->second].routed = false;
          throw std::invalid_argument(
            "Actuator field '" + field.name + "' has multiple allocators");
        }
      }
      signal_router_.add_source(actuator_signal_, allocator.name, allocator.outputs);
      for (const auto & field : allocator.inputs.fields()) {
        allocator_by_control.emplace(field.name, index);
      }
      for (const auto & field : allocator.outputs.fields()) {
        actuator_writers.emplace(field.name, index);
      }
    });
    if (binding_rejected(allocator_slot_, allocator.name)) {
      allocator.routed = false;
    }
  }

  // Missing dependencies invalidate numeric routes without deactivating their consumers.
  for (auto & controller : controllers_) {
    if (binding_rejected(controller_slot_, controller.name)) {
      continue;
    }
    controller.output_route =
      signal_router_.connect_output(command_signal_, controller.layout.control());
    controller.current_reference_route =
      signal_router_.connect_input(reference_signal_, controller.layout.reference());
    controller.enabled_axes.resize(controller.layout.control().size());
    for (const auto & field : controller.layout.control().fields()) {
      const auto destination = allocator_by_control.find(field.name);
      if (destination == allocator_by_control.end() || !allocators_[destination->second].routed) {
        controller.routed = false;
      }
    }
  }

  for (auto & allocator : allocators_) {
    if (binding_rejected(allocator_slot_, allocator.name)) {
      continue;
    }
    allocator.input_route = signal_router_.connect_input(command_signal_, allocator.inputs);
    allocator.output_route = signal_router_.connect_output(actuator_signal_, allocator.outputs);
    allocator.feedback_route = signal_router_.connect_output(realized_signal_, allocator.inputs);
    connect_state_inputs(allocator);
    for (const auto & field : allocator.inputs.fields()) {
      const auto source = control_writers.find(field.name);
      if (source == control_writers.end()) {
        allocator.routed = false;
        continue;
      }
      const auto controller = source->second;
      if (std::find(allocator.command_sources.begin(), allocator.command_sources.end(),
            controller) == allocator.command_sources.end()) {
        allocator.command_sources.push_back(controller);
      }
    }
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::collect_controller_layouts(
  const ComponentStatuses & components)
{
  for (const auto & component : components.at(controller_slot_)) {
    bind_component(controller_slot_, component.name, [&] {
      const auto plugin =
        component_instance<rpp_interfaces::ControlLaw>(controller_slot_, component.name);
      ControllerBinding binding;
      binding.name = component.name;
      binding.instance_id = component.instance_id;
      binding.layout = rpp_interfaces::ControlLawLayout(plugin->getDesc().as_struct());
      binding.state = binding.layout.state();
      binding.input_values.resize(binding.layout.reference_size());
      binding.input_valid.resize(binding.layout.reference_size());
      binding.reference_route.resize(binding.layout.reference_size());
      controllers_.push_back(std::move(binding));
    });
  }

  if (preview_only_) {
    return;
  }

  // Overlapping reference consumers are valid; warn that generator updates affect both.
  for (std::size_t left = 0; left < controllers_.size(); ++left) {
    for (std::size_t right = left + 1; right < controllers_.size(); ++right) {
      for (const auto & field : controllers_[left].layout.reference().fields()) {
        if (!rpp_interfaces::find_field(controllers_[right].layout.reference(), field.name)) {
          continue;
        }
        const auto message = "Controllers '" + controllers_[left].name + "' and '" +
          controllers_[right].name + "' share reference field '" + field.name +
          "'. Generator goal updates affect both consumers.";
        if (report_warning_) {
          report_warning_(message);
        } else {
          RPP_LOG_WARN(*context_.get_logger(), message);
        }
      }
    }
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::collect_allocators(const ComponentStatuses & components)
{
  for (const auto & component : components.at(allocator_slot_)) {
    bind_component(allocator_slot_, component.name, [&] {
      auto description =
        component_instance<rpp_interfaces::ActuationMapper>(allocator_slot_, component.name)
          ->getDesc()
          .as_struct();
      AllocatorBinding binding;
      binding.name = component.name;
      binding.instance_id = component.instance_id;
      binding.state = rpp_interfaces::FieldLayout(description.state.fields);
      binding.inputs = rpp_interfaces::FieldLayout(description.control.fields);
      binding.outputs = rpp_interfaces::FieldLayout(description.actuators.fields);
      if (binding.outputs.size() == 0) {
        throw std::invalid_argument("An allocator needs actuator outputs");
      }
      const auto infinity = std::numeric_limits<double>::infinity();
      std::vector<double> lower(binding.outputs.size(), -infinity);
      std::vector<double> upper(binding.outputs.size(), infinity);
      std::vector<double> rates(binding.outputs.size(), infinity);
      for (const auto & limit : description.limits) {
        add_limit(limit, binding.outputs, lower, upper, rates);
      }
      binding.limits = std::move(description.limits);
      allocators_.push_back(std::move(binding));
    });
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::connect_state_inputs(ComponentRoutes & component)
{
  if (!rpp_interfaces::validate_required_fields(state_layout(), component.state)) {
    component.routed = false;
    return;
  }
  component.state_route = signal_router_.connect_input(state_signal_, component.state);
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::connect_reference_generators(
  const ComponentStatuses & components)
{
  std::set<std::string> live_instances;
  // Keep configured generator contracts visible; only selected instances publish references.
  for (const auto & component : runtime_.get_subcomponent_statuses(reference_slot_)) {
    if (component.state != rpp::LifecycleState::INACTIVE &&
      component.state != rpp::LifecycleState::ACTIVE) {
      continue;
    }
    bind_component(reference_slot_, component.name, [&] {
      ReferenceGeneratorBinding binding;
      binding.name = component.name;
      binding.instance_id = component.instance_id;
      binding.plugin =
        component_instance<rpp_interfaces::ReferenceGenerator>(reference_slot_, component.name);
      auto description = binding.plugin->getDesc().as_struct();
      binding.state = rpp_interfaces::FieldLayout(description.state.fields);
      binding.outputs = rpp_interfaces::FieldLayout(description.reference.fields);
      binding.features = description.features;

      if (binding.outputs.size() == 0 ||
        description.targetMask.size() != binding.outputs.fields().size()) {
        throw std::invalid_argument("Invalid reference generator state, outputs, or target mask");
      }

      std::vector<rpp_interfaces::Field> targets;
      for (std::size_t field = 0; field < description.targetMask.size(); ++field) {
        if (description.targetMask[field]) {
          targets.push_back(binding.outputs.fields()[field]);
        }
      }
      binding.targets = rpp_interfaces::FieldLayout(std::move(targets));
      binding.target_reference_route = rpp_interfaces::field_route(binding.outputs, binding.targets);

      // Unmatched derived outputs use the earliest target deadline during sampling.
      binding.output_target_route.assign(binding.outputs.size(), missing_source);
      for (std::size_t field = 0; field < binding.outputs.fields().size(); ++field) {
        const auto target =
          rpp_interfaces::find_field(binding.targets, binding.outputs.fields()[field].name);
        if (!target ||
          !rpp_interfaces::same_field(binding.targets, *target, binding.outputs, field)) {
          continue;
        }
        const auto & input = binding.targets.ranges()[*target];
        const auto & output = binding.outputs.ranges()[field];
        for (std::size_t scalar = 0; scalar < output.size; ++scalar) {
          binding.output_target_route[output.offset + scalar] = input.offset + scalar;
        }
      }

      binding.deadlines.assign(binding.targets.size(), 0);
      binding.trajectory_timing.resize(binding.targets.size());
      binding.selected =
        std::any_of(components.at(reference_slot_).begin(), components.at(reference_slot_).end(),
          [&](const auto & selected) { return selected.instance_id == component.instance_id; });

      // A generator samples the union of times requested for the fields it produces.
      // Each controller receives only its own requested cells in reference-major order.
      std::set<std::uint64_t> times{0};
      for (const auto & controller : controllers_) {
        const auto & reference = controller.layout.reference();
        const auto & horizon = controller.layout.horizon();
        for (std::size_t field = 0; field < binding.outputs.fields().size(); ++field) {
          const auto target =
            rpp_interfaces::find_field(reference, binding.outputs.fields()[field].name);
          if (!target || !rpp_interfaces::same_field(binding.outputs, field, reference, *target)) {
            continue;
          }
          const auto & range = reference.ranges()[*target];
          for (std::size_t scalar = 0; scalar < range.size; ++scalar) {
            for (std::size_t sample = 0; sample < horizon.sample_count(); ++sample) {
              times.insert(horizon.offset(range.offset + scalar, sample));
            }
          }
        }
      }

      std::uint32_t length = 0;
      if (times.size() > 1) {
        if (times.size() > rpp_interfaces::ReferenceHorizon::max_samples) {
          throw std::invalid_argument("Combined reference horizon exceeds capacity");
        }
        binding.offsets.assign(times.begin(), times.end());
        length = static_cast<std::uint32_t>(times.size());
      }

      binding.horizon = rpp_interfaces::ReferenceHorizon(binding.outputs, binding.offsets, length);
      binding.values.resize(binding.horizon.value_count());
      binding.valid.resize(binding.horizon.value_count());
      binding.current_values.resize(binding.outputs.size());
      connect_state_inputs(binding);
      live_instances.insert(binding.instance_id);
      reference_generators_.push_back(std::move(binding));
    });
  }

  for (auto entry = bound_generator_horizons_.begin(); entry != bound_generator_horizons_.end();) {
    if (!live_instances.count(entry->first)) {
      entry = bound_generator_horizons_.erase(entry);
    } else {
      ++entry;
    }
  }

  // Many controllers may read a field, but only one selected generator may produce it.
  std::map<std::string, std::size_t> writers;
  for (std::size_t index = 0; index < reference_generators_.size(); ++index) {
    auto & generator = reference_generators_[index];
    if (!generator.selected) {
      continue;
    }
    bind_component(reference_slot_, generator.name, [&] {
      for (const auto & field : generator.outputs.fields()) {
        const auto previous = writers.find(field.name);
        if (previous != writers.end()) {
          reference_generators_[previous->second].routed = false;
          throw std::invalid_argument(
            "Reference field '" + field.name + "' has multiple generators");
        }
      }
      signal_router_.add_source(reference_signal_, generator.name, generator.outputs);
      for (const auto & field : generator.outputs.fields()) {
        writers.emplace(field.name, index);
      }
    });
    if (binding_rejected(reference_slot_, generator.name)) {
      generator.routed = false;
    }
  }

  for (auto & generator : reference_generators_) {
    if (generator.selected && !binding_rejected(reference_slot_, generator.name)) {
      generator.output_route = signal_router_.connect_output(reference_signal_, generator.outputs);
    }
  }
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::prepare_controller_references()
{
  std::map<std::string, std::size_t> generators;
  for (std::size_t index = 0; index < reference_generators_.size(); ++index) {
    const auto & generator = reference_generators_[index];
    if (!generator.selected || binding_rejected(reference_slot_, generator.name)) {
      continue;
    }
    for (const auto & field : generator.outputs.fields()) {
      generators.emplace(field.name, index);
    }
  }

  // Resolve field names and sample times once. compute() only copies the prepared cells.
  for (auto & controller : controllers_) {
    const auto & reference = controller.layout.reference();
    const auto & horizon = controller.layout.horizon();
    for (std::size_t field = 0; field < reference.fields().size(); ++field) {
      const auto source = generators.find(reference.fields()[field].name);
      if (source == generators.end()) {
        controller.routed = false;
        continue;
      }

      const auto & generator = reference_generators_[source->second];
      const auto produced_field =
        rpp_interfaces::find_field(generator.outputs, reference.fields()[field].name);
      if (!produced_field ||
        !rpp_interfaces::same_field(generator.outputs, *produced_field, reference, field)) {
        controller.routed = false;
        continue;
      }
      const auto & input = reference.ranges()[field];
      const auto & output = generator.outputs.ranges()[*produced_field];
      for (std::size_t scalar = 0; scalar < input.size; ++scalar) {
        for (std::size_t sample = 0; sample < horizon.sample_count(); ++sample) {
          const auto offset = horizon.offset(input.offset + scalar, sample);
          std::size_t output_sample = 0;
          if (!generator.offsets.empty()) {
            const auto found =
              std::lower_bound(generator.offsets.begin(), generator.offsets.end(), offset);
            if (found == generator.offsets.end() || *found != offset) {
              throw std::invalid_argument("Generator horizon is missing a requested offset");
            }
            output_sample = static_cast<std::size_t>(found - generator.offsets.begin());
          }
          const auto destination = (input.offset + scalar) * horizon.sample_count() + sample;
          controller.reference_route[destination] = {source->second,
            (output.offset + scalar) * generator.horizon.sample_count() + output_sample};
        }
      }
    }
  }
}
} // namespace rpp_control
