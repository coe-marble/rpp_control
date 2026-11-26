#include <rpp_control/ros/control_script_node.hpp>

namespace rpp_control
{
namespace helpers = node_helpers;
namespace timing = time_helpers;

ControlScriptNode::ControlScriptNode(HandoverStateFactory make_handover_state)
  : Node("control_script"),
    workspace_(helpers::declare_startup_parameter<std::string>(
      *this, "rpp_workspace", "", "RPP workspace directory.")),
    script_library_(helpers::declare_startup_parameter<std::string>(
      *this, "script_library", "rpp_control", "Registered RPP script library.")),
    script_name_(helpers::declare_startup_parameter<std::string>(
      *this, "script_name", "control_script", "Registered RPP script name.")),
    is_dev_(helpers::declare_startup_parameter<bool>(*this, "is_dev", false,
      "Development mode: ignore reference tokens and goal stamps; enable cmd_* topics.")),
    tick_interval_(
      timing::tick_interval_from_frequency(helpers::declare_startup_parameter<double>(
        *this, "control_frequency_hz", 50.0, "Positive control loop frequency in hertz."))),
    state_timeout_(helpers::declare_duration(*this, "state_timeout_sec", 1.0)),
    max_control_dt_(helpers::declare_duration(
      *this, "max_control_dt_sec", 2.0 * timing::seconds(tick_interval_))),
    safe_actuator_values_(helpers::declare_startup_parameter<std::vector<double>>(
      *this, "safe_actuator_values", {}, "Stop command in the actuator layout's field order.")),
    make_handover_state_(std::move(make_handover_state))
{
  if (script_library_.empty() || script_name_.empty()) {
    throw std::invalid_argument("script_library and script_name must be set");
  }
  timing::validate_control_periods(tick_interval_, max_control_dt_);

  state_subscription_ = create_subscription<Sample>("~/state", rclcpp::QoS(10).best_effort(),
    [this](Sample::ConstSharedPtr sample) { receive_sample(std::move(sample)); });
  goal_subscription_ = create_subscription<Goal>(
    "~/goal", rclcpp::QoS(10), [this](Goal::ConstSharedPtr goal) { receive_goal(*goal); });

  layout_publisher_ =
    create_publisher<ControlLayout>("~/layout", rclcpp::QoS(1).transient_local());
  command_publisher_ = create_publisher<Sample>("~/command", rclcpp::QoS(10));
  status_publisher_ = create_publisher<Status>("~/status", rclcpp::QoS(1));
  mission_monitor_publisher_ = create_publisher<MonitorStatuses>(
    "~/mission_monitors", rclcpp::QoS(1).durability_volatile());

  enable_service_ = create_service<SetBool>("~/enable",
    [this](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
      std::lock_guard<std::mutex> lock(mailbox_mutex_);
      if (request->data && (slot_operation_pending_ || !routes_ready_ || stop_requested_)) {
        response->success = false;
        response->message = "Control routes are unavailable. See ~/status and slot status.";
        return;
      }

      inputs_.enabled = request->data;
      if (!inputs_.enabled) {
        clear_goal();
      }

      response->success = true;
      response->message = "Transition requested. See ~/status for the worker state.";
    });

  request_reference_service_ = create_service<RequestReference>("request_external_reference",
    [this](RequestReference::Request::SharedPtr request, RequestReference::Response::SharedPtr response) {
      request_reference(*request, *response);
    });
  release_reference_service_ = create_service<ReleaseReference>("release_external_reference",
    [this](ReleaseReference::Request::SharedPtr request, ReleaseReference::Response::SharedPtr response) {
      release_reference(*request, *response);
    });

  const bool expose_navigation_interface = helpers::declare_startup_parameter<bool>(
    *this, "expose_navigation_topics", false,
    "Expose masked navigation references and FollowPath; cmd_vel requires is_dev or enable_nav2_cmd_vel.");
  publish_timer_ = create_wall_timer(tick_interval_, [this] { publish_latest_output(); });
  start_control(expose_navigation_interface);
}

ControlScriptNode::~ControlScriptNode()
{
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    stop_requested_ = true;
  }
  worker_wake_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void ControlScriptNode::receive_sample(Sample::ConstSharedPtr sample)
{
  const auto current_time = timing::timestamp_ns(now());
  std::shared_ptr<const OutputLayout> layout;
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    layout = layout_;
  }
  if (!layout || !current_time || !timing::valid_sample_stamp(*sample, *current_time)) {
    return;
  }
  const auto route = std::find_if(layout->state_routes.begin(), layout->state_routes.end(),
    [&](const auto & item) { return item.name == sample->interface_name; });
  if (route == layout->state_routes.end() || sample->values.size() != route->fields.size() ||
    !route->fields.valid_row(sample->values)) {
    return;
  }

  // Serialize the provider write with numeric reads and lifecycle operations.
  std::lock_guard<std::mutex> tick_lock(control_tick_mutex_);
  std::optional<std::uint64_t> state_time_ns;
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    if (layout != layout_ || !routes_ready_ || slot_operation_pending_ || stop_requested_) {
      return;
    }
    auto & latest = received_states_[sample->interface_name];
    // Repeating a measurement cannot refresh that resource's watchdog.
    if (latest && timing::stamp_ns(*sample) <= timing::stamp_ns(*latest)) {
      return;
    }
    latest = std::move(sample);
    inputs_.state_time_ns.reset();
    state_time_ns = assemble_state(*layout);
  }
  if (!state_time_ns) {return;}
  try {
    const auto status = script_->set_state(incoming_state_, *state_time_ns);
    if (!control_success(status)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "State provider rejected the sample (code %u).", static_cast<unsigned>(status));
      return;
    }
    // Only an accepted provider write makes the assembled timestamp available to ticks.
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    inputs_.state_time_ns = state_time_ns;
  } catch (const std::exception & error) {
    mark_worker_failed();
    RCLCPP_ERROR(get_logger(), "State provider write threw; control worker stopped: %s", error.what());
  } catch (...) {
    mark_worker_failed();
    RCLCPP_ERROR(get_logger(), "State provider write threw; control worker stopped.");
  }
}

std::optional<std::uint64_t> ControlScriptNode::assemble_state(const OutputLayout & layout)
{
  std::optional<std::uint64_t> oldest_time_ns;
  for (const auto & route : layout.state_routes) {
    const auto source = received_states_.find(route.name);
    if (source == received_states_.end() || !source->second) {
      return std::nullopt;
    }
    const auto & measurement = source->second;
    for (std::size_t axis = 0; axis < route.indices.size(); ++axis) {
      incoming_state_[route.indices[axis]] = measurement->values[axis];
    }
    const auto time_ns = timing::stamp_ns(*measurement);
    oldest_time_ns = oldest_time_ns ? std::min(*oldest_time_ns, time_ns) : time_ns;
  }
  return oldest_time_ns;
}

void ControlScriptNode::apply_pending_reset(
  ControlScript & script, WorkerBindings & bindings, std::uint64_t revision)
{
  if (bindings.reset_revision == revision) {
    return;
  }
  script.reset();
  script.cancel();
  goal_stamps_.clear();
  bindings.reset_revision = revision;
  bindings.running = false;
}

bool ControlScriptNode::receive_goal(const Goal & goal, bool token_authorized)
{
  // The ROS navigation adapter may authorize cmd_vel without an identity token.
  // Authorized inputs use the same goal setter and validation as token holders.
  if (!is_dev_ && !token_authorized &&
    !reference_claims_.authorizes(goal.interface_name, goal.identity_token)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Rejected goal for '%s': a matching reference token is required.", goal.interface_name.c_str());
    return false;
  }
  // Preparing descriptions and plugin goals may allocate. Keep that work on
  // the executor and pause numeric ticks while applying the update.
  std::lock_guard<std::mutex> tick_lock(control_tick_mutex_);
  const auto time = timing::timestamp_ns(now());
  std::shared_ptr<const OutputLayout> published;
  std::uint64_t revision;
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    if (!routes_ready_ || slot_operation_pending_ || stop_requested_) {
      return false;
    }
    published = layout_;
    revision = inputs_.reset_revision;
  }
  if (!time || (!is_dev_ && !timing::valid_stamp(goal.stamp)) ||
    timing::deadline_expired(goal.deadline_ns, *time)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Rejected goal with an invalid or expired timestamp.");
    return false;
  }
  const auto submitted = is_dev_ ? *time : timing::stamp_ns(goal.stamp);
  const auto interface =
    std::find_if(published->description.goals.begin(), published->description.goals.end(),
      [&](const auto & item) { return item.name == goal.interface_name; });
  if ((!is_dev_ && submitted > *time) || interface == published->description.goals.end() ||
    goal.values.size() != interface->layout.value_count ||
    (!goal.update_mask.empty() && goal.update_mask.size() != goal.values.size())) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Rejected goal for unknown or incompatible interface '%s'.",
      goal.interface_name.c_str());
    return false;
  }
  try {
    apply_pending_reset(*script_, worker_bindings_, revision);
    std::vector<std::uint64_t> * stamps = nullptr;
    if (!is_dev_) {
      auto & previous = goal_stamps_[script_->goal_instance_id(goal.interface_name)];
      if (previous.size() != goal.values.size()) {
        previous.assign(goal.values.size(), 0);
      }
      for (std::size_t axis = 0; axis < previous.size(); ++axis) {
        if ((goal.update_mask.empty() || goal.update_mask[axis]) && submitted < previous[axis]) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Rejected stale goal update for interface '%s'.",
            goal.interface_name.c_str());
          return false;
        }
      }
      stamps = &previous;
    }
    rpp::List<double> values;
    values.resize(goal.values.size());
    const auto & route = published->goal_routes.at(goal.interface_name);
    for (std::size_t axis = 0; axis < goal.values.size(); ++axis) {
      values[route[axis]] = goal.values[axis];
    }
    rpp::List<bool> mask;
    mask.resize(goal.update_mask.size());
    for (std::size_t axis = 0; axis < goal.update_mask.size(); ++axis) {
      mask[route[axis]] = goal.update_mask[axis];
    }
    const auto status = script_->set_point(goal.interface_name, values, mask, goal.deadline_ns);
    if (!control_success(status)) {
      return false;
    }
    if (stamps) {
      for (std::size_t axis = 0; axis < stamps->size(); ++axis) {
        if (goal.update_mask.empty() || goal.update_mask[axis]) {
          (*stamps)[axis] = submitted;
        }
      }
    }
    RCLCPP_DEBUG(get_logger(), "Updated goal for interface '%s'.", goal.interface_name.c_str());
    return true;
  } catch (const std::exception & error) {
    mark_worker_failed();
    RCLCPP_ERROR(get_logger(),
      "Goal submission threw for interface '%s'; control worker stopped: %s",
      goal.interface_name.c_str(), error.what());
  } catch (...) {
    mark_worker_failed();
    RCLCPP_ERROR(get_logger(),
      "Goal submission threw for interface '%s'; control worker stopped.",
      goal.interface_name.c_str());
  }
  return false;
}

void ControlScriptNode::clear_goal()
{
  ++inputs_.reset_revision;
}

std::optional<ControlScriptNode::Inputs> ControlScriptNode::read_tick_inputs(bool clock_moved_backwards)
{
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  if (stop_requested_) {
    return std::nullopt;
  }
  if (clock_moved_backwards) {
    inputs_.state_time_ns.reset();
    received_states_.clear();
    clear_goal();
  }
  return inputs_;
}

void ControlScriptNode::start_control(bool expose_navigation_interface)
{
  try {
    runtime_.emplace(build_runtime());
    script_.emplace(*runtime_);
    initialize_routes(*runtime_, *script_, worker_bindings_);
    create_slot_manager();
    // Validate the navigation contract before starting a worker thread. A failed
    // NavigationInterface constructor cannot leave a joinable thread behind.
    if (expose_navigation_interface) {navigation_interface_.emplace(*this);}

    worker_ = std::thread([this] { run_worker(); });
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Could not start control: %s", error.what());
    mark_worker_failed();
    return;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Could not start control after an unknown error.");
    mark_worker_failed();
    return;
  }

  RCLCPP_INFO(get_logger(), "Control worker started at %.3f Hz, state timeout %.3f s.",
    timing::frequency_hz(tick_interval_), timing::seconds(state_timeout_));
}

ControlScriptNode::Runtime ControlScriptNode::build_runtime()
{
  rpp::RppDataManager data_manager(rpp::RPP_HOME, workspace_);
  rpp::ConcurrentComponentRuntimeBuilder builder(data_manager, rpp::ClockOptions{}, true);
  return builder.build_script({rpp::LibraryScript{script_library_, script_name_}, {}, {}});
}

void ControlScriptNode::initialize_routes(Runtime & runtime, ControlScript & script, WorkerBindings & bindings)
{
  try {
    script.configure(
      safe_actuator_values_,
      [this](const std::string & error) { RCLCPP_ERROR(get_logger(), "%s", error.c_str()); },
      [this](const std::string & warning) { RCLCPP_WARN(get_logger(), "%s", warning.c_str()); });
    script.activate();
    accept_routes(runtime, script, bindings);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Initial control routing failed: %s", error.what());
  }
}

void ControlScriptNode::create_slot_manager()
{
  rpp_ros2::SlotManagerRosCallbacks callbacks;
  callbacks.before_operation = [this](const SlotOperation & operation) {
    begin_slot_operation(operation, *runtime_, *script_);
  };
  callbacks.after_access = [this] { (void)finish_slot_operation(); };
  callbacks.after_success = [this](const SlotOperation & operation) {
    auto refreshed = finish_slot_operation();
    if (!refreshed.success) {
      return refreshed;
    }
    if (operation.kind == SlotOperation::Kind::Activate ||
      operation.kind == SlotOperation::Kind::Switch) {
      const auto statuses = runtime_->get_subcomponent_statuses(operation.slot);
      const auto targets = operation.kind == SlotOperation::Kind::Switch
        ? operation.switch_request.activate
        : std::vector<std::string>{operation.target};
      for (const auto & target : targets) {
        if (std::none_of(statuses.begin(), statuses.end(), [&](const auto & status) {
              return status.name == target && status.state == rpp::LifecycleState::ACTIVE;
            })) {
          return rpp::CallbackResult::failure(
            "'" + target + "' became inactive during route binding. See the control log.");
        }
      }
    }
    return rpp::CallbackResult{};
  };
  callbacks.make_handover_state = [this](const std::type_info & state_type,
                                    const std::string & slot, const std::string & target) {
    if (!make_handover_state_) {
      return make_application_handover(state_type);
    }
    if (slot != "controller") {
      return make_handover_state_(state_type, nullptr);
    }
    const auto controller = runtime_->get_context()
                              .get_subcomponent_context(slot, target)
                              .get_instance<rpp_interfaces::ControlLaw>();
    const auto description = controller->getDesc();
    return make_handover_state_(state_type, &description);
  };
  slot_manager_.emplace(
    *this, *runtime_, rpp_ros2::SlotManagerRosOptions{}, std::move(callbacks));
}

rpp::HandoverState ControlScriptNode::make_application_handover(const std::type_info & state_type)
{
  using ControllerState = rpp_schema::rpp_interfaces::ControllerHandoverState;
  using AllocatorState = rpp_schema::rpp_interfaces::AllocatorHandoverState;
  if (state_type != typeid(ControllerState::Const) &&
    state_type != typeid(AllocatorState::Const)) {
    return {};
  }

  // Lifecycle callbacks hold the tick mutex, so the state and control layouts
  // stay consistent while this ROS-thread factory copies application data.
  const auto input = read_tick_inputs(false);
  const auto ros_time = timing::timestamp_ns(now());
  const bool has_measurement = input && input->state_time_ns && ros_time &&
    timing::measurement_fresh(*input->state_time_ns, *ros_time, state_timeout_) &&
    control_success(script_->read_state());
  if (has_measurement && !script_->state_layout().valid_row(script_->state())) {
    throw std::invalid_argument("Handover measurement does not match the shared state layout");
  }
  const bool include_previous_control = has_measurement && input->enabled &&
    script_->has_previous_control() && worker_bindings_.reset_revision == input->reset_revision;
  if (state_type == typeid(ControllerState::Const)) {
    return helpers::copy_application_handover<ControllerState>(
      *script_, has_measurement, include_previous_control, script_->controller_auxiliary_state());
  }
  return helpers::copy_application_handover<AllocatorState>(
    *script_, has_measurement, include_previous_control, script_->allocator_auxiliary_state());
}

void ControlScriptNode::run_worker()
{
  try {
    run_tick_loop(*script_, worker_bindings_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Control worker stopped: %s", error.what());
    mark_worker_failed();
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Control worker stopped after an unknown error.");
    mark_worker_failed();
  }
}

void ControlScriptNode::run_tick_loop(ControlScript & script, WorkerBindings & bindings)
{
  timing::TickClock tick_clock(tick_interval_, bindings.route_revision);
  auto next_tick = Clock::now();
  while (wait_for_tick(next_tick)) {
    std::lock_guard<std::mutex> tick_lock(control_tick_mutex_);
    const auto tick_start = Clock::now();
    next_tick = timing::next_tick_deadline(next_tick, tick_start, tick_interval_);

    const auto ros_time = now();
    const auto tick_timing = tick_clock.update(ros_time, bindings.route_revision);
    // Shutdown may have been requested while the worker waited for the tick mutex.
    const auto input = read_tick_inputs(tick_timing.reset_inputs);
    if (!input) {
      break;
    }
    if (bindings.ready) {
      queue_control_tick(script, bindings, *input, ros_time, tick_timing);
    }
  }
}

void ControlScriptNode::queue_control_tick(ControlScript & script, WorkerBindings & bindings, const Inputs & input,
  const rclcpp::Time & ros_time, const timing::TickTiming & tick_timing)
{
  auto & output = bindings.output;
  const auto & time_ns = tick_timing.time_ns;
  output.status = time_ns
    ? control_tick(script, bindings, input, *time_ns, tick_timing.interval)
    : helpers::make_status(Status::FAULT, Status::STAGE_NONE, rpp_interfaces::contract::CODE_INVALID_INPUT);
  const bool component_failure =
    output.status.state == Status::FAULT && output.status.failed_stage != Status::STAGE_NONE;
  if (output.status.state != Status::RUNNING && !component_failure) {
    if (bindings.running || output.status.state == Status::FAULT) {
      script.reset();
    } else {
      script.clear_outputs();
    }
  }
  bindings.running = output.status.state == Status::RUNNING;

  output.status.stamp = ros_time;
  for (std::size_t index = 0; index < output.actuator_values.size(); ++index) {
    output.actuator_values[index] = script.actuators()[index];
  }
  output.generated_at = time_ns;
  output.reset_revision = input.reset_revision;
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  std::swap(output, pending_output_);
  output_pending_ = true;
}

void ControlScriptNode::log_control_transition(const Status & previous, const Status & current)
{
  if (current.state == previous.state && current.failed_stage == previous.failed_stage &&
    current.rpp_status_code == previous.rpp_status_code) {
    return;
  }
  if (current.state == Status::RUNNING) {
    RCLCPP_INFO(get_logger(), "Control tick is running.");
  } else if (current.state == Status::FAULT || previous.state == Status::RUNNING) {
    RCLCPP_WARN(get_logger(), "Control execution changed (state %u, stage %u, code %u).",
      static_cast<unsigned>(current.state), static_cast<unsigned>(current.failed_stage),
      static_cast<unsigned>(current.rpp_status_code));
  }
}

bool ControlScriptNode::wait_for_tick(Clock::time_point deadline)
{
  std::unique_lock<std::mutex> lock(mailbox_mutex_);
  // wait_until releases the mutex while the worker sleeps so ROS callbacks can
  // access the mailbox. It reacquires the mutex before checking for shutdown.
  return !worker_wake_.wait_until(lock, deadline, [this] { return stop_requested_; });
}

void ControlScriptNode::begin_slot_operation(
  const SlotOperation & operation, Runtime & runtime, ControlScript & script)
{
  // Reload can call activation preparation again while this operation already
  // holds the tick lock. Keep the worker paused across the entire sequence.
  if (!slot_change_lock_.owns_lock()) {
    std::unique_lock<std::mutex> tick_lock(control_tick_mutex_);
    {
      std::lock_guard<std::mutex> lock(mailbox_mutex_);
      if (stop_requested_) {
        throw std::runtime_error("Control worker is unavailable");
      }
      slot_operation_pending_ = true;
    }
    slot_change_lock_ = std::move(tick_lock);
    // The executor may block inside lifecycle callbacks. Deliver the stop frame
    // before entering them, and start subsequent slew limiting from that frame.
    script.clear_outputs();
    std::shared_ptr<const OutputLayout> published;
    {
      std::lock_guard<std::mutex> lock(mailbox_mutex_);
      published = layout_;
    }
    if (published) {
      const auto stamp = now();
      for (const auto & route : published->command_routes) {
        Sample sample;
        sample.stamp = stamp;
        sample.interface_name = route.name;
        sample.values.resize(route.indices.size());
        for (std::size_t axis = 0; axis < route.indices.size(); ++axis) {
          sample.values[axis] = published->safe_command[route.indices[axis]];
        }
        command_publisher_->publish(sample);
      }
    }
  }
  if (operation.kind == SlotOperation::Kind::Switch ||
    (operation.kind == SlotOperation::Kind::Activate && operation.slot == "controller")) {
    auto proposed = operation.switch_request;
    if (operation.kind == SlotOperation::Kind::Activate) {
      proposed.activate = {operation.target};
    }
    auto routes = script.validate_switch_routes(operation.slot, proposed, safe_actuator_values_);
    if (!routes.success) {
      RCLCPP_ERROR(
        get_logger(), "Proposed control routes are invalid: %s", routes.message.c_str());
      routes.require_success("Validate control routes");
    }
  }
  // Reference generators need the active controllers' prediction horizon
  // before activation. Other slots proceed directly to the lifecycle operation.
  if (operation.slot != "reference") {
    return;
  }

  using Kind = SlotOperation::Kind;
  if (operation.kind != Kind::Activate && operation.kind != Kind::Switch) {
    return;
  }

  const auto generators = runtime.get_subcomponent_statuses("reference");
  const auto bind_target = [&](const std::string & name) {
    const auto target = std::find_if(generators.begin(), generators.end(),
      [&](const auto & generator) { return generator.name == name; });
    if (target != generators.end() && target->state == rpp::LifecycleState::INACTIVE) {
      script.bind_reference_before_activation(name).require_success();
    }
  };
  if (operation.kind == Kind::Activate) {
    bind_target(operation.target);
  } else {
    for (const auto & name : operation.switch_request.activate) {
      bind_target(name);
    }
  }
}

rpp::CallbackResult ControlScriptNode::finish_slot_operation()
{
  if (!slot_change_lock_.owns_lock()) {
    return {};
  }

  // A failed switch can still change the active components. Refresh routes
  // before releasing the worker, regardless of the service result.
  auto tick_lock = std::move(slot_change_lock_);
  return refresh_routes(*runtime_, *script_, worker_bindings_);
}

rpp::CallbackResult ControlScriptNode::refresh_routes(
  const Runtime & runtime, ControlScript & script, WorkerBindings & bindings)
{
  rpp::CallbackResult result;
  try {
    // Callers already detected an instance change or completed a slot operation.
    bindings.ready = false;
    script.rebuild_active_routes(safe_actuator_values_);
    accept_routes(runtime, script, bindings);
  } catch (const std::exception & error) {
    bindings.ready = false;
    result = rpp::CallbackResult::failure(error.what());
  } catch (...) {
    bindings.ready = false;
    result = rpp::CallbackResult::failure("Unknown control routing error");
  }
  if (!result.success) {
    RCLCPP_ERROR(get_logger(), "Control routes are unavailable: %s", result.message.c_str());
  }

  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  routes_ready_ = bindings.ready;
  slot_operation_pending_ = false;
  return result;
}

void ControlScriptNode::accept_routes(const Runtime & runtime, ControlScript & script, WorkerBindings & bindings)
{
  std::uint64_t pending_reset;
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    pending_reset = inputs_.reset_revision;
  }
  apply_pending_reset(script, bindings, pending_reset);
  bindings.active_instances = helpers::active_instance_ids(runtime);

  auto layout = std::make_shared<OutputLayout>();
  layout->measured_fields = script.state_layout();
  layout->state_routes =
    helpers::prepare_resource_interfaces(script.state_layout(), state_interface_definitions_);
  layout->command_routes = helpers::prepare_resource_interfaces(
    script.actuator_layout(), command_interface_definitions_);
  for (const auto & route : layout->state_routes) {
    rpp_ros_interfaces::msg::NumericInterface description;
    description.name = route.name;
    description.layout = helpers::describe_layout(route.fields);
    layout->description.state.push_back(std::move(description));
  }
  for (const auto & route : layout->command_routes) {
    rpp_ros_interfaces::msg::NumericInterface description;
    description.name = route.name;
    description.layout = helpers::describe_layout(route.fields);
    layout->description.command.push_back(std::move(description));
  }
  for (const auto & name : script.controller_names()) {
    const auto & definition = script.controller_layout(name);
    rpp_ros_interfaces::msg::ControlInterface interface;
    interface.name = name;
    interface.state = helpers::describe_layout(definition.state());
    interface.reference = helpers::describe_layout(definition.reference());
    interface.control = helpers::describe_layout(definition.control());
    interface.horizon_length = definition.horizon().horizon_length();
    if (interface.horizon_length != 0) {
      for (std::size_t scalar = 0; scalar < definition.reference().size(); ++scalar) {
        for (std::size_t sample = 0; sample < definition.horizon().sample_count(); ++sample) {
          interface.horizon_offsets_ns.push_back(definition.horizon().offset(scalar, sample));
        }
      }
    }
    layout->description.controllers.push_back(std::move(interface));
  }
  std::set<std::string> generator_instances;
  for (const auto & name : script.goal_names()) {
    const auto & available = script.goal_layout(name);
    auto previous = goal_interface_definitions_.find(name);
    if (previous == goal_interface_definitions_.end()) {
      previous = goal_interface_definitions_.emplace(name, available).first;
    } else if (available.size() != previous->second.size() ||
      !rpp_interfaces::validate_required_fields(available, previous->second)) {
      throw std::invalid_argument("Cannot redefine goal layout for generator '" + name + "'");
    }
    rpp_ros_interfaces::msg::NumericInterface goal;
    goal.name = name;
    goal.layout = helpers::describe_layout(previous->second);
    layout->goal_routes.emplace(name, rpp_interfaces::field_route(available, previous->second));
    layout->description.goals.push_back(std::move(goal));
    generator_instances.insert(script.goal_instance_id(name));
  }
  for (auto stamps = goal_stamps_.begin(); stamps != goal_stamps_.end();) {
    if (!generator_instances.count(stamps->first)) {
      stamps = goal_stamps_.erase(stamps);
    } else {
      ++stamps;
    }
  }

  incoming_state_.resize(layout->measured_fields.size());
  Output pending;
  // The executor resizes its own frame when it observes the new layout.
  for (auto * output : {&bindings.output, &pending}) {
    output->actuator_values.resize(script.actuator_layout().size());
  }
  layout->safe_command.resize(script.actuator_layout().size());
  for (std::size_t index = 0; index < layout->safe_command.size(); ++index) {
    layout->safe_command[index] = script.actuators()[index];
  }

  bindings.ready = script.ready();
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    const auto previous_revision = inputs_.reset_revision;
    if (!layout_ || !same_field_layout(layout_->measured_fields, layout->measured_fields)) {
      inputs_.state_time_ns.reset();
      ++inputs_.reset_revision;
    }
    // Surviving generators retain their goals and original deadlines.
    // A concurrent disable still needs processing on the next tick.
    if (bindings.reset_revision == previous_revision) {
      bindings.reset_revision = inputs_.reset_revision;
    }
    pending_output_ = std::move(pending);
    output_pending_ = false;
    layout_ = std::move(layout);
    routes_ready_ = bindings.ready;
  }
  ++bindings.route_revision;
  reference_claims_.retain([&](const std::string & name) {
    return layout_->goal_routes.count(name) ? script.goal_instance_id(name) : std::string{};
  });
  if (navigation_interface_) {navigation_interface_->accept_layout(layout_->description);}
  if (bindings.ready) {
    RCLCPP_INFO(get_logger(), "Control routes bound.");
  }
}

ControlScriptNode::Status ControlScriptNode::control_tick(ControlScript & script, WorkerBindings & bindings, const Inputs & input,
  std::uint64_t ros_time, Duration dt)
{
  if (!input.enabled) {
    return helpers::make_status(Status::DISABLED);
  }
  if (!input.state_time_ns ||
    !timing::measurement_fresh(*input.state_time_ns, ros_time, state_timeout_)) {
    return helpers::make_status(Status::WAITING_FOR_STATE);
  }
  if (!timing::valid_control_interval(dt, max_control_dt_)) {
    return helpers::make_status(Status::FAULT, Status::STAGE_NONE, rpp_interfaces::contract::CODE_INVALID_INPUT);
  }

  // Goal cancellation is a management callback. Wait for the executor to finish it.
  if (bindings.reset_revision != input.reset_revision) {
    return helpers::make_status(Status::WAITING_FOR_GOAL);
  }

  const auto read_status = script.read_state();
  if (!control_success(read_status)) {
    script.reset();
    return helpers::make_status(Status::FAULT, Status::STAGE_SAMPLE, read_status);
  }
  const auto result = script.step(ros_time, timing::nanoseconds(dt));
  if (!result.success()) {
    return helpers::make_status(
      Status::FAULT, helpers::describe_step_stage(result.failed_stage), result.status);
  }
  return helpers::make_status(
    script.has_active_goal() ? Status::RUNNING : Status::WAITING_FOR_GOAL);
}

void ControlScriptNode::mark_worker_failed()
{
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    // Latch the fault: publish safe commands and prevent further plugin calls.
    routes_ready_ = false;
    stop_requested_ = true;
  }
  worker_wake_.notify_all();
}

void ControlScriptNode::publish_latest_output()
{
  std::string overdue_generator;
  {
    std::unique_lock<std::mutex> tick_lock(control_tick_mutex_, std::try_to_lock);
    if (tick_lock.owns_lock() && script_) {overdue_generator = script_->trajectory_warning();}
  }
  if (!overdue_generator.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Timed trajectory for '%s' no longer covers the requested sample/horizon; refresh it before its final timestamp.",
      overdue_generator.c_str());
  }
  std::shared_ptr<const OutputLayout> layout;
  std::uint64_t reset_revision;
  bool enabled;
  bool routes_unavailable;
  bool layout_changed;
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    layout = layout_;
    reset_revision = inputs_.reset_revision;
    enabled = inputs_.enabled;
    routes_unavailable = stop_requested_ || !routes_ready_ || slot_operation_pending_;
    layout_changed = layout && layout != published_layout_;
    if (layout_changed) {
      // Only the executor touches this frame. Resize it before swapping it
      // into the worker's mailbox for the new layout.
      publishing_output_.actuator_values.resize(layout->safe_command.size());
      command_samples_.clear();
      for (const auto & route : layout->command_routes) {
        Sample sample;
        sample.interface_name = route.name;
        sample.values.resize(route.fields.size());
        command_samples_.push_back(std::move(sample));
      }
      have_output_ = false;
    }
    if (output_pending_) {
      std::swap(publishing_output_, pending_output_);
      output_pending_ = false;
      have_output_ = true;
    }
  }

  if (layout_changed) {
    layout_publisher_->publish(layout->description);
    published_layout_ = layout;
  }

  auto & output = publishing_output_;
  const auto ros_time = now();
  const bool superseded = output.reset_revision != reset_revision;
  const bool expired = timing::output_expired(
    output.generated_at, timing::timestamp_ns(ros_time), max_control_dt_);
  if (!have_output_ || routes_unavailable || superseded || expired) {
    std::uint8_t state = Status::FAULT;
    if (!routes_unavailable) {
      if (!enabled) {
        state = Status::DISABLED;
      } else if (superseded) {
        state = Status::WAITING_FOR_GOAL;
      }
    }
    output.status = helpers::make_status(state, Status::STAGE_NONE,
      state == Status::FAULT ? rpp_interfaces::contract::CODE_NOT_READY : rpp_interfaces::contract::CODE_OK);
    output.status.stamp = ros_time;
    if (layout) {
      // The executor may still hold a command from before execution was disabled.
      std::copy(
        layout->safe_command.begin(), layout->safe_command.end(), output.actuator_values.begin());
    }
  }
  if (layout) {
    for (std::size_t index = 0; index < layout->command_routes.size(); ++index) {
      const auto & route = layout->command_routes[index];
      auto & sample = command_samples_[index];
      sample.stamp = output.status.stamp;
      for (std::size_t axis = 0; axis < route.indices.size(); ++axis) {
        sample.values[axis] = output.actuator_values[route.indices[axis]];
      }
      command_publisher_->publish(sample);
    }
  }
  log_control_transition(last_published_status_, output.status);
  last_published_status_ = output.status;
  status_publisher_->publish(output.status);
  publish_mission_monitors();

  // Deliver the safe frame for a pending reset before management callbacks can block.
  process_runtime_requests();
}

void ControlScriptNode::process_runtime_requests()
{
  if (!runtime_ || !script_) {
    return;
  }

  // Plugins can request their own deactivation during a guarded call. Execute
  // those lifecycle callbacks on the ROS thread after the current tick finishes.
  std::lock_guard<std::mutex> tick_lock(control_tick_mutex_);
  try {
    std::uint64_t reset_revision;
    {
      std::lock_guard<std::mutex> lock(mailbox_mutex_);
      if (stop_requested_) {
        return;
      }
      reset_revision = inputs_.reset_revision;
    }
    apply_pending_reset(*script_, worker_bindings_, reset_revision);

    const auto result = runtime_->process_deactivations(std::chrono::milliseconds(0));
    if (!result.success) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "Deferred component deactivation failed: %s", result.message.c_str());
    }
    if (helpers::active_instance_ids(*runtime_) != worker_bindings_.active_instances) {
      (void)refresh_routes(*runtime_, *script_, worker_bindings_);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Cannot process runtime requests: %s", error.what());
  }
}
}  // namespace rpp_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<rpp_control::ControlScriptNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("control_script"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
