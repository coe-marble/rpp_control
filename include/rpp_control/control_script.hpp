#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_cpp/logger.hpp>
#include <rpp_cpp/runtime.hpp>
#include <rpp_cpp/slot_manager.hpp>
#include <rpp_plugin_types/rpp_interfaces/StateProvider.hpp>
#include <rpp_plugin_types/rpp_interfaces/ReferenceGenerator.hpp>
#include <rpp_plugin_types/rpp_interfaces/ControlLaw.hpp>
#include <rpp_plugin_types/rpp_interfaces/ActuationMapper.hpp>
#include <rpp_control/control_plugin_contract.hpp>
#include <rpp_control/auxiliary_state.hpp>
#include <rpp_control/control_auxiliary_state.hpp>
#include <rpp_control/mission_supervision.hpp>
#include <rpp_control/detail/control_bindings.hpp>
#include <rpp_control/detail/field_values.hpp>
#include <rpp_interfaces/signal_router.hpp>
#include <rpp_schema/rpp_interfaces/LimitDesc.hpp>
#include <capnp_gen/rpp_interfaces/reference/constants.capnp.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <map>
#include <new>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rpp_control
{
// Class declarations and owned state. Implementations are grouped by responsibility under detail/.
// The owner serializes goal updates, routing changes, and lifecycle operations with ticks.
template<bool Concurrent>
class BasicControlScript
{
  using ActuatorLimit = rpp_schema::rpp_interfaces::LimitDesc_Native;
  using ComponentStatuses = std::map<std::string, std::vector<rpp::ComponentStatus>>;
  using RouteId = rpp_interfaces::SignalRouter::RouteId;
  static constexpr std::size_t missing_source = std::numeric_limits<std::size_t>::max();

  // Bindings hold routes and buffers prepared before numeric execution.
  struct ComponentRoutes : detail::ComponentDescription
  {
    RouteId state_route = missing_source;
    bool routed = true;
    std::uint8_t status = rpp_interfaces::contract::CODE_NOT_READY;
  };

  struct GeneratorGoal
  {
    rpp::List<double> values;
    rpp::List<bool> mask;
    rpp::List<std::uint64_t> arrival_times_ns;
  };

  struct BoundHorizon
  {
    std::vector<std::uint64_t> offsets;
    std::uint32_t length = 0;
  };

  struct TrajectoryTiming
  {
    std::uint64_t origin_ns = 0, duration_ns = 0;
    bool timed = false, pending = false;
  };

  struct ReferenceGeneratorBinding : ComponentRoutes
  {
    rpp_interfaces::FieldLayout targets;
    rpp_interfaces::FieldLayout outputs;
    rpp_interfaces::ReferenceHorizon horizon;
    std::vector<std::uint64_t> offsets;
    // Absolute expiry per target scalar. Zero means no valid goal for that scalar.
    std::vector<std::uint64_t> deadlines;
    std::vector<TrajectoryTiming> trajectory_timing;
    std::uint64_t last_sample_ns = 0;
    bool timing_pending = false;
    std::vector<std::size_t> target_reference_route;
    std::vector<std::size_t> output_target_route;
    rpp::List<double> values;
    rpp::List<double> current_values;
    rpp::List<bool> valid;
    RouteId output_route = missing_source;
    bool selected = false;
    std::uint8_t features = 0;
    std::shared_ptr<rpp_interfaces::ReferenceGenerator> plugin;
  };

  struct ReferenceRoute
  {
    std::size_t generator = missing_source;
    std::size_t value = 0;
  };

  struct ControllerBinding : ComponentRoutes
  {
    rpp_interfaces::ControlLawLayout layout;
    // One source cell for each reference-major input value, including future samples.
    std::vector<ReferenceRoute> reference_route;
    RouteId current_reference_route = missing_source;
    RouteId output_route = missing_source;
    bool commands_active = false;
    rpp::List<double> input_values;
    rpp::List<bool> input_valid;
    rpp::List<bool> enabled_axes;
  };

  struct AllocatorBinding : ComponentRoutes
  {
    rpp_interfaces::FieldLayout inputs;
    rpp_interfaces::FieldLayout outputs;
    std::vector<ActuatorLimit> limits;
    RouteId input_route = missing_source;
    RouteId output_route = missing_source;
    RouteId feedback_route = missing_source;
    // Unique controller indices. Missing inputs make the allocator unrouted.
    std::vector<std::size_t> command_sources;
    bool commands_active = false;
  };

public:
  enum class StepStage
  {
    None,
    Sample,
    Compute,
    Map
  };

  struct StepResult
  {
    StepStage failed_stage = StepStage::None;
    std::uint8_t status = rpp_interfaces::contract::CODE_OK;

    bool success() const
    {
      return control_success(status);
    }
  };

  RPP_COMPONENTS(
    {"state_provider", "rpp_interfaces::StateProvider"},
    {"reference", "List[rpp_interfaces::ReferenceGenerator]"},
    {"controller", "List[rpp_interfaces::ControlLaw]"},
    {"allocator", "List[rpp_interfaces::ActuationMapper]"},
    {"mission_monitor", "List[rpp_interfaces::MissionMonitor]"}
  )

  // Construction and configuration.
  explicit BasicControlScript(rpp::BasicScriptRuntime<Concurrent> & runtime);

  void configure(const std::vector<double> & safe_actuator_values = {},
    std::function<void(const std::string &)> report_binding_error = {},
    std::function<void(const std::string &)> report_warning = {});

  void rebuild_active_routes(const std::vector<double> & safe_actuator_values);

  rpp::CallbackResult validate_switch_routes(const std::string & slot,
    const rpp::SwitchRequest & request, const std::vector<double> & safe_actuator_values) const;

  rpp::CallbackResult bind_reference_before_activation(const std::string & name);

  void set_actuator_limit(std::size_t index, double lower, double upper);

  // Lifecycle.
  void activate();

  void deactivate();

  bool ready() const;

  bool all_active() const;

  // Generator goals.
  std::uint8_t set_point(const std::string & name, rpp::ListConst<double> values,
    rpp::ListConst<bool> mask, std::uint64_t deadline_ns);

  std::uint8_t set_path(const std::string & name, rpp::ListConst<double> values,
    rpp::ListConst<bool> mask, std::uint64_t deadline_ns);

  std::uint8_t set_trajectory(const std::string & name, rpp::ListConst<double> values,
    rpp::ListConst<std::uint64_t> times_ns, rpp::ListConst<bool> mask, std::uint64_t deadline_ns);

  void cancel();
  std::uint8_t cancel(const std::string & name);
  std::uint8_t refresh_goal(const std::string & name, rpp::ListConst<bool> mask,
    std::uint64_t now_ns, std::uint64_t deadline_ns);
  std::uint8_t goal_features(const std::string & name) const;
  std::string trajectory_warning() const;
  bool has_active_goal() const;

  // Executor-side supervision uses the last successful numeric snapshot.
  bool has_goal_monitor(const std::string & name) const {return supervision_.has_goal_monitor(name);}
  std::vector<std::string> mission_monitor_names() const {return supervision_.monitor_names();}
  template<class Visitor>
  std::optional<std::uint64_t> visit_mission_monitor_status(const Visitor & visitor) const
  {
    return supervision_.visit_status(visitor);
  }
  std::uint8_t prepare_mission(const std::string & goal_monitor,
    const std::string & generator, rpp::ListConst<double> final_target, bool new_mission);
  typename MissionSupervision<Concurrent>::Evaluation supervise_mission(std::uint64_t time_ns);

  // Numeric execution and resets.
  StepResult step(std::uint64_t time_ns, std::uint64_t dt_ns);

  std::uint8_t sample(std::uint64_t time_ns);

  std::uint8_t compute(std::uint64_t time_ns, std::uint64_t dt_ns);

  std::uint8_t commandMap(std::uint64_t dt_ns);

  void reset();

  void clear_outputs();

  void reset_controllers();

  // Layouts, buffers, and snapshots.
  // Ingress writes the provider; the execution owner reads it before step().
  // Calls share the owner's serialization with ticks and lifecycle operations.
  std::uint8_t set_state(rpp::ListConst<double> values, std::uint64_t time_ns);

  std::uint8_t read_state();

  const rpp_interfaces::FieldLayout & state_layout() const;

  const rpp_interfaces::FieldLayout & generated_reference_layout() const;

  const rpp_interfaces::FieldLayout & controller_reference_input_layout(
    const std::string & name = "") const;

  const rpp_interfaces::ControlLawLayout & controller_layout(const std::string & name) const;

  const rpp_interfaces::FieldLayout & command_layout() const;

  const rpp_interfaces::FieldLayout & actuator_layout() const;

  std::vector<std::string> controller_names() const;

  std::vector<std::string> goal_names() const;

  const rpp_interfaces::FieldLayout & goal_layout(const std::string & name) const;

  const std::string & goal_instance_id(const std::string & name) const;

  rpp::List<double> & state();

  rpp::List<double> & references();

  rpp::List<double> & commands();

  rpp::List<bool> & enabled_axes();

  const rpp::List<double> & actuators() const;

  const rpp::List<double> & realized() const;

  bool has_previous_control() const;

  ControlAuxiliaryState controller_auxiliary_state() const;

  AuxiliaryState allocator_auxiliary_state() const;

private:
  using ActuatorCommandsByInstance = std::unordered_map<std::string, std::vector<double>>;

  // Configuration implementation.
  void prepare_execution(
    const ComponentStatuses & components, const std::vector<double> & safe_values);

  void bind_reference_generators();

  void bind_reference_generator(ReferenceGeneratorBinding & generator);

  void configure_actuator_limits(const std::vector<double> & safe_actuator_values);

  void apply_actuator_limit(std::size_t index, double lower, double upper);

  static void add_limit(const ActuatorLimit & limit, const rpp_interfaces::FieldLayout & full,
    std::vector<double> & lower, std::vector<double> & upper, std::vector<double> & rates);

  template<class Bind>
  void bind_component(const std::string & slot, const std::string & name, const Bind & bind);

  bool binding_rejected(const std::string & slot, const std::string & name) const;

  void reject_binding(const std::string & slot, const std::string & name, const std::string & why);

  // Lifecycle implementation.
  template<class Plugin>
  bool component_active(const std::string & slot, const ComponentRoutes & component) const;

  template<class Plugin>
  bool component_active(
    const std::string & slot, const std::string & name, const std::string & instance_id) const;

  bool state_source_active() const;

  template<class Include>
  ComponentStatuses components_matching(const Include & include) const;

  ComponentStatuses active_components() const;

  template<class Plugin>
  std::shared_ptr<Plugin> component_instance(const std::string & slot, const std::string & name);

  // Routing implementation.
  void build_routes(const ComponentStatuses & components);

  void collect_controller_layouts(const ComponentStatuses & components);

  void collect_allocators(const ComponentStatuses & components);

  void connect_state_inputs(ComponentRoutes & component);

  void connect_reference_generators(const ComponentStatuses & components);

  void prepare_controller_references();

  // Goals implementation.
  static bool generator_has_goal(const ReferenceGeneratorBinding & generator);

  std::uint8_t stage_goal(const std::string & name, rpp::ListConst<double> values,
    rpp::ListConst<bool> mask, std::uint64_t deadline_ns, std::uint8_t feature,
    const rpp::ListConst<std::uint64_t> * times_ns = nullptr);

  // Inputs are validated by stage_goal; the output is usable only on success.
  std::uint8_t prepare_goal(const ReferenceGeneratorBinding & generator,
    rpp::ListConst<double> values, rpp::ListConst<bool> mask, std::size_t rows,
    GeneratorGoal & goal) const;

  // Execution implementation.
  // step() validates the frame once; these stages retain output and component guards.
  std::uint8_t sample_unchecked(std::uint64_t time_ns);

  std::uint8_t compute_unchecked(std::uint64_t time_ns, std::uint64_t dt_ns);

  std::uint8_t command_map_unchecked(std::uint64_t dt_ns);

  std::uint8_t compute_actuator_commands(std::uint64_t dt_ns);

  bool apply_actuator_limits(std::uint64_t dt_ns, bool & limited);

  std::uint8_t compute_realized_commands();

  void clear_controller_output(ControllerBinding & controller);

  static std::uint8_t merge_status(std::uint8_t previous, std::uint8_t next);

  static void zero(rpp::List<double> & values);

  // State implementation.
  void restore_generator_goals(const std::vector<ReferenceGeneratorBinding> & previous_generators);

  rpp::List<double> & actuator_values();

  rpp::List<double> & realized_values();

  std::size_t generator_index(const std::string & name) const;

  std::size_t controller_index(const std::string & name) const;

  template<class Plugin>
  void append_auxiliary_states(const std::string & slot, AuxiliaryState & auxiliary) const;

  void restore_enabled_axes(const rpp_interfaces::FieldLayout & previous_layout,
    const std::vector<bool> & previous_enabled_axes);

  void restore_application_limits(const rpp_interfaces::FieldLayout & previous_layout,
    const std::vector<double> & lower, const std::vector<double> & upper);

  ActuatorCommandsByInstance remember_actuator_commands() const;

  void restore_actuator_commands(const ActuatorCommandsByInstance & commands);

  // Runtime ownership and route preparation.
  rpp::BasicScriptRuntime<Concurrent> & runtime_;
  const rpp::ComponentContext & context_;
  MissionSupervision<Concurrent> supervision_;
  std::uint64_t route_revision_ = 0;
  // The context API takes strings. Construct slot names before any numeric calls.
  const std::string state_slot_ = "state_provider";
  const std::string reference_slot_ = "reference";
  const std::string controller_slot_ = "controller";
  const std::string allocator_slot_ = "allocator";
  bool preview_only_ = false;
  std::function<void(const std::string &)> report_binding_error_;
  std::function<void(const std::string &)> report_warning_;
  std::set<std::pair<std::string, std::string>> rejected_bindings_;
  bool configured_ = false;
  bool goals_sampled_ = false;
  bool control_computed_ = false;

  // Signal IDs stay fixed while layouts and bindings are rebuilt.
  rpp_interfaces::SignalRouter signal_router_;
  const rpp_interfaces::SignalRouter::SignalId state_signal_;
  const rpp_interfaces::SignalRouter::SignalId reference_signal_;
  const rpp_interfaces::SignalRouter::SignalId command_signal_;
  const rpp_interfaces::SignalRouter::SignalId actuator_signal_;
  const rpp_interfaces::SignalRouter::SignalId realized_signal_;
  std::string state_provider_name_;
  std::shared_ptr<rpp_interfaces::StateProvider> state_provider_;
  std::string state_provider_instance_;
  std::vector<ReferenceGeneratorBinding> reference_generators_;
  std::vector<ControllerBinding> controllers_;
  std::vector<AllocatorBinding> allocators_;
  std::map<std::string, BoundHorizon> bound_generator_horizons_;

  // Actuator constraints and per-tick validity/history.
  std::vector<double> allocator_lower_bounds_;
  std::vector<double> allocator_upper_bounds_;
  std::vector<double> actuator_rate_limits_;
  std::vector<double> application_lower_bounds_;
  std::vector<double> application_upper_bounds_;
  std::vector<double> safe_actuator_values_;
  rpp::List<bool> enabled_axes_;
  std::vector<bool> active_actuators_;
  std::vector<bool> feedback_valid_;
  rpp::List<double> last_actuator_values_;
};

using ControlScript = BasicControlScript<false>;
} // namespace rpp_control

#include <rpp_control/detail/control_script_configuration.hpp>
#include <rpp_control/detail/control_script_lifecycle.hpp>
#include <rpp_control/detail/control_script_routing.hpp>
#include <rpp_control/detail/control_script_goals.hpp>
#include <rpp_control/detail/control_script_monitoring.hpp>
#include <rpp_control/detail/control_script_execution.hpp>
#include <rpp_control/detail/control_script_state.hpp>
