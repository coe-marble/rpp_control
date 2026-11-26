#pragma once

#include <rpp_control/ros/control_script_node_helpers.hpp>
#include <rpp_control/ros/control_script_time.hpp>
#include <rpp_control/ros/navigation_interface.hpp>
#include <rpp_control/ros/reference_claims.hpp>

#include <rpp_ros_interfaces/msg/control_layout.hpp>
#include <rpp_ros_interfaces/msg/control_interface.hpp>
#include <rpp_ros_interfaces/msg/control_script_status.hpp>
#include <rpp_ros_interfaces/msg/mission_monitor_statuses.hpp>
#include <rpp_ros_interfaces/msg/numeric_sample.hpp>
#include <rpp_ros_interfaces/msg/numeric_goal.hpp>
#include <rpp_ros_interfaces/msg/numeric_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rpp_ros_interfaces/srv/request_external_reference.hpp>
#include <rpp_ros_interfaces/srv/release_external_reference.hpp>

#include <rpp_control/control_script.hpp>
#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_cpp/data_manager.hpp>
#include <rpp_ros2/slot_manager_ros.hpp>
#include <rpp_schema/rpp_interfaces/AllocatorHandoverState.hpp>
#include <rpp_schema/rpp_interfaces/ControllerHandoverState.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

namespace rpp_control
{
class ControlScriptNode final : public rclcpp::Node
{
  friend class NavigationInterface;

  using Clock = time_helpers::Clock;
  using Duration = time_helpers::Duration;
  using Sample = rpp_ros_interfaces::msg::NumericSample;
  using Goal = rpp_ros_interfaces::msg::NumericGoal;
  using Status = rpp_ros_interfaces::msg::ControlScriptStatus;
  using MonitorStatuses = rpp_ros_interfaces::msg::MissionMonitorStatuses;
  using ControlLayout = rpp_ros_interfaces::msg::ControlLayout;
  using SetBool = std_srvs::srv::SetBool;
  using RequestReference = rpp_ros_interfaces::srv::RequestExternalReference;
  using ReleaseReference = rpp_ros_interfaces::srv::ReleaseExternalReference;
  using Runtime = rpp::ConcurrentScriptRuntime;
  using ControlScript = BasicControlScript<true>;
  using SlotManager = rpp_ros2::SlotManagerRos<Runtime>;
  using SlotOperation = rpp_ros2::SlotOperation;

  struct Inputs
  {
    std::optional<std::uint64_t> state_time_ns;
    std::uint64_t reset_revision = 0;
    bool enabled = false;
  };

  struct Output
  {
    std::vector<double> actuator_values;
    Status status;
    std::optional<std::uint64_t> generated_at;
    std::uint64_t reset_revision = 0;
  };

  struct OutputLayout
  {
    ControlLayout description;
    std::vector<double> safe_command;
    rpp_interfaces::FieldLayout measured_fields;
    std::vector<node_helpers::NumericInterfaceRoute> state_routes;
    std::vector<node_helpers::NumericInterfaceRoute> command_routes;
    std::map<std::string, std::vector<std::size_t>> goal_routes;
  };

  struct WorkerBindings
  {
    Output output;
    std::uint64_t reset_revision = 0;
    bool running = false;
    std::vector<std::string> active_instances;
    bool ready = false;
    std::uint64_t route_revision = 0;
  };

public:
  using ControllerDescription = rpp_schema::rpp_interfaces::ControlLawDesc::Const;
  using HandoverStateFactory =
    std::function<rpp::HandoverState(const std::type_info &, const ControllerDescription *)>;

  explicit ControlScriptNode(HandoverStateFactory make_handover_state = {});

  ~ControlScriptNode() override;

private:
  void receive_sample(Sample::ConstSharedPtr sample);

  // Caller holds the tick and mailbox mutexes. Assemble directly into the prepared input.
  // Use the oldest contributing ROS timestamp for the whole-state timeout;
  // one fresh arm cannot hide a stale base sample.
  std::optional<std::uint64_t> assemble_state(const OutputLayout & layout);

  void apply_pending_reset(
    ControlScript & script, WorkerBindings & bindings, std::uint64_t revision);

  bool receive_goal(const Goal & goal, bool token_authorized = false);

  void request_reference(const RequestReference::Request & request, RequestReference::Response & response);
  void release_reference(const ReleaseReference::Request & request, ReleaseReference::Response & response);
  bool cancel_reference(const std::string & interface);

  // Caller holds mailbox_mutex_. A revision invalidates commands already queued for publication.
  void clear_goal();

  std::optional<Inputs> read_tick_inputs(bool clock_moved_backwards);

  void start_control(bool expose_navigation_interface);

  Runtime build_runtime();

  void initialize_routes(Runtime & runtime, ControlScript & script, WorkerBindings & bindings);

  void create_slot_manager();

  rpp::HandoverState make_application_handover(const std::type_info & state_type);

  void run_worker();

  void run_tick_loop(ControlScript & script, WorkerBindings & bindings);

  void queue_control_tick(ControlScript & script, WorkerBindings & bindings, const Inputs & input,
    const rclcpp::Time & ros_time, const time_helpers::TickTiming & tick_timing);

  void log_control_transition(const Status & previous, const Status & current);

  bool wait_for_tick(Clock::time_point deadline);

  void begin_slot_operation(
    const SlotOperation & operation, Runtime & runtime, ControlScript & script);

  rpp::CallbackResult finish_slot_operation();

  rpp::CallbackResult refresh_routes(
    const Runtime & runtime, ControlScript & script, WorkerBindings & bindings);

  void accept_routes(const Runtime & runtime, ControlScript & script, WorkerBindings & bindings);

  Status control_tick(ControlScript & script, WorkerBindings & bindings, const Inputs & input,
    std::uint64_t ros_time, Duration dt);

  void mark_worker_failed();

  void publish_latest_output();
  void publish_mission_monitors();

  void process_runtime_requests();

  const std::string workspace_, script_library_, script_name_;
  const bool is_dev_;
  const Duration tick_interval_, state_timeout_, max_control_dt_;
  const std::vector<double> safe_actuator_values_;
  const HandoverStateFactory make_handover_state_;

  std::mutex control_tick_mutex_;
  std::unique_lock<std::mutex> slot_change_lock_;
  std::mutex mailbox_mutex_;
  std::condition_variable worker_wake_;
  Inputs inputs_;
  // ROS-to-Cap'n Proto conversion storage, prepared while ticks are paused.
  rpp::List<double> incoming_state_;
  std::shared_ptr<const OutputLayout> layout_;
  Output pending_output_, publishing_output_;
  bool output_pending_ = false;
  // A ready route set always has a constructed script and a committed layout.
  bool routes_ready_ = false;
  bool slot_operation_pending_ = false;
  bool stop_requested_ = false;
  // The executor owns the published layout and its current output frame.
  std::shared_ptr<const OutputLayout> published_layout_;
  Status last_published_status_;
  bool have_output_ = false;

  rclcpp::Subscription<Sample>::SharedPtr state_subscription_;
  rclcpp::Subscription<Goal>::SharedPtr goal_subscription_;
  std::optional<NavigationInterface> navigation_interface_;
  ReferenceClaims reference_claims_;
  std::map<std::string, std::vector<std::uint64_t>> goal_stamps_;
  std::map<std::string, Sample::ConstSharedPtr> received_states_;
  std::map<std::string, rpp_interfaces::FieldLayout> state_interface_definitions_,
    command_interface_definitions_;
  std::map<std::string, rpp_interfaces::FieldLayout> goal_interface_definitions_;
  std::vector<Sample> command_samples_;
  rclcpp::Publisher<ControlLayout>::SharedPtr layout_publisher_;
  rclcpp::Publisher<Sample>::SharedPtr command_publisher_;
  rclcpp::Publisher<Status>::SharedPtr status_publisher_;
  rclcpp::Publisher<MonitorStatuses>::SharedPtr mission_monitor_publisher_;
  MonitorStatuses mission_monitor_message_;
  rclcpp::Service<SetBool>::SharedPtr enable_service_;
  rclcpp::Service<RequestReference>::SharedPtr request_reference_service_;
  rclcpp::Service<ReleaseReference>::SharedPtr release_reference_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::optional<Runtime> runtime_;
  std::optional<ControlScript> script_;
  WorkerBindings worker_bindings_;
  std::optional<SlotManager> slot_manager_;
  std::thread worker_;
};
}  // namespace rpp_control
