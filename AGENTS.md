# ROS 2 Control Package Instructions

These instructions extend the global development instructions for this
ROS 2 C++ control package.

## Package Scope

This package contains robot-control functionality. Changes may affect
real hardware and must be treated as safety-sensitive.

Before editing, inspect:

- `package.xml`
- `CMakeLists.txt`
- public headers under `include/`
- implementations under `src/`
- launch and parameter files
- tests under `test/`
- interfaces used by neighboring packages

## ROS 2 Conventions

- Follow the target ROS 2 distribution and its supported API.
- Use `ament_cmake` and existing package conventions.
- Declare dependencies in both `package.xml` and `CMakeLists.txt`.
- Use standard ROS messages where they express the required semantics.
- Do not change message, service, action, topic, parameter, frame, or
  plugin names without identifying downstream compatibility effects.
- Prefer parameters for configuration over hard-coded constants.
- Add parameter descriptions, ranges, and validation where supported.
- Use ROS logging instead of `std::cout`.
- Throttle logs emitted from high-frequency paths.

## Node Design

- Keep callbacks short and non-blocking.
- Do not perform unbounded I/O in callbacks or control loops.
- Use callback groups and executors deliberately.
- Document assumptions about thread safety.
- Avoid shared mutable state; protect it explicitly when unavoidable.
- Use lifecycle nodes when startup, activation, and shutdown ordering
  are safety-relevant.
- Handle shutdown and partial initialization cleanly.

## Control-Loop Rules

- Do not allocate memory, block, log excessively, or perform file/network
  I/O inside real-time-sensitive update loops.
- Avoid unbounded containers and operations in real-time paths.
- Prepare slot-name strings, route validity, dependencies, and buffers before ticks.
  Keep goal-cancellation callbacks and transition logging on the executor.
- Use monotonic or ROS time consistently according to package semantics.
- Handle invalid, stale, missing, NaN, and infinite inputs explicitly.
- Validate command limits before publishing or writing commands.
- Preserve configured position, velocity, acceleration, effort, and
  rate limits.
- Define safe behavior for timeout, communication loss, and invalid state.
- Prefer fail-safe outputs over retaining the last command unless the
  package requirements explicitly say otherwise.
- Do not weaken watchdogs or safety limits without explicit approval.

## Interfaces and Frames

- State units in interface documentation.
- Use SI units unless a required interface specifies otherwise.
- Follow REP-103 coordinate conventions.
- Use consistent frame names and timestamps.
- Validate frame assumptions when using TF2.
- Do not silently transform or reinterpret units.

## C++ Practices

- Use the configured C++ standard.
- Prefer ROS 2 clock abstractions where simulation time matters.
- Prefer `rclcpp::Duration` and `rclcpp::Time` for ROS-time semantics.
- Use explicit ownership for nodes, publishers, subscriptions, timers,
  hardware interfaces, and controllers.
- Avoid exceptions escaping real-time update functions.
- Avoid copying large messages in high-frequency paths.
- Separate ROS communication from control algorithms where practical.
- Keep control algorithms testable without a running ROS graph.

## Parameters

- Validate every safety-relevant parameter.
- Reject invalid configurations with actionable error messages.
- Document defaults, units, valid ranges, and whether runtime updates
  are supported.
- Apply runtime parameter updates atomically.
- Do not leave the controller partially updated after validation fails.

## Source Organization

- Keep library and ROS helper headers under `include/rpp_control/`, and source
  implementations under `src/`.
- `control_script.hpp` owns class declarations, binding types, and persistent state.
  Keep nontrivial implementations in the corresponding `detail/` files.
- `control_script_configuration.hpp` owns component configuration, horizon binding,
  execution preparation, and actuator constraint configuration.
- `control_script_lifecycle.hpp` owns activation, deactivation, instance access,
  and lifecycle status queries.
- `control_script_routing.hpp` owns field compatibility, exclusive writer checks,
  and numeric connection planning. Keep lifecycle transitions and state restoration out.
- `control_script_goals.hpp` owns goal submission, validation, cancellation, and completion.
- `control_script_execution.hpp` owns numeric stages, runtime actuator limiting,
  feedback, and resets.
- `control_script_state.hpp` owns layout and buffer access, auxiliary snapshots,
  and compatible state restoration across configuration changes.
- `auxiliary_state.hpp` owns generic snapshot assembly. `control_auxiliary_state.hpp`
  adds controller history fields. Keep controller and allocator snapshots separate.
- `src/ros/control_script_node.cpp` owns ROS callbacks and worker synchronization.
  Its class declaration belongs in `include/rpp_control/ros/control_script_node.hpp`.
  Put ROS conversion and snapshot helpers in
  `include/rpp_control/ros/control_script_node_helpers.hpp`.
- `include/rpp_control/ros/control_script_time.hpp` owns time conversion, validation, freshness,
  and tick timing calculations. Keep this logic out of the node and routing files.
- `include/rpp_control/ros/navigation_interface.hpp` and `src/ros/navigation_interface.cpp`
  own optional navigation subscriptions, layout matching, and ROS-to-goal conversion.
  Receive `ControlScriptNode` directly and use C++ calls for layouts and goals;
  do not communicate with the owning node through internal publishers/subscribers.
  Run this work on the executor, outside the numeric worker.
- `src/ros/navigation_follow_path.cpp` owns the FollowPath action adapter.
  Check the selected generator's setPath feature during construction and on
  each request/submission. Forward the global path unchanged in meaning;
  pruning belongs to the reference generator and lookahead/control to the
  outer controller. Action heartbeats extend deadlines without restaging paths.
- `reference_claims.hpp` owns exclusive interface tokens. `control_script_references.cpp`
  owns claim/release services and per-interface cancellation. Execution enable
  and reference ownership are independent; do not introduce control tokens.

## Testing

For behavior changes, consider:

- unit tests for control algorithms
- parameter validation tests
- timeout and stale-data tests
- saturation and limit tests
- NaN and invalid-input tests
- lifecycle transition tests
- launch tests for ROS graph behavior
- simulation tests before hardware tests

Prefer tests that use simulated clocks and deterministic inputs.

## Validation Commands

Adapt these commands to the workspace:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build \
  --packages-select <control_package> \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash

colcon test --packages-select <control_package>
colcon test-result --verbose
