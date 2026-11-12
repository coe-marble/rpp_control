# rpp_control

Control libraries for the RPP system.

## CascadeController2D and MotionController2DImpl

`CascadeController2D` receives physical pose and twist values and produces a
normalized wrench in `[-1, 1]`. It has no scale parameters. ROS supplies the
following namespaced options to `MotionController2DImpl`:

- `controller.active_dofs` and `controller.max_wrench_rate`;
- `controller.wrench_positive_scales` and
  `controller.wrench_negative_scales`, ordered `[x, y, yaw]`;
- `controller.default_signal_tau` and `controller.default_signal_nu`;
- `controller.activate_on_start` (default `true`), which controls whether the
  controller and allocator subcomponents are activated during configuration;
- `controller.allocation_*` suppression parameters.

The signed wrench scales convert an internal normalized command to SI force and
moment before allocation. They must be the vehicle's physical full-scale wrench
limits. `cmd_tau` remains an SI external wrench reference and is not scaled.

`PIDParameters.anti_windup_gain` is in `[0, 1]`: `0.0` permits normal
integration at saturation, while `1.0` rejects integral action that would
increase a saturated output.

## Underactuated single-nozzle vehicles

A single steerable nozzle has two actuator inputs. For a nozzle at
`(jet_x, jet_y)`, its planar wrench satisfies
`tau_n = jet_x * tau_y - jet_y * tau_x`; sway and yaw are therefore not
independent. Configure a single-nozzle vehicle with active DOFs `x` and `yaw`.
When yaw is enabled, `WaterjetAllocator` gives yaw priority and derives the
nozzle lateral force from the requested yaw moment. It ignores direct sway
force instead of summing incompatible sway and yaw demands.

## Allocation-feedback priority

`MotionController2DImpl` compares residuals using its signed wrench scales,
which are also used to denormalize the internal Cascade command.

`MotionController2DImpl` can suppress one internally controlled DOF when a
higher-priority DOF causes a conflicting realized wrench. Configure the
policy with:

- `allocation_priority_dof` and `allocation_suppressed_dof` — distinct active
  DOF names, such as `yaw` and `x`.
- `allocation_residual_activation_percent` and
  `allocation_residual_deactivation_percent` — normalized residual thresholds;
  deactivation must be lower than activation.

The policy compares the previous requested and realized wrenches. A yaw
shortfall or an `x` residual suppresses the `x` cascade controller at the
activation threshold and re-enables it below the lower deactivation threshold,
even while yaw remains requested. Direct external wrench references remain
unchanged. The existing control-state integration flag shows the suppressed PID
as inactive.

## ROS command topic message types

The developer command inputs are enabled by `expose_developer_topics`.
Set `rpp_control::kUseStampedCommandTopics` in
`include/rpp_control/ros/command_types.hpp`, then rebuild the controller and
publishers that use its `CmdPose`, `CmdTwist`, and `CmdWrench` aliases.
The default `true` selects the stamped messages used by current Nav2 releases.
Set it to `false` to use the legacy unstamped interfaces.

| Topic | `false` | `true` |
| --- | --- | --- |
| `cmd_pos` | `geometry_msgs/msg/Pose` | `geometry_msgs/msg/PoseStamped` |
| `cmd_vel` | `geometry_msgs/msg/Twist` | `geometry_msgs/msg/TwistStamped` |
| `cmd_tau` | `geometry_msgs/msg/Wrench` | `geometry_msgs/msg/WrenchStamped` |

Only the wrapper changes. Reference coordinates and SI units keep their
existing meanings; no TF transformation is inferred from header frame IDs.
Header timestamps do not replace receipt-time reference handling or watchdogs.

`cmd_out` remains `std_msgs/msg/Float64MultiArray`, carrying actuator commands
consumed by `jetski_controller` rather than a pose, velocity, or wrench.

### Stamped command age filtering

`MotionControllerRos` declares two dynamically changeable ROS parameters:

- `discard_old_stamped_messages` (bool, default `true`) enables age filtering
  for stamped `cmd_pos`, `cmd_vel`, and `cmd_tau` inputs.
- `max_stamped_message_age_sec` (double, default `0.5`) is the strictly positive,
  finite maximum age in seconds, with a minimum resolution of one nanosecond.

Enabled filtering accepts only `0 <= now - header.stamp < max_age`, using the
node's ROS clock, including simulated time. Messages exactly at the limit,
older messages, future timestamps, and invalid stamps are dropped before any
reference or signal selection is updated. A zero stamp represents ROS time
zero and therefore becomes stale as the ROS clock advances. Publishers and
controller must use the same clock. Existing reference watchdogs still apply.

Unstamped inputs bypass this filter. Setting `discard_old_stamped_messages` to
`false` ignores header timestamps. Valid parameter updates take effect without
restarting; rejected atomic updates leave the filtering policy unchanged.
