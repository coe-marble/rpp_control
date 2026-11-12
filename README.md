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

### Waterjet direction and nozzle limits

`WaterjetAllocator.surge_direction_hysteresis` is a finite surge-force threshold
in newtons, in `[0, max_thrust]` (default `0.0` for compatibility). The supplied
allocator parts set it to `50.0` N. The allocator starts forward
on each activation. It switches to reverse only below the negative threshold,
and forward only above the positive threshold; at and between the thresholds it
retains direction. Set the threshold to `0.0` to restore the previous sign-based
behavior. Parameters are read at configuration; restart/reconfigure to change them.
Direction history is not transferred during allocator handover.

Hysteresis prevents small surge requests around zero from repeatedly reversing
the surge needed to produce yaw. It does not permit turning in place: a single
nozzle still needs nonzero surge to produce lateral force. A small surge request
opposite the retained direction can therefore produce surge in the retained
direction. Tune the threshold for the vehicle and controller.

The default `max_angle` and the supplied allocator parts now use
`0.5235987756` rad (30 degrees), matching the Jetski simulation nozzle. This
changes the previous `0.61` rad default. The allocator and physical/simulated
nozzle must use the same angle limits and thrust scaling: the second output is
`angle / max_angle`, so a mismatch changes the realized wrench even before
saturation. Explicit overrides must be updated in both configurations.

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

## Control time and simulation

`control_time_source` is a startup-only ROS parameter:

- `steady` (default) preserves the wall timer and monotonic `dt`, with existing
  RPP runtime receipt-time watchdogs.
- `ros` uses the node's ROS clock for the timer, `dt`, reference receipt times,
  and feedback/reference watchdogs. Set `use_sim_time: true` as well to follow
  `/clock`. A Python host is not required for this mode.

For example, launch with `--ros-args -p control_time_source:=ros -p use_sim_time:=true`
in addition to the required workspace parameters. Pausing `/clock` pauses control
updates and simulation-time watchdog aging. Configure `control_period` and
`max_control_dt` in simulation seconds; `/clock` increments must fit within
`max_control_dt`. Invalid or excessive step durations retain the existing stop
behavior. Node-clock mode does not replace the clocks of arbitrary child plugins.

A backward clock jump, a change of clock source, or a forward clock jump of at
least `max_control_dt` invalidates cached feedback and references and resets the
controller on the next enabled timer callback. That callback publishes zero
outputs; fresh feedback and references must arrive after it before control can
resume. Authority tokens and signal selections survive. Receipt at simulation
time zero is valid. A paused simulator does not publish repeated zero commands;
this mode is intended for simulation, with actuator watchdogs handled separately.

## Authority release and aborts

`ReleaseControl` releases the token authorizing `SelectSignal`. It leaves signal
selections, references and outputs unchanged; it is not a stop command.

`ReleaseExternalReference` releases the matching reference authority and clears
its cached references. On a successful release, it also zeroes stored wrench and
actuator commands. It does not disable axes, release control authority, or latch
a persistent stop: another valid reference can drive a later control update.
The service does not itself publish an actuator command; the next timer update
can recompute outputs before publishing them.

For the reported abort sequence, keep the control token while selecting
`DISABLEAXIS` for **both twist and wrench** on all axes to be disabled, release
the external reference token, then release control authority if appropriate.
These are separate service calls, not an atomic emergency stop. The `ctl_enable`
service only gates timer execution; disabling it does not publish a zero command.

## Portable component descriptions

The supplied cascade parts use `Folder` paths relative to the containing part
folder. Linked children retain their IDs; the current C++ runtime resolves them
by plugin and linked-component ID within the same parts root. The descriptions
and test fixtures no longer contain a developer-specific checkout path.
