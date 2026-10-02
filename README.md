# rpp_control

Control libraries for the RPP system.

## Capability-normalized CascadeController2D

`CascadeController2D` runs its pose and twist PID subcomponents in normalized
coordinates. Configure each vehicle with three-element scale vectors ordered as
`[x, y, yaw]`:

- `pose_scales`: position in metres and heading in radians.
- `twist_scales`: linear velocity in m/s and yaw rate in rad/s.
- `wrench_positive_scales`: positive force in N, lateral force in N, and yaw
  moment in Nm.
- `wrench_negative_scales`: magnitudes for the corresponding negative wrench
  limits.

The cascade divides pose and twist by their scales before applying the PIDs.
It bounds the normalized wrench to `[-1, 1]`, selects the directional wrench
scale, and sends the resulting SI wrench to the allocator. The allocator then
produces normalized actuator `cmd_out` values.

`cmd_tau` remains an SI wrench reference; it is not normalized. For reusable
inner-loop tuning, set PID output limits to `1.0` and provide vehicle-specific
scale vectors.

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

`MotionControllerRos` can suppress one internally controlled DOF when a
higher-priority DOF causes the allocator to realize a materially different
wrench. Configure the policy with:

- `allocation_priority_dof` and `allocation_suppressed_dof` — distinct active
  DOF names, such as `yaw` and `x`.
- `allocation_priority_request_threshold` — minimum requested higher-priority
  wrench magnitude.
- `allocation_residual_activation_threshold` and
  `allocation_residual_deactivation_threshold` — lower-priority wrench
  residual thresholds in its SI unit. Activation must exceed deactivation.

The policy compares the previous requested and realized wrenches. While the
priority request and residual exceed their thresholds, it disables the
suppressed axis for the cascade pose and twist controllers, freezing that
axis's PID state. Direct external wrench references remain unchanged. The
existing control-state integration flag shows the suppressed PID as inactive.
