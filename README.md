# rpp_control

Control libraries for the RPP system.

## CascadeController2D and MotionController2DImpl

`CascadeController2D` receives physical pose and twist values and produces a
normalized wrench in `[-1, 1]`. It has no scale parameters. Configure the
controller policy on `MotionController2DImpl`:

- `active_dofs` and `max_wrench_rate`;
- `wrench_positive_scales` and `wrench_negative_scales`, ordered `[x, y, yaw]`;
- `default_signal_tau` and `default_signal_nu`;
- allocation-feedback suppression parameters.

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
