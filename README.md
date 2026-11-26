# rpp_control

`rpp_control::BufferedStateProvider` implements the shared
`rpp_interfaces::StateProvider` contract with a preallocated state buffer.

## Control script ROS node

`control_script_node` loads a registered RPP script and runs numeric updates on
a dedicated worker. The ROS executor receives measurements and goals, handles
component lifecycle operations, and publishes command and status snapshots.
At startup, the script configures and activates its loaded components. Execution
starts disabled and is controlled through `~/enable`.

```sh
ros2 launch rpp_control control_script.launch.py
```

The launch file uses the installed workspace and the bundled
`rpp_control::control_script` description. It selects the planar cascade,
interpolating reference generator, and waterjet allocator. Register the
`rpp_interfaces` schemas and configured `rpp_control` plugins before launching.
Use `rpp_workspace`, `script_library`, and `script_name` to select a configuration.

The launch also accepts `namespace`, `use_sim_time`, `control_frequency_hz`
(default 50), and `state_timeout_sec` (default 1.0). The node's
`max_control_dt_sec` defaults to twice the control interval. Configure
`safe_actuator_values` in final actuator layout order. The default is zero
for every actuator, subject to its declared limits.

## ROS inputs and outputs

Read `~/layout` before publishing numeric inputs.

- `state` describes measurement interfaces, grouped by field resource.
- `command` describes final actuator interfaces, grouped by field resource.
- `controllers` advertises each selected controller's flat state, reference,
  control, and horizon descriptions.
- `goals` advertises reference-generator instance names and their compact target layouts.

Publish `NumericSample` on `~/state` using an advertised measurement name and field
order. Resources can update independently. The node combines them using the oldest
contributing ROS timestamp for freshness checks.

Publish `NumericGoal` on `~/goal`. Its `interface_name` selects a reference-generator
instance from `ControlLayout.goals`; `identity_token` must own that interface.
Values follow that generator's advertised target
layout. `deadline_ns` is an absolute time in the node's ROS clock. An empty
`update_mask` updates every target scalar. False entries preserve their values,
curves, and deadlines. Quaternion coefficients update together.

The bundled planar cascade includes target fields ending in `.selection`.
For pose tracking, set these fields to `2` for internal references. In the bundled
layout, a complete pose goal has values `[x, y, yaw, 0, 0, 0, 2, 2, 2]`.
Selection `1` uses the submitted inner reference, and `0` disables that reference.

Call `~/enable` with `data: true` to run control using active components.
Calling it with `data: false` stops execution, cancels all generators' accepted
goals, and resets controller history while preserving component lifecycle states.
Disabling invalidates queued commands and produces safe output.
Component lifecycle services manage plugin activation independently.

`~/command` publishes one `NumericSample` per actuator resource.
`~/status` reports execution state, failing stage, and RPP status code.
A component failure leaves affected actuator resources at their configured safe
commands while independently available branches continue. Invalid timing,
missing measurements, and disabled execution produce safe output for the whole node.

Published measurement, actuator, and goal names identify fixed numeric contracts
for the node's lifetime. Compatible internal field reordering uses prepared maps
to preserve the advertised order. Conflicting redefinitions are rejected.
Goal submission ordering is tracked per generator instance.

## Reference ownership

Call `request_external_reference` with `interface_name` from `ControlLayout.goals`
and an introspection `identity`. Success returns `identity_token`. Each whole
interface has exactly one owner; a second request fails even if its identity
matches. Include the token in every numeric or masked reference update.
This is the normal behavior with `is_dev=false` (the default).

Call `release_external_reference` with that token to cancel the interface's
goals and release ownership. Other interfaces keep their goals. Expiring a goal
or disabling execution does not release ownership. Replacing the generator
instance revokes its token. Plugin exceptions stop the control worker.
Execution uses `~/enable`; there is no control-token or signal-selection service.

## Navigation command topics

Set `expose_navigation_topics:=true` to construct the optional `NavigationInterface`
component in `control_script_node`. It accepts namespace-relative `cmd_pos`
(`PoseStamped`) and `cmd_wrench` (`WrenchStamped`) when `is_dev=true`.
`cmd_vel` (`TwistStamped`) is exposed when **`is_dev` or `enable_nav2_cmd_vel`
is true**. `is_dev` defaults to false; `enable_nav2_cmd_vel` defaults to true.
It always exposes the masked `pose_ext`
(`PoseReference`), `twist_ext` (`TwistReference`), and `wrench_ext`
(`WrenchReference`) topics. It receives `ControlScriptNode` directly:
layouts and converted goals pass through C++ calls, without an internal layout
subscription or goal publisher. Conversion runs on the ROS executor.

Masked messages use their token to select the owned reference interface. Their
six mask entries are x/y/z/roll/pitch/yaw; planar interfaces support x/y/yaw.
Unselected values and their deadlines are preserved, including cascade selectors.
A spatial pose must update all quaternion rotation axes together.

`is_dev=true` bypasses reference-token validation for both numeric and masked
goals and ignores incoming goal stamps and stamp ordering. Masked development
messages use the configured navigation target instead of token-based routing.
`enable_nav2_cmd_vel=true` enables only the plain velocity input in normal mode.
Valid `cmd_vel` messages select the configured navigation target and override
reference-token ownership, including when that interface is already claimed.
The ROS adapter submits velocity values and cascade selectors through the normal
goal setter, treating the command as authorized without claiming a token.
An active FollowPath keeps its path; its heartbeat excludes the velocity command's
fields. Once the command expires, the adapter restores the path's velocity values
and selectors without resubmitting the path or resetting its progress. With no
active path, ordinary goal expiry produces safe outputs. Cancelling references
and disabling control retain their normal behavior. Numeric and masked reference
inputs keep their existing token checks. Normal-mode velocity commands retain message-stamp
validation and ordering; development mode uses reception ROS time as before.
Both booleans are read at startup. Frame, mask, numeric-value, and expiry
validation remain enabled.

**All command types expire unless refreshed, including pose goals.**
All navigation inputs and path heartbeats use `navigation_reference_timeout_sec`,
which defaults to **0.5 seconds** and must be positive.
By default, the timeout starts at the message's stamp; future, malformed, and
already expired commands are rejected. Development mode uses reception ROS time
instead, giving each command a full timeout:

```sh
ros2 launch rpp_control control_script.launch.py \
  expose_navigation_topics:=true \
  is_dev:=true
```

`is_dev` also disables stamp validation and ordering for `~/goal`.
Explicit numeric `deadline_ns` values are always enforced, including in
development mode. Measurement timestamps are still validated.
These parameters are read at startup.

The component uses advertised goal fields that match a controller's reference
fields. Set `navigation_resource` and/or `navigation_generator` if more than
one target matches; their empty defaults require an unambiguous match.
Each message's `frame_id` must match `navigation_frame_prefix` plus the declared
reference frame. The prefix defaults to empty; set it to e.g. `jetski/` when
the ROS frame names include the vehicle namespace. This is an explicit frame-name
alias; no TF conversion is performed. Pose orientations must be unit quaternions. Planar
layouts use x/y and yaw for pose, and x/y plus angular z for twist and wrench.
The bundled cascade exposes pose and twist goals. Wrench commands require
matching force/moment goal fields and a controller that consumes them.
`WrenchController2D` can be configured as a final cascade stage to expose these
fields, disable unavailable axes, and apply physical wrench rate limits. A zero
rate disables that axis's rate limit. Expired axes become zero immediately.

## FollowPath action

`NavigationInterface` also serves the standard `nav2_msgs/action/FollowPath`
action at `follow_path`. It forwards the entire global path to the selected
reference generator's `setPath`; it does not prune, select lookahead points,
or calculate velocity commands. The reference-generator plugin prunes the
global path using current state. The outermost controller consumes that pruned
reference and can implement regulated pure pursuit; inner controllers and the
allocator retain their existing roles.

Construction requires configured routes, an unambiguous position-target
interface, and the `setPath` feature. Missing support fails immediately,
before the numeric worker starts. Every action request and every path
submission checks the current generator's features again, including after
runtime replacement. Position-only and position/orientation layouts are supported.

Enable execution before sending an action goal. The action claims the whole
reference interface, refreshes only the submitted fields' deadlines while
running, and cancels/releases them on completion, cancellation, or failure.
A heartbeat does not restage the path and cannot revive an expired goal.
Replanning replaces the path under the same token. Plugin exceptions stop
the control worker. Every active `MissionMonitor` must approve continuation through
`isOk`; exactly one selected monitor is asked for `isGoalReached`. A veto, evaluation
failure, lost monitor, or invalidated monitoring routes aborts the action and cancels
its reference while components remain active. Matching measured fields provide
diagnostic pose, speed, and distance feedback without controlling motion.

Use an empty `controller_id`, `rpp`, or the default alias `FollowPath`.
Set `goal_checker_id` to the name of an active goal-capable mission monitor.
An empty ID uses `navigation_goal_monitor`; if that is also empty, exactly one
eligible monitor must exist. The selected monitor also participates in `isOk`.
`PoseMissionMonitor` checks planar/spatial position and orientation, with optional
stopped-speed and dwell conditions. `ProgressMissionMonitor` checks displacement
within a time allowance and optionally limits tracking error. Replanning preserves
mission watchdogs; a new mission resets them through `resetHistory`.
The node publishes `~/mission_monitors` as `rpp_ros_interfaces/MissionMonitorStatuses`.
Each frame lists all active instances by name/ID, their cached continuation/arrival
results, the `isOk` explanation message, and the selected goal monitor.
Stop/error messages also accompany action failures. Its single stamp is the evaluation time;
unevaluated entries remain marked unknown. Deactivated instances disappear from
the complete list. Publication stays on the executor and never reevaluates plugins.
Nav2's `progress_checker_id` and `path_handler_id` remain unsupported: mission
monitors supply the guards, and the reference generator handles the path.
`PlanarPathReference` supplies a pruned spatial path. `InterpolatingReference`
remains available for interpolation with waypoint timing. The bundled pose PID
consumes a single pose. `RegulatedPurePursuitController2D` consumes the path fields
and produces body velocity references for a twist controller.

Reference sampling returns values and status, with no completion flag. Timed
trajectories must initially cover the bound horizon. Sampling beyond their final
timestamp produces a throttled executor warning; it never indicates arrival.

## Planar path reference generator

`rpp_control::PlanarPathReference` accepts `setPath` and `setPoint`. It projects
the measured planar position onto the accepted path, removes the traversed part,
and returns original vertices ahead of that projection. Heading interpolation
uses the shortest rotation. Progress moves forward until a new path is submitted.
The generator never selects a lookahead, computes velocity, or declares arrival.

Configure its workspace `parameters.py` with:

```python
class ComponentParameters:
    resource = "base"                 # Required; matches state and controller fields.
    frame_id = "map"
    child_frame_id = "base_link"
    path_capacity = 256                # Output capacity, 2..512 points.
    max_points = 1024                  # Maximum accepted input waypoints, 1..65536.
    max_search_distance = 20.0         # Forward arc-length search window, metres.
    max_path_length = 100.0            # Maximum emitted arc length ahead, metres.
```

Distances must be finite and positive. The search window limits shortcuts onto
nearby return legs; choose it for the expected displacement between samples and
the start of newly submitted paths. Output stops at either the distance or point
capacity. Parameters change only through inactive reconfiguration. Submission
also respects the script's existing limit of 1,048,576 expanded goal values;
each input waypoint occupies `3 * path_capacity + 5 + additional_width` values
at the plugin boundary.

The plugin defines its own state and reference layouts. State is planar position
and yaw. The first two reference fields accept pose targets; a pose update must
replace x, y, and yaw together. The script expands compact target rows to the
plugin's reference width. Without additional targets, callers provide only
`(x, y, yaw)` per waypoint. The next four fields are generated outputs:

| Reference field | Stored values | Meaning |
| --- | --- | --- |
| `<resource>.position` | 2 | Current projection x/y, metres |
| `<resource>.orientation` | 1 | Current projection yaw, radians |
| `<resource>.path.position` | `2 * path_capacity` | Flat `x0,y0,x1,y1,...`, metres |
| `<resource>.path.orientation` | `path_capacity` | One yaw per path point, radians |
| `<resource>.path.count` | 1 | Number of valid points, including the projection |
| `<resource>.path.remainingDistance` | 1 | Arc length to the global endpoint, metres |

Optional `additional_targets` appends Euclidean/angle fields for constant cascade
overrides. For example, one interface can own path, velocity, and wrench targets:

```python
from rpp_interfaces.parameter_helpers import get_velocity_reference, get_wrench_reference

class ComponentParameters:
    resource = "base"
    additional_targets = get_velocity_reference(resource=resource, inner_controller="ctl1")
    additional_targets["fields"].extend(
        get_wrench_reference(resource=resource, inner_controller="ctl0")["fields"])
```

Additional selected values must be identical in every path row; they do not form
a timed curve. An update selecting only these fields preserves path progress.
Cancellation clears the path and overrides together. The advertised target layout
contains pose followed by the additional fields. Initialize every target before
using the derived path fields, whose validity conservatively depends on all target
deadlines. The ROS interface does this for FollowPath and complete pose commands:
unused inner targets become zero and selectors switch to internal references.
FollowPath heartbeats refresh this complete initialized target set. Partial masks
retain the usual field deadlines and cannot revive expired values.

Unused capacity repeats the last valid pose; consumers use `path.count` to ignore
padding. Cancellation emits count zero. At the global endpoint, count remains
one and the final pose persists until cancellation or expiry. Mission monitors
decide arrival against the submitted final target. Remaining distance covers the
entire accepted path, including the part beyond the emitted local section.

Use `rpp_control/planar_path_layout.hpp` in a consuming C++ controller to obtain
the same fields and offsets. It must request `horizonLength = 0`: spatial points
live inside a single flat reference and carry no arrival timestamps. Positive
time horizons and `setTrajectory` return `CODE_UNSUPPORTED`. Path staging and
buffer preparation run with sampling paused. Sampling neither allocates nor logs;
retained output views remain valid across subsequent samples and rebinding.

## Regulated pure pursuit

`rpp_control::RegulatedPurePursuitController2D` is a forward-only outer control law.
It consumes the exact `PlanarPathLayout` fields produced by `PlanarPathReference`.
Both plugins must use the same resource, frames, and `path_capacity`; incompatible
layouts fail route preparation. Its state is planar position/yaw and body linear
and angular velocity. Its control is body `(vx, vy, wz)`, with `vy = 0`.
The controller requests current references (`horizonLength = 0`).

Lookahead is either fixed or scales with measured linear speed. The controller
interpolates the first forward intersection with the lookahead circle, so sparse
waypoints do not force a farther target. Curvature slows the requested surge
speed, and the angular speed limit can reduce it further. Final-approach scaling
and a braking-distance bound apply when the global endpoint is in view. The
generator's full remaining distance prevents a clipped local section from being
mistaken for the end of the mission. Pruning remains entirely in the generator.

| Parameter | Default | Meaning / units |
| --- | --- | --- |
| `resource` | required | Field resource |
| `frame_id`, `child_frame_id` | `map`, `base_link` | Path and body frames; no TF lookup |
| `path_capacity` | 256 | Must match the generator, 2..512 |
| `desired_linear_vel` | 1.0 | Maximum forward speed, m/s |
| `lookahead_dist` | 1.0 | Fixed lookahead radius, m |
| `use_velocity_scaled_lookahead_dist` | true | Use measured speed times lookahead time |
| `min_lookahead_dist`, `max_lookahead_dist` | 0.3, 2.0 | Adaptive lookahead bounds, m |
| `lookahead_time` | 1.5 | Adaptive lookahead time, s |
| `use_fixed_curvature_lookahead`, `curvature_lookahead_dist` | false, 1.0 | Use a fixed steering/regulation lookahead, m |
| `interpolate_curvature_after_goal` | false | Extend the final nonzero segment to the fixed lookahead circle |
| `regulated_linear_scaling_min_radius` | 1.0 | Below this turn radius, scale speed by radius / threshold, m |
| `regulated_linear_scaling_min_speed` | 0.1 | Curvature-scaling speed floor, m/s |
| `approach_velocity_scaling_dist` | 1.0 | Distance over which final-approach speed scales, m |
| `min_approach_linear_velocity` | 0.05 | Approach-scaling speed floor before braking, m/s |
| `max_linear_accel`, `max_linear_decel` | 1.0, 1.0 | Positive surge command rate limits, m/s² |
| `max_angular_vel`, `max_angular_accel`, `max_angular_decel` | 1.0, 2.0, 2.0 | Yaw speed, acceleration, and braking bounds, rad/s and rad/s² |
| `use_rotate_to_heading` | true | Permit rotation with zero target surge |
| `rotate_to_heading_min_angle` | 0.785398 | Enter path-heading alignment above this angle, rad |
| `rotate_to_heading_angular_vel` | 1.0 | Rotation speed bound, rad/s |
| `goal_position_distance` | 0.05 | Switch to stopping/final alignment within this endpoint distance, m |
| `heading_deadband` | 0.05 | Stop requesting heading correction within this angle, rad |

Parameters change only through inactive reconfiguration. Distances, times, rate
limits and maximum speeds must be finite and positive. Minimum speeds may be
zero and cannot exceed `desired_linear_vel`. Require minimum lookahead <= maximum,
rotation speed <= maximum yaw speed, and
`0 <= heading_deadband < rotate_to_heading_min_angle <= pi/2`.
Set the control stopping distance and heading deadband within the selected mission
monitor's tolerances. They select control behavior; they never declare action success.

Fixed curvature lookahead selects the steering and regulation point independently
of the adaptive lookahead setting, following the
[Nav2 controller convention](https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp).
Endpoint extrapolation applies only when the global endpoint is visible; it never
extends a clipped local section or changes the generator's stored path. Yaw
reversals brake to zero before accelerating in the opposite direction.

Command ramps use `previousControl` at the cascade boundary, or measured body
velocity when feedback is unavailable. Surge and yaw acceleration limits can
temporarily change the requested pursuit curvature while slowing or turning.
Hard speed bounds take precedence if feedback starts outside those bounds.
An expired path, count zero, or disabled surge/yaw axis outputs zero immediately;
there is no retained command tail. Sway is always zero. Without rotation enabled,
a lookahead behind the vehicle returns `CODE_INFEASIBLE` instead of reversing.
With rotation disabled, final orientation is not controlled. Path/pose targets
remain active until cancellation, expiry, or a mission monitor's decision.

This plugin implements geometric regulation. It has no costmap input and performs
no obstacle-cost scaling or collision prediction. Those decisions require separate
supervision or additional inputs; running a Nav2 costmap does not add them implicitly.
Numeric computation uses preallocated result buffers and performs no allocation,
logging, ROS communication, or path updates.

The bundled `path_following` script pairs the path generator with a
`CascadeController2`: `ctl1` is pure pursuit and `ctl0` is the shared twist PID.
It includes the state provider, allocator, and both mission monitors:

```sh
ros2 launch rpp_control control_script.launch.py \
  script_name:=path_following expose_navigation_topics:=true \
  navigation_generator:=path_reference navigation_goal_monitor:=goal_monitor
```

Execution starts disabled. Supply state and enable it through `~/enable` before
using `follow_path`. The reference interface is named `path_reference`.
Tune the shared twist PID and allocator for the vehicle; the bundled twist PID
gains default to zero. This script exposes pose/path goals; direct twist or wrench
overrides require a configuration that also declares those generator targets.

## Flat plugin contracts

Each `ControlLawDesc` declares its own flat `state`, `reference`, and `control`
field lists. The script prepares measurement and generated-reference inputs and
routes returned control to the configured allocators. A `CascadeController` owns
its child connections, execution order, and child lifecycle. Other controller
implementations can be configured alongside a cascade.

Controllers can consume overlapping generated reference fields. The script warns
about these overlaps, and updates to the shared generator affect every consumer.
Control and actuator fields have exclusive writers.

`horizonLength = 0` supplies one current value per reference scalar.
For positive `H` and reference width `R`, offsets contain either `H`
shared times or `R * H` per-scalar times. Reference values use index
`r * H + h`. Each horizon starts at zero and strictly increases.
Quaternion coefficients have identical sampling offsets.

Generators sample the union of times required for the fields they produce.
Each controller receives its own requested cells through prepared indices.
Horizon rebinding runs with ticks paused and atomically preserves accepted goals
and component activation state.

`ControlLaw::compute()` receives flat state and reference arrays, an
`enabledAxes` mask in control order, `inputValid` in reference value order,
previous applied control and its validity, and unsigned nanosecond time and duration.
A disabled PID axis outputs zero and clears its retained history. PID controllers
use per-value reference validity to handle expired targets.

`ActuationMapper::commandMap()` returns actuator commands and a realized-control
estimate. The script applies actuator and application limits, then calls
`feedbackMap()` with the limited actuator values. Successful feedback is routed
to the corresponding controller on the next tick for anti-windup.

## Goals and lifecycle

`ControlScript::set_point(generator_name, values, mask, deadline_ns)`,
`set_path(generator_name, rows, mask, deadline_ns)`, and
`set_trajectory(generator_name, rows, times_ns, mask, deadline_ns)` use the
generator's compact target layout. Waypoints remain flattened in waypoint-major
order. The script expands them into the generator's reference order before submission.
Goal fields are selected from reference outputs, independently of measured inputs.
The bundled planar state has six values; its goals also carry three cascade selectors.

Goals and per-scalar deadlines belong to generator instances. A controller switch
retains the surviving generator's curve and original deadlines. Replacing a
generator requires a fresh goal. Separate generator instances provide independent targets.
Reference fields that directly match target fields inherit their deadlines.
Derived reference fields use the earliest target deadline. Generators with empty
target layouts supply continuous references while active.

Deactivating a component changes that component's lifecycle state. Dependents
remain active and report execution failure while required inputs are unavailable.
Route rebuilding reconnects compatible replacements. It preserves surviving
actuator rate-limit history, application caps, and disabled control axes.

## Numeric execution

`ControlScript::step(time_ns, dt_ns)` samples generators, computes top-level
controllers, maps control to actuators, applies limits, and estimates applied control.
It validates the measured state, time step, and axis-mask width once, then calls
private unchecked stages. Direct `sample()`, `compute()`, and `commandMap()` calls
validate their own frame inputs. Generated values and component availability
remain checked inside the stages.
The script prepares buffers and field mappings outside the numeric loop.
Allocator dependencies contain unique controller indices. Numeric updates use
prepared routing validity and feedback masks rather than rebuilding names or
querying configuration error sets. Controller output validity has one flag;
goal presence is derived from accepted deadlines.
`SignalRouter` provides shared state, current-reference, control, actuator,
and realized-control buffers. Per-controller horizon inputs use prepared
generator-value indices.

The worker schedules against a steady clock. Measurement freshness, output expiry,
numeric timestamps, goal deadlines, and control durations use the node's ROS clock.
Paused or backward simulation time prevents integration. The executor serializes goal updates, horizon binding,
and lifecycle changes with ticks. Measurement reception assembles resource samples
directly into a prepared input buffer and writes the state provider. The mailbox
keeps only the oldest accepted measurement timestamp for freshness checks.
Goal cancellation and transition logging run on the ROS executor. A pending
cancellation keeps worker outputs safe until the executor finishes applying it.
Before a lifecycle operation pauses execution, the executor publishes the configured
safe commands. Actuator slew history resumes from that stop frame. Component
lifecycle states follow the requested operation.

## Controller state and handover

Controllers and allocators expose `getAuxiliaryState()` for described numeric
history and its timestamp. With ticks paused, the script gathers those snapshots
separately for controllers and allocators, retaining the oldest contributing
timestamp within each group. Cascades include their children.

The application supplies measured state and previous realized control in
`ControllerHandoverState` or `AllocatorHandoverState`. Receivers select compatible
fields. Controller handovers carry only controller auxiliary history; allocator
handovers carry only allocator auxiliary history. PID history includes integral,
previous-error, and derivative values.
Cascades also retain selected control values at their internal boundaries.

`PIDParameters.anti_windup_gain` is in `[0, 1]`. Zero permits normal integration
at saturation. One rejects integral action that would increase saturation.
Valid applied-control feedback also tracks the previous requested-versus-applied
difference. Outer cascade stages receive the reference selected for their next
stage. The innermost stage receives allocator feedback.

## Underactuated single-nozzle vehicles

A nozzle at `(jet_x, jet_y)` satisfies
`tau_n = jet_x * tau_y - jet_y * tau_x`. Sway and yaw therefore share one
physical degree of freedom. With yaw priority enabled, `WaterjetAllocator`
derives lateral nozzle force from requested yaw moment and handles surge subject
to nozzle and thrust limits. Use the controller's axis mask to select the vehicle's
controlled axes.
