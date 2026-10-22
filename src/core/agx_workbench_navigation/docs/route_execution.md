# Motion-aware route execution

A footprint is the rover's occupied shape viewed from above, expressed in metres
relative to `base_link`. Heading is its orientation in the map. The motion policy
selects the footprint, clearance allowance, turning-radius and motion limits.
The simulation outline is experimental; it is not hardware qualification.

The ROS1 task asks for a registered destination, not intermediate waypoints.
Before releasing the base, route admission searches map position and heading
using forward steering primitives and stopped rotations. Every candidate's
swept filled footprint must fit the map and current LiDAR evidence. A route must
reach the destination with its required orientation before any part is executed.
No route means task failure. This search is not a time-optimal trajectory or a
spline reconstruction optimizer.

The global plugin consumes the admitted route bound to the invocation and exact
goal timestamp. Its remaining-route check validates the uncompleted portion
against fresh obstacles; it does not silently replace the route. A newly blocked
route stops this invocation. Dynamic waiting and detours are future work.

The local follower executes typed forward/spin segments using original measured
pose and motion. Forward speed follows route limits, braking distance and steering
response. Internal stopped joins use 5 mm position and 0.015 rad heading checks;
terminal coarse arrival uses installed tolerances capped at 20 mm and 0.15 rad.
The producer limiter validates slew, measured-motion leads, curvature and lateral
acceleration. Its accepted command is checked with a finite response/stopping
sweep before publication. The command arbiter independently checks the final
command against original LiDAR returns relocated from their capture pose.
Faults publish an invocation-owned zero before their terminal diagnostic.

Tag-camera docking follows a measured stopped handover. It has its own parking
reference, straight approach, verification and directional LiDAR guards. It is
not a continuation of the global trajectory under the tabletop.

For a registered dock start, native admission creates a typed `DockDeparture`
prefix from the measured heading and the source registry's staging corridor.
The installed docking/LiDAR configuration owns its rear directional outline,
stopping allowance, drift bounds and reverse limits. The full travel outline
checks the join and every onward segment. Caller-supplied prefixes are refused.
The entire route must be admitted before any reverse command is dispatched.

The existing reverse controller executes that prefix under docking authority.
Pinning preserves the original task deadline. Onward binding remains disabled
until the native owner verifies half a second of fresh original stopped poses,
an endpoint within 20 mm/0.015 rad, the actual-to-planned join and current
remaining-route occupancy. The mission then binds the navigation producer and
refreshes its zero command while authority is still stopping for handover. It
waits for fresh active navigation authority before sending the move-base goal;
following begins at segment 1. An accepted pending grant alone cannot start
following. Preparation and dispatch reuse the same goal stamp, route identity
and original deadline. Cancellation invalidates the prepared goal.
Hardware departure qualification is disabled; the installed phase is currently
enabled only for simulation. See the [transport run guide](../../../../../docs/development/simulation/lab_navigation_transport.md).

Simulation turn-speed tuning uses `motion_limits.max_yaw_rate_radps` in
`config/navigation_motion_policy_sim.yaml`. Its supported ceiling is 0.55 rad/s;
the requested 0.55 cap permits 0.262 m/s at the minimum 0.4764 m radius, about
half the 0.5 m/s forward cap. `lateral_acceleration_mps2` remains 0.2 and
independently caps curved speed. Angular acceleration and stopping checks still
apply; this also increases the permitted stationary-spin speed. The offboard
`config/isaacsim/navigation_motion_policy.yaml` is a symlink to this installed
simulation policy, so the two names do not require separate edits.

Python loading, native loading and route validation enforce the same ceiling;
larger values fail startup. Configuration edits within the supported bounds
require a simulator restart but no rebuild. Changing a compiled ceiling requires
a rebuild first. The hardware policy remains disabled and retains its 0.4 rad/s
configuration; simulation tuning does not authorize hardware execution.

Configuration ownership:

- `navigation_motion_policy_sim.yaml` (or the disabled hardware policy): route
  geometry, sampling/search, forward/spin speed, acceleration and turning limits.
  The offboard policy symlink points to these same bytes.
- `ranger_local_planner.yaml`: endpoint tolerance, measured feedback/progress
  leases, limiter time cap and steering-response slowdown.
- `rover_execution.yaml`: shared docking/departure phase scaling. Coarse-route
  non-unit scales are rejected as unsupported
  by the admitted route implementation.
- Registry: destination pose and docking backoff. Workflow YAML: task order and
  destination selection. Neither is a duplicate navigation tuning surface.

See [motion limits](motion_limits.md), [base authority](base_authority.md), the
[navigation task](../../../../../docs/development/tasks/navigation.md) and
[current simulation inspection](../../../../../docs/development/simulation/lab_navigation_entry_grasp.md).

Initial spins reserve stopping angle from the larger of original measured yaw rate and the last limited command, multiplied by original feedback age, one control interval and the 0.1 s response allowance. The approach aims inside the unchanged endpoint tolerance and still requires fresh stopped dwell before forward travel.

A spin that settles outside its endpoint tolerance may correct only an error of at most 3 degrees, after fresh measured stop. Correction stays in the same segment/invocation, caps yaw speed at 0.05 rad/s, allows two attempts and expires after 10 original source-time seconds. Its complete turn from the actual pose, remaining route and finite stopping sweeps must be clear. The original steady task deadline also remains in force. Forward motion is released only after the original heading tolerance and stop dwell pass. Larger errors, stale state, drift, blocked sweeps, exhausted attempts and cancellation end with owned zero; failure diagnostics include target, error, tolerance and attempts.
