# Effective base motion limits

`RangerCommandLimiter` applies the final source-side normal-motion limits after
the desired control law and before collision checking/publication. The reference
planner settings are 0.25 m/s² forward acceleration, 0.30 m/s² forward deceleration,
0.40 rad/s² angular acceleration/deceleration and 0.20 m/s² lateral acceleration.
`max_angular_accel_radps2` is explicit and required in the ROS planner configuration.
It shares the 0.40 reference with visual docking's angular acceleration setting.
For admitted routes these limits come from the installed motion policy.

The first command after startup/reset is zero. Subsequent changes use elapsed ROS
source time capped by `rate_limit_max_dt_s`; paused time grants no additional speed
change, and clock regression stops immediately. The preferred command uses the
existing common interpolation factor for forward speed and yaw rate. It is kept
exactly when the emitted tuple satisfies all limits. Failure of that line does
not prove that the supported command set is empty: a local interval fallback
intersects independent forward/yaw slew and signed measured-progress bounds with
the configured forward speed, curvature cone/rate and lateral acceleration.
For fixed forward speed `v>0`, feasible yaw is the interval
`[max(yaw_low, curvature_low*v, -lateral_limit/v),
  min(yaw_high, curvature_high*v, lateral_limit/v)]`.
Its nonemptiness gives one bounded forward-speed interval. The fallback clamps
desired forward speed into that interval and then desired yaw into its yaw
interval. It introduces no optimizer or lateral/crab command. There is no enforced cruise-speed minimum for admitted routes. The spin speed cap applies
only to pure spin; forward arcs retain their configured radius/lateral bounds.

Curvature-rate history comes from the previous accepted positive-forward arc.
After zero or spin, the limiter imposes the radius cone on a new arc without
inventing a previous curvature; the desired control law retains its own curvature
initialization. For two positive-forward endpoints already inside the permitted
curvature interval, velocity interpolation keeps curvature between those
endpoints. A direct helper caller requesting a larger curvature jump now uses
the fallback instead of bypassing that rate. Exact stopping zero has no defined
curvature and remains a valid endpoint.

Spin-to-arc and arc-to-spin changes slow the current mode to zero before starting
the next mode. Normal terminal braking and mode handoffs cannot increase forward
speed or yaw magnitude or introduce opposite yaw. A retained nonzero command
keeps its motion state rather than reporting `GOAL_REACHED`. Pure-spin reversal
behavior at the source is unchanged; the final arbiter still owns the measured
stopped handoff. A pure-spin tail at or below the driver's floating-point epsilon
would select straight mode: the source instead reaches exact zero when slew
permits, or retains the nearest representable spin value above that threshold.

Both preferred and fallback outputs pass the same strict emitted-tuple checks:
original forward/yaw slew, signed fixed leads, speed limits, curvature cone and
applicable rate, lateral product, and actual forward/yaw radius quotient. The
quotient check uses the loaded configured radius; departure admission already
requires it to be at least the active vendor minimum. No epsilon enlarges an
accepted bound. Wider intermediate arithmetic and bounded inward representable
repairs handle rounded endpoints. A pinned yaw endpoint may require moving
forward speed inward and recalculating yaw. If no strict witness is found after
bounded repair, the source fails zero with `command feasibility numerical boundary`;
this is not a claim that every real-valued feasible command was absent. A truly
empty interval retains `measured progress incompatible with source slew`.

The local producer checks the exact limited command with finite response and
braking sweeps against current raw map/LiDAR evidence. It uses the installed
motion-policy footprint and padding, including occupied interior cells. It does
not use the retired full-circle clearance query. The costmap's physical clearing
footprint remains a separate setting. The `collision_footprint` effective-report
field now reports the actual route polygon; travel admission checks containment
and limits against that report.

The final arbiter independently checks the command and original capture pose.
A stale source, changed policy, expired task, tracking escape or newly blocked
route stops the invocation. Failure diagnostics follow an operation-owned zero;
publication ordering is not physical stop proof. Completion still requires fresh
measured stopping and the original endpoint checks.

The installed motion policy owns all coarse speed, acceleration, radius and
curvature limits. The current reference policy retains 0.25 m/s² acceleration,
0.30 m/s² deceleration, 0.40 rad/s² angular acceleration and 0.20 m/s² lateral
acceleration. `rover_execution` scaling remains supported for docking/departure;
non-unit coarse route scaling is rejected because it would disagree with the
admitted profile. There are no lookahead, spin-minimum or collision-circle knobs
for this route implementation.

See [route execution](route_execution.md) for active parameters and the
[current inspection guide](../../../../../docs/development/simulation/lab_navigation_entry_grasp.md).
Simulation tests do not establish physical acceleration, steering response,
footprint coverage or real-robot qualification. Hardware execution remains gated
by its installed policy and measured commissioning evidence.
