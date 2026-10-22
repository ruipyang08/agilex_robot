# Qualified default posture and travel readiness

`travel_profile.hpp` and `travel_policy.hpp` are onboard, ROS-independent support
APIs. The policy reads detached observations and issues no commands, physical
effects, permits or hold releases. The ROS navigation adapter owns the approved
arm/lift path, live monitoring and authority protocol. A task result is never
posture or clearance evidence.

## Qualification and profile admission

The onboard package default `config/travel_profile.yaml` is deliberately disabled
for both simulation and hardware. It contains no invented enabled posture or
envelope. Offboard `run_sim.sh --profile workbench_manipulation` explicitly
selects the simulation-only empty-start profile at
`config/isaacsim/travel_profiles/original_default_empty.yaml`. The complete
schema in `agx_navigation_tasks/test/travel_profile_fixture.yaml` uses synthetic unit-test values and
is not installed or suitable for physical execution.

Before enabling a separate profile, record physical evidence for every permitted
starting pose, load state and path to the same default arm/lift posture. A profile
that permits held travel must also qualify the retained tube's complete collision
envelope. Its conservative base footprint and speed/acceleration envelope must
cover every load state it permits. Record robot/model identity, source
configuration, tested paths, measurements and limitations in the qualification
evidence identified by the profile. Simulation evidence enables only
`simulation`; hardware requires its own physical trials. The parser checks the
declaration, not the truth of a qualification report.

The exact schema has these top-level fields:

| Field | Contents |
| --- | --- |
| `schema_version` | Integer `1` |
| `profile_id` | Stable named profile |
| `simulation`, `hardware` | `enabled` boolean and `evidence_id` |
| `robot` | Frozen arm group, arm/gripper/lift state topics, lift service and observer settings |
| `parameters` | `arm`, `lift`, `motion`, `held_load`, `base`, `timing`, `starts` |

Both `robot` and `parameters` may be null only for a disabled placeholder.
Unknown, duplicate and missing fields are rejected. Qualified profiles require
finite explicit positive bounds, unique named joints and starts, and a strictly
convex base footprint containing the base origin. There are no implicit numeric
defaults that authorize motion.
The preparation deadline is capped at 3600 seconds for the bounded worker's
software representation; this maximum does not qualify a physical timeout.

`robot.observer` supplies the lightweight `agxcore::TubeObserverConfig`, including
gripper open/close commands, command tolerance, all observation ages, stop dwell,
speed and cumulative measured position ranges. Navigation can create an
independent live observer without resolving a grasp fixture or perception config.
`parameters.arm` gives each default joint's position, tolerance and stopped speed.
`parameters.lift` gives the default position, tolerance and stopped speed. Motion
limits apply to the approved preparation path; base limits describe the one
shared empty/loaded navigation envelope. The adapter must bind actual planner and
arbiter limits to `base.id` and its recorded values. Matching posture alone does
not prove that a larger object fits the qualified envelope.

`held_load` also declares `attachment_link`, three-element `center_xyz_m`,
four-element `orientation_xyzw` and an explicit `touch_links` list. Together
with `max_radius_m` and `max_length_m`, these describe a conservative attached
cylinder enclosing the actual held tube. Its center and quaternion are relative
to the attachment link; the cylinder's local Z axis is its length axis. All
values must be finite, the quaternion must be unit length, and touch-link names
must be nonempty, valid and unique. The motion adapter must resolve every link
against the robot model, restrict allowed contacts to actual gripper links, and
attach this geometry before planning any loaded path. The parser validates the
declaration; it cannot establish kinematic membership or physical containment.
Do not allow the tube to touch arm, lift, base or environment links by broadening
the contact exemption. Enclosure, contact links and the full approved path remain
part of the profile's physical qualification evidence.

Each `starts` entry identifies a qualified path, named arm positions with its own
per-joint tolerances, lift position/tolerance and explicit empty/held permissions.
A profile may cover only empty starts, only held starts, or both; it must contain
at least one permitted start. Runtime start evaluation rejects a load that no
matching start authorizes. For example, home may permit only empty travel, while
a separately qualified phase-one return may permit a held tube and an explicitly
measured range of tool yaw. Wide starting tolerances do not relax the tighter
default endpoint tolerances. The entire declared start range and path must be
physically qualified; this is not a universal retract from arbitrary equipment.

```cpp
TravelProfile ParseTravelProfile(const std::string& yaml,
                                const std::string& source_path = {});
TravelProfile LoadTravelProfile(const YAML::Node& node);
void ValidateTravelProfile(const TravelProfile&);
bool TravelEnabled(const TravelProfile&, TravelMode);
```

Parsing performs no file or ROS I/O. `ParseTravelProfile` computes
`config_id = "sha256:" + SHA256(exact YAML bytes)`. `source_path` is provenance and
does not affect identity. The node overload hashes `YAML::Dump(node)`; callers
freezing an on-disk file should use the text overload to preserve byte identity.
Profiles and observations are value copies. `TravelPolicy` retains its own
validated profile; later caller mutations cannot change an active preparation.

## Measured start, preparation and endpoint

`EvaluateStartClearance(profile, mode, sample, source_now, steady_now)` requires
fresh named joint/lift measurements matching a declared starting pose and fresh
known empty or supported single-held custody. It returns that start and its
approved path ID. Stale historical clearance alone does not prevent a newly
measured, qualified start. Current captured-in-fixture/unsupported clearance,
unknown custody, unresolved objects, pending transfers or a state fault reject
the start. A held tube must retain its exact identity and container profile.

The adapter follows this sequence for every departure, including an already
default robot and navigation after a previous ordinary failure:

1. Establish and measure a stopped base hold and stopped command producers.
2. Call `BeginPreparation` and obtain the named starting path. It pins the load,
   object, authority session and hold epoch, and starts a fixed steady deadline.
3. Execute only that qualified path while calling `CheckPreparation` from the
   bounded supervisor. This consumes the start certificate while the same
   custody, fresh arm/lift evidence, held identity and command guard remain valid.
4. After successful joined arm/lift command completion and the final preparation
   guard, call the observer's one-shot `ArmMotionWindow`. It captures both clocks
   and fresh actual anchors under the observation mutex. Pass that exact returned
   generation/barrier to `CommandsFinished`; callbacks between arming and binding
   already belong to the window. Already-default execution follows the same rule.
   Never open the gripper to stow a held tube.
5. Feed fresh measurements to `Observe`. Publish a new safe-clear state effect
   only from the qualified measured geometry, then use the resulting state copy.
   Arm, lift, producer, gripper and clearance observations must all be newer than
   the original completion barrier. Matching arm/lift endpoints, original strict
   physical stationary intervals, quiescent producers and safe clearance must persist
   over source-time and receipt-time settled dwell. Retained motion/rate bounds
   also apply during final settling; exceeding them faults without reanchoring.
6. Only ready output can support the separate generation-bound departure permit.
   The authority server still rechecks hold and stopped base prerequisites.
7. Readiness requires continuous strict stationarity through permit/release waits.
   After the release RPC is actually accepted, retain that accepted-release event, then
   under the policy mutex recheck strict current readiness, window and ownership
   through `BeginTravel`. Only its success enables bounded retention during
   navigation. Failure returns through ordinary stopping with no navigation
   dispatch. Early or repeated transitions are rejected; the window never resets.

The arm/lift observations carry the observer's original
`stationary_source_since_s` and `stationary_receipt_since_s`. Each must be finite,
positive and no later than that axis's current original measurement evidence.
Both interval starts must also be no later than that axis's first settled
measurement in both clock domains. A nonstationary callback followed by a
stationary callback between policy polls therefore breaks continuity even when
the latest flags are true and the retained rate peak fits the travel bounds.
Before readiness this restarts only the policy's settled dwell at the current
sample. From readiness through successful `BeginTravel`, it latches failure with
the affected axis, original since-times and pinned settled evidence. The retained
window, generation, barrier and extrema never reset. After `BeginTravel`, bounded
retention applies without requiring that strict stationary interval to continue.

Changing custody, a new pending/unknown state, loss of hold/command guard, invalid
or regressing clocks, stale/out-of-order observations or a missed supervision
interval latches failure. A paused ROS clock cannot stop the steady deadline;
it also cannot advance source-time settled dwell. Re-reading a cached sample
does not extend dwell. The existing endpoint settling check remains, and the
independent retained range/rate guards can conservatively fault before dwell
completes. A later healthy or stationary sample cannot repair that fault.

Semantic transport revision is deliberately not pinned while the approved
preparation commands run: new measured clearance may replace old or aged
clearance. At readiness the policy pins `robot.transport.revision`, alongside
the original custody identity. The result exposes that revision for the permit
and continuing readiness lease, rather than the store's every-write revision.

```cpp
TravelPolicy policy(profile, TravelMode::Simulation);
StartClearance start = policy.BeginPreparation(sample, ros_now, steady_now);
// Bounded adapter executes start.path_id, continually checking preparation.
auto window = observer->ArmMotionWindow();  // Joined commands and final guard first.
policy.CommandsFinished(window, ros_now_after_commands, steady_now_after_commands);
TravelDecision ready = policy.Observe(new_sample, new_ros_now, new_steady_now);
// Continue with subsequent new samples until measured dwell is satisfied.
// After an actually accepted hold release, recheck strict readiness:
auto departed = policy.BeginTravel(released_sample, released_ros_now, released_steady_now);
// Dispatch navigation only if departed.ready.
```

`TravelHoldObservation` must come from the measured base guard and worker
supervisor. Producer observations describe joined/inactive command producers;
their timestamps cannot be refreshed merely by publishing readiness. Position
and velocity observations come from physical sensor messages, with their original
source timestamps and local monotonic receipt times. Readiness booleans in a
recipe or a previous task's success cannot supply these inputs.

Source timestamps here mean the original supplied message stamps. Hardware
acquisition age, resolution and accuracy require separate commissioning evidence;
retaining a publisher stamp does not establish those properties.

Producer quiescence excludes an active or unjoined local worker, a latched
`exit_required`, and active/joinable/coalesced/final stop RPC jobs or sealed
stopping. Original fresh terminal arm planning/execution statuses are still
required. The readiness path only copies stop-job state under its mutex; it
never reaps, joins or launches a retry. The worker's terminal exit outcome is
published before its active flag clears, and the shared exit flag only latches
true. A returned lift RPC is not physical stop or removal of a position-hold
target. Physical stationary flags remain separate arm/lift observations.
Each arm status stream keeps its own source/receipt high-water marks. Invalid
stamps or regressions latch travel evidence invalid for this workflow owner;
the combined minimum timestamp cannot hide an old terminal status behind a
newer active one. Legitimate actionlib state changes within one source-clock
tick remain accepted while preserving that stamp's original receipt. Later
valid status updates remain available to the independent terminal stop path,
but cannot erase the travel evidence fault.

## Continuing travel supervision

`CheckTravel` is available after readiness and continuously rechecks the same
default posture, quiescent producers, fresh custody/clearance and pinned semantic
revision. Before `BeginTravel` it additionally requires the same continuous strict
stationary intervals certified by the settled dwell. It does not require
manipulation hold, since successful departure releases that hold. The adapter
must keep the hold before release and supervise
base authority independently throughout travel. Fresh identical observations
may keep readiness without changing the semantic revision. Lost retention,
unsupported load, posture drift, stale feedback or a changed revision fails
closed and cannot be repaired by reusing this policy instance.

Every `Observe` and `CheckTravel` after command completion requires the exact
pinned motion-window generation/barrier and fresh original unique evidence.
Every recorded position extremum must fit the unchanged default-posture
tolerance; every maximum absolute reported rate must fit the unchanged joint/lift
profile limit. Retained spans also use the existing observer
`stopped_arm_range_rad` / `stopped_lift_range_m` (typically 0.002 rad / 0.001 m).
This includes final settling, permit waits, release and the entire travel phase.
Range checks compare representable interval endpoints, so an adjacent one-mm
height tick is not rejected by subtractive floating-point roundoff; no physical
epsilon or new configured margin is added. The first failed stream/joint, extrema,
bound and original extremum evidence are retained in the failure detail.

Invalid, stale or changed-window rejection records the consumption source/steady
clocks, valid/fresh flags, per-stream freshness checks, both original arm/lift
evidence, and actual/expected generation and barrier fields. It also includes the
observer's prior failure context, preserving the separate snapshot clocks when
that read was already stale. This distinguishes observer staleness, evidence that
ages before consumption, and token/validity rejection without changing any guard
or limit. Displayed sources are capped at 96 bytes and prior detail at 1536 bytes,
with `...` on truncation; original sample fields remain unchanged. Formatting
failure produces a generic `window fault` with the same Fault outcome. Neither
diagnostics nor later healthy input reset the first policy failure.

The [observer window guide](../../agx_xarm_pick/docs/motion_window.md) describes
all-callback coverage, duplicate handling and permanent invalidation. Wire lift
velocity remains integer mm/s. In the simulation facade, a reported -1 mm/s
results from rounding approximately -0.5 to -1.5 mm/s. Hardware quantization and
accuracy are not established by this diagnostic. Acceptance compares reported
measurements, not a guaranteed unquantized physical speed. No sensor feedback,
profile values, terminal stop rules or simulator model are changed; hardware
travel remains disabled.

`Abort` latches cancellation, deadline or fault. The adapter stops through its
bounded physical stop contract, preserves committed custody effects and asserts
hold; the policy itself cannot stop a worker or certify physical stopping.

## Verification scope

`travel_profile_test.cpp` checks exact schema admission, content identity,
disabled hardware, explicit bounds, named starts and shared envelope validity.
`travel_policy_test.cpp` checks empty/held starts, unsupported starting geometry,
wide qualified starting ranges, post-command barriers, cached observations,
settled dwell, revision pinning, retention/custody changes, cancellation,
deadline, clock domains, supervision gaps and stale/nonfinite/out-of-order data.
`workflow_travel_retention_test` links the production observation buffer and
injects hidden arm/lift motion between policy polls. It checks settling restart,
ready-to-release rejection, exact dwell boundaries, both original interval clocks,
paused-source contradictions and bounded retention after departure. The host
adapter test also checks that all four original interval fields are copied.
These tests establish software contracts only. Enabled physical profiles and
the ROS adapter's ordered-command/race qualification require separate evidence.

## Bounded ROS motion adapter

`MoveToTravelPosture(node, profile, start, load, observer, guard, stop)` takes the
exact `StartClearance` returned by the policy and the current `TransportView`.
It resolves that named start/path against the frozen profile and rechecks the
measured starting posture. This xarm6 adapter accepts ordered `joint1` through
`joint6` and compares MoveIt's finite `lifting_joint` position with fresh actual
lift feedback before planning.

Arm planning carries the qualified single-held cylinder in its `RobotState`,
using the configured center/orientation, maximum radius/length and exact object
identity. The attachment must belong to the AG95 gripper subtree, and collision
touch exemptions must be strict gripper descendants. Unknown existing attached
loads are rejected. The same payload model is present in current-scene collision
checks for the arm trajectory and vertical lift sweep. The arm trajectory must
remain inside the qualified named-start-to-default joint corridor and end at the
default target. Scene samples supplement physical qualification; they do not prove
continuous collision clearance or qualify an arbitrary starting condition.

Travel trajectories supply position, velocity and acceleration at every
waypoint. Both the active onboard `position_controllers/JointTrajectoryController`
and the simulation trajectory sampler therefore use quintic interpolation.
`travel_spline.hpp` represents that complete boundary-value polynomial in
Bernstein form. Subdivided convex hulls conservatively bound all interior
positions and derivative extrema: endpoint derivatives alone are insufficient.
Admission rejects an interior corridor excursion and uniformly stretches time
until the full velocity/acceleration bounds fit the profile, then rechecks the
rounded ROS durations. Collision samples follow this quintic curve rather than
straight waypoint chords. `travel_spline_test.cpp` includes zero-derivative
endpoints with nonzero interior speed and an interior overshoot regression.

Hardware qualification must additionally bind the controller's initial bridge:
Noetic JTC can drop a time-zero first waypoint and splice from its previous
desired state to the first future waypoint. Physical joint feedback alone does
not establish that desired state. The simulation adapter starts from the supplied
first waypoint instead. This initial-bridge contract remains a hardware
qualification boundary. The host rejects hardware composed travel even if a
profile claims hardware qualification; enabling it requires implementation and
validation of that bridge, followed by physical qualification.

The bounded supervisor checks the measured corridor during execution: lift stays
at its qualified start during arm motion, and arm stays at measured default during
lift motion. The actual production `RunTravelMotionSequence` rechecks interruption
after every command/observation callback. It waits for fresh measured stopped arm
endpoint before issuing lift commands, then for measured stopped lift endpoint.
The policy's separate post-command endpoint/dwell barrier still follows both.
Lift speed is rounded down to integral mm/s and must remain positive; target
quantization must fit the qualified tolerance. Overflowing wire values reject.

`RunBoundedMotion` accepts at most 3600 s and rejects durations smaller than one
steady-clock tick before launching a worker. Cancellation/fault/deadline is latched.
A vendor call already past its interruption read may still be in flight, so the
independent bounded stop path is repeated while joining and again after an
interrupted/failed worker joins. Joining is not measured stop confirmation. An
unjoined worker returns `exit_required=true` and `UnconfirmedStop`; the host must
flush evidence and exit while the independent watchdog retains stopping.

`bounded_motion_test.cpp` and `travel_motion_sequence_test.cpp` exercise fixed
bounds, final callback interruption, delayed successful/failed lift-speed replies,
no later position command, repeated/final stops, named-path identity, corridor
rejection, lift wire limits, measured model disagreement and gripper-link scoping.
The adapter never authorizes base release; that remains contingent on the policy,
measured base hold, current readiness and the authority permit protocol.

## Admission against effective base settings

The ROS host calls `AdmitTravelEnvelope(node, profile)` before dispatch and
retains the returned `TravelEnvelopeAdmission`. Its `Guard()` participates in
preparation and travel supervision. `Digest()` and `SnapshotYaml()` record the
admitted configuration in the run manifest; `Detail()` explains a rejection.
The adapter owns a dedicated callback queue and admits within a wall-clock
bound (3 seconds by default, explicitly configurable only within `(0, 60]`).
The ROS adapter is built into the workflow executable; the installed pure
library exports `EvaluateTravelEnvelope` and `TravelEnvelopeMonitor` for
independent contract testing.

Admission consumes five version-1 effective configuration reports at
`/workbench_nav/effective_config/{local_planner,global_planner,base_arbiter,docking,lidar}`.
Each owning process reports its actual cached settings and a fresh process or
plugin session ID. Reports are latched and repeat on a 0.2-second wall-clock
heartbeat. Reading edited YAML files or parameter-server values is not proof of
what a running controller loaded. The local report contains both its cached
collision-checker footprint and the costmap's current padded footprint; the
global report contains its current padded footprint, including the polygon
used for a configured robot radius. Changing those footprints invalidates the
admission. Dynamic footprint changes are not supported by this workflow.

The pure check requires every qualified footprint vertex to be inside the
local collision, local costmap, global costmap and LiDAR protected polygons.
Only finite, ordered, strictly convex polygons are admitted. The LiDAR
self-return inset must itself lie inside the qualified footprint: a larger
excluded region could hide an obstacle outside that volume. The reported
height filter must include the base plane; this is a planar envelope check,
not a new claim that all obstacles at arm or payload height are detectable.
The profile's physical path/load qualification remains necessary.

The arbiter's independent x/y speed ceilings are checked as a vector norm.
Coarse angular speed is bounded by both its spin limit and its maximum speed
at minimum turn radius. Coarse planar acceleration combines the larger
longitudinal acceleration/deceleration with lateral acceleration. Docking uses
a conservative bound for both body-frame component ramps and body rotation,
as well as the final-approach and rollback bounds. All routine bounds must be
at or below the profile's qualified limits. The exact loaded
`rover_execution` policy is validated in both planner and docking reports;
all scales/coefficients must be in `(0,1]`, and the two policies must agree.
These are checks, not runtime changes to any controller settings.

The arbiter reports fixed measured-speed leads, complete pose-feedback and
source/receipt/future-age policy,
and the explicit `measured_progress_contraction` / `producer_source_time` policy.
Admission rejects legacy final-slew fields and other policy names. Producer
interpolation is checked before outgoing collision checks; final scalar
contraction preserves the checked path. Source-time derivative bounds do not
certify final publication slew or physical acceleration. Those still require
recorded qualification against this profile. All policy fields enter the frozen
identity; missing fields or a changed mounting frame invalidate admission.
The arbiter minimum turning radius must not exceed the planner's minimum radius.
See the [onboard command-limit guide](../../agx_workbench_navigation/docs/motion_limits.md).

Normal terminal approaches brake through their producer before reporting arrival.
Emergency stops and cancellation can still issue immediate zero; their physical
deceleration and retention need separate stop/load qualification. Measured
progress bounds command buildup during steering alignment, not actuator response.

After admission, missing or multiple publishers, a changed report, a process
restart, reversed receipt time, or a heartbeat gap over two seconds permanently
invalidates that admission. A later heartbeat cannot silently restore it.
Report publishers observe configuration, not physical motion; existing live
odometry, authority, sensor, action and watchdog checks remain required for
motion and process health. The report heartbeat is not a replacement for those
faster physical guards.

The current supported load is a manually identified container type. Admission
matches the exact container profile ID and checks the admitted grasp geometry,
including dimensional tolerances, against the travel cylinder bounds. The run
manifest freezes the grasp configuration and its source identities. The
container declaration currently has no mass or contents field: the travel
profile's mass ceiling is a physical qualification assumption for that
identified type, not a value inferred from gripper contact, perception, a task
success result, or runtime weighing. Changing tube contents or physical mass
requires renewed qualification. Hardware loaded travel remains disabled until
that qualification exists; no database or inventory identity is implied.
