# Base holds, departure permits and explicit fault reset

The onboard `base_command_arbiter.py` remains the autonomous publisher of vendor
`/cmd_vel` (`geometry_msgs/Twist`). Its ROS-independent `AuthorityArbiter` owns
hold/stop generations, departure permits, command selection and safety checks.
The ROS wrapper serializes every callback and output tick with one `RLock`.
Pure callers must provide the same single-owner serialization.

The guarded departure services, readiness lease and per-operation producer
fencing are implemented support. Workflow release remains disabled until measured
travel preparation and host integration are qualified. The complete producer,
relay and consumer protocol must be deployed together.
Every ROS motion grant requires a measured guarded departure, including the first
journey after arbiter boot or restart. Standalone callers must route through the
workflow preparation path; an ordinary authority request cannot bypass it. The
legacy ROS-free state-machine primitives exist for unit compatibility only and
are not a transport entry point.

## Hold identity and confirmation

`BaseAuthorityState` adds `session_id`, `hold_epoch`, `grant_id` and
`source_goal_id` to its existing state,
source, reason and `manipulation_hold_active` fields. Each arbiter construction
creates a boot-unique session. `hold_epoch` is the monotonic generation for new
holds, normal stop/NONE requests, cancellation, fault entry and explicit reset.
An epoch is an identity, not a time or an indication that stopping is confirmed.
Every explicit emergency-stop assertion advances it, including while already
faulted, so an older queued reset cannot clear a newer stop. Observing the same
latched fault on subsequent output ticks does not advance it. The first fault
cause remains latched in both cases.

`/workbench_nav/acquire_base_hold` uses `AcquireBaseHold`:

```text
request: run_id, operation_id, reason
response: accepted, stopped_confirmed, session_id, hold_epoch, reason
```

`accepted` acknowledges the assertion. `stopped_confirmed` additionally requires
fresh valid odometry below both measured velocity thresholds for the configured
dwell, and sustained zero output after the hold began. An accepted assertion can
therefore have `stopped_confirmed=false`, including while faulted. No arm task may
treat assertion acknowledgement alone as measured stopping.

Repeating the same run/operation/reason while its generation remains current
rechecks stopping without creating a new generation. Reusing its operation with
different content or after another stop/release is rejected. Use a new operation
ID for a new stop. `/workbench_nav/hold_for_manipulation` remains a legacy
`std_srvs/Trigger` stop-only endpoint; each call invalidates departure intent.
Repeated assertions preserve an already established physical stopped/zero dwell,
so legacy polling can still obtain confirmation. Neither endpoint clears faults.

## Readiness is measured onboard evidence

`/workbench_nav/travel_readiness` uses `TravelReadiness`:

```text
header, run_id, departure_id, session_id, hold_epoch, sequence, state_revision,
config_id, travel_profile_id, arm_ready, lift_ready, producers_stopped,
clearance_verified, payload_state, payload_ref, container_profile_id, ready, reason
```

`payload_state` is `UNKNOWN=0`, `EMPTY=1` or `HOLDING=2`. Empty evidence has no
payload/profile reference. Held evidence requires both. Unknown, inconsistent or
missing payload evidence never grants departure. Config/profile IDs must identify
the immutable qualified configuration used by the producer.

The onboard adapter rechecks the measured arm/lift posture, physical clearance,
payload retention and live supervision for each observation. `producers_stopped`
means the arm/lift command producers are stopped; it does not require the base
navigation command producer to stop during travel. `state_revision` binds the
semantic transport/custody revision, not a counter incremented by every healthy
sensor heartbeat. Revision changes, custody/config/profile changes, changed
readiness flags and invalid evidence revoke an outstanding permit. They fault an
active travel lease. Healthy observation refresh must preserve that semantic
revision; it must not preserve stale underlying measurements.

The message header uses ROS source time. The receiver supplies its own steady
receipt time; no sender exports a steady-clock timestamp. Both age checks apply.
Sequence and source stamp must strictly advance within a run/departure/generation.
Duplicate or reordered messages cannot refresh evidence. The sender must not
re-stamp stale arm/lift/gripper measurements and call them fresh readiness. The
aggregate message cannot independently prove those underlying measurement ages.

## Prepare and consume an expiring permit

`/workbench_nav/prepare_base_departure` uses `PrepareBaseDeparture`:

```text
request: run_id, departure_id, expected_session_id, expected_hold_epoch,
         readiness_sequence, state_revision
response: accepted, reason, permit_id, session_id, hold_epoch, valid_for_s
```

Preparation requires the matching current hold, authority NONE, measured stopped
and zero-output dwell, fresh valid LiDAR, and matching live readiness. It does not
release the hold or grant motion. It binds the observed readiness sequence and
semantic contents and stores a fixed expiration on the server's steady clock.
The same request returns the same permit with its remaining lifetime; retries
and later healthy heartbeat sequence/stamp updates never extend expiration.
Changed semantic evidence invalidates it. A departure ID cannot be reused to
renew an expired/consumed/invalidated operation.

An identical heartbeat can arrive while the prepare RPC is in flight. Initial
preparation accepts its requested sequence only if the server actually observed
it recently and its complete semantic binding still matches the current fresh
readiness. A bounded 128-entry history is cleared on invalid/changed readiness or
any stop generation. An arbitrary lower sequence, expired observation, or a
sequence from before changed-and-restored readiness is rejected. This tolerance
does not change the permit's fixed expiry or its exact idempotent request identity.

`/workbench_nav/release_base_hold` uses `ReleaseBaseHold`:

```text
request: run_id, departure_id, expected_session_id, expected_hold_epoch, permit_id
response: accepted, reason, session_id, hold_epoch
```

Under the same arbiter lock, release verifies identity and fixed expiration,
rechecks current held/stopped/zero evidence and readiness, consumes the permit
once, clears both command caches and leaves authority NONE with zero output.
It does not itself grant NAVIGATION or DOCKING. A later explicit authority grant
is required. Duplicate consumption is rejected; an acknowledgement is not a
reusable capability. A new stop/hold/fault invalidates outstanding permits even
if a release service worker arrives late.

The caller's release-call wall budget must exceed the **maximum configured server
permit lifetime plus a transport margin**. The server permits at most 2 seconds;
use at least a 3-second call budget when relying on that public maximum (1 second
transport margin). A worker arriving after this caller timeout encounters server
expiry even when a newer stop message is delayed. Do not extend a task deadline
to finish the RPC. On cancellation, stop local dispatch and assert a new hold
immediately. Unknown release outcomes require observation of authority and hold
generation; they do not authorize blind release retries or later motion.

After release, the lease continuously checks fresh readiness. Nonready or changed
semantic evidence faults immediately on receipt; missing/stale evidence faults
on the next steady output tick. Fault output stays zero. Lease expiry does not
depend on ROS time progressing. Source selection, command freshness, LiDAR
direction checks, transition zero and measured handover checks remain active.

## Configuration

Authority configuration uses `schema_version: 3`. In addition to the existing
limits, freshness and handover mappings, it requires explicit `motion` and
`pose_feedback` mappings. The installed [authority configuration](../config/authority.yaml)
shows their exact schema. Earlier authority files must be migrated; the workflow
recipe remains version 2. Rebuild the ROS messages and consumers together: the
authority state now carries canonical signed motion evidence for both producers.
There is no ROS switch to bypass measured progress.

`motion` supplies fixed measured-speed leads and the supported minimum turning
radius. Normal source-time interpolation belongs to each command producer, before
its collision checks. The final arbiter performs measured-progress contraction;
it does not claim a symmetric final-publication slew bound. `pose_feedback` supplies frames, valid original
sample intervals, discontinuity bounds and fixed-transform/quaternion tolerances.
See [measured command limiting](motion_limits.md#measured-progress-at-the-final-command-boundary)
for their semantics and physical qualification boundary. An optional exact
`departure` mapping can override these defaults:

```yaml
departure:
  departure_permit_lifetime_s: 0.50
  travel_readiness_receipt_s: 0.25
  travel_readiness_header_s: 0.35
```

All durations must be finite and positive. Permit lifetime is bounded at 2 seconds;
each readiness age bound is at most 5 seconds. Header future skew uses existing
`maximum_header_future_skew_s`. These are software bounds, not qualified physical
posture/retention settings. Profile selection and physical qualification remain
the adapter's responsibility.

## Fault reset is separate from ordinary stopping

`SetBaseAuthority(NONE)` now preserves FAULT and its original reason. The mission
port's ordinary relinquish/cleanup continues using NONE; it must not call reset.

`/workbench_nav/reset_base_fault` uses `ResetBaseFault`:

```text
request: expected_session_id, expected_hold_epoch, reason
response: accepted, state, reason
```

Explicit operator/qualification reset checks the current generation and existing
recovery predicates: recovered LiDAR including the original blocked direction,
and fresh stopped odometry dwell. With fenced commands it also requires revoked
grants and a zero arbiter output generated within two output periods (0.10 s).
The ROS arbiter selects these fenced predicates in its constructor, including a
fault before the first source grant; there is no ROS parameter to bypass them.
A revoked producer cannot supply fresh authorized heartbeats: every old envelope,
including zero, is rejected. Revocation plus measured stopping replaces the old
unfenced source-heartbeat requirement; simulation must not fabricate zero source
packets to pass reset. The ROS-free legacy state-machine path retains its original
source-heartbeat predicate. Successful reset invalidates old
departure intent and clears the fault, while preserving manipulation hold. It
does not restore motion or clear the separate requirement for guarded departure.
The simulation acceptance `ResetController` is an explicit reset client and uses
this service with the currently observed generation. The workflow watchdog and
mission cleanup are not reset clients.

## Validation

Host checks from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
uv run --no-sync --extra sim pytest -p no:cacheprovider -q \
  tests/test_workbench_authority.py tests/test_workbench_package_contract.py \
  tests/test_workbench_launch_contract.py tests/test_workbench_acceptance.py
```

Tests exercise permit expiry/reuse, stale session/epoch, stop/fault races, invalid
and reordered readiness, live lease loss, explicit reset, and production ROS
callbacks through transport doubles. They do not qualify a live ROS graph,
physical travel or payload retention. Build all changed
ROS message consumers together before running the updated protocol; its message
MD5 changes are intentionally incompatible with already running old consumers.

## Per-operation command authorization

`/workbench_nav/set_authority` uses the incremented `SetBaseAuthority` protocol:

```text
request: requested_state, expected_session_id, source_goal_id
response: accepted, state, reason, session_id, grant_id, source_goal_id
```

Read the current boot session from `BaseAuthorityState`. The mission port creates
one unique source operation ID and retries that same request until the handover
completes. The accepted grant is bound to that session, source topic and operation.
It cannot be reused after stop, hold, fault, shutdown or a different source grant.
A restart creates a new session. A fresh operation requires fresh stopped odometry
dwell, clear valid LiDAR and its released workflow leg's live readiness lease. Repeating a current request does not replace its grant.

Both internal command topics carry `AuthorizedBaseCommand`:

```text
header, session_id, grant_id, source_goal_id, sequence, twist
```

The topic determines NAVIGATION or DOCKING. The original ROS source timestamp is
captured before computation. A positive uint64 sequence advances per grant.
Identity mismatch, reused sequence, source-time regression, future/stale time,
nonfinite or unsupported Twist fields never refresh command evidence. Header age
is bounded separately at 0.25 s by the default `AuthorityConfig.command_header_s`;
the same check continues while cached output is considered. Receipt freshness uses
the receiver's own steady clock. No cross-process steady timestamp is transmitted.
Only the arbiter publishes autonomous vendor `/cmd_vel`, still `geometry_msgs/Twist`.

`RangerLocalPlanner` and `DockingPort` produce envelopes at computation, retaining
an immutable captured binding. They do not borrow a newer authority-state topic's
grant to relabel old work. The move_base-required standard Twist output goes to
`/workbench_nav/diagnostics/move_base_cmd_vel`, outside the actuator route.

The sole controlled intra-mission source transition is NAVIGATION to DOCKING.
The mission first observes its owned move_base goal succeeded, then measures base
settling before tag acquisition and handover. NAVIGATION remains selected with its
terminal zero during these phases. A stale cached **zero** remains zero; a stale
nonzero faults. The arbiter requires that navigation zero plus fresh stopped dwell
before replacing its grant and clearing commands. It preserves the same departure
lease and hold epoch for this one leg. The new DOCKING grant initially reports
STOPPING_FOR_HANDOVER: the docking producer supplies its newly bound zero heartbeat,
and the existing measured handover gate completes before motion. Generic NONE,
cancellation, fault or a new hold still revoke the lease.

## Binding a navigation plan to its operation

`/workbench_nav/bind_navigation_source` uses `BindBaseCommandSource`:

```text
request: session_id, grant_id, source_goal_id, expected_goal (PoseStamped)
response: accepted, reason
```

The port confirms any previously owned move_base goal has an observed terminal
status (LOST is not confirmation), binds, then sends the exact stamped map-frame
goal. The binding mutex drains prior computation/publication, clears the old plan
and resets sequence. It emits zero and cannot compute motion until a plan with the
matching endpoint and **original** goal timestamp is installed. A new binding's
nonzero timestamp must strictly increase across the plugin process, even across
arbiter sessions; an exact repeated binding is idempotent. Clock rewind or a
same-time new goal therefore fails closed rather than reusing a plan identity.

`nav_core::setPlan` has no action goal ID. Endpoint geometry alone is insufficient
for a same-destination revisit. The original goal timestamp is the plan nonce:
RangerGlobalPlanner preserves it on the exact endpoint, including delayed results.
After validating identity, the local planner normalizes only its private control
copy to latest-transform time so a long trip does not exhaust map-to-odom TF history.
The stored authorization endpoint keeps its original timestamp.

Stock move_base can deliver a pending plan from an earlier operation after a
new binding. Under the same command-gate mutex, `DeliverPlan` distinguishes
`Installed`, `SupersededDelivery` and `Rejected`. Only a structurally valid
map-frame endpoint with a nonzero nonce strictly older than the active binding
qualifies for discard. The ROS `setPlan` adapter acknowledges that delivery so
it cannot abort the newer operation merely by surfacing an old buffer. It does
not install the old path or change readiness, goal state, plan receipt,
controller/limiter history, progress guard, pose snapshot or command identity.
The existing boolean `AcceptPlan` API still returns true only for installation.

Before a matching current plan arrives, completion remains false and command
computation returns false with an owned zero. An installed current plan survives
later superseded deliveries. Missing binding, wrong frame, malformed endpoint,
zero/future nonce and equal-nonce endpoint mismatch still reject; empty input
still invalidates the current plan. Waiting and discarded deliveries cannot
renew the original progress allowance or the existing finite controller patience
and task budgets. A matching plan arriving after progress expiry cannot revive
the operation. No old nonce or geometry is relabeled as current.

The tested Noetic image contains move_base 1.17.3 and tf2 0.7.10. Their
[goalToGlobalFrame implementation](https://github.com/ros-planning/navigation/blob/1.17.3/move_base/src/move_base.cpp)
transforms the original stamped input; the
[same-frame transform](https://github.com/ros/geometry2/blob/0.7.10/tf2/src/buffer_core.cpp)
preserves a nonzero requested timestamp. An installed-tf2 test verifies map-frame
stamp preservation. Preserve this dependency behavior when deploying or upgrading;
never recover a lost nonce by assigning the current binding to an incoming plan.

## Simulation relay and qualification boundary

The simulation command guard and fault overlay forward the original envelope
unchanged exactly once per callback. They do not repeat cached commands, invent
sequences, restamp delayed input, or emit producer zeros on shutdown. Expiry is
owned by the onboard arbiter. The simulation-only authority profile extends command
wall receipt leases to 2 s for slow physics, while the original source-time limit
remains 0.25 s. Stopping the relay or pausing the source therefore cannot sustain a
command indefinitely. The retired ROS-free tuple repeater remains a diagnostic
utility only and is not on the command route.

Focused regression coverage includes delayed prior-operation packets, wrong source
or goal, replay, restart, source expiry despite recent receipt, stopped source
handover, producer bind-before-dispatch, old computation and same-destination plan
rejection, and identity-preserving relay callbacks. Run the C++ gate test and Catkin
planner GTest together with `tests/test_workbench_command_producers.py`,
`tests/test_workbench_command_guard.py` and `tests/test_workbench_fault_overlay.py`.
These checks prove software behavior; physical stopping and loaded travel remain
separate qualification gates.
