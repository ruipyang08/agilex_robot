# grasp_test_tube

This onboard ROS1 task performs the qualified tube grasp using a frozen task
configuration and requested cap color. The current task starts from a confirmed
held base arrival, with the expected fixture installation, a verified empty
gripper, and the configured centered arm/lift preparation envelope. Required
camera, detector, arm, lift and gripper interfaces are admitted before execution.
An entry condition is not a substitute for continuous base and cancellation
supervision during arm motion.

`GraspEffectAdapter` translates the grasp package's task-neutral `TubeEffectSink`
into the shared `RobotStateStore`. Its constructor receives the state store,
run/task/operation IDs, container profile and expected revision. Create it as a
`shared_ptr`, pass `Sink()` to `RunTubeMissionMonitored`, then call `Finish()` on
the returned outcome. The sink keeps its adapter and store alive for a worker
that is still stopping. Every update checks the expected revision; a rejected
sink stops the mission before another command.

One color-selected tube has a run-local object ID `<operation>/selected-tube`.
This is an operation identity, not a registered physical serial number. The
adapter does not invent an exact rack slot from a color selection. Future tasks
can use registered objects through the same shared object-reference contract.

| Observed boundary | Shared physical state |
|---|---|
| Verified open, or verified empty insertion aperture | Gripper empty |
| Transfer admitted before close | Pending transfer to gripper; travel blocked |
| Fresh stable caught feedback | Holding; captured in fixture |
| Measured extraction plus retained feedback | Holding; extracted and safe clear |
| Qualified empty missed-pick return with measured endpoint | Empty and safe clear |
| Later retained feedback | Refresh custody without inventing new clearance |
| Lost or stale retention | Current custody/clearance unknown; preserve last confirmed custody and pending provenance |

Effect IDs are `<operation>/effect/<sequence>`. `Finish()` references the IDs
already applied by the sink and does not apply them again. Capture followed by
failed extraction retains caught-in-fixture custody. Failed return retains prior
verified extraction clearance, but travel still requires its own current arm,
lift, retention and base qualification. Late cancellation changes completion
status without reverting physical history. Reporting after a later release
cannot restore earlier held custody.

Only a search/target-absent failure with a known empty gripper, accepted effects
and joined worker is classified as ordinary. This classifies the cause and
does not assert stopping. Continuation additionally requires the runner's
qualified stop hook, fresh empty custody and verified clearance reconciliation.
Other failures remain terminal. A joined worker alone is not a stop
acknowledgement. A blocked worker requires process exit and external watchdog
stop ownership. Generic cleanup never opens a held or uncertain gripper.

The returned read-only `TubeLiveObserver` remains usable after the mission.
Subsequent travel tasks must inspect current stamped observations; a grasp's
final payload enum is historical evidence and cannot independently authorize
navigation.

The optional known-empty missed-pick return also requires an injected synchronous
held-base predicate. The monitored runner keeps this separate from its generic
cancellation predicate; absent proof disables recovery. It rechecks the current
held lease after the empty effect and immediately before recovery motion.

## Configuration and current host binding

The modular form is `{id: pick, type: grasp_test_tube, config: taskA.yaml,
cap_color: green}`. The legacy form uses `fixture_profile` instead of `config`.
Both are admitted before any task starts; see [configuration](../configuration.md).
A selected installation must match the planned preceding station. A potential
held payload from an earlier continuing grasp prevents another grasp during
admission, even if a particular execution might miss its target.

The host creates `GraspEffectAdapter(state, run_id, task_id, operation_id,
container_profile, expected_revision)` and passes its sink into:

```cpp
agxcore::RunTubeMissionMonitored(node, fixture, color, keep_running,
                               frozen_config, effect_sink, options, held_base);
```

`held_base` defaults absent and is separate from generic cancellation. The current
host supplies its synchronous measured held-base guard in both roles and keeps a
shared owner alive through worker exit. It retains the independently qualified
navigation observer across the grasp, rather than replacing its stop thresholds
with generic mission defaults. Effects reach the state store before a later
cancellation/result boundary; `Finish` only packages their existing IDs.

The concrete software binding is implemented and ROS/C++ contract tested. New
retained-tube navigation and safe missed-pick continuation physical qualification
remain pending; hardware requires separate evidence.

## Optional passive effect observations

`GraspEffectAdapter` offers an optional diagnostic observation journal.
It is disabled by default. The workflow host can enable capture and publish
original detached rows only when its strictly typed private
`~publish_grasp_effect_observations` boolean is true for execution. Check-only
never enables capture or starts its timer; no launch or acceptance flag enables
this by default. See [configuration](../configuration.md).
`EnableObservationJournal()` succeeds once before the first effect delivery
attempt, including an attempt whose original control allocation throws.
`TakeObservations()` returns detached rows and clears their queue occupancy only.
It never refreshes store state, evidence clocks, sequence numbers or the sticky
lost-evidence flag. `TryTakeObservations()` offers the same detached batch after
a nonblocking lock attempt. Its `nullopt` means acquisition did not succeed,
including a possible spurious failure; it is not an acquired empty batch or
evidence of a specific lock owner. A failed attempt changes no rows, sequence or
gap. An acquired empty batch still reports the current sequence and sticky gap.
Unexpected lock errors propagate to the optional caller. Both drain methods
share the same caller-locked move/reset operation and perform no Snapshot,
clock read, callback, logging or formatting. None of these methods changes the
sink or `Finish()` contract, and this API does not activate a consumer or promise
a realtime latency bound.

A row retains the original bound run/task/operation/object identity and TubeEffect,
the actual optional `StateUpdate`, returned acceptance bool, a separately read
store revision/gripper/transport/selected-object projection, and the matching
current effect record when one was appended. Snapshot happens after application
and may already differ in revision or freshness. Its values never replace the
original update. A replay, early adapter rejection or MeasuredStop has no store
application; returning true does not manufacture an Applied result. A real store
Duplicate update does not create a new effect record. Same-ID conflicts select
the actual Conflict revision/evidence rather than an earlier Applied record.
Ordinary Retained refreshes are excluded from this compact journal, while their
original effects continue to update the store normally.

At most 64 queued rows are retained, preserving the earliest records on overflow.
Every string retained in a row, including nested evidence and holder references,
is limited to 512 bytes without truncation. Overflow, oversized identity, missing
required record or optional capture failure sets a sticky `gap`; drains cannot
clear it or reset the local sequence. Original physical evidence, including
rejected nonfinite values, is never repaired. A gap makes diagnostic evidence
incomplete and does not reject or alter the control effect. Capture exceptions
are contained; exceptions from the original control operation retain their
existing behavior. No callback, serialization, publishing or diagnostic I/O runs
inside the adapter/store locks.

These bounds limit retained output, not capture latency or temporary allocation:
the existing `RobotStateStore::Snapshot()` copies its complete objects/effects
history under the store lock before projection. No hard realtime guarantee is
claimed. Enabled capture and transport cost validation and physical retention-loss
admission remain separate work. A journal row alone proves no native tube identity,
fixture clearance, detach, physical separation or hardware retention behavior.

### Detached observation JSON

The source-private `detail::EncodeGraspEffectObservation` helper encodes a
caller-owned detached row into one strict JSON object. It is compiled into the
workflow library and used by the optional host pump after a successful detached
drain. Capture and transport remain disabled by default. The private header is
not an installed consumer API. The caller must not mutate the row concurrently.

The schema has `schema_version: 1` and `kind: "grasp_effect_observation"`.
Run/task/operation identity comes from the row; `invocation_id` is its run ID,
`/`, and task ID. The operation must equal that invocation plus `/grasp`.
Source event, optional actual application, acceptance, separate snapshot fields
and optional matched effect retain their own values. Every field in the detached
graph is emitted, including explicit null optionals. Encoding does not infer an
application, repair custody or validate physical consistency. It reads no store,
adapter, clocks or active workflow context.

Enums retain both their exact underlying integer and enumerator name; an unknown
integer has a null name. Unsigned revisions/sequences retain their full 64-bit
integer value. Finite evidence doubles round-trip through binary64 decimal text,
including signed zero. Nonfinite evidence uses a tagged object with
`classification` (`nan`, `positive_infinity` or `negative_infinity`) and the exact
16-digit lowercase `ieee754_bits`; it never emits bare NaN/Infinity. These tags
preserve rejected evidence and do not make it valid for control.

Retained strings must be valid UTF-8 and at most 512 bytes. Valid bytes, embedded
NUL and JSON escapes are preserved without truncation or Unicode normalization.
Run and task must be nonempty; their coupled operation constraint means they
cannot independently reach 512 bytes in a valid envelope. Invalid strings or
identity throw `std::invalid_argument` with a fixed bounded reason; allocation
and formatting failures propagate. Output stays private until the complete
string is returned, so a failure exposes no partial record and changes no state.

The native codec fixture combines authentic adapter/store rows with explicitly
synthetic fidelity vectors. Its vector-output mode is checked by the offboard
strict parser, including original-context delivery to `WorkflowTrace`. Native
execution alone does not establish independent parsed-field fidelity. Encoding
alone proves neither delivery nor completeness. The optional host transport
contains failures and retains the failed original sequence as an evidence gap.
Physical retention-loss qualification remains separate.

The isolated `grasp_host_binding_ros` characterization fixture calls the actual
`Workflow::Grasp`, monitored runner, effect adapter and state store while replacing
only the grasp leaf's out-of-line methods with labeled test evidence. It covers
constructor and post-join effects, joined-worker versus unfinished-stop outcomes,
and runner terminal ordering with a fake stop-hook boundary. Those four original
cases keep transport disabled. Additional opt-in cases exercise the same host,
monitor, adapter and store with journal publication, contention and error barriers.
The isolated unjoined case runs the unchanged monitor's 15-second stop timeout
in a child process and exits without destroying its still-running worker. Its
historical fake-leaf label `leaf-post-join-outcome` identifies an Outcome effect;
on that detached path the label is not proof of joining. No fixture uses a
physical robot, simulator or MoveIt execution. Check the native link map when
modifying the fixture: its executable must supply the test leaf methods without
selecting the production mission object. These characterizations do not qualify
physical grasp, retention, stop performance or travel.

### Optional host transport and evidence completeness

After complete recipe/interface admission, the host prepares fixed slots for
admitted grasp tasks. Each has its original recipe index and run/task/operation
identity. At task entry, the actual new adapter is enabled before its sink is
exposed to the monitor, and its ownership is published once through an atomic
ready fact. A later task cannot relabel or replace that slot's producer. Optional
preparation/binding failures leave the original task and effect path unchanged;
unrepresentable diagnostic identity is reported by recipe index in the local
summary only, never truncated or published with another task's identity.

A separate 0.1-second wall timer holds only a weak Workflow reference. Each
callback takes a nonwaiting claim, fairly selects at most one eligible slot and
makes one `TryTakeObservations()` attempt. It encodes and publishes at most 64
detached rows directly on `/workbench_workflow/events`, with their original
identity and evidence. It does not use the ordinary active-task Event formatter,
read a new store snapshot or substitute a current timestamp. Busy or spurious
acquisition failure changes no observed sequence/gap metadata and retries only
on a later callback. Encoding/publication holds no adapter, store or control lock.
The claim excludes concurrent pumps and optional metadata snapshots; no control
writer waits for it. ROS publication/allocation costs still require measurement.

The host records actual monitor return, worker join, exit requirement, completed
adapter Finish and runner terminal notification as separate monotonic facts.
A final acquisition requires returned, joined, Finish-complete and terminal facts
before the acquisition. Completion during a normal batch needs another callback's
acquisition. A successful final acquisition, including empty, closes draining
once; an unjoined or unknown producer remains incomplete. A joined producer with
`exit_required` can close its journal but still has the original terminal stop
failure. A never-started task becomes `not_started` only after Run returns or
its admitted execution path exits exceptionally; that fact does not imply a
producer returned or joined.

Each row's local publish return is counted. An encoding or publication exception
sets a sticky transport gap with the original failed sequence, continues later
rows and never requeues the detached row. Drain errors retain unknown acquisition
metadata. Status events use strict `schema_version: 1` and
`kind: "grasp_effect_journal_status"`, the original envelope and fixed producer,
drain, transport and availability groups. Unknown join/exit and sequence/gap
facts are explicit nulls. Counters saturate with an overflow flag. Status is
attempted only after changed facts/accounting, and terminal status once; failed
status attempts remain in local metadata and are not retried solely for failure.
A status cannot certify its own future publication success.

The report's optional `grasp_effect_observations` member records requested/active
state, Run-return/preparation/pump facts and a single nonwaiting metadata snapshot.
If the claim is busy it reports unavailable/in-flight instead of reading mutable
counters. Successful snapshots retain terminal-status attempted/local-return
flags separately from the wire status. Formatting failure yields null without
changing the task result. The existing 200 ms reporting grace and `_Exit` policy
are unchanged: there is no synchronous final drain, flush, handshake or diagnostic
join. Stop, cleanup, guards, readiness and heartbeat perform no journal work.
Normal timer/spinner destruction after safe terminal work can wait for an
existing callback; it does not prove delivery.

Local publish return is not subscriber acknowledgement. Capture/transport gaps,
missing sequences or final status, failed/in-flight summary, and an unclosed
producer leave evidence incomplete. These diagnostic facts do not authorize
motion, alter custody, recover a failure or establish physical retention loss.
