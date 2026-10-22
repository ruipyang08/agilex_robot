# Measured robot state and result metadata

`agx_robot_workflow/robot_state.hpp` is an installed, ROS-independent support
API implemented by `agx_robot_task_runtime`. It stores verified physical effects in
memory. It does not subscribe to ROS, actuate anything, persist inventory or
reconstruct state from task success. The ROS workflow host connects grasp effect
sinks and independent live observations to this store, and uses its current
transport view to guard preparation and travel. The store itself remains free of
ROS and actuator dependencies.

## Ownership and clocks

One `RobotStateStore` owns writes. Each operation locks the store, validates its
revision and updates related views atomically. `Snapshot()` returns a detached
value; changing a copy cannot modify the store, and later writes cannot change an
earlier snapshot. Tasks should receive snapshots or a narrow adapter, not mutable
storage. The store is noncopyable because it owns its synchronization primitive.

```cpp
using Clock = RobotStateStore::Clock;  // std::function<double()>
RobotStateStore store(source_clock, steady_clock, 1.0);
RobotStateSnapshot snapshot = store.Snapshot();
```

Inject `source_clock` in the same domain as ROS message stamps, including simulated
ROS time, and `steady_clock` in the same monotonic seconds domain used when the
adapter receives a message. These callbacks must be nonblocking, must not reenter
the store and must be safe for concurrent callers. The age limit must be finite
and positive. Both clock ages must be finite, nonnegative and at most that limit.
Future stamps, time moving backwards, nonfinite stamps/clocks and old evidence
cannot establish readiness. The store does not convert between clock domains.
Reading does not refresh timestamps or increment revisions. Adapter code must not
replace an old source stamp with its current time.

Startup has no objects/effects, unknown gripper/load and no travel clearance.
Recreating the store produces the same unknown startup state. There is no database,
automatic restart recovery, remembered inventory or automatic conflict reset.

## Public values

| Value | Fields and meaning |
| --- | --- |
| `ObjectRef` | `id`, `ObjectIdentity::{Registered, RunLocal}`, `container_profile`; a color is not an identity. ID and profile must be nonempty and a known ID cannot change identity/profile. |
| `HolderRef` | `HolderKind::Gripper` with empty fixture/slot fields, or `HolderKind::FixtureSlot` with nonempty `fixture_instance` and exact `slot`, such as `rack-1` and `A2`. |
| `Evidence` | `source`, `source_stamp_s`, `receipt_steady_s`, `run_id`, `task_id`, `operation_id`; source/run/task/operation identity is required. |
| `PendingTransfer` | Operation ID, object, optional source and destination references. Missing source/destination means unknown, never an invented slot. |
| `ObjectLocation` | Object, `last_confirmed_holder`/`last_confirmed_evidence`, `LocationCertainty::{Unknown, Confirmed, Uncertain}`, `current_evidence`, optional `pending`, `revision`, computed `fresh`. Historical confirmation remains when present location becomes uncertain. |
| `GripperView` | `GripperOccupancy::{Unknown, Empty, Holding}`, optional object and evidence, revision and computed freshness. Only `Holding` has a current object. |
| `TransportView` | `TransportLoad::{Unknown, Empty, SingleHeld, Unsupported}`, independent `TravelEligibility::{Unknown, CapturedInFixture, SafeClear, Unsupported}`, object, custody/clearance evidence, pending/uncertain/ready flags and revision. |
| `RobotStateSnapshot` | Object map keyed by identity, gripper/transport views, effect journal, overall revision and latched `halted`. |

The transport load is the currently supported single-gripper load projection. It
is not proof of an unloaded rover or an inventory of rover-mounted storage.
Adapters for any additional load/fixture must represent it as unsupported until a
qualified model is installed. Exact fixture-slot references establish a future
identity boundary; they do not implement rack inventory or placement algorithms.

`Confirmed` records what the evidence established. It is usable as current
confirmation only when `fresh` is also true. An object's historical holder must
never be used as current custody when its certainty is `Uncertain`.
Transport readiness additionally requires fresh gripper and clearance evidence,
`SafeClear`, no pending transfer, no unresolved custody and no halt latch.
Captured-in-fixture is custody, not safe clearance for travel. Readiness is one
state prerequisite, not a base-motion permit or a hardware qualification.

`RobotStateSnapshot::revision` is the write revision used by `BeginTransfer` and
`ApplyEffect`. `TransportView::revision` instead changes when load/object,
clearance eligibility, pending state, uncertainty or readiness changes during a
write. Fresh repeated custody/clearance observations update their actual source
and receipt evidence without advancing that semantic transport revision. A
custody-only refresh with `Unknown` eligibility preserves independent clearance
evidence and its original age; use an explicit `TravelEligibility` effect with
`Unknown` to invalidate clearance. A fresh observation after evidence expires
cannot reuse the previous ready revision. Reads still never mutate either
revision; a stale read reports `ready=false`, so readiness consumers must check
freshness continuously as well as binding the semantic revision.

## Transfers and verified effects

```cpp
StateUpdate BeginTransfer(StateRevision expected_revision,
                          const std::string& operation_id,
                          const ObjectRef& object,
                          std::optional<HolderRef> source,
                          std::optional<HolderRef> destination);
StateUpdate ApplyEffect(const PhysicalEffect& effect,
                        const Evidence& evidence,
                        const std::string& operation_id,
                        StateRevision expected_revision);
```

`BeginTransfer` records intent as pending before custody changes. It rejects stale
revisions, invalid references, another pending transfer and a halted store without
mutating state. A gripper source must be freshly observed holding that object; a
gripper destination must be freshly observed empty. A supplied known source must
match fresh confirmed custody. Source and destination cannot be identical or both
unknown. Beginning a transfer invalidates previous clearance evidence.

`PhysicalEffect` contains an effect `id`, `kind`, optional `object`/`holder` and
`eligibility`. Its kinds are:

- `GripperEmpty`: measured empty gripper; any previously held object becomes
  uncertain. This does not invent a destination for a lost object.
- `GripperHolding`: measured custody of the named object. A capture can supply
  `CapturedInFixture`; clearance can be verified separately.
- `GripperUncertain`: retention/custody is no longer confirmed. The current
  gripper becomes unknown and clearance becomes unknown. Known held, explicitly
  named and pending-transfer objects become uncertain; their last confirmed
  holder/evidence and pending transfer remain intact. No holder or travel
  eligibility may be supplied. This never pretends an uncertain transfer finished.
- `Released`: measured release of the named object; the gripper becomes empty.
  An optional holder requires verified support there. Without it, the object's
  current location becomes uncertain even if the planned destination was known.
- `ObjectAtHolder`: observed object at an exact holder. A contradictory change
  from confirmed custody requires a matching pending transfer; it cannot silently
  replace a current holder. A fixture-holder observation cannot carry travel
  eligibility, and an unrelated fixture observation cannot update gripper clearance.
- `TravelEligibility`: independent measured clearance or unsupported-load
  evidence. Its optional object must match current gripper occupancy/identity.

Except for unknown release support, effects completing a pending transfer must
match its object, operation and destination. There is no task-failure rollback.
Unfinished transfers remain pending until physical evidence resolves them.
Ordinary failures, cancellation and deadlines do not themselves apply an effect.
Callers must report real effects before returning a later failure/cancellation.

`StateUpdate` contains `UpdateStatus::{Applied, Duplicate, Rejected, Conflict}`,
the current revision, `halt` and detail. Effects require fresh, finite, attributed
evidence. A physical revision conflict, invalid evidence, conflicting custody or
reuse of an effect ID with different content records the incoming evidence in the
journal, marks affected current custody uncertain, makes gripper state unknown
and latches `halted`. The caller must stop; the store does not command a stop.
Historical confirmation remains available for diagnosis, never rollback.

Freshness alone is insufficient: source and monotonic receipt stamps must not go
backwards relative to accepted evidence for each affected object, gripper or
clearance view. These per-view high-water marks survive conflicting observations.
A new effect ID and current expected revision cannot make a delayed capture
overwrite a later release. Transfer operation IDs cannot be reused by
`BeginTransfer`, and a completed transfer's operation cannot modify a later
custody operation. Pending-operation matching still applies. Continuous evidence
for unchanged current custody may keep its current operation ID.

An exact replay of effect ID, effect content, evidence and operation is
`Duplicate`. It does not mutate state or refresh evidence, including after the
original evidence has aged out. The replay's expected revision is ignored.
Further verified observations may record physical reality after a conflict, but
they do not clear the halt latch or authorize a new transfer. Explicit recovery
and reconciliation are deferred.

## Example physical histories

1. Observe empty, begin transfer of run-local `tube-1` into the gripper, then apply
   capture evidence. The pending transfer clears and custody is confirmed. If
   extraction or return subsequently fails, the captured object stays in the
   gripper view; `CapturedInFixture` still prevents departure.
2. Independently observe safe clearance for that same object. With fresh custody
   and no conflict/pending operation, the single-held transport view is ready.
3. Begin gripper-to-`rack-1/A2` transfer. Apply release only after it occurs. If
   support at `A2` was measured, current location is confirmed there. If support
   was not measured, omit the holder: the gripper is confirmed empty, the object
   is uncertain, and its old held record is historical. Later withdrawal failure
   cannot restore held custody in either case.

## Task adapter state binding

New tasks use the invocation's `TaskRuntimePorts` rather than creating another
store. Take `snapshot().revision` for a proposed update, and use the pinned
`operation_id` with `begin_transfer` before physical custody change.
Populate `Evidence` with the measured source timestamp, the original steady
receipt timestamp and the ports' run/task/operation IDs. Never replace an old
measurement's timestamp with the time of the state write.

Call `apply_effect` when the physical event is verified, inspect its
`StateUpdate`, and attach accepted effect IDs/current revision to the returned
`StepResult`. Rejected/conflicting updates cannot be reported as reconciled
success. Do not retry a conflict by blindly reading a newer revision; physical
custody and operation ownership must be reconciled first.

For example, a placement task begins transfer from the currently held tube to a
known fixture slot. Once release is observed, apply `Released` immediately.
Include the destination holder only if support there was measured; otherwise
record uncertain location. If withdrawal then fails or cancellation arrives,
return that failure with the release effect reference. The gripper remains
empty and historical holding is not restored. Clearance still needs its own
fresh evidence before travel.

The active binding deliberately permits verified effects after cancellation,
but rejects new transfer intent. It rejects updates after expiry or with a
different run/task/operation identity. Finish effect reporting and worker drain
before the next task is activated; do not bind delayed work to a later task.
The in-memory store has no restart recovery or automatic physical reconciliation.

## Result support types

`runner.hpp` exports the following classifications:

```cpp
enum class FailureKind { None, Ordinary, Cancelled, Deadline, Fault, UnconfirmedStop };
enum class StopState { NotConfirmed, Confirmed };
enum class RunCompletion { Succeeded, CompletedWithFailures, Failed, Cancelled };
enum class FailurePolicy { Stop, Continue };
```

`StepResult` retains its first two aggregate fields, `ok` and `detail`, then adds
`physical_effect_ids`, `state_revision`, `failure` and `stop`. Legacy
`StepResult{false, "detail"}` defaults to terminal `Fault` and `NotConfirmed`;
`StepResult{true, "detail"}` defaults to `None`. Classification initializes once
with the aggregate; code that subsequently assigns `ok` must set `failure` too.
`TaskResult` and `Result` append effect IDs/revision/failure/stop fields, and
`Result` additionally appends `completion` and the ordered `failed_task_ids`.

`Run` preserves applied effect IDs and state revisions from execution, bounded
stop and cleanup before checking late cancellation. `GraspEffectAdapter::Finish`
references effects already applied synchronously; it never replays them. The
ROS host writes these fields into task/run summaries. Explicit continuation
produces `CompletedWithFailures`, retains each failed task and keeps `success`
false; the complete exit contract is in [execution](execution.md).

## Validation and limits

The robot-state test covers initial unknown and measured empty/holding,
independent clearance, snapshot isolation, both clocks, stale/future/nonfinite
observations, partial task failure/cancellation, observed/unknown release,
revision/operation conflicts, conflicting holder observations, replay/ID reuse,
unsupported transport, restart and concurrent writes. CMake keeps assertions on.

From the repository root, the pure support test needs only C++17 and threads:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Wno-missing-field-initializers \
  -UNDEBUG -pthread -Iagilex_ws/src/core/agx_robot_task_runtime/include \
  -Iagilex_ws/src/core/agx_task_workflow/include \
  agilex_ws/src/core/agx_robot_task_runtime/test/robot_state_test.cpp \
  agilex_ws/src/core/agx_robot_task_runtime/src/robot_state.cpp \
  -o /tmp/workflow_robot_state_test
/tmp/workflow_robot_state_test
```

These checks do not validate ROS measurements, loaded travel, mechanical release,
hardware stopping or deployment. Runtime adapters are integrated, but software
coverage alone does not qualify a physical travel profile or a robot stop.
Simulation acceptance and real-robot qualification remain separate gates.
