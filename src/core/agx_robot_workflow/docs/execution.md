# Execution guards and stopping

The ROS-independent guards and lifecycle runner are integrated into the built-in
ROS workflow host. Navigation and grasp bind measured entry/stop hooks, shared
custody effects, continuous guards and per-task deadlines. New third-party tasks
must provide their own bounded adapters; generic metadata cannot stop hardware.

## Deadline and cancellation

`ExecutionContext` in `include/agx_task_workflow/execution_context.hpp`
receives a steady-clock function, a cancellation predicate, an optional fault
predicate and a progress callback. Its finite future deadline is fixed at construction;
a positive duration that cannot advance the representable clock value is rejected.
`Check()` latches the first observed cancellation, deadline or fault; clock regression or
nonfinite time fails as a fault. `ReportProgress()` checks before and after the
callback. Publishing progress does not extend the deadline.

The host reads private `~task_timeout_s` once before admission, accepting only a
finite positive integer or floating-point value that produces a representable
future steady deadline. Omission retains **900 seconds**. Booleans, strings,
containers, nonfinite/nonpositive values, overflow and durations below the clock's
representable resolution are rejected. Each `BeginTask` constructs one context
from that immutable duration before the task's first guard or progress event.
Preparation, readiness, release, navigation and arrival share its original
deadline; entering navigation never starts another budget. A newly admitted task
gets its own budget. This setting is per task, not per recipe or motion phase.

`run_workbench_workflow.py --task-timeout-s 1800` forwards an optional private
override. Omission sends no override. Both execution and check-only summaries
report `task_timeout_s` and `task_timeout_clock: "steady"`; an invalid setting
reports a null resolved duration and fails validation. The onboard boundary
remains authoritative, including representability at admission and at each new
task. No task YAML or physical profile field is added.

For an active task the context's cause also governs the host runtime guard and
runner cancellation binding. Raw cancellation, runtime faults and envelope faults
all enter that latch. Within one previously uninterrupted check, cancellation
takes precedence over fault (including an invalid clock), then deadline at
`now >= deadline`. A physical readiness failure observed by `CheckTravel` enters
the fault input before that check. Withdrawing otherwise healthy readiness because
the task is already interrupted does not manufacture a physical fault. A later
readiness failure, cancellation or envelope fault cannot replace an already
observed Deadline; an earlier Cancelled/Fault likewise remains first. This is
observed ordering, not reconstruction of events between checks.

Navigation requires an active, uninterrupted context before dispatch. It captures
the effective cause before cancelling the action and rechecks after retrieving
completion evidence. Stop and cleanup retain their separate existing bounds
(normally 5 seconds each), worker-drain obligations and physical effect reporting.
Extending task execution does not extend freshness, RPC or stop limits and does
not allow another task after terminal deadline failure.

A task must check at every bounded motion/wait boundary and at final completion.
The helper does not stop hardware itself, interrupt an arbitrary blocking RPC or
prove that a worker has stopped. The adapter owns cancellation and its measured
stop acknowledgement. It must not dispatch later work while a command-producing
worker could resume. ROS time is for source observations; deadlines and receipt
ages use steady wall time so pausing simulation cannot suspend stopping.

The nested navigation mission has separate phase caps in the clock passed to its
coordinator. The ROS wrapper supplies ROS/source time and polls on wall time;
the simulation's existing 1200-second coarse-navigation limit therefore remains
a source-time phase limit. It is not a new overall task deadline and is unchanged
by `~task_timeout_s`. An offboard acceptance harness may impose its own outer wall
budget; selecting a longer task budget requires accounting for that independently.

## Writing a bounded device adapter

Capture the fixed invocation ports once inside execution. Check `ports.check()`
before commands, during every wait and after receiving a result. Limit phase
waits to the earlier of their own cap and `ports.deadline_steady_s()`. Use the
same steady clock domain as the host. Never renew the task deadline on progress.
Verify the actual outcome inside `execute` before returning success; the runner
has no later generic verification callback.

A synchronous ROS1 service call can block despite a deadline checked immediately
before it. For services that can stall, use a supervised transport/worker with
bounded polling, retained ownership and a bounded drain policy, or an action
protocol with explicit goal ownership/cancellation. Cancelling the local service
link does not prove the remote handler or hardware stopped. Never detach a
command-producing worker and return success or confirmed stop.

On failure, task-owned `stop(timeout_s)` must request stopping and establish
measured acknowledgement within its own bound. `cleanup(timeout_s)` accounts for
owned workers and reports any remaining physical effects; it runs even when
stopping fails. These are failure-path hooks, not normal-success finalizers.
The runner only measures callback overruns after return and cannot preempt them.
The host's shared robot fallback stop is selected by declared actuator resources;
an external device requires its own bounded stop path.

If stopping or drain cannot be established, report terminal
`UnconfirmedStop`, prevent later dispatch and retain the relevant process-loss
protection. The production grasp host can flush its summary then exit when its
worker cannot be joined; that policy does not automatically supervise a new
equipment worker. Such a task must define independent watchdog/inhibit behavior
for abrupt host loss. Exiting a process is not a physical stop acknowledgement.

The example callback only logs a value and returns synchronously. Its no-motion
stop hook is valid for that example and must be replaced for actuator work.
The fake ROS adapter tests availability waits; they do not qualify arbitrary
blocking service callbacks or real device stopping.

## Measured base guard

`BaseGuard` in `include/agx_robot_task_runtime/base_guard.hpp` consumes a coherent
`BaseSample` copied by the ROS adapter under its observation lock. Receipt times
and the supplied steady time must use the same clock. The adapter derives
`command_zero` from every Twist component, rejecting nonfinite values, and
records watchdog readiness only from a positive live heartbeat.

`CheckStopped()` requires fresh authority/odometry/output and watchdog evidence,
valid authority and odometry source times, finite pose/velocity, authority NONE, the requested
hold state, zero output, measured stopped speeds and inactive navigation goals
with independently fresh source and receipt timestamps. Optionally it also
requires a terminal owned navigation goal identity. Combining timestamps with a minimum is not
a substitute for validating each source; a future authority/status header fails
the guard even when the odometry header is current. It checks one
sample: the caller must still establish the required stopped dwell and receive
the authority's hold acknowledgement.

`CaptureHold()` stores a measured pose and authority session/epoch. An optional
station identifies an already-verified arrival; the guard does not perform tag
docking or verify station geometry. `CheckHeld()` requires the same current hold,
fresh evidence, displacement/yaw limits and the existing sustained-motion dwell.
An isolated velocity spike may satisfy that dwell policy, but stale state,
nonzero output, displaced pose or lost hold fails immediately.

`InvalidateArrival()` removes the station authorization while retaining the
measured hold needed during departure preparation. `ArrivalMatches()` checks the
stored station and current held-state evidence; a navigation adapter must also
verify the destination's measured geometry before using a same-station shortcut.
Use one task-thread owner for the stateful held-pose/dwell guard.

## Task exit obligations

Every actuator adapter must identify how to cancel, bound the wait for stopping,
verify stopping and report physical effects that already happened. A joined
thread is not evidence of stopped hardware. A successful release followed by
failed withdrawal must not restore historical held custody. Gripper release is
never generic cleanup for a held tube.

The independent workflow watchdog retains navigation cancellation, base hold,
arm stop and lift stop paths. A blocked hold RPC cannot suppress the other paths.
The existing blocked-grasp-worker process-exit rule remains in force. Hold
assertion can safely arrive late; departure release requires its separate
generation-bound, expiring protocol and cannot reuse an unguarded detached RPC.

## Invocation guard ownership

The host clears the previous invocation's selected travel profile, envelope and
policy before the prepared `begin` callback selects its adapters. Grasp selects
its own immutable config and observer; an observation task inherits no travel
guard. Observed envelope faults latch in the host and remain faults after local
selection changes. Cancellation, custody, and unjoined-worker obligations persist.

Each module explicitly certifies its owned effects/work through
`continuation_check`; the host independently checks cancellation/faults, pending
stop workers and unresolved shared robot state. Navigation/grasp certification
still requires measured transport/held-base reconciliation. A synchronous
observation task can certify its own quiescence without a gripper observer.
A missing certificate rejects continuation. The next task still runs its own
measured entry check.

## Sequential execution and continuation

`Run` consumes each bound task's optional `begin`, `entry_check`, `execute`,
`stop`, `cleanup` and optional `continuation_check` hooks. It checks cancellation and the optional runtime guard before and
after progress callbacks and task boundaries. Full task definitions require all
three lifecycle hooks and a finite positive `stop_timeout_s`. Entry rejection is
terminal before that task's motion callback; it is not an ordinary execution
failure that may be skipped. Legacy four-field task bindings remain supported,
with false results terminal by default.

`Ports` retains its original cancellation, terminal fallback stop, phase and task
event fields, then appends:

```cpp
std::function<FailureKind()> runtime_guard;
std::function<bool(const PreparedTask&, const StepResult&)> continuation_allowed;
std::function<double()> steady_now;
std::function<void(const PreparedTask&)> task_begin;
```

The host's runtime guard returns `None` only while cancellation/deadline/fault
supervision permits execution. Robot tasks' `continuation_check` verifies that
reported effects and current measured state are fresh, supported and free of
pending transfer or unknown custody/clearance. The host's `continuation_allowed`
also rejects unresolved shared state. The runner has no robot
state store or ROS dependency and does not guess those facts from task success.
The runtime guard, task certificate and host eligibility callback are all
required for continuation; an absent
callback cannot authorize it. `steady_now` is optional and defaults to
`std::chrono::steady_clock` seconds.

`task_begin` is an optional admission-only hook. For each new iteration the
runner first checks cancellation, then calls it exactly once before checking the
new task's runtime context or publishing its phase/running event. The host sets
the active task identity and constructs its new fixed deadline there. It must not
issue motion or clear a persistent fault/cancellation latch. A cancellation,
fault or exception from this hook prevents entry and dispatch. Keeping this out
of `entry_check` prevents the previous task's identity and deadline from leaking
into the next task's initial progress/guard checks.

The pure predicate is:

```cpp
MayContinue(FailurePolicy policy, FailureKind kind, StopState stopped,
            bool reconciled, bool terminal)
```

It returns true only for `Continue`, `Ordinary`, `Confirmed`, reconciled effects
and no terminal condition. The runner also requires a complete task contract,
successful bounded stop/cleanup, positive measured-state eligibility and fresh
runtime checks. Cancel, deadline, fault, unknown/pending state, missing guards,
unconfirmed stop and next-task entry rejection prevent subsequent dispatch.
There are no retries or fault-latch resets.

Failure handling calls each task's stop and cleanup hook at most once. Cleanup
still runs if stopping throws or cancellation arrives during stopping. Physical
effect IDs and state revisions from execution, stop and cleanup are merged before
checking a late interruption, so a capture or release is retained in the result.
Stop/cleanup overrun, clock regression or invalid time cannot certify a safe exit.
The runner measures their supplied bound after they return; it cannot kill or
preempt an arbitrary C++ callback. The host must supply bounded ports and retain
the process-exit rule when a command-producing worker cannot be joined. Each hook
receives the task's bound separately; that number is not proof of wall-bounded
physical behavior.

`Ports::stop` is a terminal fallback, attempted once when the task has no stop hook
or task stop/cleanup could not confirm a safe stop. A successful per-task stop is
not called again because reporting later throws. Observer exceptions cannot skip
the physical cleanup paths. A void fallback stop never supplies a fabricated
`StopState::Confirmed` acknowledgement.

The ROS host also owns the lift `mode=-2` and base-hold RPC workers. Fresh
stationary telemetry cannot confirm a task stop while either worker remains
unjoined. `StopTask` closes ordinary RPC scheduling for that invocation. If an
earlier stop RPC is still in flight, it coalesces one final stop behind that
reply, covering a motion command accepted after the earlier stop acted. After
both final workers join, the host starts a new observation barrier and base
dwell; arm, lift and base evidence must be newer than that barrier.

A pending RPC at the stop deadline makes cleanup terminal and latches
`blocked_child_exit`. Diagnostics identify `lift_stop_rpc_pending` and
`hold_stop_rpc_pending` separately. Unstarted retries are discarded at timeout;
the terminal fallback cannot create fresh RPC workers after cleanup. Physical
effects and their state revision survive the failure. The host remains alive
through summary flush, then exits without waiting indefinitely for transport.
Worker captures contain only the copied service client and shared completion
flag, so exceptional destruction can detach a pending stop-only leaf without
accessing a destroyed workflow. `BeginTask` rejects pending or terminal workers;
only a new admitted invocation after quiescent cleanup reopens stop scheduling.

The isolated `task_workflow_stop_rpc_ros.test` holds actual ROS lift/hold replies
while publishing fresh stopped feedback and checks delayed completion and bounded
terminal exit. `tests/test_workflow_stop_rpcs.py` compiles the host's real methods
against controllable transport leaves to check final retry ordering, a fake
device's intervening motion, post-completion observation barriers, next-task
rearming, preserved effects and destruction with a late reply. These tests do
not establish physical actuator stopping performance.

Task-returned physical effects and diagnostic text are retained even when a
concurrent cancellation or runtime fault takes priority immediately afterward.
For example, a verified release followed by a failed withdrawal remains recorded
in both the state effects and the failure explanation.

## Status, watchdog and operator exit codes

An allowed ordinary failure remains `status="failed"` in task history and emits
`TASK_FAILED`. The runner then checks the next task's measured entry conditions.
It does not emit terminal `STOPPING` or `FAILED` and try to clear those latches.
The watchdog arms/keeps supervision active for `TASK_FAILED`; heartbeat loss
between tasks still stops the robot.

After all tasks finish, any continued failures produce
`RunCompletion::CompletedWithFailures`, `success=false`, `FailureKind::Ordinary`
and terminal phase `COMPLETED_WITH_FAILURES`. `Result::failed_task_ids` preserves
all failed task IDs, and task/run metadata retains physical effect references and
state revisions. The host maps this completion to exit code `3`. The operator CLI
preserves that code and reports that the workflow completed with task failures.
All-success remains exit `0`; terminal failure remains `1`; existing operator
cancellation and transport-error codes retain their meanings.

The watchdog disarms on `SUCCEEDED` or `COMPLETED_WITH_FAILURES`, while preserving
any already-latched fault. `TASK_FAILED` does not clear faults. Tasks cannot emit
these reserved workflow phases themselves.

For an integrated, qualified missed-pick → navigation scenario, the expected
history is:

| Invocation | Task result | Stop/state requirement |
| --- | --- | --- |
| Missed pick | `failed`, `Ordinary` | Confirmed stopped, observed empty, clear and no pending transfer |
| Independent navigation | `succeeded`, `None` | New entry checks pass and fresh held arrival is established |
| Whole run | `CompletedWithFailures`, `success=false`, exit `3` | Missed-pick failure remains in history |

The runner and physical adapters implement this ordering. Pure task-port tests
and isolated ROS fakes verify its software boundaries. A qualified physical
missed-pick exit followed by travel still requires the separate recorded
simulation acceptance; unit tests alone do not enable a profile.

## Validation

`test/base_guard_test.cpp` checks stale/missing/future evidence, session/epoch
changes, displacement, wrapped yaw, goal state, arrival invalidation and sustained
motion. `test/execution_context_test.cpp` checks fixed deadlines, latched causes,
clock failures and cancellation from the progress callback. Runner transition
tests also verify task identity before reporting and replacement of an expired
previous context before the next task runtime check. These are software
contract checks, not physical stopping qualification.

`test/runner_test.cpp` additionally checks the continuation truth table, legacy
behavior, failed-task history, next-entry rejection, effect preservation,
cancellation during stop/cleanup, changed eligibility/runtime guards, reporting
exceptions, reserved phases and callbacks returning after their bounds.
`tests/test_workflow_watchdog.py` exercises terminal/nonterminal supervision and
independent stop paths. `tests/test_workflow_cli_cancellation.py` checks exit code
propagation and owned-child signal forwarding, including a failed first forward.

`tests/test_workflow_deadline.py` compiles the real host deadline, readiness and
navigation methods plus its runner bindings against controllable transport leaves,
linked with the production context, composed navigation and sequential runner.
It covers preparation-budget reuse, navigation beyond 900 seconds with an explicit
longer budget, exact expiry, late fault/cancel/readiness callbacks, physical effects,
confirmed stop, missing contexts and paused source time. Its CLI tests execute the
generated shell argument block against a local fake executable. These host checks
do not run ROS or certify physical stopping. The isolated ROS test also exercises
the actual private-parameter types and check-only summary fields.

`test/task_workflow_ros.test` runs the actual ROS host against an isolated fake
graph. It admits an A → B → A recipe without grasp parameters, checks that
check-only does not replace an executing workflow, and rejects missing/disabled
profiles, unknown later task types, excessive reported control limits, rejected
holds, mismatched hold sessions, stale producer source stamps, and caught feedback
without established object identity before arm or navigation actuation. Its
`workflow_ros_travel.yaml` and `workflow_ros_effective.yaml` are synthetic test
inputs; they must never be installed as robot or simulator qualification.

Run this rostest on its own ROS master. Composed-port tests verify preparation
and release sequencing and cancellation. Successful physical preparation and
hold release require the separate rendered simulation acceptance; this fake
graph does not fabricate MoveIt motion.

## Qualification and extensions

The built-in adapters are connected; physical stop/race and multi-station travel
qualification remain pending under the implementation plan. New external equipment
must provide its own stop/effect contract; robot stopping does not undo an
instrument setting.

### Emergency stop observation

`BaseSample.watchdog_received` means a watchdog heartbeat was observed;
`watchdog_ready` separately reports permission to run. Normal stopped, held and
arrival checks require a fresh heartbeat **and** readiness, authority `NONE`, the
required hold state and zero output. Every hold, including a departure hold with
no owned navigation goal, also requires fresh inactive goal-status source and
receipt evidence. Owned terminal goal identity is additionally checked for an
arrival hold. A watchdog that withdraws readiness on
`STOPPING` must not appear to have stopped reporting its heartbeat.

Before sending emergency stop requests, the host calls
`BaseGuard::BeginEmergencyStop(steady_now, ros_now)`. This invalidates its captured
hold and arrival and starts an observation barrier. Repeated
`ObserveEmergencyStop(sample, steady_now, ros_now)` calls can confirm fresh,
measured inhibited authority `NONE` or `FAULT` with an asserted manipulation hold,
zero output, no active navigation goal, and fresh watchdog receipt even when its
readiness is false. If an owned goal is present it must also be terminal. All
required receipts and authority, odometry and goal source stamps must be after the
barrier; a pre-request stationary observation cannot confirm a later stop.

Confirmation requires advancing odometry source and receipt times across
`motion_dwell_s`, measured velocity below the stopped limits, and cumulative
position/yaw variation within the smaller of the normal guard limits and
`emergency_displacement_limit_m` / `emergency_yaw_limit_rad` (defaults 0.002 m and
0.005 rad). These are software acceptance bounds, not a claim about braking
performance or sensor accuracy. Motion, excessive drift, stale/missing evidence,
a clock regression, a changed authority session/state, a regressing hold epoch, or a monitoring gap
restarts the dwell. Monotonically increasing hold epochs from repeated emergency
inhibition do not restart this physical stop dwell; normal captured holds still
require the exact captured epoch. Re-reading one cached observation cannot complete it.

This result certifies only the observed base stop. It creates no arrival or held
base authorization, clears no runtime fault, and cannot itself permit task
continuation. Arm/lift stop confirmation and bounded stop-callback execution
remain separate host responsibilities. Any later authorized operation must pass
the normal readiness checks and explicitly capture a fresh hold.

Action-status source timestamps use the base authority's installed
`maximum_header_future_skew_s` allowance, consistently in both the shared base
checks and arm/navigation producer checks. Independent ROS clock subscribers can
receive adjacent ticks in different callback order. This allowance never changes
original stamps, increases source/receipt age limits, or permits future local
receipt times. Sensor freshness and post-command measurement barriers remain
separate requirements.
