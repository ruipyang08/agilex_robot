# Tube grasp behavior

This deployable ROS1 package owns the grasp behavior. It receives an immutable
`ResolvedTubeTaskConfig`; see [configuration](docs/configuration.md) for the
versioned domain graph and the legacy ROS parameter snapshot adapter. The task
color is operation intent. Configured metric detection uses the admitted robot
detector endpoint with `_configured` appended. The target selection endpoint is
currently `/select_tube_target_2d_configured` because the robot schema does not yet
declare a separate target selection endpoint.

The workflow-facing adapter is documented in
[grasp_test_tube](../agx_robot_workflow/docs/tasks/grasp_test_tube.md); recipe
selection is described in [workflow configuration](
../agx_robot_workflow/docs/configuration.md). The mission does not depend on
workflow state. Its public effect and observation types can be consumed by other
onboard adapters.

## Physical effects and failures

`TubeEffectSink` receives synchronous, ordered observations independent of the
workflow package. `TransferBegun` is delivered before the final close command.
The command itself does not prove capture. Fresh stamped gripper feedback must
prove caught state before `Captured`; that establishes custody while the tube
may still be inside the fixture. Successful measured extraction and retained
gripper feedback establish `ExtractedSafeClear`. A failed extraction or return
does not undo earlier physical evidence.

`RetentionUncertain` invalidates current payload knowledge when caught feedback
is lost or stale. It retains the actual source and receipt timestamps, including
stale timestamps; the receiving workflow can therefore reject stale evidence
and halt without treating dispatch time as a new device observation. “Uncaught”
alone never establishes empty. Empty requires the qualified open state, or a
verified insertion aperture before this operation has issued its final close.

`TubeMissionOutcome` includes the effect sequence, payload, clearance and sink
acceptance. Reporting an outcome does not replay effects. Sink rejection latches
stop before subsequent commands. A generic failure or stop never opens a held
gripper. The existing pre-close recovery may retreat and open only while custody
is still known to be empty.

For `TargetAbsent` or `SearchExhausted` before a transfer, a currently verified
empty gripper can return through the existing phase-one recovery planner while
the base remains held. Only a verified measured endpoint followed by another
empty observation emits `EmptySafeClear`. The original missed-pick status is
preserved; failed recovery is a motion fault. This path is never entered for
cancellation, stale/fault causes, held/unknown custody or an issued close.

## Live observation and stopping

`TubeLiveObserver` owns independent read-only subscriptions and can survive the
mission object. Its typed snapshot includes gripper state, all six arm positions
and velocities, and measured lift position, native measured velocity and stationarity. Both message source
stamp and local steady receipt time must remain fresh. Unstamped, future-dated,
nonfinite, missing-joint or incomplete velocity feedback cannot qualify stop.
The ROS observer captures both freshness clocks while the observation buffer is
locked, so a concurrent callback cannot insert a measurement newer than the
snapshot's clocks. The pure buffer retains its explicit-clock-value overload for
deterministic tests; live callers use its clock-function overload. Clock functions
must not reenter the buffer. This preserves strict rejection of future evidence.

The independent [one-shot motion window](docs/motion_window.md) retains every
reported arm/lift position extremum and absolute rate peak after joined command
completion. Its original generation/barrier, unique measurement receipts and
latched failures support bounded travel retention without changing strict stop
observations. It has no reset and grants no hardware qualification.

`ArmLiftStoppedSince()` requires two advancing, fresh arm/lift observations after
the stop request, with near-zero arm velocity and non-running zero-speed lift
feedback held for the observation settle interval. Measured positional range
must also stay within the configured arm/lift tolerances throughout that dwell;
contradictory observed speed or positional-range evidence cannot qualify stopping.
These checks do not prove zero motion below the telemetry's resolution. Defaults are a
1e-6 rad/s arm threshold, 0.002 rad arm range, 0.001 m lift range and 0.2 s
interval; these are observation rules and grant
no motion permission. A lift stop inhibit flag does not invalidate independent
measured speed feedback. Other lift faults invalidate the measurement.

`RunTubeMissionMonitored` separates worker join from measured stopping. It sends
arm/lift stop requests and bounds worker/RPC cleanup plus measured stopping to
15 s total; fresh measured evidence gets at most the final 1 s of that budget.
A blocked worker or stop RPC returns `exit_required`; the caller must
flush its report and terminate the process as documented by the runner. Joining
or accepting a stop RPC alone never sets `measured_stop_confirmed`.

Host checks compile the production journal, observation buffer and workflow
adapter without ROS. Boundary tests execute the mission's final grasp command
sequence and the actual monitored runner with injected devices. The ROS adapter
and mission must also be built in the offboard ROS1 container before deployment.

`TubeLiftObservation::velocity_mps` uses the vendor `LiftMotorMsg.backSpeed`
measurement, converting signed integer mm/s to m/s. The vendor driver derives
that field from measured motor speed; it is not a command. Differentiating the
separate integer-mm `backHeight` would turn a one-mm height tick at 60 Hz into a
false 0.06 m/s speed. Missing or nonfinite native speed invalidates the sample.
Stopped evidence still requires the non-running flag, measured zero speed and
the measured positional-range dwell above.

An identical lift measurement with the same source timestamp preserves the first
receipt time and dwell history. It cannot extend source or receipt freshness or
supply another advancing post-stop sample. Changed payloads at the same source
timestamp, regressed timestamps or receipts, and a changed source invalidate the
observation. Fresh advancing measurements are required to recover valid evidence.

The factory overload `MakeTubeLiveObserver(node, TubeObserverInterfaces,
TubeObserverConfig)` lets navigation retain a separately qualified observer
without loading grasp fixture configuration. The overload taking resolved grasp
configuration delegates to the same implementation. The monitored runner's final
optional `held_base` predicate is distinct from `keep_running`: cancellation-only
callers cannot authorize empty missed-pick recovery. Callback/state ownership must
outlive a stopping worker; the workflow captures its shared owner by value.

New carry-after-grasp and missed-pick continuation physical qualification remains
pending. Earlier grasp evidence does not qualify a new travel profile or onboard
hardware stopping.

### Mission stop RPC ownership

`RunTubeMissionMonitored` owns both mission completion and the mission's separate
lift-stop transport. `TubeGraspMission::FinishStopRequests(timeout_s)` is called
after the motion worker joins: it sends a final `mode=-2` stop, or queues one
behind an earlier stop whose reply is still pending. This covers a position
request accepted after the earlier stop acted. The final RPC must join before
new arm/lift measurements can confirm stopping. Repeated callbacks can still
publish arm-stop inhibition; they cannot schedule additional lift RPCs after
final cleanup is sealed.

The existing `worker_stop_timeout_s` is one deadline for worker joining, stop RPC
completion and subsequent observation; `measured_stop_timeout_s` only caps the
last portion. Pending transport at that deadline sets `exit_required`, even if
the motion worker joined or old feedback looked stationary. The caller must
flush its report and terminate before another task. Unstarted retries are
cancelled. A pending stop-only callback captures its own service client and
completion flag, so bounded destruction does not join indefinitely or access
mission state. This is not permission to continue after detaching it.

Every failure return, including cancellation during the final `Outcome` callback,
uses this cleanup and preserves the latest custody/effect journal. The report
cannot reuse a measured-stop flag from before the final stop completion.
`tests/test_tube_mission_runner.py` checks these report/budget boundaries.
`tests/tube_stop_rpc_harness.py` extracts the production stop methods for both
host fake-transport tests and an isolated roscpp mock service; it exercises
concurrent stop calls, an accepted late position request, ordered final stop,
blocked reply, and destruction with a late callback. It avoids constructing
MoveIt and does not establish physical stopping performance.
