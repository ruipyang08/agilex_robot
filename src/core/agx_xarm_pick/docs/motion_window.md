# One-shot motion observation window

`TubeObservationBuffer::ArmMotionWindow(source_clock, steady_clock)` starts one
task-neutral window of reported arm/lift motion. `TubeLiveObserver` exposes the
same operation and returns a token containing a unique generation and the
original source/steady barrier. This is additional observation state; it does
not change the existing strict stationary fields or `ArmLiftStoppedSince()`.

The caller arms only after its bounded command worker has actually joined and
its final preparation guard succeeds. Both clocks and the current finite, fresh
actual arm/lift anchors are captured under the buffer mutex. Navigation creates
a new observer for each task. An arm attempt consumes that observer's one shot,
including an attempt with bad clocks or missing/stale anchors. A second attempt
is rejected and invalidates the existing window. There is no reset or recovery
operation within a task.

Every incoming arm/lift callback updates the independent window under that same
mutex, before any legacy invalid/duplicate early return. Six fixed arm entries
and one lift entry retain minimum position, maximum position and maximum absolute
reported velocity, with original source/receipt evidence for each extremum.
Advancing queued measurements from before the barrier also contribute extrema;
they cannot count as post-command evidence. An out-and-back movement or rate peak
between snapshots remains visible. Becoming stationary later cannot erase it.

After arming, invalid/nonfinite input, a changed source, regressing source or
receipt clocks, contradictory same-stamp payloads, or an excessive source or
receipt gap permanently invalidates the window. The first failure and its
original evidence remain available without logging or I/O in callbacks. Identical
repeats retain the first receipt of their unique measurement; a separate receipt
high-water mark detects callback reordering. Repeats cannot renew freshness or
repair invalidation. Before arming, a valid advancing measurement may recover
from an old continuity gap and provide a new fresh anchor; historical gaps are
not part of a window that has not started. Pre-arm duplicates still cannot renew
that anchor's original receipt.

`Snapshot()` returns legacy observations and the window together, read-only.
Window freshness uses its last original unique arm/lift evidence in both clock
domains. A stale read does not mutate the buffer; the consuming travel policy
latches that failed observation. Later advancing callbacks also detect an
excessive original measurement gap. An absent/default window is invalid and
cannot authorize travel, including with older observer implementations.

A valid but stale snapshot annotates only its returned window with the first
rejected stream (arm before lift), both original arm/lift evidence, the snapshot's
source and steady clocks, and the existing freshness limits. Its failure evidence
identifies that first stream. A later healthy snapshot has no such annotation;
an actual callback failure keeps its original latched detail and evidence.
Displayed source names are capped at 96 bytes with `...` on truncation; the
original evidence fields remain intact. Formatting occurs only for a failed read,
uses no additional clock or I/O, and leaves the original stale result intact if
diagnostic construction fails.

The workflow applies its existing posture/rate limits and the observer's existing
`stopped_arm_range_rad` / `stopped_lift_range_m` to the retained extrema. The
observer itself is independent of task targets and workflow control. Strict
preparation and cancellation stopping still use the original stationary/dwell
rules. For preparation and the ready-to-release wait, workflow also consumes the
existing `stationary_source_since_s` and `stationary_receipt_since_s` maintained
by every callback. A hidden interruption restarts policy dwell before readiness
and faults after readiness until successful `BeginTravel`, even if the current
stationary flags recovered. This never resets the independent motion window;
bounded retention continues after departure. See
[travel preparation](../../agx_robot_workflow/docs/travel_preparation.md)
for checkpoint binding and the explicit transition after accepted hold release.

The lift rate remains signed `LiftMotorMsg.backSpeed` in integer mm/s, converted
to m/s. In the simulation facade, a reported -1 mm/s results from rounding
approximately -0.5 to -1.5 mm/s. Hardware quantization and accuracy are not
established by this diagnostic.
A one-mm reported height span likewise does not reveal sub-mm movement
at its endpoints. This window preserves every reported callback, not unobserved
physical extrema or finer encoder resolution. It grants no hardware qualification;
hardware travel remains disabled in the shipped profile.

Host checks compile the production observer, policy and adapter methods:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_travel_retention.py tests/test_travel_producers.py tests/test_travel_release_boundary.py tests/test_tube_live_observer.py tests/test_workflow_stop_rpcs.py
```

The C++ `tube_motion_window_test` and `workflow_travel_retention_test` are also
registered with CTest. Host success does not replace the required offboard ROS1
container build or frozen physical qualification before deployment.
