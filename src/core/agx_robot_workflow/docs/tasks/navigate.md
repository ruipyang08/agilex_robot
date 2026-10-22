# navigate

`navigate` reaches a registered workbench and establishes a newly measured arrival
hold. The only task-specific input is `target`; optional common `config` selects
a qualified travel profile. It can follow another navigation or grasp when live
custody, clearance and actuator state admit empty or supported single-tube travel.
The same-station case still performs preparation and fresh arrival checks.

```yaml
id: approach_destination
type: navigate
target: wb-101
```

The source/target stations, profile and evidence identity must already be
installed. See [configuration](../configuration.md) for path resolution, fallback
selection and execution-mode rules. The example inherits the installation
profile; add a common `config` field only when explicitly selecting another
qualified profile. The shipped default profile is disabled.
Hardware composed navigation is unconditionally blocked until the initial
joint-trajectory-controller bridge is validated; a profile label cannot bypass it.

## Description and admission

`RegisterBuiltinTasks` supplies the `TaskDefinition`. Its prepare factory binds
`NavigateParameters{target, shared_ptr<const TravelProfile>}` to the composed
adapter without commanding motion. The host resolves each profile first, verifies
its enabled execution mode, checks compatible robot interfaces, then prepares the
whole recipe. An unadmitted `config` never silently falls back to legacy navigation.

Declared resources are base, arm, lift, gripper and camera. Required interfaces
include the navigation action; authority, odometry, output command and goal/status
observations; workflow watchdog; typed acquire/prepare/release services; a consumer
for travel readiness; configured joint/gripper/lift feedback and lift control;
MoveIt planning/execution/planning-scene interfaces; and an arm-stop consumer.
The five effective-config publisher streams are also admitted and continuously
supervised against the qualified travel envelope. An unrelated custom task does
not inherit these dependencies just because navigation is registered.

## Physical sequence

`RunNavigationTask(NavigationTaskPorts, PhaseSink)` checks its runtime guard before
and after each phase callback and physical port. Earlier applied effects remain
in the result if a later boundary is cancelled or fails.

| Phase | Required physical boundary |
| --- | --- |
| `PREPARE_DEPARTURE` | Invalidate previous arrival authorization; acquire a typed hold with current session/epoch, zero output, fresh source/receipt evidence and measured stopped base |
| `PREPARE_TRAVEL` | Observe supported current custody and an admitted starting posture; while held, execute the profile's arm-then-lift preparation with continuous clearance/collision and retained-load checks |
| `VERIFY_TRAVEL_READY` | Observe post-command stopped producers, measured endpoint and settle evidence; establish a fresh readiness lease |
| `RELEASE_BASE` | Prepare a fixed-expiry single-use permit bound to run/departure/hold/revision/readiness sequence, then release that exact hold |
| `NAVIGATE` | Dispatch the registered workbench action; keep posture, custody, clearance, localization and command-authority supervision active |
| `VERIFY_STOP` | Require the owned navigation goal terminal, fresh odometry, zero output and stopped dwell |
| `HOLD_BASE` | End travel readiness, acquire a new destination hold and capture its current session/epoch and measured pose |
| `VERIFY_ARRIVAL` | Commit the held arrival after final live checks |

The navigation action internally owns coarse travel, visual tag alignment and
final approach. The recipe does not need entries for those phases. Source-derived
continuous localization replaces per-leg reseeding; see
[localization](../../../agx_workbench_navigation/docs/localization.md).

The profile and implementation details of path verification, live planning-scene
checks, temporary attached tube geometry, interpolation and actuator stopping
are in [travel preparation](../travel_preparation.md). Command publication,
action completion and a previous grasp result alone cannot establish travel
readiness. A blocked producer, stale observation, lost retention or inconsistent
readback prevents release or stops the active operation.

## Custody, stopping and outcomes

Navigation consumes a `TransportView` without changing holder identity. Empty
requires current verified empty feedback. A held tube requires an already-known
object, compatible container, retained feedback and safe clearance. A caught
sensor cannot identify a tube after restart. Pending transfers, uncertainty,
unsupported loads or a latched state fault block departure. Navigation never
opens the gripper to simplify recovery.

The workflow owns bounded stop/cleanup hooks. It cancels only owned navigation,
requests base hold and arm/lift stop through independent paths, then waits for
fresh post-stop observations. Worker join or a successful stop RPC does not prove
physical stopping. A command-producing worker that cannot be joined requires
process exit and independent watchdog stopping; later tasks cannot run.

A task can succeed only after the new held arrival is verified. Any cancellation,
deadline, fault, readiness loss or unconfirmed stop is terminal. Navigation does
not convert those failures to ordinary solely because the recipe says
`on_failure: continue`. A preceding ordinary missed pick may permit navigation
only after its bounded stop and fresh empty/safe-clear reconciliation succeed.

## Verification boundary

Pure tests exercise every stage failure, cancellation before release and late
cancellation after an applied arrival effect. Built-in tests admit repeated
navigation and reject unadmitted configurations. Isolated ROS tests exercise the
real host's configuration/dependency checks and pre-actuator failure paths.
Two-station empty/held travel and missed-pick continuation still require measured
simulation qualification; hardware qualification remains separate and pending.
