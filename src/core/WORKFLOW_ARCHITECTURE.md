# Onboard workflow architecture

The onboard computer runs Ubuntu 20.04 and ROS1. The offboard computer edits and
validates the same deployable `agilex_ws/` sources in Docker. Isaac Sim/ROS2 supply
simulation interfaces and independent observations. RViz displays reported state;
it is not a second motion or custody authority.

| Module | Responsibility and source guide |
| --- | --- |
| `agx_task_workflow` | [General contract, recipe admission and sequential runner](agx_task_workflow/README.md) |
| `agx_robot_task_runtime` | [Optional robot state, guards and invocation ports](agx_robot_task_runtime/README.md) |
| `agx_navigation_tasks` | [Navigation definition, configuration and departure/arrival sequence](agx_navigation_tasks/README.md) |
| `agx_tube_tasks` | [Tube grasp definition, configuration, execution and effect reconciliation](agx_tube_tasks/README.md) |
| `agx_robot_workflow` | [ROS application: service bindings, registration, supervision and YAML recipes](agx_robot_workflow/README.md) |
| Task extension | [Definitions/description/requirements](agx_task_workflow/docs/task_contract.md), [template](agx_task_workflow/docs/task_template.md) |
| New task starting point | [Buildable record_value and exact build/test commands](agx_task_workflow/examples/record_value/README.md) |
| Configuration | [Version-2 recipe, per-task references, freeze and provenance](agx_robot_workflow/docs/configuration.md) |
| Shared measured state | [Object/holder references, transfers, uncertainty and revisions](agx_robot_workflow/docs/robot_state.md) |
| Execution | [Deadlines, bounded stop, cleanup and explicit continuation](agx_robot_workflow/docs/execution.md) |
| Reusable navigation task | [Task contract](agx_robot_workflow/docs/tasks/navigate.md), [travel posture and envelope](agx_robot_workflow/docs/travel_preparation.md) |
| Navigation behavior | [Authority, localization and control limits](agx_workbench_navigation/README.md) |
| Grasp adapter | [Task contract and applied physical effects](agx_robot_workflow/docs/tasks/grasp_test_tube.md) |
| Grasp behavior | [Immutable mission, effect journal and live observer](agx_xarm_pick/README.md), [domain schema](agx_xarm_pick/docs/configuration.md) |
| Perception | [Configured requests, immutable histories and digest boundary](../sensors/camera/agx_tube_perception/README.md) |

Dependencies point from workflow adapters to domain behavior and robot interfaces.
Grasp effects and live observations are task-neutral; grasp does not depend back
on workflow state. Task definitions describe requirements/effects without running
the task. `PreparationState` is prospective planning data; only measured effects
update current custody. Generic cleanup never releases a held object.

The extension boundary is compile-time registration plus per-invocation
`TaskRuntimePorts`. The host activates fixed identity and deadline before entry;
optional `RobotTaskPorts` shares that lifetime for measured robot-state access.
Task adapters own verification during execution and bounded
failure stopping. Descriptive capabilities require implemented measured checks;
they are not a planner. Resource declarations select shared robot stopping and
do not implement concurrent execution. New device behavior remains onboard.

Repeated navigation and stopped ordinary-failure continuation are implemented in
software. The installed travel profile stays disabled until measured qualification.
The normal one-station two-terminal simulation and Tasks 1–3 have user-confirmed
manual acceptance. New two-station/loaded scenarios and real-robot qualification
remain separate and pending.
No test fixture, state-store success or cached profile alone grants physical motion.


## Future work

Recognition/alignment improvements, grasp-speed investigation, automatic
loading-area intake, persistent inventory and registered fixture slots remain
future work. An onboard rack needs verified placement/retrieval and exact slot
custody through the state API. Equipment tasks and general station compatibility
will extend the current task contracts. A planner, retry/resume orchestration and
navigation strategy optimization, including nearby-station crab travel, are not
implemented by this milestone.
