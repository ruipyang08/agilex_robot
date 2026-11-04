# Robot workflow application

This onboard ROS1 package admits and executes ordered version-2 tasks through a
registry. The deployable sources are in `agilex_ws/`; the offboard computer builds
and tests those same sources in Noetic Docker. Isaac Sim and ROS2 supply simulated
interfaces and independent physical observations. They do not own task behavior.

The general contract and runner are exported by
[`agx_task_workflow`](../agx_task_workflow/README.md). Measured robot state and
guards belong to [`agx_robot_task_runtime`](../agx_robot_task_runtime/README.md).
Navigation and grasp definitions are peer modules:
[`agx_navigation_tasks`](../agx_navigation_tasks/README.md) and
[`agx_tube_tasks`](../agx_tube_tasks/README.md). This package is the application
host. The ROS host binds installed services, physical effect
sinks, live observers, bounded stopping, base hold/release and navigation.
Repeated navigation is implemented for an admitted empty or supported single-tube
load. Explicit ordinary-failure continuation preserves failed-task history and
requires confirmed stop, task-owned certification and shared-state reconciliation
before any next task. Observation tasks do not require gripper readiness.

Start with these source-local guides:

- [Build and inspect record_value](../agx_task_workflow/examples/record_value/README.md), then follow
  the [complete task authoring and integration procedure](../agx_task_workflow/docs/task_template.md)
- [Configuration and offline freeze rules](docs/configuration.md)
- [Task definitions and admission](../agx_task_workflow/docs/task_contract.md), with a [copyable template](../agx_task_workflow/docs/task_template.md)
- [Measured custody, object references and result metadata](docs/robot_state.md)
- [Execution, cancellation, stopping and continuation](docs/execution.md)
- [Travel posture, path verification and effective control envelope](docs/travel_preparation.md)
- Concrete tasks: [navigate](docs/tasks/navigate.md), [grasp_test_tube](docs/tasks/grasp_test_tube.md)
  and `place_test_tube` ([configuration and known gaps](../agx_place_tasks/docs/configuration.md))

This package exports an executable, launch files and recipes, with no task
library or public C++ headers. Consumers register the selected peer modules
directly. `test/downstream_consumer` checks those modules together from the
relocated installation; the general package has its own minimal independent
consumer. Source tests and isolated ROS fakes validate software behavior, not
physical qualification.

## Run a recipe

The canonical onboard executable is `rosrun agx_robot_workflow run_robot_workflow`.
The offboard command is `python3 scripts/run_robot_workflow.py`; append `--check`
for admission without dispatch, or `--config path/to/recipe.yaml` to select
another workflow. [navigate_dock_grasp.yaml](config/navigate_dock_grasp.yaml) is
the default recipe. Its task list supplies the complete navigate → grasp order.

The earlier `scripts/run_workbench_workflow.py` command forwards to the same
CLI, including configuration, cancellation and exit status. There is no remaining
`agx_workbench_workflow` source package or second workflow implementation.
ROS node names, `/workbench_workflow/*` topics/parameters and the existing summary
path remain stable so the current watchdog and operator tools keep working.
In a repository checkout, the two-terminal inspection guide is
`docs/development/simulation/workbench_workflow_operation.md`.

## Application boundary

`RegisterRobotTasks` connects module-owned definitions to installed services;
`PrepareRecipe` and `Run` belong to the general framework. `RunNavigationTask`
owns navigation's departure/arrival sequence. `ExecuteGrasp` in `agx_tube_tasks`
owns empty-custody admission, effect-adapter creation and result reconciliation.
The host supplies the monitored ROS mission, keeps its owner alive for worker
callbacks, and records observations and terminal worker obligations.

Shared robot selection, observed base state, travel readiness publication,
effective-envelope admission and physical stopping remain coordinated here.
These bindings deliberately know about this installed robot; this application
is not a universal ROS driver. New equipment modules supply their own bindings
without changing the general framework or hardcoding a workflow sequence.

Generic `TaskRuntimePorts` pins run/task/operation identity, fixed deadline and
interruption checks. Optional `RobotTaskPorts` adds the existing state store and
a measured held-arrival check for tasks that declare it. Production
`workflow_node.cpp` registers `record_value` before `PrepareRecipe`; adding YAML
alone cannot install another type. Success verification is inside execution.
Failure stop/cleanup and new equipment process-loss protection remain adapter
responsibilities. Observation-only tasks do not acquire shared actuator stops.

The installed `application.launch` selects the profile and execution mode once
for later recipes; [installation defaults](docs/configuration.md#installation-defaults)
describes bringup and explicit overrides. The offboard CLI inherits these choices
when given only `--config recipe.yaml`.

The installed default travel profile disables both simulation and hardware.
Two-station empty travel, retained-tube travel, and missed-pick continuation each
require their own recorded physical acceptance. Their new physical qualification
is pending; previous one-station grasp/navigation results do not qualify these
new combinations. Real-robot qualification is separate from simulation.

The user manually accepted Tasks 1–3 and the normal two-terminal simulation
workflow. Its offboard launcher selects the measured neutral empty-start
simulation profile, separately from the disabled onboard package default.
See [installation defaults](docs/configuration.md#installation-defaults).

Hardware composed navigation also has an unconditional module preparation gate while the
controller's initial desired-state trajectory bridge remains unvalidated.
Changing a profile's enabled/evidence fields cannot bypass that gate.
