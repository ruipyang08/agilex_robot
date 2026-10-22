# General task workflow

This deployable C++17 package owns task definitions, strict version-2 recipe
parsing, whole-recipe preparation, sequential execution and invocation lifetimes.
Its build dependencies are Catkin, yaml-cpp and platform threads. It has no ROS
node, navigation, manipulation or robot-state dependency.

Public headers are under `include/agx_task_workflow/`; the exported library is
`task_workflow`. Begin with the [record-value example](examples/record_value/README.md).
Optional robot state and guards are owned by the separate `agx_robot_task_runtime`
package. The existing `agx_robot_workflow` application binds both packages.

Task metadata uses validated lowercase dotted identifiers, such as
`instrument.centrifuge` and `instrument.lid_closed`. A descriptive ID does not
provide a runtime guard or authorize continuation. Factories must still bind
entry, execution, bounded stop and cleanup callbacks. Measured prerequisite and
effect checks belong to the selected domain adapter.

`PreparationState::planning` stores typed prospective values by namespaced key.
Use value types containing owned data, never live device handles or mutable
shared state. `Get<T>` returns a copy; `Set<T>` replaces the value and rejects a
type change. A continuing task's prospective success writes are discarded;
its expected effects remain conditional. These values never replace measured
execution state.

`TaskRuntimeBinding` pins run/task/operation identity and deadline. Optional domain
ports use `InvocationToken::WithActive` so expiry waits for an admitted operation
to finish. Token callbacks must be bounded and non-reentrant. Expired ports reject
use, including when a new invocation reuses the same task ID.

The optional peer `agx_navigation_tasks` and `agx_tube_tasks` packages and
`agx_robot_workflow` application host are
implemented. In a repository checkout, `docs/development/tasks/README.md` is the
indexed catalog for registered tasks and future design-only modules.
