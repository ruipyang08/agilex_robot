# Tube tasks

`agx_tube_tasks` owns the `grasp_test_tube` task definition, immutable configuration
preparation, execution admission/reconciliation, grasp effect adapter and detached
observation encoding. It is a
peer of `agx_navigation_tasks`, with the same `agx_task_workflow` contract and
optional robot state from `agx_robot_task_runtime`. Physical grasp behavior remains
in the deployable ROS1 `agx_xarm_pick` package.

```yaml
schema_version: 2
workflow_id: collect_tube
tasks:
  - id: approach
    type: navigate
    target: wb-100
  - id: collect
    type: grasp_test_tube
    fixture_profile: table_grid_3x3_v1
    cap_color: green
```

The example is the physically commissioned legacy selection. A modular
`config: path/to/tube_task.yaml` instead selects a rack installation, container,
behavior and robot configuration graph. Its path is relative to the workflow
file; references inside that graph are relative to each containing file.
`cap_color` is an operation argument, not object identity. Configuration changes
reuse the same task implementation. No physically qualified modular graph is
currently shipped; the complete source fixture is synthetic schema/test data.
This stage preserves the supported external rack and required preceding station
arrival. Robot-mounted rack grasp/place uses separate, independently developed
known-pose task implementations and does not reuse this task's
recognition/alignment behavior.

## Integration

Include `agx_tube_tasks/grasp_test_tube_task.hpp` and call
`RegisterTubeTasks(registry, installation, ports)`. `TubeConfigResolver` reads and
freezes configurations during preparation using `ResolveTubeTaskConfig` and
validates the color-bound motion config. It performs no live waits or commands.
The legacy inline `fixture_profile` path remains compatible: the application
snapshots legacy ROS parameters once before preparation. `config` and
`fixture_profile` are mutually exclusive.

`TubeTaskPorts` binds execution, cancellation, measured entry, bounded stop and
cleanup, read-only installed-service admission, invocation observer selection and
continuation certification. The host checks cross-task robot/interface
compatibility after the whole recipe prepares. The factory retains prospective
arrival/custody rules; runtime adapters separately require measured held/stopped
base evidence and confirmed empty custody. Grasp effects remain recorded when
verified even if later extraction or return fails. Cleanup never releases a tube
just to satisfy the next task.

`grasp_execution.hpp` exposes `ExecuteGrasp(request, ports)` for the ROS
application's execution binding. The module checks measured held/empty custody,
creates the effect adapter, dispatches the injected monitored mission and
reconciles its result. The application supplies the shared state, invocation
identity, live guards and observation callbacks. It must latch an unjoined
worker's exit obligation in `mission_returned` before effect reconciliation.
The monitored mission's retained callbacks must keep their robot owner alive.

The exported library is `tube_tasks`. A consumer declares only this package:

```cmake
find_package(catkin REQUIRED COMPONENTS agx_tube_tasks)
target_include_directories(my_task_host PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(my_task_host ${catkin_LIBRARIES})
```

`test/consumer` is built against a relocated install with the application host
and navigation-task package physically absent. The source depends on neither.
The existing ROS application and two-terminal commands remain in
`agx_robot_workflow`. When that optional application is installed, its
`docs/tasks/grasp_test_tube.md` contains the operator guide and physical
qualification limits. The source-local
[configuration](docs/configuration.md) and [inspection](docs/inspection.md)
guides provide shorter module-specific entry points.
