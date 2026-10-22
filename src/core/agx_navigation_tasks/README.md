# Navigation tasks

`agx_navigation_tasks` owns the `navigate` task definition, qualified travel
configuration, departure/arrival sequence, travel policy, motion adapter and
live effective-envelope admission. It is a peer of `agx_tube_tasks`; both use
`agx_task_workflow` and optional measured robot support from
`agx_robot_task_runtime`.

```yaml
schema_version: 2
workflow_id: approach_station
tasks:
  - id: approach
    type: navigate
    target: wb-100
```

`target` is a registered destination ID, resolved from the installation's
navigation registry. `config` is optional when a qualified default travel
profile is installed. An explicit `config: path/to/qualified_profile.yaml` is
relative to the containing workflow file.
The same task definition supports different destinations and qualified profiles.

## Integration

Include `agx_navigation_tasks/navigate_task.hpp` and call
`RegisterNavigationTasks(registry, installation, ports)`. `NavigationInstallation`
snapshots the registry path, default travel profile and execution mode.
`NavigationConfigResolver` validates and caches immutable file contents during
preparation. An explicit per-task `config` takes precedence over the installed
default. The application snapshots private `~travel_profile` ahead of global
`/workbench_workflow/travel_profile`. Profiles must qualify the selected mode;
the unvalidated hardware initial-trajectory bridge remains blocked in code.

`NavigateTaskPorts` binds composed execution, cancellation, measured entry,
bounded stop/cleanup, read-only installation admission, invocation selection and
continuation certification. The lower-level `NavigationTaskPorts` in
`navigation_task.hpp` composes the departure/arrival sequence. The application
binds ROS services to these APIs; metadata alone does not provide device control.
No live waits or commands run while preparing a recipe. The host performs
cross-task robot compatibility and effective-envelope/interface admission after
all factories succeed. Prepared invocations retain their selected profile.

The exported library is `navigation_tasks`. A consumer declares only this package:

```cmake
find_package(catkin REQUIRED COMPONENTS agx_navigation_tasks)
target_include_directories(my_task_host PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(my_task_host ${catkin_LIBRARIES})
```

`test/consumer` is built against a relocated install with the application host
and tube-task package physically absent. The source has no dependency on either.
The current ROS application and existing two-terminal commands remain in
`agx_robot_workflow`; lower-level navigation uses `agx_workbench_navigation`.
When the optional application is installed, its `docs/tasks/navigate.md`
contains the operator guide and physical qualification limits. The source-local
[configuration](docs/configuration.md) and [inspection](docs/inspection.md)
guides summarize module ownership and acceptance.
