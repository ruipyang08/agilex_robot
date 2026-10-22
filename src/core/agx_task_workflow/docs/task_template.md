# Author a task against the general contract

Start with the [working example](../examples/record_value/README.md) and
[contract](task_contract.md). Build a new task in its own deployable ROS1 package,
depending on `agx_task_workflow` and only the domain services it actually needs.
Do not depend on the application host to obtain the general API.

1. Define one reusable behavior and its success evidence. Separate invocation
   arguments from installation/configuration data. Variations supported by the
   same behavior should select configuration rather than create copied types.
2. Declare a stable type, contract version, exact input schema, interface
   dependencies, namespaced resources, measured prerequisites and effects.
   Common recipe fields `id`, `type`, `config` and `on_failure` are not custom inputs.
3. Admit and freeze configuration before dispatch. Validate the complete graph,
   resolve paths relative to each containing file, and retain source/content
   identity. If configuration is unsupported, reject `config_path` explicitly.
4. Bind entry, execution, bounded stop and cleanup callbacks through injected
   services. Preparation performs no live waits or actuation. Request invocation
   ports during entry/execution and check interruption around external callbacks.
   Bind read-only `admission_check` if the installed service needs additional
   admission, and `begin` to select invocation-specific adapters. To support
   `on_failure: continue`, supply `continuation_check` certifying owned work and
   effects after stop/cleanup; the host's shared invariant checks also must pass.
5. Verify actual results before success. Record physical effects when observed,
   including effects that remain after later failure. Never infer custody from
   a task's name or clear it merely to make the next task pass.
6. Test complete recipe admission, invalid later tasks, interruption, stop and
   cleanup, effect preservation and independent installation. A device-free
   callback is useful for contract tests; physical adapters need their own
   simulation and manual inspection evidence.
7. Register the definition in the selected application and bind its installation
   services. Install public headers, libraries, configs and the exact tested
   example/inspection instructions. Verify a copied downstream consumer.

## Current integration boundary

`agx_navigation_tasks` and `agx_tube_tasks` are peer modules using this contract.
Their factories resolve and freeze their own configurations. The existing
`agx_robot_workflow` application snapshots installation parameters, registers
the modules, calls whole-recipe preparation, then admits their frozen selections.
Adding a configured task requires its registration and installed-service binding;
it requires no task-name branch in a central configuration loader. See each
module's `task_config.hpp` and registration header for the integration boundary.

The standalone example uses only generic invocation ports. Robot tasks may depend
on `agx_robot_task_runtime` for the authoritative custody store, base guards and
optional held-arrival ports. Table height, rack geometry and reference frames are
domain inputs, not universal prerequisites or a mandatory WorkbenchTask parent.
The current physical grasp adapter still supports its qualified external rack;
robot-mounted motion requires a suitable adapter and calibration.

For the existing application, use its documented two-terminal simulation and
inspect the rover, lift and arm after software checks. A contract test does not
qualify a new fixture, new motion or hardware deployment.
