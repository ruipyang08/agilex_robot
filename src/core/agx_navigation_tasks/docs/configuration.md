# Navigation task configuration

The workflow invocation is:

```yaml
- id: approach
  type: navigate
  target: wb-100
```

`target` selects an installed destination ID; it is not a raw tag number or map
pose. The example uses the application's qualified default travel profile. Add
`config: path/to/qualified_profile.yaml` to select another admitted profile;
explicit paths resolve relative to the workflow file. The module freezes the
selected profile during recipe preparation and records its content identity in
the result.

Travel profiles describe supported execution mode, posture/custody envelope,
planning and control configuration evidence. A profile label cannot qualify a
new load, scene, controller or hardware mode. Disabled or mismatched profiles
reject before commands. Installation target geometry remains in the navigation
registry owned by the application.

When the optional `agx_robot_workflow` application is installed, see its
`docs/configuration.md` for defaults and `docs/tasks/navigate.md` for measured
admission and completion conditions.
