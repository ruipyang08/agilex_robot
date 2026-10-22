# External-rack grasp configuration

The currently commissioned invocation is:

```yaml
- id: collect
  type: grasp_test_tube
  fixture_profile: table_grid_3x3_v1
  cap_color: green
```

The modular form replaces `fixture_profile` with
`config: path/to/tube_task.yaml`. That graph freezes the external-rack fixture,
support/station, tube/container, behavior and robot descriptions. References
resolve relative to their containing document. The full schema and synthetic
two-installation example are in the
[domain configuration guide](../../agx_xarm_pick/docs/configuration.md). They are
test data, not physically qualified rack installations. No physically qualified
modular graph is currently shipped.

`cap_color` is an operation selection; it is not a physical object serial
number. `fixture_profile` and `config` cannot be supplied together.

This task requires a preceding compatible held station arrival because parking
changes the external rack pose and the implementation uses camera recognition
and visual alignment. Robot-mounted rack pickup/place uses separate known-pose
task implementations and configuration; it is not enabled by changing this
fixture label.

When the optional `agx_robot_workflow` application is installed, its
`docs/configuration.md` and `docs/tasks/grasp_test_tube.md` describe application
defaults and physical behavior.
