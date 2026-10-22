# Workflow configuration and immutable admission

The recipe chooses an ordered operation. A task configuration owns its physical
geometry, calibrated observations, qualified behavior and robot interfaces.
`LoadConfig` parses the recipe, module-owned configuration resolvers freeze the referenced
configuration, and `PrepareRecipe` validates every invocation before any task
entry or command is dispatched. Configuration validation is not physical
qualification.

## Recipe version 2

```yaml
schema_version: 2
workflow_id: move_and_collect
tasks:
  - id: source
    type: navigate
    target: wb-100
    config: profiles/travel.yaml
  - id: collect
    type: grasp_test_tube
    config: profiles/pick_table_a.yaml
    cap_color: green
    on_failure: continue
  - id: destination
    type: navigate
    target: wb-101
    config: profiles/travel.yaml
```

This is a schema example; the referenced files and station registrations must
exist and be qualified for the selected execution mode. It does not install a
travel profile or qualify carrying a tube.

The root accepts exactly `schema_version`, `workflow_id` and `tasks`. Version is
integer `2`; the workflow ID and task IDs/types are nonempty strings. The task list
is nonempty and IDs are unique. Common task fields are `id`, `type`, optional
`config`, and optional `on_failure` (`stop` by default, or `continue`). Other fields
are validated by the registered task definition. Duplicate/unknown keys, wrong
scalar types and unknown task types are rejected. There is no shell execution,
recursive merge, interpolation, retry, resume or runtime plugin discovery.

A relative `config` path resolves against the recipe file's directory, never the
launching shell's working directory. `ParseConfig` without a source filename
rejects relative configuration paths. `LoadConfig(path)` supplies that origin.
The selected configuration is an input to admission, not a live parameter link.

The host's optional private `~task_timeout_s` is separate from recipe and physical
configuration. It is read once as a finite positive numeric duration, defaulting
to 900 seconds per task, and must form a representable future steady deadline.
The offboard wrapper forwards `--task-timeout-s SECONDS` only when supplied.
Execution and check-only summaries record `task_timeout_s` and
`task_timeout_clock: "steady"`. `BeginTask` creates the fixed per-task budget
before physical task preparation and subsequent phases. Whole-recipe
configuration and interface admission occurs before `BeginTask` and is outside
that budget. Stop/cleanup and subordinate navigation phase limits remain separate.
See [execution](execution.md) for validation, clock and first-cause semantics.

## Concrete task recipe shapes

The production host registers the software-only example:

```yaml
schema_version: 2
workflow_id: record_value_example
tasks:
  - id: record_sample
    type: record_value
    value: sample-1
```

Use the [example guide](../../agx_task_workflow/examples/record_value/README.md) to run it without a
simulator. It rejects `config`; it needs no navigation or grasp configuration.
The standalone example admits only its own registered type. The production host
also registers `navigate` and `grasp_test_tube`.

A navigation-only recipe using the installed destination input is:

```yaml
schema_version: 2
workflow_id: approach_source
tasks:
  - id: source
    type: navigate
    target: wb-100
```

It inherits the installation's travel profile and mode. The station must exist
in the live registry; the selected profile and interfaces must pass admission.
The repository operator guide owns commands for the normal measured empty start.

A modular grasp workflow first establishes the required held arrival:

```yaml
schema_version: 2
workflow_id: collect_from_installed_rack
tasks:
  - id: source
    type: navigate
    target: wb-100
  - id: collect
    type: grasp_test_tube
    config: task_configs/rack_a.yaml
    cap_color: green
```

This is a schema-valid authoring example, not a supplied physical installation.
Create and qualify the referenced `tube_task` graph using the
[domain schema](../../agx_xarm_pick/docs/configuration.md); its installation
station must match the held arrival. `task_configs/rack_a.yaml` is relative to
the recipe, and each graph reference is relative to its own containing file.
For example, place a copied recipe at `/opt/team/recipes/workflow.yaml` with
its graph under `/opt/team/recipes/task_configs/`; launching from `/tmp`
does not change those bindings. Preserve that layout when relocating.
The parser/resolver tests exercise these rules with synthetic fixtures;
those fixtures are not installed equipment qualification.

Omit `fixture_profile` for modular grasp. Legacy `fixture_profile` plus
`cap_color` remains supported by the original default recipe. Neither style
changes task behavior through shared-parameter mutation.

New equipment YAML is design-only until its type is implemented and registered.
The host does not resolve new task configuration automatically; see the
[required integration edits](../../agx_task_workflow/docs/task_template.md).

## Optional grasp observation transport

`~publish_grasp_effect_observations` is a private boolean read once by the
workflow host, default false. Explicit integers, strings and other non-booleans
are configuration errors before dispatch. This is neither a recipe field nor a
physical profile setting. True has no capture, timer or diagnostic-publication
effect in check-only mode. No installed launch or acceptance wrapper enables it.

For an executing admitted recipe, true prepares stable grasp-observation slots
and permits the existing adapters' optional journals to feed the existing events
topic. Invalid diagnostic identity or optional preparation/binding failure makes
that evidence unavailable without rejecting or changing an otherwise admitted
task. No task-count or control-identity restriction is added. The optional report
member `grasp_effect_observations` records availability and drain/publication
facts independently of the authoritative result. Missing/incomplete diagnostics
never become successful task or physical evidence. See the
[grasp observation contract](tasks/grasp_test_tube.md#optional-host-transport-and-evidence-completeness)
for original identity, closure, errors and unchanged stop/report boundaries.
Enabled capture and publication costs still require validation before any
physical retention-loss experiment; this setting alone grants no qualification.

## Built-in selections

| Task | Task-specific fields | Configuration choice |
| --- | --- | --- |
| `navigate` | `target` | Per-task `config`, otherwise nonempty private `~travel_profile`, otherwise global `/workbench_workflow/travel_profile` |
| `grasp_test_tube`, modular | `cap_color` | Required task `config` points to a `tube_task` reference graph |
| `grasp_test_tube`, legacy | `fixture_profile`, `cap_color` | Existing loaded `/tube_grasp` and selected `/tube_fixture_profiles` snapshot |

A grasp task cannot supply both `config` and `fixture_profile`. The legacy
fixture selection must match the already-loaded profile; the recipe does not
rewrite an active ROS parameter or switch global profiles. Requested color must
be present in the frozen behavior's classifier map.

The host parses `~execution_mode`, falling back to
`/workbench_workflow/execution_mode` and then `hardware`. Only `hardware` and
`simulation` are accepted. Every navigation task must have an enabled profile
with an evidence identity for that mode. The installed default travel profile
has both modes disabled. Source test fixtures are synthetic and are never a
replacement for measured qualification.

The full repository's offboard guide
`docs/development/simulation/workbench_workflow_operation.md`
shows how simulator readiness and navigation admission are reported separately.
On admission failure, the workflow reports the canonical travel-profile source
along with the disabled execution mode so an operator can distinguish an
unqualified installed default from a simulator bringup failure.

The current host additionally rejects every hardware composed-navigation request:
the hardware joint-trajectory controller can splice its previous desired state
into a new arm trajectory, and that initial bridge is not yet validated. An
`enabled` hardware label cannot bypass this implementation gate.

Travel profiles are strict version-1 typed documents, described in
[travel preparation](travel_preparation.md). Their `config_id` hashes exact file
bytes; moving the file preserves the ID, but comments or formatting changes
produce a new travel ID. This differs deliberately from the normalized tube
domain graph ID below. The host retains one const travel snapshot for each
canonical selected path during admission.

For the current physical adapter, travel must select `xarm6` and six ordered
joints `joint1` through `joint6`. All travel tasks use the same robot interfaces.
Travel and grasp must agree on joint/gripper/lift feedback, lift command endpoint,
and the open/close/command-tolerance observation contract. A profile may not
command one device while the shared observer reads another.

Navigation also admits five cached effective-configuration reports: local
planner, global planner, base arbiter, docking, and LiDAR. It checks loaded speed,
acceleration and footprint limits against the profile, pins producer sessions and
content, and supervises report freshness throughout execution. It does not infer
loaded controls from a YAML file that a running producer may no longer use. See
[the envelope admission contract](travel_preparation.md#admission-against-effective-base-settings).

## Tube domain graph

`agx_xarm_pick::ResolveTubeTaskConfig(path)` returns
`shared_ptr<const ResolvedTubeTaskConfig>`. The complete key schema and numerical
constraints are maintained beside the resolver in
[the tube configuration guide](../../agx_xarm_pick/docs/configuration.md).

```text
pick_table_a.yaml: tube_task
  installation -> rack_a_on_table_a.yaml: fixture_installation
    fixture_profile -> type_a.yaml: rack_profile
    support_surface -> table_a.yaml: support_surface
  container -> tube.yaml: container_profile
  behavior -> pickup.yaml: tube_behavior
  robot -> robot.yaml: robot_profile
```

A second installation can reference the same rack geometry and a different
measured table/support transform. `fixture_instance_id` identifies the installed
rack; `fixture_profile_id` identifies its reusable geometry. Slot IDs/layout
belong to the rack profile. Current occupancy belongs to measured robot state,
not a static configuration document. Color remains operation intent in YAML.

Each domain document has its own strict version/kind/id. References resolve
relative to their containing document. Missing references, cycles, duplicate
keys, unknown keys, incompatible kinds/frames, nonfinite quantities, malformed
units and mixed ownership are rejected. Each canonical file is read once in one
resolution; no recursive merge or global ROS parameter rewrite occurs.

The tube `config_id` is `sha256:` plus a normalized content digest including
schema versions and reference identities. Mapping order, comments, number
spelling and relocation do not change it. Source provenance separately records
each file's kind/id, exact byte SHA256 and absolute source path. Absolute paths
are provenance only; relative file graphs can be copied offline unchanged.
Existing const snapshots are unaffected by later edits. A later admission sees
the edited input. Shared files across separately selected task graphs are frozen
per resolution; deploy configuration revisions between runs, not during admission.

`BindTubeTaskMotion(snapshot, color)` binds the selected classifier and rejects
features unsupported by the current mission: required fixture tag registration,
tilting the rack out of the horizontal plane, or unsupported planning/camera
frames and context type. Valid domain geometry alone does not imply that a motion
adapter can execute it. The current frames are `link_base` and
`cam1_color_optical_frame`; horizontal rack yaw is permitted.

## Frozen perception and legacy adaptation

The workflow injects the same const tube snapshot into the mission and its target
provider. Metric requests use the admitted `robot.detector_service` with
`_configured` appended. The target endpoint is
`/select_tube_target_2d_configured`; version 1 has no configurable target endpoint.

`MakeTubePerceptionProjection(snapshot, color)` produces only geometry, frames,
perception settings and tolerances. `FillConfiguredTubePerceptionRequest` sends
those typed values, graph/fixture identities, a projection SHA256, color and
`not_before`; metric detection also sends its mode. No host paths, arm motion
settings or gripper commands are exposed in this projection. Responses must echo
all four configuration identity strings before the caller accepts their evidence.

The server independently hashes the actual projection and binds its content to
the graph/fixture/color identity. It cannot reconstruct omitted motion/robot facts
from a subset, so the graph ID remains a trusted caller assertion, not an
authentication or whole-graph verification claim. Its bounded registry rejects
rebinding or capacity overflow. Immutable sessions keep temporal observations
separate for complete identity plus fixture and color. The perception package's
`README.md` documents the wire format, hash encoding, transport compatibility and
cache limits. Locate it with `rospack find agx_tube_perception`; this works in both
source and relocated install workspaces.

`SnapshotLegacyTubeTaskConfig` reads the existing ROS subtrees once and adapts
the combined profile into the same mission injection path. Legacy perception
startup is normalized through the same frozen projection/session machinery.
The combined legacy YAML remains readable unchanged. No temporary parameter
mutation is used to emulate task-local configuration.

## Admission, reports and offline copying

After all selections and factories succeed, the host checks the union of declared
ROS interfaces. Missing observed publishers, command subscribers or service/action
surfaces prevent execution. Check-only uses a distinct anonymous node and performs
these read-only checks without calling task entry, motion, stop or cleanup hooks.
On success it prints `Validation passed. No tasks were executed.` and reports
`completion: "Validated"`, `validation_only: true`, `valid: true`, and exit code
0. The execution-only `success` field remains false and task statuses remain
`not_run`. A failed check reports `completion: "Failed"` and a nonzero exit code.
Validation does not prove reachability, retention or stopping.

Run summaries retain task/configuration identities, source provenance, travel
profile and effective-envelope digests, and applied physical effect references.
A later cancellation or task failure does not remove verified capture/release
history. `on_failure: continue` still requires an ordinary failure, confirmed stop,
reconciled effects, fresh supported state and the next task's entry checks; see
[execution](execution.md).

Copy the entire referenced graph with the deployable `agilex_ws` source and keep
its relative paths. Onboard execution requires only ROS1 and installed system
libraries/Python 3.8 dependencies, not this repository's offboard `uv`, Isaac or
ROS2 environment. The offboard wrapper accepts `--travel-profile PATH` and
`--execution-mode simulation`; onboard invocation passes the corresponding
private ROS parameters. Without an installed or explicit mode the host falls
back to hardware; the combined simulation bringup installs simulation mode.

## Installation defaults

The installed `application.launch` publishes
`/workbench_workflow/travel_profile` and `/workbench_workflow/execution_mode`
from its `travel_profile` and `execution_mode` arguments. Its defaults are the
package's disabled `config/travel_profile.yaml` and `hardware`. Selecting a path
never enables or qualifies that profile, and the hardware controller-splice gate
still applies.

The combined simulation application passes `execution_mode:=simulation`.
The normal `run_sim.sh --profile workbench_manipulation` command selects
`config/isaacsim/travel_profiles/original_default_empty.yaml`, which authorizes
only the measured neutral empty start. `--travel-profile PATH` overrides that
selection; the path must exist below the mounted workspace. Direct onboard use
continues to inherit the disabled package default. The launcher never selects a
candidate or synthetic test fixture.

After installation, `run_workbench_workflow.py --config recipe.yaml` inherits
those settings. Its optional `--travel-profile` and `--execution-mode` arguments
supply only explicit private overrides; omitted flags write no empty private
parameters and do not rewrite the shared installation. The onboard host still
validates the selected mode, profile qualification and complete task recipe
before any command. A task's own `config` remains the most specific profile
selection.
