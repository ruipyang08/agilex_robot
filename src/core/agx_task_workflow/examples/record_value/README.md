# Record-value task example

This runnable example registers a task, admits a complete YAML recipe and records
an inline string through an injected callback. It uses only `agx_task_workflow`;
no ROS master, robot-state store, navigation or manipulation package is needed.
The callback checks the active invocation before and after execution.

The reusable definition is installed as
`include/agx_task_workflow/examples/record_value_task.hpp`. `main.cpp` is a small
application host. The current example accepts `value` and rejects `config`;
configuration-based variants are a later implementation stage.

From the repository root, build the existing simulation workspace:

```bash
docker compose -f docker/noetic/compose.yaml build ros1-dev
docker compose -f docker/noetic/compose.yaml run --rm ros1-dev \
  python3 scripts/build_ros1_workspaces.py --profile simulation
```

Validate and run the example in Docker:

```bash
docker compose -f docker/noetic/compose.yaml run --rm ros1-dev bash -lc '
set -e
source /opt/ros/noetic/setup.bash
source agilex_ws/devel/setup.bash
rosrun agx_task_workflow record_value_example --check agilex_ws/src/core/agx_task_workflow/examples/record_value/workflow.yaml
rosrun agx_task_workflow record_value_example agilex_ws/src/core/agx_task_workflow/examples/record_value/workflow.yaml
'
```

Expected output includes `Validated 1 task(s); no callbacks ran.` and then
`record_value: sample-1`. Change `value` in the recipe to inspect reuse of the same
definition. Missing, empty, non-string or unknown input fields fail admission.

`CMakeLists.txt` builds a copied example against the exported package. For a
stronger independence check, build only the general package and consume its
relocated install in a second container without the source tree or network:

```bash
python3 scripts/verify_workflow_task_exports.py --core-only \
  --output tmp/task_core_exports_review
```

Choose a new output directory for each verification. The result records source
hashes, image identity and isolated build/consumer logs. The full application
export check is:

```bash
python3 scripts/verify_workflow_task_exports.py \
  --output tmp/workflow_exports_review
```

Read the [contract](../../docs/task_contract.md) and
[authoring procedure](../../docs/task_template.md) before adding device behavior.
