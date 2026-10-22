# Task definition contract

The public C++17 API is in namespace `agx_task_workflow`, exported by the
`task_workflow` library. The generic package depends on yaml-cpp and platform
threads; it does not construct ROS nodes or require robot state.

| Header | Responsibility |
| --- | --- |
| `config.hpp` | Strict version-2 recipes and source-relative configuration paths |
| `task_contract.hpp` | Definitions, typed input schemas, dependency and effect descriptions |
| `task_registry.hpp` | Registration, detached descriptions and whole-recipe preparation |
| `planning_state.hpp` | Copyable prospective domain values |
| `runner.hpp` | Sequential entry, execution, stop, cleanup and continuation |
| `execution_context.hpp` | One fixed monotonic deadline and sticky interruption cause |
| `task_runtime_ports.hpp` | Invocation identity, checks, deadline and shared lifetime token |

## Definition and preparation

`RegisterTask(registry, definition)` registers a unique task type. A complete
`TaskDefinition` contains a positive `contract_version`, description, input
schema, preparation callback, interfaces, resources, requirements and expected
effects. `DescribeTask` returns a copy without preparing or executing anything.
Prefer `PrepareRecipe` over calling a factory directly: it validates the entire
recipe and unions compatible interface declarations before dispatch.

`TaskSpec` owns `id`, `type`, optional `config_path` and `failure_policy`. Other
YAML fields become task parameters. Supported input types are nonempty string,
boolean, integer and finite number. Duplicate/unknown/missing fields reject.
Factories own domain ranges and immutable configuration admission. Preparation
must not wait for live observations, request runtime ports or command devices.

The factory returns `PreparedTask` with matching identity, execution callback,
entry check, stop, cleanup and a positive finite stop timeout. Use `TASK_RUNNING`
as the entry phase for new types; `NAVIGATE` and `GRASP` remain accepted for the
existing application's reporting compatibility. Incomplete legacy factories
cannot authorize continuing after a failure.

Three optional bindings separate preparation from live installation/runtime work:

- `admission_check()` performs read-only installed-service checks after **every**
  task has prepared. The host calls these, then checks cross-task compatibility
  and the union of required interfaces, before any task begins.
- `begin()` selects the invocation's adapters after host context activation and
  before entry. It must return promptly and issue no motion commands. The runner
  checks interruption before and after it.
- `continuation_check(result)` certifies the task's owned effects and work after
  bounded failure stop/cleanup. It must reject unresolved effects or workers.
  The host separately checks persistent shared invariants. No certificate means
  no continuation, including for tasks that declare no robot resources.

## Dependencies and prospective state

Resource, state-requirement and expected-effect kinds are lowercase namespaced
IDs, for example `lab.fridge`, `lab.fridge_closed` or `lab.temperature_recorded`.
Each dot-separated segment starts with a letter and contains letters, digits or
underscores. Duplicate resources reject. Timing remains `Entry` or `Continuous`;
effect conditions remain `TaskSucceeded` or `VerifiedPhysicalEvent`.

These IDs describe contracts. They do not allocate devices, install guards,
verify effects or permit continuation. Task adapters provide those operations.
Robot-specific IDs and measured state belong to `agx_robot_task_runtime`.

Interfaces retain typed `Service`, `Action` or `Topic` declarations with logical
key, endpoint, ROS type and topic direction. A bound endpoint may resolve a
declared empty endpoint; it cannot change a fixed endpoint, key, type or direction.
Conflicting declarations reject the whole recipe. Runtime interface availability
and handshake checks remain application/adapter responsibilities.

`PreparationState::planning.Get<T>(key)` returns a detached value, defaulting to
`T{}` if absent. `Set<T>` validates the namespaced key and rejects changing an
existing key's type. Values must own their data and contain no mutable shared
state, runtime stores or live device handles. This is prospective planning data.
Adapters separately verify measured execution prerequisites.

Preparation is transactional: each factory receives a copied prospective state.
For a task with `on_failure: continue`, its success-state writes are discarded
and expected effects remain in `conditional_effects`. Later factories must not
assume its success. A failed later admission dispatches no earlier task.

## Runtime and interruption

The host activates `TaskRuntimeBinding` immediately before the runner's first
task guard. Ports pin run, task and operation identity plus a fixed deadline.
Factories may capture a ports provider, but call it only during execution.
Old ports expire when the binding expires, is destroyed or activates another
invocation. Reusing a task ID does not revive old ports.

Optional domain services use `InvocationToken::WithActive` to share the same
invocation lock. Their callbacks must be bounded and must not re-enter the token
or binding. The lock serializes an admitted operation with expiry. Cancellation
is distinct from expiry: domain adapters may still record verified facts for an
active interrupted invocation, while refusing new commands or transfer intent.

Execution success must follow measured success verification. The runner preserves
reported effects/revisions through failure stop and cleanup. Continuing requires
an ordinary failure, confirmed stop, completed cleanup, a task-owned continuation
certificate and explicit host reconciliation. A descriptive effect or valid resource ID supplies none of those
permissions. Cancellation, deadline, fault and unconfirmed stop remain terminal.

See the [authoring procedure](task_template.md) and
[standalone example](../examples/record_value/README.md).
