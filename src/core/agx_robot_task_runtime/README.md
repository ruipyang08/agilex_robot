# Optional robot task runtime

This deployable library provides measured robot custody/effects, base guards and
robot-specific invocation ports. It depends on `agx_task_workflow`, with no
navigation, grasp, MoveIt or application-host dependency.

The exported library is `robot_task_runtime`; public headers live under
`include/agx_robot_task_runtime/`. `RobotTaskRuntimeBinding` composes the generic
invocation binding and adds the host's authoritative `RobotStateStore` and an
optional measured held-arrival callback. All ports use the generic invocation's
lock and expiry. Interrupted active tasks may report verified physical effects;
they cannot start a new transfer. Expired tasks cannot apply effects.

`task_metadata.hpp` declares robot-owned descriptive resource/requirement/effect
IDs. `robot_sequence_plan.hpp` contains the prospective robot value used by the
existing navigation/grasp factories. Neither is part of the general task contract.

Physical state still comes from observed evidence, including its original source
and receipt timestamps. The extraction preserves conflict, custody, pending-work
and stopping rules. It does not qualify any new physical motion or fixture.
