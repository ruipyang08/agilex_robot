# Onboard workbench navigation

This package owns deployable ROS1 workbench navigation on the onboard computer.
The same source runs in the offboard Noetic container against simulator state.

- [Base authority and command protocol](docs/base_authority.md): measured holds,
  fixed-expiry departure permits, continuous readiness, operation-bound producer
  commands, stopped navigation-to-docking handover and explicit fault reset.
- [Continuous localization](docs/localization.md): fresh source pose and odometry,
  connected TF, bounded stationary AMCL updates, and Isaac-owned simulation pose.
- [Motion-aware route execution](docs/route_execution.md): admission, following,
  parameter ownership and docking handover.
- [Effective motion limits](docs/motion_limits.md): startup/transition acceleration,
  final-command collision checks and the physical qualification boundary.
- `RangerGlobalPlanner` retains the original map-goal timestamp on its endpoint;
  `RangerLocalPlanner` validates that nonce before accepting a plan and publishes
  `AuthorizedBaseCommand` directly. Standard move_base Twist is diagnostic only.
- `ros_mission_ports.py` binds the previous-goal-terminal navigation operation
  before dispatch and captures docking authorization before computation.

The final vendor `/cmd_vel` remains Twist, with `base_command_arbiter.py` as its
sole autonomous publisher. Install producer messages/services, planner library,
mission ports and all simulation relays together: old raw internal Twist senders
are incompatible with the operation-fenced protocol. Guarded workflow departure
must remain disabled until the measured travel profile and host integration pass
their qualification gates.

The [workflow navigation task](../agx_robot_workflow/docs/tasks/navigate.md)
composes qualified preparation, readiness-bound release, this navigation action
and a fresh destination hold. It accepts only the destination as operation intent;
[configuration selection](../agx_robot_workflow/docs/configuration.md) owns
qualified profiles. New task authors should start at the
[workflow module guide](../agx_robot_workflow/README.md). New two-station empty,
loaded and missed-pick continuation physical qualification remains pending;
software implementation and earlier single-station evidence do not enable it.

The workflow host currently blocks hardware composed travel because the hardware
controller's initial desired-state trajectory splice is not validated. Mode/profile
labels cannot override this gate.

The registration CLI accepts `--alignment-longitudinal-m` (default `0.0`) as an
explicit docking offset, separate from the marker pose fitted from camera/TF
observations. The final distance is the parking front reference plus the parking
safety distance plus this offset. The registry writer rejects nonfinite offsets
and negative final distances. For the existing simulation tag-centered convention,
`-0.36` cancels its measured/configured `0.36 m` front reference when the safety
distance is zero; it does not translate the registered tag or staging pose.
