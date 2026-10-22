# Navigation task inspection

Software checks must establish recipe/profile admission, module registration,
independent consumer linkage, cancellation and bounded stop behavior. A physical
simulation inspection additionally confirms that the rover departs only after
travel readiness, reaches the selected destination, stops and establishes the
new measured hold.

In a repository checkout, use
`docs/development/simulation/workbench_workflow_operation.md` for the current
two-terminal commands. Inspect the task result for `status: succeeded`,
`detail: fresh measured destination arrival hold` and the selected target. A
successful action result without the final stopped hold is not task success.
Simulation acceptance does not qualify onboard hardware or an untested travel
profile.
