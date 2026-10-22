# External-rack grasp inspection

Software verification covers frozen configuration identity, required preceding
arrival, empty-custody admission, effect reconciliation, cancellation and
independent module consumption. Manual simulation inspection must confirm the
perception-guided sequence: preparation, search, visual alignment, layout
snapshot, pregrasp, insertion, closure, extraction and retained return.

In a repository checkout, use
`docs/development/simulation/workbench_workflow_operation.md` for the current
two-terminal commands. The workflow result must report the grasp task
`succeeded`; custody/effect records
must reflect observed capture and extraction. Camera alignment alone is not task
success. A missed pick or later extraction failure preserves its measured
partial state. Simulation evidence does not qualify hardware or a different
fixture configuration.
