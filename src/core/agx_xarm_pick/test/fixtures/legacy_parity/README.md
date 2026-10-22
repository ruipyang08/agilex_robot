# Legacy/modular execution parity input

This test-only graph represents the control and perception values in the current
`config/test_tube_fixture_profiles.yaml`, `test_tube_grasp.yaml`,
`test_tube_preparation.yaml`, `test_tube_search.yaml`, and `arm_execution.yaml`.
The two loaders receive independent inputs; the test does not construct expected
values by calling either loader's setters or duplicating its conversion rules.
Update this graph deliberately when the commissioned legacy values change.

The modular schema also requires rack dimensions/slots, station association and
support dimensions that legacy execution did not own. These remain synthetic
test metadata inherited from the two-table fixture, not physical calibration or
permission to dispatch this graph. The legacy adapter must leave absent facts
absent. Configuration IDs, provenance and context-source labels differ by design.

`TubeConfiguration.AllLegacyControlAndPerceptionFieldsMatchModularInput` compares
all 99 current motion/configuration fields, nested fields, robot interfaces,
classifiers, existing domain geometry and canonical perception bytes. Its grouped
test helper must be extended whenever those typed configuration structs gain a
field. A 1e-12 floating-point roundoff allowance covers radians/degrees conversion;
it is not a motion, observation or qualification tolerance.
