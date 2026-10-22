# Resolved tube task configuration

`agx_xarm_pick::ResolveTubeTaskConfig(path)` is the ROS-independent C++ resolver
for a tube task's physical and qualified behavior configuration. It returns
`std::shared_ptr<const ResolvedTubeTaskConfig>` or throws `std::invalid_argument`.
The resolver never reads or writes the ROS parameter server and never commands a
robot. The returned `motion` value is the typed projection for the existing
mission, alongside typed domain identities and source provenance.

The legacy ROS parameter loader and combined `test_tube_fixture_profiles.yaml`
remain readable through the existing entrypoints. The new resolver does not
reinterpret or silently migrate those files. In particular, a legacy combined
profile is not a valid new task file.

## Ownership and the reference graph

Each document is a mapping with required `schema_version: 1`, `kind`, and `id`.
All keys listed below are required unless explicitly marked optional. Unknown
keys, duplicate mapping keys, multiple YAML documents, YAML merge keys, nulls,
reference cycles, incompatible document kinds, and recursive aliases are rejected.
There are no recursive merges or precedence rules.

```text
taskA.yaml (tube_task)
  installation -> installationA.yaml (fixture_installation)
                    fixture_profile -> rack.yaml (rack_profile: typeA)
                    support_surface -> tableA.yaml (support_surface)
  container -> tube.yaml (container_profile)
  behavior -> behavior.yaml (tube_behavior)
  robot -> robot.yaml (robot_profile)

taskB.yaml uses installationB.yaml -> the same rack.yaml + tableB.yaml
```

The complete synthetic graph is in `agilex_ws/src/core/agx_xarm_pick/test/fixtures/tube_task_config/`. It is test
input, not a production calibration. Table A and table B have distinct measured
heights, transforms, stations, and fixture instance IDs. They share the exact
same `typeA` rack and tube/behavior/robot files. No Isaac scene properties are
part of this onboard configuration graph; simulation setup remains offboard.

The task contains references only, besides its document header:

```yaml
schema_version: 1
kind: tube_task
id: pickA
installation: installationA.yaml
container: tube.yaml
behavior: behavior.yaml
robot: robot.yaml
```

Target color is operation intent and is deliberately absent from this file.
The behavior owns qualified color classifiers; it does not choose which tube to
pick. The mission adapter must check the requested color exists and copy that
classifier into its per-operation state.

| Kind | Fields besides the document header | Responsibility |
| --- | --- | --- |
| `tube_task` | `installation`, `container`, `behavior`, `robot` | References to four domain documents |
| `fixture_installation` | `fixture_profile`, `support_surface`, `station_id`, `parent_frame`, `fixture_frame`, `measured_transform`, `registration` | One uniquely identified physical installation |
| `rack_profile` | `dimensions_xyz_m`, `layout`, `slots`, `supported_container_profiles` | Reusable rack geometry and supported tube profile IDs |
| `support_surface` | `station_id`, `frame_id`, `dimensions_xyz_m`, `surface_height_above_floor_m`, `height_tolerance_m` | Measured table/support geometry |
| `container_profile` | `body_diameter_m`, `body_height_m`, `cap_diameter_m`, `cap_height_m`, `diameter_tolerance_m`, `height_tolerance_m` | Reusable tube dimensions |
| `tube_behavior` | `task_type`, `perception`, `motion` | Qualified perception and manipulation settings |
| `robot_profile` | `planning_frame`, `color_frame`, `interfaces`, `lift`, `transforms`, `gripper`, `robot_state_max_age_sec`, `arm_execution` | Robot geometry, capabilities, and ROS interfaces |

Strings must be nonempty strings. Unquoted numeric/boolean values cannot be used
as IDs. Numeric fields must be finite YAML numbers, not quoted strings; count
fields must be decimal integers. Booleans use unquoted `true` or `false`. Distances
are metres, angles are radians, durations are seconds. Vendor gripper command,
force, and speed values retain their explicitly qualified native units. HSV and
pixel values are detector units; execution coefficients are dimensionless.

## Installation, support, rack, and container schemas

`fixture_installation.id` is the `fixture_instance_id`, distinct from the rack's
`fixture_profile_id`. It must be unique for each physical installation. Its
`parent_frame` and support `frame_id` must equal the robot `planning_frame`; its
`station_id` must equal the support's `station_id`. The fixture child frame must
differ from the parent. Version 1 does not resolve arbitrary TF graphs.

`measured_transform` contains exactly:

- `translation_xyz_m`: three finite numbers, in the parent frame.
- `quaternion_xyzw`: four finite numbers, unit norm within `1e-6`.

`registration` contains exactly `required`, `maximum_translation_error_m`, and
`maximum_yaw_error_rad`, plus optional string `tag_id`. Required registration
requires a tag ID. Tolerances must be positive, with yaw at most pi radians.
Accepting registration metadata does not implement tag registration; the runtime
adapter must reject an unsupported registration requirement before motion.

The support dimensions have three positive components. Its measured height and
height tolerance must be positive. The installation's zero-lift Z must agree
with `surface_height_above_floor_m - arm_base_height_at_zero_lift_above_floor_m`
within the height tolerance and stay inside the qualified workspace. Thus a new
table installation must supply its actual measurements, not inherit a table
height from a generic rack.

Rack `layout` contains exactly `rows`, `columns`, `center_pitch_m`,
`rack_height_above_surface_m`, `pitch_tolerance_m`, `maximum_missing_slots`,
`minimum_layout_inliers`, and `maximum_lattice_rms_error_m`. Rows and columns are
positive counts; the required inlier count equals the total slots minus allowed
missing slots. Rack height may be zero. Pitch/tolerance/error and cap-clearance
relationships preserve the existing mission's validation.

`slots` is a nonempty sequence of mappings with exactly `id`, `row`, `column`,
and `center_xy_m`. IDs and integer coordinates are unique. Every layout coordinate
is present. Centers must agree with the square lattice pitch within `1e-9` m;
columns increment X and rows increment Y. Rack dimensions must contain the lattice
including cap diameter. `supported_container_profiles` is a nonempty sequence of
unique profile IDs and must contain the selected container's ID.

All six container dimensions/tolerances are positive. Cap diameter must exceed
body diameter. The resolver also checks compatibility with rack pitch, insertion
clearance, gripper aperture calibration, and grasp offset.

## Behavior schema

`task_type` is currently `test_tube_pickup`. `perception` contains exactly:

- `roi_normalized`: `[x_min, y_min, x_max, y_max]`, ordered within `[0, 1]`.
- `minimum_layout_saturation`: integer in `[1, 255]`.
- `color_classifiers`: a nonempty mapping from color names to mappings containing
  exactly `hsv_lower` and `hsv_upper`, each three integers with valid ordered HSV
  bounds. This is classifier calibration, not target selection.
- `cap_top_height_band_m`, `minimum_cap_circularity`, `maximum_cap_axis_ratio`,
  `maximum_target_temporal_spread_px`, `maximum_target_center_error_px`,
  `maximum_recentering_iterations`.
- `required_consistent_frames`, `synchronizer_queue_size`, `max_sync_skew_sec`,
  `detection_timeout_sec`, `maximum_frame_age_sec`, `max_interframe_spread_m`.
- `workspace_min_xyz_m` and `workspace_max_xyz_m`: three numbers each, ordered on
  all axes.

`motion` contains exactly `preparation_clearance_above_cap_m`,
`pregrasp_clearance_m`, `grasp_offset_below_cap_top_m`, `lift_clearance_m`,
`downward_axis_tolerance_rad`, `planning_timeout_sec`, `gripper_timeout_sec`,
`hold_duration_sec`, `cartesian_step_m`, `arm_speed`, `preparation`, and `search`.
Positive clearances and timing limits are mandatory. The grasp offset must fit
inside the cap height. Downward tolerances are positive and at most pi/2 radians.

`arm_speed` requires `default_coefficient`; it may additionally contain any of
`phase1_preparation`, `phase2_preparation`, `search`, `visual_alignment`,
`pregrasp_adjustment`, `insertion`, `extraction`, `return_with_tube`, and `recovery`.
These coefficients and the robot's velocity/acceleration scales must be in
`(0, 1]`. The resolver computes the per-phase products once.

`preparation` has exactly two mappings:

- `phase_one`: `joint_positions_rad` (six numbers), `expected_tcp_xyz_m` (three
  numbers), `downward_axis_tolerance_rad`.
- `phase_two`: `centerline_y_m`, `downward_axis_tolerance_rad`.

`search` contains exactly `centerline_y_m`, `initial_centerline_tolerance_m`,
`centerline_joint_position_rad`, `centerline_joint_tolerance_rad`,
`initial_tcp_x_bounds_m`, `initial_tcp_z_bounds_m`, `view_region`,
`visual_alignment`, `observation_y_positions_m`, `downward_axis_tolerance_rad`,
`minimum_adjacent_view_overlap`, `settle_duration_sec`, and
`detector_attempts_per_waypoint`.

`view_region` contains exactly `x_bounds_m`, `preferred_x_m`,
`clearance_above_cap_bounds_m`, `preferred_clearance_above_cap_m`,
`sampling_step_m`, `entry_y_m`, `waypoint_y_tolerance_m`,
`maximum_candidate_attempts`, and `planning_time_per_candidate_sec`.

`visual_alignment` contains exactly `u_axis_gain_m_per_px`,
`v_axis_gain_m_per_px`, `target_center_tolerance_px`,
`required_centered_observations`, `maximum_step_m`, `minimum_retry_step_m`,
`maximum_total_displacement_m`, `maximum_iterations`, `stationary_target_retries`,
and `minimum_pixel_error_improvement_px`.

The resolver reuses the existing pure search, view-region, alignment, lift, and
geometry contracts. Search waypoints must descend strictly in Y, contain at least
three positions, and fit the qualified workspace. The complete lift uncertainty
and search-clearance envelope must also fit. These constraints are not relaxed
when splitting a previously combined configuration into domain files.

## Robot schema

`interfaces` contains exactly `detector_service`, `joint_state_topic`,
`gripper_command_topic`, and `gripper_state_topic`; these and the lift interface
names must be absolute ROS names.

`lift` contains exactly `service_name`, `state_topic`,
`arm_base_height_at_zero_lift_above_floor_m`, `minimum_position_m`,
`maximum_position_m`, `target_tolerance_m`, `state_max_age_sec`,
`motion_timeout_sec`, and `maximum_speed_request_m_s`. The target must fit the
lift range. The speed must convert exactly to positive integer native mm/s;
tolerance converts to rounded native mm only after range checking.

`transforms` contains exactly `link6_to_grasp_tcp_xyz_m`,
`link6_to_grasp_tcp_quaternion_xyzw`, `link6_to_camera_xyz_m`, and
`link6_to_camera_quaternion_xyzw`. Quaternions must be unit length. The camera
optical and grasp approach axes must agree within the qualified downward-axis
tolerance.

`gripper` contains exactly `fully_open_command`, `close_command`,
`insertion_radial_clearance_m`, `finger_thickness_m`, `finger_width_m`,
`tube_clearance_margin_m`, `command_gap_samples`, `gripper_force`,
`gripper_speeds`, `gripper_command_tolerance`, and `gripper_state_max_age_sec`.
`command_gap_samples` is a sequence of `[native_command, gap_m]` pairs; commands
and gaps must increase, cover the open/closed command range, and provide enough
insertion aperture. `gripper_speeds` contains exactly `preclose`, `final_close`,
and `recovery`, each in native `[1, 100]`; force is in native `[20, 100]`.

`arm_execution` contains exactly `velocity_scale` and `acceleration_scale`, each
in `(0, 1]`. `robot_state_max_age_sec` is positive.

## Resolution, identity, and adapter boundary

References resolve relative to their containing file, not the process working
directory. Absolute references are accepted for commissioning but relative paths
are recommended for offline copying. Each canonical file is read once during a
resolution. Reference paths are never reopened by the returned snapshot. If files
change later, an existing result remains unchanged; a new resolution sees the new
files. A deployment should replace its configuration graph between operations.

`config_id` is `sha256:` plus a digest of canonical domain documents and reference
identities. Mapping order, comments, numeric spelling, and reference file paths do
not affect that ID. It includes schema versions and all supplied domain facts.
`sources` holds kind, ID, SHA256 of the exact input bytes, and an absolute path for
each file. Source hashes therefore detect comment/format changes even when the
resolved configuration identity is unchanged. Absolute paths are provenance only,
so copying the same graph onto an offline onboard computer preserves `config_id`.

The resolver, runtime binder, immutable mission injection and configured
perception clients are implemented. `BindTubeTaskMotion(snapshot, color)` selects
the requested classifier and validates the current adapter before motion. The
mission, target provider and metric requests retain that same snapshot. The
workflow host resolves and admits the whole recipe first; it never rewrites a
shared active profile to switch installations.

Version 1 rejects required tag registration, nonhorizontal rack installation,
unsupported planning/camera frames and unsupported context type at binding.
It supports horizontal yaw and the `link_base` / `cam1_color_optical_frame`
frames. The mission projection retains its established units, with explicit
radians-to-degrees and metres-to-native-mm conversions where the vendor surface
requires them. There is no generic TF graph resolver or recursive merger.

`SnapshotLegacyTubeTaskConfig` adapts the previously loaded ROS subtrees to the
same const mission interface. The original combined profile stays readable;
subsequent parameter changes do not alter an admitted mission. Its provenance
records the captured parameter content rather than pretending it came from the
new file graph. Legacy data does not gain fabricated physical slot identity.

Configured perception receives only its typed projection, not motion/gripper
settings or host paths. It recomputes the projection digest, pins identity/content
bindings and isolates request histories. The perception protocol guide is
`README.md` in the directory returned by `rospack find agx_tube_perception`, in
both source and relocated install workspaces. See also the [workflow selection
guide](../../agx_robot_workflow/docs/configuration.md). A domain-valid profile
is still subject to adapter and physical qualification gates.

The ROS test `tube_config_snapshot.test` includes
`AllLegacyControlAndPerceptionFieldsMatchModularInput`. It compares the current
legacy configuration files against an independent equivalent modular test graph:
all 99 `TubeTaskMotionConfig` fields and nested values, robot interfaces, color
classifiers, existing domain geometry, and every canonical perception wire field.
The grouped test-only comparator must grow with these typed structures. Only
floating-point roundoff from angle-unit conversion is tolerated. Configuration
IDs/provenance and the two context-source labels are asserted separately; absent
legacy rack slots/station metadata are not invented. The test graph's additional
schema metadata is synthetic and grants no physical permission.

The focused Noetic ROS test passed both snapshot/mutation and complete execution
parity checks. Host `tests/test_tube_legacy_parity.py` also checks nested-field
mismatch detection using the production resolver/binder. These are software
compatibility checks, not a travel or hardware qualification.

Run the production resolver tests on the offboard computer with yaml-cpp,
OpenSSL development libraries, and a C++17 compiler installed:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests/test_resolved_tube_config.py
```

The Python test runner compiles and calls the actual C++ parser; it contains no
second YAML interpretation or runtime schema implementation.
