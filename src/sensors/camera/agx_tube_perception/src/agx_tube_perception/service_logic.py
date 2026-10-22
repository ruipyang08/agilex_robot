"""ROS-independent safety helpers for the tube-cap detector service."""

from dataclasses import dataclass
from math import ceil, floor, isfinite
from numbers import Integral, Real
from types import MappingProxyType
from typing import Tuple

import numpy as np


SUCCESS = 0
INVALID_REQUEST = 1
TARGET_ABSENT = 2
AMBIGUOUS_TARGET = 3
STALE_FRAME = 4
TF_UNAVAILABLE = 5
OUT_OF_WORKSPACE = 6
INCONSISTENT_TARGET = 7
LAYOUT_UNDERCONSTRAINED = 8
PROFILE_MISMATCH = 9
_INT64_MIN = -(1 << 63)
_INT64_MAX = (1 << 63) - 1


@dataclass(frozen=True)
class TimingFailure:
    status: int
    detail: str


@dataclass(frozen=True)
class LiftTarget:
    millimetres: int
    metres: float


@dataclass(frozen=True)
class RigidTransform:
    """A ROS-independent parent-to-child rigid transform."""

    translation_xyz: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]


def _stamp_ns(value, name):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError("{0} must be an integer nanosecond stamp".format(name))
    if value < 0:
        raise ValueError("{0} must be nonnegative".format(name))
    return int(value)


def _positive_seconds(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{0} must be a finite positive number".format(name))
    result = float(value)
    if not isfinite(result) or result <= 0.0:
        raise ValueError("{0} must be a finite positive number".format(name))
    return result


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{0} must be a finite number".format(name))
    result = float(value)
    if not isfinite(result):
        raise ValueError("{0} must be a finite number".format(name))
    return result


def _finite_nonnegative(value, name):
    result = _finite_number(value, name)
    if result < 0.0:
        raise ValueError("{0} must be nonnegative".format(name))
    return result


def compute_lift_target(
    table_surface_m,
    zero_lift_arm_base_m,
    minimum_m,
    maximum_m,
):
    """Compute one absolute lift request using C++ llround semantics."""
    table_surface = _finite_number(table_surface_m, "table surface")
    zero_lift_arm_base = _finite_number(
        zero_lift_arm_base_m, "zero-lift arm base height"
    )
    minimum = _finite_number(minimum_m, "minimum lift position")
    maximum = _finite_number(maximum_m, "maximum lift position")
    if minimum > maximum:
        raise ValueError("lift position bounds must be ordered")
    target_m = table_surface - zero_lift_arm_base
    if target_m < minimum or target_m > maximum:
        raise ValueError("derived lift target is outside commissioned bounds")
    target_mm = target_m * 1000.0
    if target_mm < _INT64_MIN or target_mm > _INT64_MAX:
        raise ValueError("derived lift target exceeds signed 64-bit millimetres")
    rounded_mm = (
        floor(target_mm + 0.5)
        if target_mm >= 0.0
        else ceil(target_mm - 0.5)
    )
    return LiftTarget(int(rounded_mm), float(rounded_mm) / 1000.0)


def classify_lift_feedback(
    has_feedback,
    initialized,
    back_height_mm,
    target_height_mm,
    expected_target_height_mm,
    stamp_ns,
    now_ns,
    wall_receipt_age_sec,
    maximum_age_sec,
    tolerance_mm,
    faulted,
):
    """Return a stale-frame failure unless measured vendor lift state is safe."""
    if type(has_feedback) is not bool or not has_feedback:
        return TimingFailure(STALE_FRAME, "lift feedback is missing")
    if type(initialized) is not bool or not initialized:
        return TimingFailure(STALE_FRAME, "lift feedback is uninitialized")
    if type(faulted) is not bool or faulted:
        return TimingFailure(STALE_FRAME, "lift feedback reports a fault")
    height_values = (
        ("measured lift height", back_height_mm),
        ("reported lift target height", target_height_mm),
        ("expected lift target height", expected_target_height_mm),
    )
    for name, value in height_values:
        if isinstance(value, bool) or not isinstance(value, Integral):
            return TimingFailure(
                STALE_FRAME, "{0} must be an integer".format(name)
            )
        if value < 0:
            return TimingFailure(
                STALE_FRAME, "{0} must be nonnegative".format(name)
            )
    if isinstance(tolerance_mm, bool) or not isinstance(tolerance_mm, Integral):
        return TimingFailure(
            STALE_FRAME, "lift target tolerance must be an integer"
        )
    if tolerance_mm < 0:
        return TimingFailure(
            STALE_FRAME, "lift target tolerance must be nonnegative"
        )
    try:
        source_stamp_ns = _stamp_ns(stamp_ns, "lift feedback stamp")
        current_stamp_ns = _stamp_ns(now_ns, "current lift stamp")
        receipt_age = _finite_nonnegative(
            wall_receipt_age_sec, "lift receipt age"
        )
        maximum_age = _positive_seconds(
            maximum_age_sec, "lift maximum age"
        )
    except ValueError as error:
        return TimingFailure(STALE_FRAME, str(error))
    if source_stamp_ns == 0:
        return TimingFailure(STALE_FRAME, "lift feedback stamp is zero")
    stamp_age_ns = current_stamp_ns - source_stamp_ns
    if stamp_age_ns < 0:
        return TimingFailure(
            STALE_FRAME, "lift feedback stamp is in the future"
        )
    if (
        stamp_age_ns > int(maximum_age * 1_000_000_000)
        or receipt_age > maximum_age
    ):
        return TimingFailure(STALE_FRAME, "lift feedback is stale")
    if int(target_height_mm) != int(expected_target_height_mm):
        return TimingFailure(
            STALE_FRAME, "lift feedback target does not match the request"
        )
    if abs(int(back_height_mm) - int(expected_target_height_mm)) > int(
        tolerance_mm
    ):
        return TimingFailure(
            STALE_FRAME,
            "measured lift height is outside requested target tolerance",
        )
    return None


def resolve_lift_relative_z(zero_lift_z_m, back_height_mm):
    """Translate one zero-lift link_base z value by measured lift feedback."""
    zero_lift_z = _finite_number(zero_lift_z_m, "zero-lift z")
    if isinstance(back_height_mm, bool) or not isinstance(
        back_height_mm, Integral
    ):
        raise ValueError("measured lift height must be an integer")
    if back_height_mm < 0:
        raise ValueError("measured lift height must be nonnegative")
    return zero_lift_z - int(back_height_mm) / 1000.0


def lift_height_batch_is_consistent(
    request_height_mm,
    current_height_mm,
    observation_heights_mm,
    target_height_mm,
    tolerance_mm,
):
    """Require request, current, target, and successful frames in one span."""
    if isinstance(tolerance_mm, bool) or not isinstance(tolerance_mm, Integral):
        return False
    if tolerance_mm < 0 or isinstance(observation_heights_mm, (str, bytes)):
        return False
    try:
        observations = tuple(observation_heights_mm)
    except TypeError:
        return False
    heights = (
        request_height_mm,
        current_height_mm,
        target_height_mm,
    ) + observations
    if any(
        isinstance(height, bool)
        or not isinstance(height, Integral)
        or height < 0
        for height in heights
    ):
        return False
    return max(heights) - min(heights) <= tolerance_mm


def classify_sensor_timing(
    color_stamp_ns,
    camera_info_stamp_ns,
    now_stamp_ns,
    maximum_age_sec,
):
    """Return a stale-frame failure when synchronized RGB stamps are unusable."""
    color_ns = _stamp_ns(color_stamp_ns, "color stamp")
    camera_info_ns = _stamp_ns(camera_info_stamp_ns, "camera info stamp")
    now_ns = _stamp_ns(now_stamp_ns, "current stamp")
    maximum_age_ns = int(
        _positive_seconds(maximum_age_sec, "maximum age") * 1_000_000_000
    )

    if color_ns == 0 or camera_info_ns == 0:
        return TimingFailure(STALE_FRAME, "synchronized sensor stamp is zero")
    if color_ns != camera_info_ns:
        return TimingFailure(
            STALE_FRAME,
            "color image and camera info require an exact stamp match",
        )
    age_ns = now_ns - color_ns
    if age_ns < 0:
        return TimingFailure(STALE_FRAME, "synchronized frame stamp is in the future")
    if age_ns > maximum_age_ns:
        return TimingFailure(
            STALE_FRAME,
            "synchronized frame exceeded the configured age limit",
        )
    return None


class ObservationEpoch:
    """Keep one outcome per increasing measurement stamp and reset on rollback."""

    def __init__(self, outcomes):
        self._outcomes = outcomes
        self._last_stamp_ns = None
        self.epoch = 0

    def begin(self, stamp_ns):
        stamp_ns = _stamp_ns(stamp_ns, "observation stamp")
        if self._last_stamp_ns is None:
            self._last_stamp_ns = stamp_ns
            return True
        if stamp_ns == self._last_stamp_ns:
            return False
        if stamp_ns < self._last_stamp_ns:
            self._outcomes.clear()
            self.epoch += 1
        self._last_stamp_ns = stamp_ns
        return True

    def append(self, outcome):
        stamp_ns = _stamp_ns(outcome.stamp_ns, "observation stamp")
        if stamp_ns != self._last_stamp_ns:
            raise ValueError("observation must match the active measurement stamp")
        self._outcomes.append(outcome)

    def select_batch(self, not_before_ns, required_count):
        not_before_ns = _stamp_ns(not_before_ns, "request cutoff")
        if (
            isinstance(required_count, bool)
            or not isinstance(required_count, Integral)
            or required_count <= 0
        ):
            raise ValueError("required count must be a positive integer")
        eligible = [
            outcome
            for outcome in self._outcomes
            if outcome.stamp_ns >= not_before_ns
        ]
        if len(eligible) < required_count:
            return None
        batch = eligible[-int(required_count) :]
        stamps = [outcome.stamp_ns for outcome in batch]
        if any(current <= previous for previous, current in zip(stamps, stamps[1:])):
            return None
        return batch


def batch_is_stale(observations, now_stamp_ns, maximum_age_sec):
    """Reject old or future observations in a candidate service batch."""
    now_ns = _stamp_ns(now_stamp_ns, "current stamp")
    maximum_age_ns = int(
        _positive_seconds(maximum_age_sec, "maximum age") * 1_000_000_000
    )
    for observation in observations:
        stamp_ns = _stamp_ns(observation.stamp_ns, "observation stamp")
        age_ns = now_ns - stamp_ns
        if age_ns < 0 or age_ns > maximum_age_ns:
            return True
    return False


def classify_batch_status(statuses):
    """Return the common status or INCONSISTENT_TARGET for mixed outcomes."""
    values = tuple(statuses)
    if not values:
        raise ValueError("status batch must be nonempty")
    if len(set(values)) != 1:
        return INCONSISTENT_TARGET
    return values[0]


def detection_status(code):
    """Map every known pure-detector failure code without a silent default."""
    statuses = {
        "target_absent": TARGET_ABSENT,
        "ambiguous_target": AMBIGUOUS_TARGET,
        "tf_unavailable": TF_UNAVAILABLE,
        "invalid_image": TF_UNAVAILABLE,
        "out_of_workspace": OUT_OF_WORKSPACE,
        "inconsistent": INCONSISTENT_TARGET,
        "inconsistent_target": INCONSISTENT_TARGET,
        "layout_underconstrained": LAYOUT_UNDERCONSTRAINED,
        "profile_mismatch": PROFILE_MISMATCH,
    }
    try:
        return statuses[code]
    except KeyError:
        raise RuntimeError("unexpected detector code: {0}".format(code))


def three_vector(value, name):
    """Return three finite floats while rejecting string-like iterables."""
    if isinstance(value, (str, bytes)):
        raise ValueError("{0} must contain three numeric values".format(name))
    try:
        result = tuple(float(component) for component in value)
    except (TypeError, ValueError):
        raise ValueError("{0} must contain three numeric values".format(name))
    if len(result) != 3 or not all(isfinite(component) for component in result):
        raise ValueError("{0} must contain three finite values".format(name))
    return result


def _exact_mapping(value, expected_keys, name):
    if not isinstance(value, dict):
        raise ValueError("{0} must be a mapping".format(name))
    actual = set(value)
    expected = set(expected_keys)
    missing = sorted(expected - actual)
    unexpected = sorted(actual - expected)
    if missing or unexpected:
        raise ValueError(
            "{0} keys are invalid (missing={1}, unexpected={2})".format(
                name, missing, unexpected
            )
        )
    return value


def _mapping_with_optional_keys(
    value, required_keys, optional_keys, name
):
    if not isinstance(value, dict):
        raise ValueError("{0} must be a mapping".format(name))
    actual = set(value)
    required = set(required_keys)
    allowed = required | set(optional_keys)
    missing = sorted(required - actual)
    unexpected = sorted(actual - allowed)
    if missing or unexpected:
        raise ValueError(
            "{0} keys are invalid (missing={1}, unexpected={2})".format(
                name, missing, unexpected
            )
        )
    return value


def _nonempty_string(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("{0} must be a nonempty string".format(name))
    return value


def _finite_positive(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{0} must be a finite positive number".format(name))
    result = float(value)
    if not isfinite(result) or result <= 0.0:
        raise ValueError("{0} must be a finite positive number".format(name))
    return result


def validate_mission_context(mission, expected_profile_id, expected_color):
    """Validate the Task-8 mission/profile boundary without ROS dependencies."""

    mission = _exact_mapping(
        mission,
        (
            "active_profile_id",
            "target_cap_color",
            "context",
            "lift",
            "perception",
            "transforms",
            "motion",
        ),
        "tube grasp mission",
    )
    profile_id = _nonempty_string(
        mission["active_profile_id"], "active profile id"
    )
    target_color = _nonempty_string(
        mission["target_cap_color"], "target cap color"
    )
    if profile_id != expected_profile_id:
        raise ValueError("active profile id does not match selected profile")
    if target_color != expected_color:
        raise ValueError("target cap color does not match fixture profile")

    context = _exact_mapping(
        mission["context"],
        (
            "source",
            "planning_frame",
            "nominal_fixture_center_xyz_m",
            "maximum_translation_error_m",
            "maximum_yaw_error_deg",
            "tag_registration_required",
        ),
        "tube grasp context",
    )
    if context["source"] != "configured_demo":
        raise ValueError("context source must be configured_demo")
    planning_frame = context["planning_frame"]
    if planning_frame != "link_base":
        raise ValueError(
            "context planning_frame must be exactly link_base"
        )
    fixture_center = three_vector(
        context["nominal_fixture_center_xyz_m"],
        "nominal fixture center",
    )
    translation_error = _finite_positive(
        context["maximum_translation_error_m"],
        "maximum translation error",
    )
    yaw_error = _finite_positive(
        context["maximum_yaw_error_deg"], "maximum yaw error"
    )
    if type(context["tag_registration_required"]) is not bool:
        raise ValueError("tag_registration_required must be a boolean")
    if context["tag_registration_required"]:
        raise ValueError(
            "tag registration must be disabled for configured_demo context"
        )

    lift = _mapping_with_optional_keys(
        mission["lift"],
        (
            "service_name",
            "state_topic",
            "arm_base_height_at_zero_lift_above_floor_m",
            "minimum_position_m",
            "maximum_position_m",
            "target_tolerance_m",
            "state_max_age_sec",
            "motion_timeout_sec",
        ),
        ("maximum_speed_request_mm_s",),
        "tube grasp lift",
    )
    lift_service_name = _nonempty_string(
        lift["service_name"], "lift service name"
    )
    lift_state_topic = _nonempty_string(
        lift["state_topic"], "lift state topic"
    )
    if not lift_service_name.startswith("/") or not lift_state_topic.startswith(
        "/"
    ):
        raise ValueError("lift service and state names must be absolute")
    zero_lift_arm_base = _finite_nonnegative(
        lift["arm_base_height_at_zero_lift_above_floor_m"],
        "zero-lift arm base height",
    )
    minimum_lift_position = _finite_nonnegative(
        lift["minimum_position_m"], "minimum lift position"
    )
    maximum_lift_position = _finite_nonnegative(
        lift["maximum_position_m"], "maximum lift position"
    )
    if minimum_lift_position > maximum_lift_position:
        raise ValueError("lift position bounds must be ordered")
    target_tolerance = _finite_nonnegative(
        lift["target_tolerance_m"], "lift target tolerance"
    )
    state_max_age = _positive_seconds(
        lift["state_max_age_sec"], "lift state maximum age"
    )
    motion_timeout = _positive_seconds(
        lift["motion_timeout_sec"], "lift motion timeout"
    )

    perception = _exact_mapping(
        mission["perception"],
        (
            "required_consistent_frames",
            "synchronizer_queue_size",
            "max_sync_skew_sec",
            "detection_timeout_sec",
            "maximum_frame_age_sec",
            "max_interframe_spread_m",
            "workspace_min_xyz_m",
            "workspace_max_xyz_m",
        ),
        "tube grasp perception",
    )
    for field in ("required_consistent_frames", "synchronizer_queue_size"):
        value = perception[field]
        if isinstance(value, bool) or not isinstance(value, Integral) or value <= 0:
            raise ValueError("{0} must be a positive integer".format(field))
    for field in (
        "max_sync_skew_sec",
        "detection_timeout_sec",
        "maximum_frame_age_sec",
        "max_interframe_spread_m",
    ):
        _finite_positive(perception[field], field)
    workspace_min = three_vector(
        perception["workspace_min_xyz_m"], "workspace minimum"
    )
    workspace_max = three_vector(
        perception["workspace_max_xyz_m"], "workspace maximum"
    )
    if any(low >= high for low, high in zip(workspace_min, workspace_max)):
        raise ValueError("workspace bounds must be strictly ordered")

    _exact_mapping(
        mission["transforms"],
        (
            "link6_to_grasp_tcp_xyz_m",
            "link6_to_grasp_tcp_quaternion_xyzw",
            "link6_to_camera_xyz_m",
            "link6_to_camera_quaternion_xyzw",
        ),
        "tube grasp transforms",
    )
    # Execution validates the speed values. Perception accepts either explicit
    # schema while consuming only its unchanged search/observation settings.
    configured_motion = mission["motion"]
    arm_keys = ("arm_speed",) if isinstance(configured_motion, dict) and "arm_speed" in configured_motion else ("velocity_scaling", "acceleration_scaling")
    gripper_keys = ("gripper_speeds",) if isinstance(configured_motion, dict) and "gripper_speeds" in configured_motion else ("gripper_speed",)
    motion = _mapping_with_optional_keys(
        mission["motion"],
        (
            "gripper_force",
            "gripper_command_tolerance",
            "gripper_state_max_age_sec",
            "robot_state_max_age_sec",
            "planning_timeout_sec",
            "gripper_timeout_sec",
            "hold_duration_sec",
            "cartesian_step_m",
            "search",
        ) + arm_keys + gripper_keys,
        ("preparation",),
        "tube grasp motion",
    )
    search = _exact_mapping(
        motion["search"],
        (
            "centerline_y_m",
            "initial_centerline_tolerance_m",
            "centerline_joint_position_rad",
            "centerline_joint_tolerance_rad",
            "initial_tcp_x_bounds_m",
            "initial_tcp_z_bounds_m",
            "view_region",
            "visual_alignment",
            "observation_y_positions_m",
            "downward_axis_tolerance_deg",
            "minimum_adjacent_view_overlap",
            "settle_duration_sec",
            "detector_attempts_per_waypoint",
        ),
        "tube grasp search",
    )
    _exact_mapping(
        search["view_region"],
        (
            "x_bounds_m",
            "preferred_x_m",
            "clearance_above_cap_bounds_m",
            "preferred_clearance_above_cap_m",
            "sampling_step_m",
            "entry_y_m",
            "waypoint_y_tolerance_m",
            "maximum_candidate_attempts",
            "planning_time_per_candidate_sec",
        ),
        "tube grasp view region",
    )
    _exact_mapping(
        search["visual_alignment"],
        (
            "u_axis_gain_m_per_px",
            "v_axis_gain_m_per_px",
            "target_center_tolerance_px",
            "required_centered_observations",
            "maximum_step_m",
            "minimum_retry_step_m",
            "maximum_total_displacement_m",
            "maximum_iterations",
            "stationary_target_retries",
            "minimum_pixel_error_improvement_px",
        ),
        "tube grasp visual alignment",
    )
    return MappingProxyType(
        {
            "source": "configured_demo",
            "planning_frame": planning_frame,
            "nominal_fixture_center_xyz_m": fixture_center,
            "maximum_translation_error_m": translation_error,
            "maximum_yaw_error_deg": yaw_error,
            "tag_registration_required": False,
            "lift": MappingProxyType(
                {
                    "service_name": lift_service_name,
                    "state_topic": lift_state_topic,
                    "arm_base_height_at_zero_lift_above_floor_m": (
                        zero_lift_arm_base
                    ),
                    "minimum_position_m": minimum_lift_position,
                    "maximum_position_m": maximum_lift_position,
                    "target_tolerance_m": target_tolerance,
                    "state_max_age_sec": state_max_age,
                    "motion_timeout_sec": motion_timeout,
                }
            ),
        }
    )


def _camera_matrix(camera_matrix_k):
    if isinstance(camera_matrix_k, (str, bytes)):
        raise ValueError("camera intrinsics must contain nine numeric values")
    try:
        raw_values = tuple(camera_matrix_k)
    except (TypeError, ValueError):
        raise ValueError("camera intrinsics must contain nine numeric values")
    if any(isinstance(value, (bool, np.bool_)) for value in raw_values):
        raise ValueError("camera intrinsics must not contain booleans")
    try:
        values = np.asarray(raw_values, dtype=np.float64)
    except (TypeError, ValueError):
        raise ValueError("camera intrinsics must contain nine numeric values")
    if values.shape != (9,) or not np.all(np.isfinite(values)):
        raise ValueError("camera intrinsics must contain nine finite values")
    if values[0] <= 0.0 or values[4] <= 0.0:
        raise ValueError("camera intrinsics require positive focal lengths")
    return values.reshape((3, 3))


def _transform_components(transform):
    try:
        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        raw_translation = (translation.x, translation.y, translation.z)
        raw_quaternion = (
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        if any(
            isinstance(value, (bool, np.bool_))
            for value in raw_translation + raw_quaternion
        ):
            raise ValueError("transform values must not be booleans")
        translation_xyz = np.asarray(
            raw_translation, dtype=np.float64
        )
        quaternion_xyzw = np.asarray(
            raw_quaternion, dtype=np.float64
        )
    except (AttributeError, TypeError, ValueError):
        raise ValueError("transform must contain numeric translation and quaternion")
    if not np.all(np.isfinite(translation_xyz)):
        raise ValueError("transform translation must be finite")
    if not np.all(np.isfinite(quaternion_xyzw)):
        raise ValueError("transform quaternion must be finite")
    norm = float(np.linalg.norm(quaternion_xyzw))
    if abs(norm - 1.0) > 1e-6:
        raise ValueError("transform quaternion must be unit length")
    x, y, z, w = quaternion_xyzw
    rotation = np.asarray(
        (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        ),
        dtype=np.float64,
    )
    return rotation, translation_xyz


def transform_camera_point(camera_xyz, transform):
    """Transform one finite camera-frame point into the planning frame."""

    point = np.asarray(three_vector(camera_xyz, "camera point"), dtype=np.float64)
    rotation, translation = _transform_components(transform)
    result = rotation @ point + translation
    return tuple(float(value) for value in result)


def intersect_pixel_ray_with_horizontal_plane(
    pixel_uv, plane_z_m, camera_matrix_k, transform
):
    """Intersect a calibrated camera pixel ray with a planning-frame Z plane."""

    matrix = _camera_matrix(camera_matrix_k)
    rotation, translation = _transform_components(transform)
    try:
        u, v = pixel_uv
        u = float(u)
        v = float(v)
        plane_z_m = float(plane_z_m)
    except (TypeError, ValueError):
        raise ValueError("pixel and cap plane must contain finite numbers")
    if not np.isfinite(u) or not np.isfinite(v) or not np.isfinite(plane_z_m):
        raise ValueError("pixel and cap plane must contain finite numbers")
    ray_camera = np.asarray(
        (
            (u - matrix[0, 2]) / matrix[0, 0],
            (v - matrix[1, 2]) / matrix[1, 1],
            1.0,
        ),
        dtype=np.float64,
    )
    ray_planning = rotation @ ray_camera
    denominator = float(ray_planning[2])
    if abs(denominator) < 1e-9:
        raise ValueError("camera ray is parallel to cap plane")
    scale = (plane_z_m - float(translation[2])) / denominator
    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError("cap plane intersection is behind camera")
    point = translation + scale * ray_planning
    if not np.all(np.isfinite(point)):
        raise ValueError("cap plane intersection is non-finite")
    return tuple(float(value) for value in point)


def projected_ellipse_diameter_m(
    ellipse, plane_z_m, camera_matrix_k, transform
):
    """Project fitted ellipse axes onto the cap plane and average diameters."""

    try:
        center, axes, angle_deg = ellipse
        center_u, center_v = map(float, center)
        axis_a_px, axis_b_px = map(float, axes)
        angle_rad = np.deg2rad(float(angle_deg))
    except (TypeError, ValueError):
        raise ValueError("ellipse must contain numeric center, axes, and angle")
    values = (center_u, center_v, axis_a_px, axis_b_px, angle_rad)
    if not all(np.isfinite(value) for value in values):
        raise ValueError("ellipse values must be finite")
    if axis_a_px <= 0.0 or axis_b_px <= 0.0:
        raise ValueError("ellipse axes must be positive")
    directions = (
        np.asarray((np.cos(angle_rad), np.sin(angle_rad))),
        np.asarray((-np.sin(angle_rad), np.cos(angle_rad))),
    )
    center_uv = np.asarray((center_u, center_v), dtype=np.float64)
    diameters = []
    for axis_px, direction in zip((axis_a_px, axis_b_px), directions):
        offset = 0.5 * axis_px * direction
        first = intersect_pixel_ray_with_horizontal_plane(
            center_uv - offset, plane_z_m, camera_matrix_k, transform
        )
        second = intersect_pixel_ray_with_horizontal_plane(
            center_uv + offset, plane_z_m, camera_matrix_k, transform
        )
        diameters.append(float(np.linalg.norm(np.subtract(second, first))))
    return float(np.mean(diameters))


def validate_cap_diameter(
    observed_diameter_m, expected_diameter_m, tolerance_m
):
    """Return a physical cap diameter only when it matches the profile."""

    observed = _finite_positive(observed_diameter_m, "observed diameter")
    expected = _finite_positive(expected_diameter_m, "expected diameter")
    tolerance = _finite_positive(tolerance_m, "diameter tolerance")
    if abs(observed - expected) > tolerance:
        raise ValueError(
            "observed cap diameter is outside the configured tolerance"
        )
    return observed


def _strict_vector(value, length, name):
    if isinstance(value, (str, bytes)):
        raise ValueError(
            "{0} must contain {1} finite numbers".format(name, length)
        )
    try:
        raw = tuple(value)
    except TypeError:
        raise ValueError(
            "{0} must contain {1} finite numbers".format(name, length)
        )
    if len(raw) != length or any(
        isinstance(component, (bool, np.bool_))
        or not isinstance(component, Real)
        or not isfinite(float(component))
        for component in raw
    ):
        raise ValueError(
            "{0} must contain {1} finite numbers".format(name, length)
        )
    return tuple(float(component) for component in raw)


def _unit_quaternion(value, name):
    quaternion = _strict_vector(value, 4, name)
    if abs(float(np.linalg.norm(quaternion)) - 1.0) > 1e-6:
        raise ValueError("{0} must be unit length".format(name))
    return quaternion


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotation_from_quaternion(quaternion):
    x, y, z, w = quaternion
    return np.asarray(
        (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        ),
        dtype=np.float64,
    )


def _compose_rigid(left, right):
    left_rotation = _rotation_from_quaternion(left.quaternion_xyzw)
    translation = (
        np.asarray(left.translation_xyz, dtype=np.float64)
        + left_rotation
        @ np.asarray(right.translation_xyz, dtype=np.float64)
    )
    quaternion = _quaternion_multiply(
        left.quaternion_xyzw, right.quaternion_xyzw
    )
    return RigidTransform(
        translation_xyz=tuple(float(value) for value in translation),
        quaternion_xyzw=tuple(float(value) for value in quaternion),
    )


def _inverse_rigid(transform):
    x, y, z, w = transform.quaternion_xyzw
    inverse_quaternion = (-x, -y, -z, w)
    inverse_rotation = _rotation_from_quaternion(inverse_quaternion)
    translation = -inverse_rotation @ np.asarray(
        transform.translation_xyz, dtype=np.float64
    )
    return RigidTransform(
        translation_xyz=tuple(float(value) for value in translation),
        quaternion_xyzw=inverse_quaternion,
    )


def commissioned_search_camera_transform(
    search_tcp_xyz,
    link6_to_tcp_xyz,
    link6_to_tcp_quaternion_xyzw,
    link6_to_camera_xyz,
    link6_to_camera_quaternion_xyzw,
):
    """Compose the commissioned roll-pi search TCP into planning-to-camera."""

    planning_to_tcp = RigidTransform(
        translation_xyz=_strict_vector(
            search_tcp_xyz, 3, "search TCP translation"
        ),
        quaternion_xyzw=(1.0, 0.0, 0.0, 0.0),
    )
    link6_to_tcp = RigidTransform(
        translation_xyz=_strict_vector(
            link6_to_tcp_xyz, 3, "link6-to-TCP translation"
        ),
        quaternion_xyzw=_unit_quaternion(
            link6_to_tcp_quaternion_xyzw, "link6-to-TCP quaternion"
        ),
    )
    link6_to_camera = RigidTransform(
        translation_xyz=_strict_vector(
            link6_to_camera_xyz, 3, "link6-to-camera translation"
        ),
        quaternion_xyzw=_unit_quaternion(
            link6_to_camera_quaternion_xyzw,
            "link6-to-camera quaternion",
        ),
    )
    planning_to_link6 = _compose_rigid(
        planning_to_tcp, _inverse_rigid(link6_to_tcp)
    )
    return _compose_rigid(planning_to_link6, link6_to_camera)


def camera_point_from_planning(planning_xyz, planning_to_camera):
    """Transform one planning-frame point into commissioned optical axes."""

    if not isinstance(planning_to_camera, RigidTransform):
        raise ValueError("planning-to-camera must be a RigidTransform")
    point = np.asarray(
        _strict_vector(planning_xyz, 3, "planning point"),
        dtype=np.float64,
    )
    rotation = _rotation_from_quaternion(
        planning_to_camera.quaternion_xyzw
    )
    camera = rotation.T @ (
        point - np.asarray(
            planning_to_camera.translation_xyz, dtype=np.float64
        )
    )
    return tuple(float(value) for value in camera)


def _layout_point_map(observation):
    grid_u = tuple(observation.grid_u)
    grid_v = tuple(observation.grid_v)
    points = tuple(observation.observed_cap_tops)
    if not (
        len(grid_u)
        == len(grid_v)
        == len(points)
        == observation.lattice_inlier_count
    ):
        raise RuntimeError("layout arrays do not match fitted inlier count")
    result = {}
    for grid_x, grid_y, point in zip(grid_u, grid_v, points):
        if (
            isinstance(grid_x, bool)
            or not isinstance(grid_x, Integral)
            or isinstance(grid_y, bool)
            or not isinstance(grid_y, Integral)
        ):
            raise RuntimeError("layout grid keys must be integers")
        key = (int(grid_x), int(grid_y))
        if key in result:
            raise RuntimeError("layout grid keys must be unique")
        result[key] = np.asarray(
            _strict_vector(point, 3, "layout point"), dtype=np.float64
        )
    if (0, 0) not in result:
        raise RuntimeError("layout does not contain the target grid key")
    return result


def newest_coherent_observation(
    observations, maximum_spread_m, require_layout=False
):
    """Validate a temporal batch and return only its newest coherent sample."""

    values = tuple(observations)
    if not values:
        raise ValueError("observation batch must be nonempty")
    spread = _finite_positive(maximum_spread_m, "maximum spread")
    from .detector import DetectionError, aggregate_points

    aggregate_points(
        [observation.planning_xyz for observation in values], spread
    )
    newest = max(values, key=lambda observation: observation.stamp_ns)
    if not require_layout:
        return newest, tuple(float(value) for value in newest.planning_xyz)

    newest_map = _layout_point_map(newest)
    layout_targets = []
    for observation in values:
        points = _layout_point_map(observation)
        _strict_vector(observation.grid_axis_u, 2, "grid axis u")
        _strict_vector(observation.grid_axis_v, 2, "grid axis v")
        target = np.asarray(
            _strict_vector(
                observation.planning_xyz, 3, "target planning point"
            )
        )
        if np.linalg.norm(target - points[(0, 0)]) > spread:
            raise DetectionError(
                "inconsistent",
                "target and layout target point exceed configured spread",
            )
        layout_targets.append(points[(0, 0)])
    aggregate_points(layout_targets, spread)
    return newest, tuple(
        float(value) for value in newest_map[(0, 0)]
    )
