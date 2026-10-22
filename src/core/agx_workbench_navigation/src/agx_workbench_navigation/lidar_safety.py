"""ROS-free directional safety evaluation for the raw rover LiDAR cloud."""

from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path
import struct
from typing import Mapping, Optional, Sequence, Tuple, Union

from .contracts import (
    DirectionalClearance,
    MotionDirection,
    SafetyHealth,
    Velocity2D,
)


FLOAT32 = 7
MAX_POINTCLOUD_POINTS = 250000
MAX_POINTCLOUD_BYTES = 16*1024*1024


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{0} must be a finite numeric value".format(name))
    try:
        converted = float(value)
    except OverflowError:
        raise ValueError("{0} must be a finite numeric value".format(name))
    if not math.isfinite(converted):
        raise ValueError("{0} must be a finite numeric value".format(name))
    return converted


def _positive(value, name):
    converted = _finite_number(value, name)
    if converted <= 0.0:
        raise ValueError("{0} must be positive".format(name))
    return converted


def _nonnegative(value, name):
    converted = _finite_number(value, name)
    if converted < 0.0:
        raise ValueError("{0} must be nonnegative".format(name))
    return converted


class DuplicateStampFilter:
    """Suppress only consecutive duplicate publish stamps.

    A rewind is deliberately allowed through so the downstream authority
    arbiter can detect and fail closed on a ROS-time regression.  Isaac may
    execute both a sensor callback and a timer callback within one simulation
    tick, however, and those callbacks must not produce two messages with the
    same header stamp.
    """

    def __init__(self):
        self._last_stamp_s = None

    def accept(self, stamp_s):
        candidate = _nonnegative(stamp_s, "publish_stamp_s")
        is_duplicate = (
            self._last_stamp_s is not None
            and candidate == self._last_stamp_s
        )
        self._last_stamp_s = candidate
        return not is_duplicate


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("{0} must be a positive integer".format(name))
    return value


def _mapping(value, name):
    if not isinstance(value, Mapping):
        raise ValueError("{0} must be a mapping".format(name))
    return value


def _strict_keys(value, name, expected):
    expected_set = set(expected)
    actual_set = set(value)
    missing = sorted(expected_set - actual_set)
    unknown = sorted(actual_set - expected_set)
    if missing:
        raise ValueError(
            "{0} is missing required keys: {1}".format(name, ", ".join(missing))
        )
    if unknown:
        raise ValueError(
            "{0} contains unknown keys: {1}".format(name, ", ".join(unknown))
        )


def _nonempty_string(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("{0} must be a non-empty string".format(name))
    return value


@dataclass(frozen=True)
class Point3D:
    """One XYZ sample in ``base_link``; non-finite samples remain filterable."""

    x: float
    y: float
    z: float

    def __post_init__(self):
        for field_name in ("x", "y", "z"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError("{0} must be numeric".format(field_name))
            try:
                converted = float(value)
            except OverflowError:
                raise ValueError("{0} must be numeric".format(field_name))
            object.__setattr__(self, field_name, converted)


@dataclass(frozen=True)
class CompactXYZ:
    """Packed float32 XYZ backed by immutable bytes, without per-point objects."""

    packed: bytes

    def __post_init__(self):
        if (type(self.packed) is not bytes or len(self.packed) % 12
                or len(self.packed) // 12 > MAX_POINTCLOUD_POINTS):
            raise ValueError("compact XYZ size exceeds the bound")

    def __len__(self):
        return len(self.packed) // 12

    @property
    def xyz_m(self):
        import numpy as np
        return np.frombuffer(self.packed, dtype='<f4').reshape((-1, 3))

    def __iter__(self):
        return (Point3D(*values) for values in struct.iter_unpack('<fff', self.packed))


@dataclass(frozen=True)
class SafetyEnvelope:
    """Physical footprint and raw-point filtering boundaries."""

    front_x_m: float
    rear_x_m: float
    left_y_m: float
    right_y_m: float
    min_obstacle_z_m: float
    max_obstacle_z_m: float
    self_padding_m: float
    minimum_points_per_zone: int

    def __post_init__(self):
        for field_name in (
            "front_x_m",
            "rear_x_m",
            "left_y_m",
            "right_y_m",
            "min_obstacle_z_m",
            "max_obstacle_z_m",
        ):
            object.__setattr__(
                self,
                field_name,
                _finite_number(getattr(self, field_name), field_name),
            )
        object.__setattr__(
            self,
            "self_padding_m",
            _nonnegative(self.self_padding_m, "self_padding_m"),
        )
        object.__setattr__(
            self,
            "minimum_points_per_zone",
            _positive_integer(
                self.minimum_points_per_zone, "minimum_points_per_zone"
            ),
        )
        if self.front_x_m <= self.rear_x_m or self.left_y_m <= self.right_y_m:
            raise ValueError("footprint bounds are invalid")
        if self.min_obstacle_z_m >= self.max_obstacle_z_m:
            raise ValueError("obstacle height bounds are invalid")
        if (
            self.rear_x_m + self.self_padding_m
            >= self.front_x_m - self.self_padding_m
            or self.right_y_m + self.self_padding_m
            >= self.left_y_m - self.self_padding_m
        ):
            raise ValueError(
                "self_padding_m inset must leave a nonempty trusted self volume"
            )

    @property
    def circumscribed_radius_m(self):
        return max(
            math.hypot(x_value, y_value)
            for x_value in (self.front_x_m, self.rear_x_m)
            for y_value in (self.left_y_m, self.right_y_m)
        )


@dataclass(frozen=True)
class SafetyConfig:
    """Validated deployable LiDAR safety configuration."""

    model: str
    topic: str
    expected_frame: str
    target_frame: str
    max_age_s: float
    minimum_rate_hz: float
    minimum_valid_points_per_cloud: int
    sensor_range_max_m: float
    envelope: SafetyEnvelope
    static_clearance_m: float
    sensor_latency_s: float
    command_latency_s: float
    brake_latency_s: float
    minimum_braking_deceleration_mps2: float

    def __post_init__(self):
        for field_name in ("model", "topic", "expected_frame", "target_frame"):
            object.__setattr__(
                self,
                field_name,
                _nonempty_string(getattr(self, field_name), field_name),
            )
        if self.topic != "/rslidar_points":
            raise ValueError("topic must be /rslidar_points")
        if self.expected_frame != "rslidar":
            raise ValueError("expected_frame must be rslidar")
        if self.target_frame != "base_link":
            raise ValueError("target_frame must be base_link")
        for field_name in (
            "max_age_s",
            "minimum_rate_hz",
            "sensor_range_max_m",
            "static_clearance_m",
            "minimum_braking_deceleration_mps2",
        ):
            object.__setattr__(
                self,
                field_name,
                _positive(getattr(self, field_name), field_name),
            )
        for field_name, configuration_name in (
            ("sensor_latency_s", "sensor_s"),
            ("command_latency_s", "command_s"),
            ("brake_latency_s", "brake_s"),
        ):
            object.__setattr__(
                self,
                field_name,
                _nonnegative(getattr(self, field_name), configuration_name),
            )
        object.__setattr__(
            self,
            "minimum_valid_points_per_cloud",
            _positive_integer(
                self.minimum_valid_points_per_cloud,
                "minimum_valid_points_per_cloud",
            ),
        )
        if not isinstance(self.envelope, SafetyEnvelope):
            raise ValueError("envelope must be a SafetyEnvelope")

    @property
    def footprint_circumscribed_radius_m(self):
        return self.envelope.circumscribed_radius_m

    @classmethod
    def from_mapping(cls, value):
        raw = _mapping(value, "lidar safety configuration")
        _strict_keys(
            raw,
            "lidar safety configuration",
            (
                "schema_version",
                "model",
                "topic",
                "expected_frame",
                "target_frame",
                "max_age_s",
                "minimum_rate_hz",
                "minimum_valid_points_per_cloud",
                "sensor_range_max_m",
                "footprint",
                "filter",
                "static_clearance_m",
                "latency",
                "minimum_braking_deceleration_mps2",
            ),
        )
        if raw["schema_version"] != 1:
            raise ValueError("schema_version must be 1")
        footprint = _mapping(raw["footprint"], "footprint")
        filter_config = _mapping(raw["filter"], "filter")
        latency = _mapping(raw["latency"], "latency")
        _strict_keys(
            footprint,
            "footprint",
            ("front_x_m", "rear_x_m", "left_y_m", "right_y_m"),
        )
        _strict_keys(
            filter_config,
            "filter",
            (
                "min_obstacle_z_m",
                "max_obstacle_z_m",
                "self_padding_m",
                "minimum_points_per_zone",
            ),
        )
        _strict_keys(
            latency,
            "latency",
            ("sensor_s", "command_s", "brake_s"),
        )
        return cls(
            model=raw["model"],
            topic=raw["topic"],
            expected_frame=raw["expected_frame"],
            target_frame=raw["target_frame"],
            max_age_s=raw["max_age_s"],
            minimum_rate_hz=raw["minimum_rate_hz"],
            minimum_valid_points_per_cloud=raw[
                "minimum_valid_points_per_cloud"
            ],
            sensor_range_max_m=raw["sensor_range_max_m"],
            envelope=SafetyEnvelope(
                front_x_m=footprint["front_x_m"],
                rear_x_m=footprint["rear_x_m"],
                left_y_m=footprint["left_y_m"],
                right_y_m=footprint["right_y_m"],
                min_obstacle_z_m=filter_config["min_obstacle_z_m"],
                max_obstacle_z_m=filter_config["max_obstacle_z_m"],
                self_padding_m=filter_config["self_padding_m"],
                minimum_points_per_zone=filter_config[
                    "minimum_points_per_zone"
                ],
            ),
            static_clearance_m=raw["static_clearance_m"],
            sensor_latency_s=latency["sensor_s"],
            command_latency_s=latency["command_s"],
            brake_latency_s=latency["brake_s"],
            minimum_braking_deceleration_mps2=raw[
                "minimum_braking_deceleration_mps2"
            ],
        )


@dataclass(frozen=True)
class DecodedPointCloud:
    """Fail-closed result of validating and decoding PointCloud2 metadata."""

    state: SafetyHealth
    points: Union[Tuple[Point3D, ...], CompactXYZ]
    reason: str

    def __post_init__(self):
        if not isinstance(self.state, SafetyHealth):
            raise ValueError("state must be a SafetyHealth")
        if not isinstance(self.points, CompactXYZ) and (
            not isinstance(self.points, tuple) or not all(
                isinstance(point, Point3D) for point in self.points)):
            raise ValueError("points must be an immutable Point3D tuple")
        if not isinstance(self.reason, str):
            raise ValueError("reason must be a string")


@dataclass(frozen=True)
class SafetyDecision:
    """One immutable directional-clearance and health decision."""

    state: SafetyHealth
    blocked_directions: MotionDirection
    clearances: DirectionalClearance
    required_stop_distance_m: float
    reason: str
    valid_point_count: int
    filtered_point_count: int

    def __post_init__(self):
        if not isinstance(self.state, SafetyHealth):
            raise ValueError("state must be a SafetyHealth")
        if not isinstance(self.blocked_directions, MotionDirection):
            raise ValueError("blocked_directions must be a MotionDirection")
        if not isinstance(self.clearances, DirectionalClearance):
            raise ValueError("clearances must be a DirectionalClearance")
        object.__setattr__(
            self,
            "required_stop_distance_m",
            _nonnegative(
                self.required_stop_distance_m, "required_stop_distance_m"
            ),
        )
        if not isinstance(self.reason, str) or not self.reason:
            raise ValueError("reason must be a non-empty string")
        for field_name in ("valid_point_count", "filtered_point_count"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError("{0} must be a nonnegative integer".format(field_name))

    @property
    def forward_clearance_m(self):
        return self.clearances.forward_m

    @property
    def reverse_clearance_m(self):
        return self.clearances.reverse_m

    @property
    def left_clearance_m(self):
        return self.clearances.left_m

    @property
    def right_clearance_m(self):
        return self.clearances.right_m

    @property
    def rotation_clearance_m(self):
        return self.clearances.rotation_m


@dataclass(frozen=True)
class LidarEvidence:
    """Immutable source-specific facts captured by the ROS monitor."""

    points: Optional[Tuple[Point3D, ...]]
    cloud_stamp_s: Optional[float]
    tf_stamp_s: Optional[float]
    cloud_stamps_s: Tuple[float, ...]
    velocity: Optional[Velocity2D]
    odometry_stamp_s: Optional[float]
    cloud_failure_reason: Optional[str]
    odometry_failure_reason: Optional[str]

    @property
    def cloud_rate_hz(self):
        if len(self.cloud_stamps_s) < 2:
            return None
        span_s = self.cloud_stamps_s[-1] - self.cloud_stamps_s[0]
        if span_s <= 0.0:
            return None
        return float(len(self.cloud_stamps_s) - 1) / span_s


class ClockAlignmentWait:
    """Bound a cross-topic clock join without accepting or renewing evidence.

    The caller publishes nothing while waiting. Its previous published proof
    continues to age at the consumer, including the consumer's wall watchdog.
    Neither a new future sample nor a stopped ROS clock renews this wall lease.
    """

    def __init__(self, max_wait_s):
        self._max_wait_s = _positive(max_wait_s, "max_wait_s")
        self._started_s = None

    def defer(self, decision, evidence, *, now_s, receipt_s):
        future_stamp = {
            "cloud_stamp_in_future": evidence.cloud_stamp_s,
            "odometry_stamp_in_future": evidence.odometry_stamp_s,
            "tf_stamp_in_future": evidence.tf_stamp_s,
        }.get(decision.reason)
        if future_stamp is None or not 0.0 < future_stamp - now_s <= self._max_wait_s:
            self._started_s = None
            return False
        if self._started_s is None:
            self._started_s = receipt_s
        return 0.0 <= receipt_s - self._started_s < self._max_wait_s


class LidarEvidenceTracker:
    """Keep cloud/TF and odometry health independent across callbacks."""

    def __init__(self, maximum_rate_samples=20):
        if (
            isinstance(maximum_rate_samples, bool)
            or not isinstance(maximum_rate_samples, int)
            or maximum_rate_samples < 2
        ):
            raise ValueError("maximum_rate_samples must be an integer of at least 2")
        self._maximum_rate_samples = maximum_rate_samples
        self._points = None
        self._cloud_stamp_s = None
        self._tf_stamp_s = None
        self._cloud_stamps_s = ()
        self._velocity = None
        self._odometry_stamp_s = None
        self._cloud_failure_reason = None
        self._odometry_failure_reason = None

    @staticmethod
    def _reason(value, name):
        if not isinstance(value, str) or not value:
            raise ValueError("{0} must be a non-empty string".format(name))
        return value

    def record_cloud_failure(self, reason):
        self._cloud_failure_reason = self._reason(reason, "cloud failure reason")
        self._points = None
        self._cloud_stamp_s = None
        self._tf_stamp_s = None
        self._cloud_stamps_s = ()

    def record_valid_cloud(self, *, points, cloud_stamp_s, tf_stamp_s):
        try:
            snapshot = tuple(points)
        except TypeError:
            self.record_cloud_failure("cloud_evidence_invalid")
            return False
        if not all(isinstance(point, Point3D) for point in snapshot):
            self.record_cloud_failure("cloud_evidence_invalid")
            return False
        try:
            cloud_stamp = _finite_number(cloud_stamp_s, "cloud_stamp_s")
            tf_stamp = _finite_number(tf_stamp_s, "tf_stamp_s")
        except ValueError:
            self.record_cloud_failure("cloud_evidence_invalid")
            return False
        if self._cloud_stamps_s and cloud_stamp <= self._cloud_stamps_s[-1]:
            self.record_cloud_failure("cloud_stamp_nonmonotonic")
            return False
        self._points = snapshot
        self._cloud_stamp_s = cloud_stamp
        self._tf_stamp_s = tf_stamp
        self._cloud_stamps_s = (
            self._cloud_stamps_s + (cloud_stamp,)
        )[-self._maximum_rate_samples :]
        self._cloud_failure_reason = None
        return True

    def record_odometry_failure(self, reason):
        self._odometry_failure_reason = self._reason(
            reason, "odometry failure reason"
        )
        self._velocity = None
        self._odometry_stamp_s = None

    def record_valid_odometry(self, velocity, *, stamp_s):
        if not isinstance(velocity, Velocity2D):
            self.record_odometry_failure("odometry_invalid")
            return False
        try:
            stamp = _finite_number(stamp_s, "odometry_stamp_s")
        except ValueError:
            self.record_odometry_failure("odometry_invalid")
            return False
        self._velocity = velocity
        self._odometry_stamp_s = stamp
        self._odometry_failure_reason = None
        return True

    def snapshot(self):
        return LidarEvidence(
            points=self._points,
            cloud_stamp_s=self._cloud_stamp_s,
            tf_stamp_s=self._tf_stamp_s,
            cloud_stamps_s=self._cloud_stamps_s,
            velocity=self._velocity,
            odometry_stamp_s=self._odometry_stamp_s,
            cloud_failure_reason=self._cloud_failure_reason,
            odometry_failure_reason=self._odometry_failure_reason,
        )


def required_stop_distance(equivalent_speed_mps, config):
    """Return static clearance plus latency travel and braking distance."""

    if not isinstance(config, SafetyConfig):
        raise ValueError("config must be a SafetyConfig")
    speed = _nonnegative(equivalent_speed_mps, "equivalent_speed_mps")
    total_latency_s = (
        config.sensor_latency_s
        + config.command_latency_s
        + config.brake_latency_s
    )
    return (
        config.static_clearance_m
        + speed * total_latency_s
        + speed * speed / (2.0 * config.minimum_braking_deceleration_mps2)
    )


def resolve_lidar_config_path(package_lookup):
    """Resolve the installed package share config through the ROS package index."""

    if not callable(package_lookup):
        raise ValueError("package_lookup must be callable")
    try:
        package_path = Path(package_lookup("agx_workbench_navigation"))
    except Exception as exc:
        raise ValueError(
            "cannot resolve installed agx_workbench_navigation package: {0}".format(
                exc
            )
        )
    config_path = (package_path / "config/lidar_safety.yaml").resolve()
    if not config_path.is_file():
        raise ValueError(
            "installed LiDAR safety configuration is missing: {0}".format(
                config_path
            )
        )
    return config_path


def unhealthy_decision(
    config,
    health,
    reason,
    required_stop_distance_m=0.0,
):
    """Create a fail-closed state for unavailable or invalid sensor evidence."""

    if not isinstance(config, SafetyConfig):
        raise ValueError("config must be a SafetyConfig")
    if health not in (SafetyHealth.STALE, SafetyHealth.INVALID):
        raise ValueError("unhealthy state must be STALE or INVALID")
    if not isinstance(reason, str) or not reason:
        raise ValueError("reason must be a non-empty string")
    return SafetyDecision(
        state=health,
        blocked_directions=MotionDirection.ALL,
        clearances=DirectionalClearance(
            forward_m=0.0,
            reverse_m=0.0,
            left_m=0.0,
            right_m=0.0,
            rotation_m=0.0,
            sensor_max_m=config.sensor_range_max_m,
        ),
        required_stop_distance_m=required_stop_distance_m,
        reason=reason,
        valid_point_count=0,
        filtered_point_count=0,
    )


def evidence_decision(evaluator, evidence, *, now_s):
    """Evaluate one atomic monitor snapshot without conflating source failures."""

    if not isinstance(evaluator, LidarSafetyEvaluator):
        raise ValueError("evaluator must be a LidarSafetyEvaluator")
    if not isinstance(evidence, LidarEvidence):
        raise ValueError("evidence must be LidarEvidence")
    if evidence.cloud_failure_reason is not None:
        return unhealthy_decision(
            evaluator.config,
            SafetyHealth.INVALID,
            evidence.cloud_failure_reason,
        )
    if evidence.odometry_failure_reason is not None:
        return unhealthy_decision(
            evaluator.config,
            SafetyHealth.INVALID,
            evidence.odometry_failure_reason,
        )
    if (
        evidence.points is None
        or evidence.cloud_stamp_s is None
        or evidence.tf_stamp_s is None
    ):
        return unhealthy_decision(
            evaluator.config,
            SafetyHealth.STALE,
            "cloud_unavailable",
        )
    if evidence.velocity is None or evidence.odometry_stamp_s is None:
        return unhealthy_decision(
            evaluator.config,
            SafetyHealth.STALE,
            "odometry_unavailable",
        )
    return evaluator.evaluate(
        points=evidence.points,
        velocity=evidence.velocity,
        stamp_s=evidence.cloud_stamp_s,
        now_s=now_s,
        odometry_stamp_s=evidence.odometry_stamp_s,
        tf_available=True,
        tf_stamp_s=evidence.tf_stamp_s,
        cloud_rate_hz=evidence.cloud_rate_hz,
    )


def _field_value(field, name):
    if isinstance(field, Mapping):
        return field.get(name)
    return getattr(field, name, None)


def _invalid_decode(reason):
    return DecodedPointCloud(
        state=SafetyHealth.INVALID,
        points=(),
        reason=reason,
    )


def decode_xyz_points(
    *,
    data,
    width,
    height,
    fields,
    point_step,
    row_step,
    is_bigendian,
    compact=False,
):
    """Validate PointCloud2 layout metadata and decode its exact FLOAT32 XYZ."""

    if isinstance(width, bool) or not isinstance(width, int) or width < 0:
        return _invalid_decode("width_invalid")
    if isinstance(height, bool) or not isinstance(height, int) or height <= 0:
        return _invalid_decode("height_invalid")
    if (
        isinstance(point_step, bool)
        or not isinstance(point_step, int)
        or point_step <= 0
    ):
        return _invalid_decode("point_step_invalid")
    if isinstance(row_step, bool) or not isinstance(row_step, int) or row_step < 0:
        return _invalid_decode("row_step_invalid")
    if not isinstance(is_bigendian, bool):
        return _invalid_decode("endianness_invalid")
    if width*height > MAX_POINTCLOUD_POINTS or row_step*height > MAX_POINTCLOUD_BYTES:
        return _invalid_decode("cloud_size_limit")
    if isinstance(fields, (str, bytes)) or not isinstance(fields, Sequence):
        return _invalid_decode("fields_invalid")

    xyz_fields = {}
    for field in fields:
        name = _field_value(field, "name")
        if name not in ("x", "y", "z"):
            continue
        if name in xyz_fields:
            return _invalid_decode("duplicate_xyz_field")
        xyz_fields[name] = field
    if set(xyz_fields) != {"x", "y", "z"}:
        return _invalid_decode("missing_xyz_field")

    offsets = {}
    for name in ("x", "y", "z"):
        field = xyz_fields[name]
        datatype = _field_value(field, "datatype")
        count = _field_value(field, "count")
        offset = _field_value(field, "offset")
        if datatype != FLOAT32 or count != 1:
            return _invalid_decode("xyz_field_not_float32")
        if (
            isinstance(offset, bool)
            or not isinstance(offset, int)
            or offset < 0
            or offset + 4 > point_step
        ):
            return _invalid_decode("xyz_field_out_of_bounds")
        offsets[name] = offset
    field_ranges = sorted(
        (offset, offset + 4, name) for name, offset in offsets.items()
    )
    for previous, current in zip(field_ranges, field_ranges[1:]):
        if current[0] < previous[1]:
            return _invalid_decode("xyz_fields_overlap")

    minimum_row_step = point_step * width
    if row_step < minimum_row_step:
        return _invalid_decode("row_step_too_small")
    try:
        buffer = memoryview(data)
    except TypeError:
        return _invalid_decode("buffer_invalid")
    if buffer.ndim != 1 or buffer.itemsize != 1:
        return _invalid_decode("buffer_invalid")
    if len(buffer) != row_step * height:
        return _invalid_decode("buffer_length_mismatch")

    byte_order = ">" if is_bigendian else "<"
    if compact:
        import numpy as np
        layout = np.dtype({'names': ['x', 'y', 'z'],
            'formats': [byte_order+'f4']*3,
            'offsets': [offsets[name] for name in ('x', 'y', 'z')],
            'itemsize': point_step})
        view = np.ndarray((height, width), dtype=layout, buffer=buffer,
                          strides=(row_step, point_step))
        xyz = np.stack([view[name] for name in ('x', 'y', 'z')], axis=-1)
        return DecodedPointCloud(SafetyHealth.CLEAR,
            CompactXYZ(xyz.astype('<f4', copy=False).tobytes()), 'decoded')
    points = []
    try:
        for row_index in range(height):
            row_offset = row_index * row_step
            for column_index in range(width):
                point_offset = row_offset + column_index * point_step
                values = []
                for name in ("x", "y", "z"):
                    values.append(
                        struct.unpack_from(
                            byte_order + "f",
                            buffer,
                            point_offset + offsets[name],
                        )[0]
                    )
                points.append(Point3D(*values))
    except (struct.error, TypeError, ValueError):
        return _invalid_decode("buffer_unpack_failed")
    return DecodedPointCloud(
        state=SafetyHealth.CLEAR,
        points=tuple(points),
        reason="decoded",
    )


def inset_self_exclusion(envelope, footprint):
    """Intersection of the original trusted inset with the convex body outline.

    Never suppress a return merely because it lies inside a larger rectangle.
    The original inset remains; a second inset would classify known body surfaces
    as obstacles. Clipping handles short edges in measured convex outlines.
    """
    from .motion_policy import _validate_candidate_polygon
    footprint=tuple(tuple(p) for p in footprint)
    _validate_candidate_polygon(footprint)
    area=sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(footprint,footprint[1:]+footprint[:1]))
    if area<0:footprint=tuple(reversed(footprint))
    inset=envelope.self_padding_m
    polygon=[(envelope.rear_x_m+inset,envelope.right_y_m+inset),
             (envelope.front_x_m-inset,envelope.right_y_m+inset),
             (envelope.front_x_m-inset,envelope.left_y_m-inset),
             (envelope.rear_x_m+inset,envelope.left_y_m-inset)]
    for a,b in zip(footprint,footprint[1:]+footprint[:1]):
        ex,ey=b[0]-a[0],b[1]-a[1];bound=0.0
        def distance(p):return ex*(p[1]-a[1])-ey*(p[0]-a[0])-bound
        clipped=[]
        for previous,current in zip(polygon[-1:]+polygon[:-1],polygon):
            d0,d1=distance(previous),distance(current)
            if (d0>=0)!=(d1>=0):
                fraction=d0/(d0-d1)
                clipped.append(tuple(x+fraction*(y-x) for x,y in zip(previous,current)))
            if d1>=0:clipped.append(current)
        polygon=clipped
        if len(polygon)<3:raise ValueError('body has no trusted self volume')
    # Remove representation-only collinear/duplicate vertices for the report.
    cleaned=[]
    for i,p in enumerate(polygon):
        a,b=polygon[i-1],polygon[(i+1)%len(polygon)]
        cross=(p[0]-a[0])*(b[1]-p[1])-(p[1]-a[1])*(b[0]-p[0])
        if abs(cross)>1e-12:cleaned.append(p)
    if len(cleaned)<3:raise ValueError('invalid body exclusion polygon')
    return tuple(cleaned)


class LidarSafetyEvaluator:
    """Evaluate deterministic directional swept volumes in ``base_link``."""

    def __init__(self, config, self_footprint_xy_m=None):
        if not isinstance(config, SafetyConfig):
            raise ValueError("config must be a SafetyConfig")
        self.config = config
        self.self_exclusion_xy_m=(inset_self_exclusion(config.envelope,self_footprint_xy_m)
            if self_footprint_xy_m is not None else None)

    def required_stop_distance(self, velocity):
        if not isinstance(velocity, Velocity2D):
            raise ValueError("velocity must be a Velocity2D")
        equivalent_speed_mps = max(
            abs(velocity.linear_x),
            abs(velocity.linear_y),
            abs(velocity.angular_z)
            * self.config.footprint_circumscribed_radius_m,
        )
        return required_stop_distance(equivalent_speed_mps, self.config)

    def _unhealthy(self, health, reason, required_distance=0.0):
        return unhealthy_decision(
            self.config,
            health,
            reason,
            required_stop_distance_m=required_distance,
        )

    def _health_error(
        self,
        *,
        stamp_s,
        now_s,
        odometry_stamp_s,
        tf_available,
        tf_stamp_s,
        cloud_rate_hz,
        required_distance,
    ):
        try:
            now = _finite_number(now_s, "now_s")
        except ValueError:
            return self._unhealthy(
                SafetyHealth.INVALID, "now_invalid", required_distance
            )
        if now < 0.0:
            return self._unhealthy(
                SafetyHealth.INVALID, "now_invalid", required_distance
            )
        try:
            cloud_stamp = _finite_number(stamp_s, "stamp_s")
        except ValueError:
            return self._unhealthy(
                SafetyHealth.INVALID, "cloud_stamp_invalid", required_distance
            )
        if cloud_stamp > now:
            return self._unhealthy(
                SafetyHealth.INVALID,
                "cloud_stamp_in_future",
                required_distance,
            )
        if now - cloud_stamp > self.config.max_age_s:
            return self._unhealthy(
                SafetyHealth.STALE, "cloud_stale", required_distance
            )
        if odometry_stamp_s is None:
            return self._unhealthy(
                SafetyHealth.STALE, "odometry_unavailable", required_distance
            )
        try:
            odometry_stamp = _finite_number(
                odometry_stamp_s, "odometry_stamp_s"
            )
        except ValueError:
            return self._unhealthy(
                SafetyHealth.INVALID,
                "odometry_stamp_invalid",
                required_distance,
            )
        if odometry_stamp > now:
            return self._unhealthy(
                SafetyHealth.INVALID,
                "odometry_stamp_in_future",
                required_distance,
            )
        if now - odometry_stamp > self.config.max_age_s:
            return self._unhealthy(
                SafetyHealth.STALE, "odometry_stale", required_distance
            )
        if tf_available is not True:
            return self._unhealthy(
                SafetyHealth.INVALID, "tf_unavailable", required_distance
            )
        if tf_stamp_s is None:
            return self._unhealthy(
                SafetyHealth.INVALID, "tf_stamp_invalid", required_distance
            )
        try:
            tf_stamp = _finite_number(tf_stamp_s, "tf_stamp_s")
        except ValueError:
            return self._unhealthy(
                SafetyHealth.INVALID, "tf_stamp_invalid", required_distance
            )
        if tf_stamp > now:
            return self._unhealthy(
                SafetyHealth.INVALID, "tf_stamp_in_future", required_distance
            )
        if now - tf_stamp > self.config.max_age_s:
            return self._unhealthy(
                SafetyHealth.INVALID, "tf_stale", required_distance
            )
        if cloud_rate_hz is None:
            return self._unhealthy(
                SafetyHealth.STALE,
                "cloud_rate_unavailable",
                required_distance,
            )
        try:
            cloud_rate = _finite_number(cloud_rate_hz, "cloud_rate_hz")
        except ValueError:
            return self._unhealthy(
                SafetyHealth.INVALID,
                "cloud_rate_invalid",
                required_distance,
            )
        if cloud_rate <= 0.0:
            return self._unhealthy(
                SafetyHealth.INVALID,
                "cloud_rate_invalid",
                required_distance,
            )
        if cloud_rate < self.config.minimum_rate_hz:
            return self._unhealthy(
                SafetyHealth.STALE,
                "cloud_rate_below_minimum",
                required_distance,
            )
        return None

    def evaluate(
        self,
        *,
        points,
        velocity,
        stamp_s,
        now_s,
        odometry_stamp_s=None,
        tf_available=True,
        tf_stamp_s=None,
        cloud_rate_hz=None,
    ):
        if not isinstance(velocity, Velocity2D):
            return self._unhealthy(SafetyHealth.INVALID, "velocity_invalid")
        required_distance = self.required_stop_distance(velocity)
        health_error = self._health_error(
            stamp_s=stamp_s,
            now_s=now_s,
            odometry_stamp_s=odometry_stamp_s,
            tf_available=tf_available,
            tf_stamp_s=tf_stamp_s,
            cloud_rate_hz=cloud_rate_hz,
            required_distance=required_distance,
        )
        if health_error is not None:
            return health_error
        try:
            snapshot = tuple(points)
        except TypeError:
            return self._unhealthy(
                SafetyHealth.INVALID, "point_data_invalid", required_distance
            )
        if not all(isinstance(point, Point3D) for point in snapshot):
            return self._unhealthy(
                SafetyHealth.INVALID, "point_data_invalid", required_distance
            )

        finite_points = tuple(
            point
            for point in snapshot
            if math.isfinite(point.x)
            and math.isfinite(point.y)
            and math.isfinite(point.z)
        )
        if len(finite_points) < self.config.minimum_valid_points_per_cloud:
            return self._unhealthy(
                SafetyHealth.INVALID,
                "insufficient_valid_points",
                required_distance,
            )

        envelope = self.config.envelope
        obstacle_points = self.filtered_obstacles(finite_points)
        footprint_intrusion = any(
            envelope.rear_x_m <= point.x <= envelope.front_x_m
            and envelope.right_y_m <= point.y <= envelope.left_y_m
            for point in obstacle_points
        )
        intrusion_clearance = [0.0] if footprint_intrusion else []
        sensor_max = self.config.sensor_range_max_m
        forward_candidates = intrusion_clearance + [
            point.x - envelope.front_x_m
            for point in obstacle_points
            if point.x >= envelope.front_x_m
            and envelope.right_y_m <= point.y <= envelope.left_y_m
        ]
        reverse_candidates = intrusion_clearance + [
            envelope.rear_x_m - point.x
            for point in obstacle_points
            if point.x <= envelope.rear_x_m
            and envelope.right_y_m <= point.y <= envelope.left_y_m
        ]
        left_candidates = intrusion_clearance + [
            point.y - envelope.left_y_m
            for point in obstacle_points
            if point.y >= envelope.left_y_m
            and envelope.rear_x_m <= point.x <= envelope.front_x_m
        ]
        right_candidates = intrusion_clearance + [
            envelope.right_y_m - point.y
            for point in obstacle_points
            if point.y <= envelope.right_y_m
            and envelope.rear_x_m <= point.x <= envelope.front_x_m
        ]
        rotation_candidates = [
            max(
                0.0,
                math.hypot(point.x, point.y)
                - self.config.footprint_circumscribed_radius_m,
            )
            for point in obstacle_points
        ]
        direction_candidates = (
            (MotionDirection.FORWARD, forward_candidates, required_distance),
            (MotionDirection.REVERSE, reverse_candidates, required_distance),
            (MotionDirection.LEFT, left_candidates, required_distance),
            (MotionDirection.RIGHT, right_candidates, required_distance),
            (
                MotionDirection.ROTATION,
                rotation_candidates,
                self.config.static_clearance_m,
            ),
        )
        blocked = MotionDirection.NONE
        blocked_names = []
        for direction, candidates, threshold_m in direction_candidates:
            count = sum(candidate <= threshold_m for candidate in candidates)
            if footprint_intrusion or count >= envelope.minimum_points_per_zone:
                blocked |= direction
                blocked_names.append(direction.name.lower())

        def nearest(candidates):
            if not candidates:
                return sensor_max
            return min(sensor_max, max(0.0, min(candidates)))

        clearances = DirectionalClearance(
            forward_m=nearest(forward_candidates),
            reverse_m=nearest(reverse_candidates),
            left_m=nearest(left_candidates),
            right_m=nearest(right_candidates),
            rotation_m=nearest(rotation_candidates),
            sensor_max_m=sensor_max,
        )
        return SafetyDecision(
            state=(
                SafetyHealth.BLOCKED
                if blocked != MotionDirection.NONE
                else SafetyHealth.CLEAR
            ),
            blocked_directions=blocked,
            clearances=clearances,
            required_stop_distance_m=required_distance,
            reason=(
                "blocked:{0}".format(",".join(blocked_names))
                if blocked_names
                else "clear"
            ),
            valid_point_count=len(finite_points),
            filtered_point_count=len(obstacle_points),
        )

    def filtered_obstacles(self, points):
        """Exact existing height/self filter, without a zone-count rejection.

        Independent filled sweeps must retain even an isolated obstacle return.
        Directional evaluation keeps its existing count thresholds separately.
        """
        envelope = self.config.envelope
        return tuple(p for p in points if all(math.isfinite(v) for v in (p.x, p.y, p.z)) and
                     envelope.min_obstacle_z_m <= p.z <= envelope.max_obstacle_z_m and
                     not self._is_self_return(p))

    def _is_self_return(self, point):
        if self.self_exclusion_xy_m is not None:
            polygon=self.self_exclusion_xy_m
            return all((b[0]-a[0])*(point.y-a[1])-(b[1]-a[1])*(point.x-a[0])>=0
                for a,b in zip(polygon,polygon[1:]+polygon[:1]))
        envelope = self.config.envelope
        padding = envelope.self_padding_m
        return (
            envelope.rear_x_m + padding
            <= point.x
            <= envelope.front_x_m - padding
            and envelope.right_y_m + padding
            <= point.y
            <= envelope.left_y_m - padding
        )
