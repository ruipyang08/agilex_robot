"""ROS-independent validation and filtering for floor-tag observations."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from numbers import Real
from statistics import median
from typing import Optional, Sequence, Tuple


class TagFilterError(ValueError):
    """The tag-filter configuration or clock fact is invalid."""


def _stable_value_key(value):
    return (
        type(value).__module__,
        type(value).__qualname__,
        repr(value),
    )


def _deep_freeze(value, sort_unordered=False):
    if isinstance(value, Mapping):
        raise TagFilterError("observation sequences cannot contain mappings")
    if isinstance(value, (set, frozenset)):
        if not sort_unordered:
            raise TagFilterError("ordered pose components cannot be a set")
        frozen = [_deep_freeze(item, sort_unordered=True) for item in value]
        try:
            return tuple(sorted(frozen))
        except TypeError:
            return tuple(sorted(frozen, key=_stable_value_key))
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return tuple(
            _deep_freeze(item, sort_unordered=sort_unordered) for item in value
        )
    return value


@dataclass(frozen=True)
class TagObservation:
    """One marker and camera origin in ``base_link`` at the detector stamp."""

    marker_id: int
    detected_marker_ids: Tuple[int, ...]
    stamp_s: float
    frame_id: str
    translation_m: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]
    camera_position_m: Optional[Tuple[float, float, float]] = None

    def __post_init__(self):
        for field_name, sort_unordered in (
            ("detected_marker_ids", True),
            ("translation_m", False),
            ("quaternion_xyzw", False),
            ("camera_position_m", False),
        ):
            if field_name == "camera_position_m" and self.camera_position_m is None:
                continue
            try:
                frozen = _deep_freeze(
                    getattr(self, field_name),
                    sort_unordered=sort_unordered,
                )
            except TagFilterError:
                raise TagFilterError(
                    "{0} contains an invalid unordered value".format(field_name)
                )
            object.__setattr__(self, field_name, frozen)


@dataclass(frozen=True)
class FilteredTagPose:
    """Robust marker geometry and conservative residuals in ``base_link``."""

    marker_id: int
    stamp_s: float
    frame_id: str
    forward_m: float
    lateral_m: float
    floor_height_m: float
    approach_yaw_rad: float
    forward_residual_m: float
    lateral_residual_m: float
    floor_height_residual_m: float
    floor_normal_residual_rad: float
    approach_yaw_residual_rad: float


@dataclass(frozen=True)
class TagFilterDecision:
    """Result of processing one observation."""

    accepted: bool
    reason: str
    acquired: bool
    filtered_pose: Optional[FilteredTagPose]
    window: Tuple[FilteredTagPose, ...]
    diagnostic_marker_ids: Tuple[int, ...]


@dataclass(frozen=True)
class _AcceptedGeometry:
    stamp_s: float
    translation_m: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]
    approach_yaw_rad: float
    floor_normal_error_rad: float


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TagFilterError("{0} must be a finite numeric value".format(name))
    try:
        converted = float(value)
    except (OverflowError, TypeError, ValueError):
        raise TagFilterError("{0} must be a finite numeric value".format(name))
    if not math.isfinite(converted):
        raise TagFilterError("{0} must be a finite numeric value".format(name))
    return converted


def _positive(value, name):
    converted = _finite(value, name)
    if converted <= 0.0:
        raise TagFilterError("{0} must be positive".format(name))
    return converted


def _vector(value, length, name):
    if (
        isinstance(value, (str, bytes))
        or not isinstance(value, Sequence)
        or len(value) != length
    ):
        raise TagFilterError("{0} must contain {1} finite values".format(name, length))
    return tuple(
        _finite(component, "{0}[{1}]".format(name, index))
        for index, component in enumerate(value)
    )


def _unit_vector(value, length, name):
    vector = _vector(value, length, name)
    norm = math.sqrt(sum(component * component for component in vector))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise TagFilterError("{0} must be a unit vector".format(name))
    return vector


def _unit_quaternion(value):
    quaternion = _vector(value, 4, "quaternion_xyzw")
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise TagFilterError("quaternion_xyzw must be a unit quaternion")
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


def _quaternion_conjugate(quaternion):
    return (
        -quaternion[0],
        -quaternion[1],
        -quaternion[2],
        quaternion[3],
    )


def _rotate_vector(quaternion, vector):
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, vector_quaternion),
        _quaternion_conjugate(quaternion),
    )
    return (rotated[0], rotated[1], rotated[2])


def _dot(left, right):
    return sum(left[index] * right[index] for index in range(3))


def _wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _angle_distance(left, right):
    return abs(_wrap_angle(left - right))


def _circular_median(values):
    candidates = sorted(_wrap_angle(value) for value in values)
    return min(
        candidates,
        key=lambda candidate: (
            sum(_angle_distance(candidate, value) for value in candidates),
            candidate,
        ),
    )


def _quaternion_distance(left, right):
    dot = abs(sum(left[index] * right[index] for index in range(4)))
    return 2.0 * math.acos(min(max(dot, -1.0), 1.0))


def _quaternion_medoid(values):
    return min(
        values,
        key=lambda candidate: (
            sum(_quaternion_distance(candidate, value) for value in values),
            candidate,
        ),
    )


class TagObservationFilter:
    """Validate requested floor tags and retain a fresh robust pose window."""

    def __init__(
        self,
        requested_marker_id,
        required_samples,
        min_span_s,
        max_age_s,
        max_range_m,
        expected_floor_z_m,
        max_floor_height_error_m,
        max_floor_normal_error_rad,
        max_translation_jump_m,
        max_angular_jump_rad,
        expected_frame="base_link",
        expected_floor_normal=(0.0, 0.0, 1.0),
        camera_position_m=(0.0, 0.0, 0.0),
    ):
        if (
            isinstance(requested_marker_id, bool)
            or not isinstance(requested_marker_id, int)
            or not 100 <= requested_marker_id <= 199
        ):
            raise TagFilterError(
                "requested_marker_id must be an integer from 100 through 199"
            )
        if (
            isinstance(required_samples, bool)
            or not isinstance(required_samples, int)
            or required_samples < 2
        ):
            raise TagFilterError("required_samples must be an integer of at least 2")
        validated_min_span = _positive(min_span_s, "min_span_s")
        validated_max_age = _positive(max_age_s, "max_age_s")
        if validated_min_span > validated_max_age:
            raise TagFilterError("min_span_s must not exceed max_age_s")
        if not isinstance(expected_frame, str) or expected_frame != "base_link":
            raise TagFilterError("expected_frame must be base_link")

        self.requested_marker_id = requested_marker_id
        self.required_samples = required_samples
        self.min_span_s = validated_min_span
        self.max_age_s = validated_max_age
        self.max_range_m = _positive(max_range_m, "max_range_m")
        self.expected_floor_z_m = _finite(
            expected_floor_z_m,
            "expected_floor_z_m",
        )
        self.max_floor_height_error_m = _positive(
            max_floor_height_error_m,
            "max_floor_height_error_m",
        )
        self.max_floor_normal_error_rad = _positive(
            max_floor_normal_error_rad,
            "max_floor_normal_error_rad",
        )
        if self.max_floor_normal_error_rad > math.pi:
            raise TagFilterError("max_floor_normal_error_rad must not exceed pi")
        self.max_translation_jump_m = _positive(
            max_translation_jump_m,
            "max_translation_jump_m",
        )
        self.max_angular_jump_rad = _positive(
            max_angular_jump_rad,
            "max_angular_jump_rad",
        )
        if self.max_angular_jump_rad > math.pi:
            raise TagFilterError("max_angular_jump_rad must not exceed pi")
        self.expected_frame = expected_frame
        self.expected_floor_normal = _unit_vector(
            expected_floor_normal,
            3,
            "expected_floor_normal",
        )
        self.camera_position_m = _vector(
            camera_position_m,
            3,
            "camera_position_m",
        )
        self._accepted = []
        self._accepted_keys = set()
        self._window = ()
        self._filtered_pose = None
        self._acquired = False

    @property
    def window(self):
        """The current fresh, bounded sample window."""

        return self._window

    @property
    def filtered_pose(self):
        """The robust estimate for the current window, if one exists."""

        return self._filtered_pose

    @property
    def acquired(self):
        """Whether sample count, span, freshness, and geometry all hold."""

        return self._acquired

    def update(self, observation, now_ros_s):
        """Validate an observation and update the robust sample window."""

        now = self._validated_now(now_ros_s)
        self._prune(now)

        rejection, diagnostic_ids = self._validate_observation(
            observation,
            now,
        )
        if rejection is not None:
            return self._decision(False, rejection, diagnostic_ids)

        geometry = self._geometry(observation)
        key = self._observation_key(observation)
        if key in self._accepted_keys:
            return self._decision(
                False,
                "duplicate_observation",
                diagnostic_ids,
            )
        if self._accepted and observation.stamp_s <= self._accepted[-1].stamp_s:
            return self._decision(
                False,
                "non_monotonic_stamp",
                diagnostic_ids,
            )

        prospective_window = (self._accepted + [geometry])[-self.required_samples :]
        consistency_rejection = self._window_consistency_rejection(prospective_window)
        if consistency_rejection is not None:
            return self._decision(
                False,
                consistency_rejection,
                diagnostic_ids,
            )

        self._accepted.append(geometry)
        self._accepted_keys.add(key)
        if len(self._accepted) > self.required_samples:
            self._accepted = self._accepted[-self.required_samples :]
            self._rebuild_keys()
        self._rebuild_outputs()
        return self._decision(True, "accepted", diagnostic_ids)

    def refresh(self, now_ros_s):
        """Refresh ROS-time state; mission loops must call this during dropouts."""

        now = self._validated_now(now_ros_s)
        self._prune(now)
        return self._decision(False, "state_refreshed", ())

    def _validated_now(self, now_ros_s):
        now = _finite(now_ros_s, "now_ros_s")
        if now < 0.0:
            raise TagFilterError("now_ros_s must be nonnegative")
        return now

    def _validate_observation(self, observation, now):
        if not isinstance(observation, TagObservation):
            return "invalid_observation", ()
        marker_id = observation.marker_id
        if (
            isinstance(marker_id, bool)
            or not isinstance(marker_id, int)
            or not 0 <= marker_id <= 65535
        ):
            return "invalid_marker_id", ()
        try:
            detected_ids = tuple(observation.detected_marker_ids)
        except (TypeError, ValueError):
            return "invalid_marker_id_set", ()
        if not detected_ids or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 0 <= value <= 65535
            for value in detected_ids
        ):
            return "invalid_marker_id_set", ()
        wrong_ids = tuple(
            sorted(
                {value for value in detected_ids if value != self.requested_marker_id}
            )
        )
        if marker_id != self.requested_marker_id:
            return "wrong_marker_id", wrong_ids
        if detected_ids.count(self.requested_marker_id) > 1:
            return "duplicate_marker_id", (self.requested_marker_id,)
        if detected_ids.count(marker_id) != 1:
            return "invalid_marker_id_set", wrong_ids

        try:
            stamp = _finite(observation.stamp_s, "stamp_s")
        except TagFilterError:
            return "invalid_stamp", wrong_ids
        if stamp < 0.0:
            return "invalid_stamp", wrong_ids
        age = now - stamp
        if age < 0.0:
            return "future_stamp", wrong_ids
        if age > self.max_age_s:
            return "stale", wrong_ids
        if observation.frame_id != self.expected_frame:
            return "wrong_frame", wrong_ids

        try:
            translation = _vector(
                observation.translation_m,
                3,
                "translation_m",
            )
        except TagFilterError:
            return "non_finite_pose", wrong_ids
        try:
            quaternion = _unit_quaternion(observation.quaternion_xyzw)
        except TagFilterError:
            return "invalid_quaternion", wrong_ids
        try:
            camera_position = (
                self.camera_position_m
                if observation.camera_position_m is None
                else _vector(observation.camera_position_m, 3, "camera_position_m")
            )
        except TagFilterError:
            return "invalid_camera_position", wrong_ids
        range_m = math.sqrt(
            sum(
                (translation[index] - camera_position[index]) ** 2
                for index in range(3)
            )
        )
        if range_m <= 0.0 or range_m > self.max_range_m:
            return "impossible_range", wrong_ids
        if (
            abs(translation[2] - self.expected_floor_z_m)
            > self.max_floor_height_error_m
        ):
            return "impossible_height", wrong_ids

        normal = _rotate_vector(quaternion, (0.0, 0.0, 1.0))
        marker_to_camera = tuple(
            camera_position[index] - translation[index] for index in range(3)
        )
        if _dot(normal, marker_to_camera) <= 0.0:
            return "back_facing", wrong_ids
        normal_dot = min(
            max(_dot(normal, self.expected_floor_normal), -1.0),
            1.0,
        )
        if math.acos(normal_dot) > self.max_floor_normal_error_rad:
            return "wrong_floor_normal", wrong_ids
        arrow = _rotate_vector(quaternion, (1.0, 0.0, 0.0))
        if math.hypot(arrow[0], arrow[1]) <= 1e-9:
            return "invalid_arrow_projection", wrong_ids
        return None, wrong_ids

    def _geometry(self, observation):
        translation = _vector(
            observation.translation_m,
            3,
            "translation_m",
        )
        quaternion = _unit_quaternion(observation.quaternion_xyzw)
        normal = _rotate_vector(quaternion, (0.0, 0.0, 1.0))
        normal_dot = min(
            max(_dot(normal, self.expected_floor_normal), -1.0),
            1.0,
        )
        arrow = _rotate_vector(quaternion, (1.0, 0.0, 0.0))
        return _AcceptedGeometry(
            stamp_s=float(observation.stamp_s),
            translation_m=translation,
            quaternion_xyzw=quaternion,
            approach_yaw_rad=math.atan2(arrow[1], arrow[0]),
            floor_normal_error_rad=math.acos(normal_dot),
        )

    def _observation_key(self, observation):
        return (
            int(observation.marker_id),
            float(observation.stamp_s),
            tuple(float(value) for value in observation.translation_m),
            tuple(float(value) for value in observation.quaternion_xyzw),
        )

    def _prune(self, now):
        if any(now < accepted.stamp_s for accepted in self._accepted):
            self._accepted = []
        else:
            self._accepted = [
                accepted
                for accepted in self._accepted
                if now - accepted.stamp_s <= self.max_age_s
            ]
        self._rebuild_keys()
        self._rebuild_outputs()

    def _rebuild_keys(self):
        self._accepted_keys = {
            (
                self.requested_marker_id,
                accepted.stamp_s,
                accepted.translation_m,
                accepted.quaternion_xyzw,
            )
            for accepted in self._accepted
        }

    def _robust_center(self, accepted):
        return _AcceptedGeometry(
            stamp_s=max(item.stamp_s for item in accepted),
            translation_m=tuple(
                median(item.translation_m[index] for item in accepted)
                for index in range(3)
            ),
            quaternion_xyzw=_quaternion_medoid(
                [item.quaternion_xyzw for item in accepted]
            ),
            approach_yaw_rad=_circular_median(
                [item.approach_yaw_rad for item in accepted]
            ),
            floor_normal_error_rad=max(
                item.floor_normal_error_rad for item in accepted
            ),
        )

    def _window_consistency_rejection(self, accepted):
        for left_index, left in enumerate(accepted):
            for right in accepted[left_index + 1 :]:
                translation_spread = math.sqrt(
                    sum(
                        (left.translation_m[index] - right.translation_m[index]) ** 2
                        for index in range(3)
                    )
                )
                if translation_spread > self.max_translation_jump_m:
                    return "translation_jump"
                if (
                    _quaternion_distance(
                        left.quaternion_xyzw,
                        right.quaternion_xyzw,
                    )
                    > self.max_angular_jump_rad
                ):
                    return "angular_jump"
        return None

    def _rebuild_outputs(self):
        if not self._accepted:
            self._window = ()
            self._filtered_pose = None
            self._acquired = False
            return
        center = self._robust_center(self._accepted)
        window = []
        for accepted in self._accepted:
            window.append(
                FilteredTagPose(
                    marker_id=self.requested_marker_id,
                    stamp_s=accepted.stamp_s,
                    frame_id=self.expected_frame,
                    forward_m=accepted.translation_m[0],
                    lateral_m=accepted.translation_m[1],
                    floor_height_m=accepted.translation_m[2],
                    approach_yaw_rad=accepted.approach_yaw_rad,
                    forward_residual_m=abs(
                        accepted.translation_m[0] - center.translation_m[0]
                    ),
                    lateral_residual_m=abs(
                        accepted.translation_m[1] - center.translation_m[1]
                    ),
                    floor_height_residual_m=abs(
                        accepted.translation_m[2] - self.expected_floor_z_m
                    ),
                    floor_normal_residual_rad=(accepted.floor_normal_error_rad),
                    approach_yaw_residual_rad=_angle_distance(
                        accepted.approach_yaw_rad,
                        center.approach_yaw_rad,
                    ),
                )
            )
        self._window = tuple(window)
        self._filtered_pose = FilteredTagPose(
            marker_id=self.requested_marker_id,
            stamp_s=center.stamp_s,
            frame_id=self.expected_frame,
            forward_m=center.translation_m[0],
            lateral_m=center.translation_m[1],
            floor_height_m=center.translation_m[2],
            approach_yaw_rad=center.approach_yaw_rad,
            forward_residual_m=max(item.forward_residual_m for item in self._window),
            lateral_residual_m=max(item.lateral_residual_m for item in self._window),
            floor_height_residual_m=max(
                item.floor_height_residual_m for item in self._window
            ),
            floor_normal_residual_rad=max(
                item.floor_normal_residual_rad for item in self._window
            ),
            approach_yaw_residual_rad=max(
                item.approach_yaw_residual_rad for item in self._window
            ),
        )
        span = self._accepted[-1].stamp_s - self._accepted[0].stamp_s
        self._acquired = (
            len(self._accepted) >= self.required_samples
            and span >= self.min_span_s
            and self._window_consistency_rejection(self._accepted) is None
        )

    def _decision(self, accepted, reason, diagnostic_marker_ids):
        return TagFilterDecision(
            accepted=accepted,
            reason=reason,
            acquired=self._acquired,
            filtered_pose=self._filtered_pose,
            window=self._window,
            diagnostic_marker_ids=tuple(diagnostic_marker_ids),
        )
