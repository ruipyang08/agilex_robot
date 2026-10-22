"""Source-time base progress from measured poses, independent of ROS.

The wrapper must provide the complete T_child_base transform at the original
odometry stamp. This module never substitutes identity, reads a clock, or uses
message Twist as motion evidence. Results are finite-interval averages, not an
instantaneous acceleration guarantee or proof that an upstream sensor is fresh.
"""
from dataclasses import dataclass
import math
from numbers import Real
from typing import Optional


def _number(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('expected finite number')
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise ValueError('expected finite number')
    if not math.isfinite(result):
        raise ValueError('expected finite number')
    return result


def _frame(value):
    return (isinstance(value, str) and bool(value) and
            not value.startswith('/') and not any(c.isspace() for c in value))


@dataclass(frozen=True)
class PoseIntervalConfig:
    parent_frame: str
    child_frame: str
    command_frame: str
    min_interval_s: float
    max_interval_s: float
    max_translation_step_m: float
    max_rotation_step_rad: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    quaternion_norm_tolerance: float
    extrinsic_translation_tolerance_m: float
    extrinsic_rotation_tolerance_rad: float

    def __post_init__(self):
        for name in ('parent_frame', 'child_frame', 'command_frame'):
            if not _frame(getattr(self, name)):
                raise ValueError('invalid exact frame: ' + name)
        if self.parent_frame in (self.child_frame, self.command_frame):
            raise ValueError('odometry parent must differ from moving frames')
        names = (
            'min_interval_s', 'max_interval_s', 'max_translation_step_m',
            'max_rotation_step_rad', 'max_linear_speed_mps', 'max_angular_speed_radps',
            'quaternion_norm_tolerance', 'extrinsic_translation_tolerance_m',
            'extrinsic_rotation_tolerance_rad',
        )
        for name in names:
            value = _number(getattr(self, name))
            if value <= 0:
                raise ValueError(name + ' must be positive')
            object.__setattr__(self, name, value)
        if self.min_interval_s >= self.max_interval_s:
            raise ValueError('source interval bounds are reversed')
        if self.max_rotation_step_rad >= math.pi:
            raise ValueError('rotation discontinuity bound must be below pi')
        if (self.quaternion_norm_tolerance >= .01 or
                self.extrinsic_rotation_tolerance_rad >= .01):
            raise ValueError('quaternion/extrinsic tolerances must remain numerical tolerances')


@dataclass(frozen=True)
class PoseInterval:
    valid: bool
    reason: str
    vx_mps: Optional[float]
    vy_mps: Optional[float]
    wz_radps: Optional[float]
    source_stamp_s: Optional[float]
    receipt_s: Optional[float]
    interval_s: Optional[float]


def _vector(value, count):
    if value is None or isinstance(value, (str, bytes)):
        raise ValueError('invalid vector')
    try:
        if len(value) != count:
            raise ValueError('invalid vector length')
        return tuple(_number(value[i]) for i in range(count))
    except (TypeError, IndexError, KeyError):
        raise ValueError('invalid vector')


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by + ay*bw + az*bx - ax*bz,
            aw*bz + az*bw + ax*by - ay*bx,
            aw*bw - ax*bx - ay*by - az*bz)


def _inverse(q):
    return (-q[0], -q[1], -q[2], q[3])


def _rotate(q, vector):
    return _multiply(_multiply(q, vector + (0.,)), _inverse(q))[:3]


def _quaternion(value, tolerance):
    q = _vector(value, 4)
    norm = math.hypot(*q)
    if not math.isfinite(norm) or abs(norm - 1.) > tolerance:
        raise ValueError('quaternion is not unit length')
    # Correct only validated representation roundoff. A scaled quaternion is
    # rejected above, never silently interpreted as a valid orientation.
    return tuple(v / norm for v in q)


def _rotation_vector(current, previous):
    """Shortest finite rotation in parent axes; q and -q are equivalent."""
    delta = _multiply(current, _inverse(previous))
    if delta[3] < 0:
        delta = tuple(-v for v in delta)
    sine = math.hypot(*delta[:3])
    angle = 2. * math.atan2(sine, delta[3])
    if sine == 0:
        return (0., 0., 0.), 0.
    return tuple(v * (angle / sine) for v in delta[:3]), angle


class PoseIntervalEstimator:
    """Single-owner estimator; its ROS wrapper serializes callbacks.

    Invalid evidence clears the pair and returns unknown velocities. Source and
    receipt high-water marks, and the first accepted fixed extrinsic, survive
    invalidation. Recovery requires a new baseline and a later valid endpoint;
    a rejected endpoint is never reused as that baseline. Reconfigure by making
    a new estimator while authority is stopped, not by resetting these fences.
    """

    def __init__(self, config):
        if not isinstance(config, PoseIntervalConfig):
            raise ValueError('PoseIntervalConfig is required')
        self.config = config
        self._previous = None
        self._extrinsic = None
        self._source_highwater = None
        self._receipt_highwater = None

    @property
    def current_base_pose(self):
        """Original accepted (source, receipt, XYZ, XYZW), or unavailable.

        Tuple contents are immutable. Publication never changes these stamps.
        """
        return self._previous

    def _deny(self, reason, source=None, receipt=None):
        self._previous = None
        return PoseInterval(False, reason, None, None, None, source, receipt, None)

    def _timestamps(self, source, receipt):
        source, receipt = _number(source), _number(receipt)
        if source <= 0 or receipt < 0:
            raise ValueError('invalid source/receipt timestamp')
        return source, receipt

    def _advance_highwaters(self, source, receipt):
        if self._source_highwater is None or source > self._source_highwater:
            self._source_highwater = source
        if self._receipt_highwater is None or receipt > self._receipt_highwater:
            self._receipt_highwater = receipt

    def invalidate(self, reason, *, source_stamp_s=None, receipt_s=None):
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError('invalidation reason is required')
        try:
            source, receipt = self._timestamps(source_stamp_s, receipt_s)
        except ValueError:
            source, receipt = None, None
        if source is not None:
            self._advance_highwaters(source, receipt)
        return self._deny(reason, source, receipt)

    def update(self, position_xyz, quaternion_xyzw, child_to_base_xyz,
               child_to_base_xyzw, *, source_stamp_s, receipt_s,
               parent_frame, child_frame, command_frame):
        try:
            source, receipt = self._timestamps(source_stamp_s, receipt_s)
        except ValueError:
            return self._deny('invalid_timestamps')
        previous_source, previous_receipt = self._source_highwater, self._receipt_highwater
        self._advance_highwaters(source, receipt)
        if previous_source is not None and source <= previous_source:
            return self._deny('source_stamp_not_increasing', source, receipt)
        if previous_receipt is not None and receipt <= previous_receipt:
            return self._deny('receipt_not_increasing', source, receipt)
        c = self.config
        if (parent_frame != c.parent_frame or child_frame != c.child_frame or
                command_frame != c.command_frame):
            return self._deny('frame_mismatch', source, receipt)
        try:
            position = _vector(position_xyz, 3)
            child_q = _quaternion(quaternion_xyzw, c.quaternion_norm_tolerance)
            offset = _vector(child_to_base_xyz, 3)
            relative_q = _quaternion(child_to_base_xyzw, c.quaternion_norm_tolerance)
            # Compose actual base origin and orientation before differencing;
            # subtracting child poses then rotating cannot cancel sensor offset.
            translated = _rotate(child_q, offset)
            base_position = tuple(a+b for a, b in zip(position, translated))
            base_q = _multiply(child_q, relative_q)
            if not all(math.isfinite(v) for v in base_position + base_q):
                raise ValueError('nonfinite composed pose')
        except (ValueError, OverflowError):
            return self._deny('invalid_pose_or_extrinsic', source, receipt)
        if self._extrinsic is not None:
            old_offset, old_relative_q = self._extrinsic
            shift = math.hypot(*(a-b for a, b in zip(offset, old_offset)))
            _, rotation = _rotation_vector(relative_q, old_relative_q)
            if (not math.isfinite(shift) or shift > c.extrinsic_translation_tolerance_m or
                    rotation > c.extrinsic_rotation_tolerance_rad):
                return self._deny('extrinsic_changed', source, receipt)
        else:
            self._extrinsic = (offset, relative_q)
        current = (source, receipt, base_position, base_q)
        if self._previous is None:
            self._previous = current
            return PoseInterval(False, 'first_sample', None, None, None, source, receipt, None)
        old_source, _, old_position, old_q = self._previous
        interval = source - old_source
        if not c.min_interval_s <= interval <= c.max_interval_s:
            return self._deny('source_interval_out_of_bounds', source, receipt)
        displacement = tuple(a-b for a, b in zip(base_position, old_position))
        distance = math.hypot(*displacement)
        rotation_world, angle = _rotation_vector(base_q, old_q)
        if not math.isfinite(distance) or distance > c.max_translation_step_m:
            return self._deny('translation_discontinuity', source, receipt)
        if not math.isfinite(angle) or angle > c.max_rotation_step_rad:
            return self._deny('rotation_discontinuity', source, receipt)
        if distance / interval > c.max_linear_speed_mps:
            return self._deny('linear_rate_exceeded', source, receipt)
        if angle / interval > c.max_angular_speed_radps:
            return self._deny('angular_rate_exceeded', source, receipt)
        linear = _rotate(_inverse(base_q), tuple(v/interval for v in displacement))
        angular = _rotate(_inverse(base_q), tuple(v/interval for v in rotation_world))
        if not all(math.isfinite(v) for v in linear + angular):
            return self._deny('nonfinite_interval_velocity', source, receipt)
        self._previous = current
        return PoseInterval(True, 'ok', linear[0], linear[1], angular[2],
                            source, receipt, interval)
