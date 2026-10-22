"""ROS-independent types shared by deployable workbench navigation nodes."""

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import math
from numbers import Real


class ActionResultCode(IntEnum):
    """Values generated on the result side of NavigateToWorkbench.action."""

    OK = 0
    INVALID_REGISTRY = 10
    NAVIGATION_REJECTED = 20
    NAVIGATION_ABORTED = 21
    NAVIGATION_TIMEOUT = 22
    ROUTE_UNAVAILABLE = 23
    ROUTE_SEARCH_LIMIT = 24
    ROUTE_INVALIDATED = 25
    ROUTE_BLOCKED = 26
    ROUTE_EVIDENCE_UNAVAILABLE = 27
    ROUTE_START_MISMATCH = 28
    LOCALIZATION_UNAVAILABLE = 30
    TF_UNAVAILABLE = 31
    CAMERA_NOT_READY = 40
    TAG_ACQUISITION_TIMEOUT = 41
    TAG_EVIDENCE_INVALID = 42
    HANDOFF_INVALID = 43
    BLIND_TRAVEL_LIMIT = 44
    LIDAR_BLOCKED = 45
    LIDAR_STALE = 46
    LIDAR_INVALID = 47
    AUTHORITY_DENIED = 50
    AUTHORITY_FAULT = 51
    DOCKING_TIMEOUT = 60
    DOCKING_OSCILLATION = 61
    FINAL_TOLERANCE_NOT_REACHED = 62
    BLIND_DEVIATION = 63
    ALIGNMENT_RETRY_EXHAUSTED = 64
    ODOM_NOT_SETTLED = 70
    CANCELLED = 80
    INTERNAL_CONFIGURATION_ERROR = 90


class ActionPhase(IntEnum):
    """Values generated on the feedback side of NavigateToWorkbench.action."""

    LOOKUP = 0
    COARSE_NAVIGATION = 1
    BASE_SETTLING = 2
    TAG_ACQUISITION = 3
    AUTHORITY_HANDOVER = 4
    FINAL_DOCKING = 5
    FINAL_SETTLING = 6
    SUCCEEDED = 7
    FAILED = 8
    CANCELLED = 9
    DEPARTING_DOCK = 10
    DEPARTURE_HANDOVER = 11
    DEPARTURE_AUTHORITY = 12


class BaseAuthority(IntEnum):
    """SetBaseAuthority and BaseAuthorityState authority constants."""

    NONE = 0
    NAVIGATION = 1
    STOPPING_FOR_HANDOVER = 2
    DOCKING = 3
    FAULT = 4


class SafetyHealth(IntEnum):
    """LidarSafetyState safety-state constants."""

    CLEAR = 0
    BLOCKED = 1
    STALE = 2
    INVALID = 3


class MotionDirection(IntFlag):
    """LidarSafetyState blocked-direction bit constants."""

    NONE = 0
    FORWARD = 1
    REVERSE = 2
    LEFT = 4
    RIGHT = 8
    ROTATION = 16
    ALL = 31


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{0} must be a finite numeric value".format(name))
    try:
        converted = float(value)
    except OverflowError:
        raise ValueError("{0} must be a finite numeric value".format(name))
    if not math.isfinite(converted):
        raise ValueError("{0} must be a finite numeric value".format(name))
    return converted


@dataclass(frozen=True)
class PlanarPose:
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self):
        object.__setattr__(self, "x_m", _finite(self.x_m, "x_m"))
        object.__setattr__(self, "y_m", _finite(self.y_m, "y_m"))
        object.__setattr__(self, "yaw_rad", _finite(self.yaw_rad, "yaw_rad"))


@dataclass(frozen=True)
class PlanarError:
    position_m: float
    yaw_rad: float

    def __post_init__(self):
        object.__setattr__(self, "position_m", _finite(self.position_m, "position_m"))
        object.__setattr__(self, "yaw_rad", _finite(self.yaw_rad, "yaw_rad"))


@dataclass(frozen=True)
class Velocity2D:
    linear_x: float
    linear_y: float
    angular_z: float

    def __post_init__(self):
        object.__setattr__(self, "linear_x", _finite(self.linear_x, "linear_x"))
        object.__setattr__(self, "linear_y", _finite(self.linear_y, "linear_y"))
        object.__setattr__(self, "angular_z", _finite(self.angular_z, "angular_z"))


@dataclass(frozen=True)
class StampedVelocity:
    stamp_s: float
    velocity: Velocity2D

    def __post_init__(self):
        object.__setattr__(self, "stamp_s", _finite(self.stamp_s, "stamp_s"))
        if not isinstance(self.velocity, Velocity2D):
            raise ValueError("velocity must be a Velocity2D")


@dataclass(frozen=True)
class OdomPose2D:
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self):
        object.__setattr__(self, "x_m", _finite(self.x_m, "x_m"))
        object.__setattr__(self, "y_m", _finite(self.y_m, "y_m"))
        object.__setattr__(self, "yaw_rad", _finite(self.yaw_rad, "yaw_rad"))


@dataclass(frozen=True)
class DirectionalClearance:
    forward_m: float
    reverse_m: float
    left_m: float
    right_m: float
    rotation_m: float
    sensor_max_m: float

    def __post_init__(self):
        sensor_max_m = _finite(self.sensor_max_m, "sensor_max_m")
        if sensor_max_m <= 0.0:
            raise ValueError("sensor_max_m must be positive")
        object.__setattr__(self, "sensor_max_m", sensor_max_m)
        for field_name in (
            "forward_m",
            "reverse_m",
            "left_m",
            "right_m",
            "rotation_m",
        ):
            clearance = _finite(getattr(self, field_name), field_name)
            if clearance < 0.0 or clearance > sensor_max_m:
                raise ValueError(
                    "{0} must be within the finite sensor maximum".format(field_name)
                )
            object.__setattr__(self, field_name, clearance)


@dataclass(frozen=True)
class FinalErrorEstimate:
    error: PlanarError
    is_estimated: bool
    source: str

    def __post_init__(self):
        if not isinstance(self.error, PlanarError):
            raise ValueError("error must be a PlanarError")
        if not isinstance(self.is_estimated, bool):
            raise ValueError("is_estimated must be a bool")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a non-empty string")
