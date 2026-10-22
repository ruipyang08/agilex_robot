"""Immutable sensor-capture provenance for route and stopping-sweep checks."""
from dataclasses import dataclass
import math
from numbers import Real
from typing import Tuple, Union

from .contracts import SafetyHealth
from .lidar_safety import Point3D, CompactXYZ, MAX_POINTCLOUD_POINTS


MAX_OBSTACLE_POINTS = MAX_POINTCLOUD_POINTS


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError('finite numeric capture value required')
    return float(value)


def _digest(value):
    return (isinstance(value, str) and value.startswith('sha256:') and len(value) == 71 and
            all(c in '0123456789abcdef' for c in value[7:]))


@dataclass(frozen=True)
class CapturePose:
    position_xyz_m: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]

    def __post_init__(self):
        p, q = tuple(self.position_xyz_m), tuple(self.orientation_xyzw)
        if len(p) != 3 or len(q) != 4:
            raise ValueError('capture transform requires XYZ/XYZW')
        p, q = tuple(map(_finite, p)), tuple(map(_finite, q))
        if abs(sum(v*v for v in q)-1.) > 1e-6:
            raise ValueError('normalized capture quaternion required')
        object.__setattr__(self, 'position_xyz_m', p)
        object.__setattr__(self, 'orientation_xyzw', q)


@dataclass(frozen=True)
class ObstacleCapture:
    cloud_sequence: int
    cloud_stamp_s: float
    transform_stamp_s: float
    transform_static: bool
    receipt_steady_s: float
    source_frame: str
    # odom -> base_link at the cloud's original stamp, never the newest pose.
    capture_pose: CapturePose
    points: Union[Tuple[Point3D, ...], CompactXYZ]
    policy_digest: str
    filter_digest: str

    def __post_init__(self):
        if type(self.cloud_sequence) is not int or not 0 <= self.cloud_sequence < 2**32:
            raise ValueError('original uint32 cloud sequence required')
        for name in ('cloud_stamp_s', 'transform_stamp_s', 'receipt_steady_s'):
            object.__setattr__(self, name, _finite(getattr(self, name)))
        if (self.cloud_stamp_s <= 0 or self.receipt_steady_s < 0 or self.transform_stamp_s < 0 or
                type(self.transform_static) is not bool or
                (self.transform_stamp_s == 0) != self.transform_static):
            raise ValueError('original capture stamps and explicit static transform required')
        if (not isinstance(self.source_frame, str) or not self.source_frame or
                not isinstance(self.capture_pose, CapturePose) or
                not _digest(self.policy_digest) or not _digest(self.filter_digest)):
            raise ValueError('capture frame, pose and installed digests required')
        if not isinstance(self.points, (tuple, CompactXYZ)) or len(self.points) > MAX_OBSTACLE_POINTS:
            raise ValueError('immutable obstacle points exceed the bound')
        if isinstance(self.points, CompactXYZ):
            import numpy as np
            if not np.isfinite(self.points.xyz_m).all():
                raise ValueError('finite filtered obstacle points required')
            return
        if any(not isinstance(p, Point3D) or not all(math.isfinite(v) for v in (p.x, p.y, p.z))
               for p in self.points):
            raise ValueError('finite filtered obstacle points required')


def snapshot_health(capture, sensor_health, source_now_s, receipt_now_s,
                    source_max_age_s, receipt_max_age_s):
    if capture is None or not isinstance(capture, ObstacleCapture) or type(sensor_health) is not SafetyHealth:
        return SafetyHealth.INVALID
    if sensor_health in (SafetyHealth.INVALID, SafetyHealth.STALE):
        return sensor_health
    try:
        source_now, receipt_now, source_limit, receipt_limit = map(_finite,
            (source_now_s, receipt_now_s, source_max_age_s, receipt_max_age_s))
    except ValueError:
        return SafetyHealth.INVALID
    if source_limit <= 0 or receipt_limit <= 0:
        return SafetyHealth.INVALID
    source_ages = [source_now-capture.cloud_stamp_s]
    if not capture.transform_static:
        source_ages.append(source_now-capture.transform_stamp_s)
    receipt_age = receipt_now-capture.receipt_steady_s
    if receipt_age < 0 or any(age < 0 for age in source_ages):
        return SafetyHealth.INVALID
    if receipt_age > receipt_limit or any(age > source_limit for age in source_ages):
        return SafetyHealth.STALE
    return sensor_health
