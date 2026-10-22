"""ROS-free odometry projection, dwell, and speed-limit helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real


@dataclass(frozen=True)
class PathProjection:
    along_m: float
    cross_m: float
    yaw_drift_rad: float


def project_pose(origin, pose):
    """Project pose displacement into the fixed origin heading."""

    dx = _finite(pose.x_m - origin.x_m, "projection_dx")
    dy = _finite(pose.y_m - origin.y_m, "projection_dy")
    yaw_delta = _finite(pose.yaw_rad - origin.yaw_rad, "projection_yaw_delta")
    cosine = math.cos(origin.yaw_rad)
    sine = math.sin(origin.yaw_rad)
    return PathProjection(
        along_m=_finite(cosine * dx + sine * dy, "projection_along"),
        cross_m=_finite(-sine * dx + cosine * dy, "projection_cross"),
        yaw_drift_rad=_finite(
            math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)),
            "projection_yaw_drift",
        ),
    )


def advance_dwell(started_s, condition, now_s, dwell_s):
    """Return ``(new_started_s, complete)`` for one continuous predicate."""

    if not isinstance(condition, bool):
        raise ValueError("condition must be a bool")
    now = _finite(now_s, "now_s")
    dwell = _finite(dwell_s, "dwell_s")
    if dwell < 0.0:
        raise ValueError("dwell_s must be nonnegative")
    if not condition:
        return None, False
    if started_s is None:
        return now, dwell == 0.0
    started = _finite(started_s, "started_s")
    if now < started:
        raise ValueError("now_s must not precede started_s")
    return started, now - started + 1e-12 >= dwell


def bounded_speed_magnitude(
    *,
    distance_to_stop_m,
    previous_mps,
    maximum_mps,
    minimum_mps,
    acceleration_mps2,
    slowdown_distance_m,
    elapsed_s,
):
    """Return one nonnegative stopping- and rate-limited speed magnitude."""

    distance = _finite(distance_to_stop_m, "distance_to_stop_m")
    previous = _finite(previous_mps, "previous_mps")
    maximum = _finite(maximum_mps, "maximum_mps")
    minimum = _finite(minimum_mps, "minimum_mps")
    acceleration = _finite(acceleration_mps2, "acceleration_mps2")
    slowdown_distance = _finite(slowdown_distance_m, "slowdown_distance_m")
    elapsed = _finite(elapsed_s, "elapsed_s")
    if distance < 0.0:
        raise ValueError("distance_to_stop_m must be nonnegative")
    if previous < 0.0:
        raise ValueError("previous_mps must be nonnegative")
    if maximum <= 0.0:
        raise ValueError("maximum_mps must be positive")
    if minimum < 0.0 or minimum > maximum:
        raise ValueError("minimum_mps must be between zero and maximum_mps")
    if acceleration <= 0.0:
        raise ValueError("acceleration_mps2 must be positive")
    if slowdown_distance <= 0.0:
        raise ValueError("slowdown_distance_m must be positive")
    if elapsed < 0.0:
        raise ValueError("elapsed_s must be nonnegative")
    if distance == 0.0:
        return 0.0

    stopping_cap = math.sqrt(2.0 * acceleration * distance)
    slowdown_cap = maximum * min(1.0, distance / slowdown_distance)
    target = max(minimum, min(maximum, stopping_cap, slowdown_cap))
    maximum_change = acceleration * elapsed
    bounded = max(
        max(0.0, previous - maximum_change),
        min(previous + maximum_change, target),
    )
    return min(maximum, max(0.0, bounded))


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{0} must be a finite numeric value".format(name))
    try:
        converted = float(value)
    except (OverflowError, TypeError, ValueError):
        raise ValueError("{0} must be a finite numeric value".format(name))
    if not math.isfinite(converted):
        raise ValueError("{0} must be a finite numeric value".format(name))
    return converted
