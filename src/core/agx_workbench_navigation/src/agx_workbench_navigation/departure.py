"""Measured reverse departure from a registered dock before coarse execution.

The static global costmap includes a docked rover in its inflated obstacle
region. Route admission checks this phase and the complete onward route before
motion. This controller uses the docking command path to reach the source
station's staging corridor, then yields to ``move_base`` only after a fresh
measured stop and native route revalidation. Task history cannot prove motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .contracts import OdomPose2D
from .docking import DockingConfig
from .docking_motion import bounded_speed_magnitude, project_pose


class DepartureSelectionError(ValueError):
    """A nearby registered dock cannot support a measured reverse departure."""


@dataclass(frozen=True)
class DeparturePlan:
    source_id: str
    distance_m: float


@dataclass(frozen=True)
class DepartureStep:
    command_mps: float = 0.0
    progress_m: float = 0.0
    complete: bool = False
    failure_reason: str = ""


def _yaw(orientation):
    if len(orientation) != 4:
        raise DepartureSelectionError("invalid registered dock orientation")
    x, y, z, w = (float(value) for value in orientation)
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise DepartureSelectionError("invalid registered dock orientation")
    if abs(x * x + y * y + z * z + w * w - 1.0) > 1.0e-3:
        raise DepartureSelectionError("invalid registered dock orientation")
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def select_registered_departure(pose, workbenches, target_id, max_retreat_m):
    """Select a source dock from current map pose, never task history.

    An open-space start returns ``None``. A pose near a station with an unsafe
    orientation or an unreachable staging pose raises instead of silently
    sending a global goal from inside the inflated dock region.
    """

    if not isinstance(pose, OdomPose2D):
        raise DepartureSelectionError("measured map pose unavailable")
    maximum = float(max_retreat_m)
    if not math.isfinite(maximum) or maximum <= 0.0:
        raise DepartureSelectionError("invalid reverse departure limit")
    nearby = []
    for entry in workbenches:
        marker = entry.map_pose["position_m"]
        mx, my = float(marker[0]), float(marker[1])
        if math.hypot(pose.x_m - mx, pose.y_m - my) <= 0.50:
            nearby.append(entry)
    if not nearby:
        return None
    if len(nearby) != 1 or nearby[0].workbench_id == target_id:
        raise DepartureSelectionError("ambiguous registered departure dock")

    source = nearby[0]
    marker_yaw = _yaw(source.map_pose["orientation_xyzw"])
    if _angle_error(pose.yaw_rad, marker_yaw) > 0.20:
        raise DepartureSelectionError("registered departure heading mismatch")
    stage = source.staging_pose["position_m"]
    dx = float(stage[0]) - pose.x_m
    dy = float(stage[1]) - pose.y_m
    along = math.cos(pose.yaw_rad) * dx + math.sin(pose.yaw_rad) * dy
    cross = -math.sin(pose.yaw_rad) * dx + math.cos(pose.yaw_rad) * dy
    distance = -along
    if not all(math.isfinite(v) for v in (dx, dy, cross, distance)) or (
        not 0.50 <= distance <= maximum or abs(cross) > 0.15
    ):
        raise DepartureSelectionError("registered staging pose is not a bounded reverse corridor")
    return DeparturePlan(source.workbench_id, distance)


class ReverseDepartureController:
    """One bounded, odometry-driven retreat with rear-clearance admission."""

    def __init__(self, plan, config):
        if not isinstance(plan, DeparturePlan) or not isinstance(config, DockingConfig):
            raise ValueError("reverse departure requires a plan and docking config")
        if not 0.0 < plan.distance_m <= config.max_rollback_travel_m:
            raise ValueError("reverse departure exceeds the configured travel limit")
        self._plan = plan
        self._config = config
        self._origin = None
        self._started_s = None
        self._last_s = None
        self._last_speed_mps = 0.0
        self._stopped_since_s = None
        self._terminal = None

    def start(self, pose, now_s):
        if self._origin is not None or not isinstance(pose, OdomPose2D):
            raise ValueError("reverse departure requires one measured odometry origin")
        now = float(now_s)
        if not math.isfinite(now) or now < 0.0:
            raise ValueError("invalid reverse departure clock")
        self._origin = pose
        self._started_s = now
        self._last_s = now

    def step(self, pose, linear_speed_mps, angular_speed_radps,
             odom_fresh, rear_clear, now_s):
        if self._origin is None:
            raise RuntimeError("reverse departure has not started")
        if self._terminal is not None:
            return self._terminal
        now = float(now_s)
        if not math.isfinite(now) or now < self._last_s:
            return self._fail("departure_deviation")
        elapsed = now - self._last_s
        self._last_s = now
        if not odom_fresh or not isinstance(pose, OdomPose2D):
            return self._fail("odometry_not_fresh")
        speeds = (float(linear_speed_mps), float(angular_speed_radps))
        if not all(math.isfinite(v) and v >= 0.0 for v in speeds):
            return self._fail("odometry_not_fresh")
        projection = project_pose(self._origin, pose)
        progress = -projection.along_m
        remaining = self._plan.distance_m - progress
        if (abs(projection.cross_m) > self._config.max_cross_track_m or
                abs(projection.yaw_drift_rad) > self._config.max_yaw_drift_rad or
                progress < -self._config.rollback_completion_tolerance_m or
                progress > self._plan.distance_m + self._config.rollback_overrun_allowance_m or
                progress > self._config.max_rollback_travel_m):
            return self._fail("departure_deviation", progress)
        if now - self._started_s > self._config.rollback_timeout_s:
            return self._fail("departure_timeout", progress)
        if remaining <= self._config.rollback_completion_tolerance_m:
            self._last_speed_mps = 0.0
            stopped = (
                speeds[0] <= self._config.stop_linear_speed_mps and
                speeds[1] <= self._config.stop_angular_speed_radps
            )
            if not stopped:
                self._stopped_since_s = None
            elif self._stopped_since_s is None:
                self._stopped_since_s = now
            elif now - self._stopped_since_s >= self._config.stop_dwell_s:
                self._terminal = DepartureStep(progress_m=progress, complete=True)
                return self._terminal
            return DepartureStep(progress_m=progress)
        if rear_clear is not True:
            return self._fail("lidar_blocked", progress)
        speed = bounded_speed_magnitude(
            distance_to_stop_m=remaining,
            previous_mps=self._last_speed_mps,
            maximum_mps=self._config.max_rollback_speed_mps,
            minimum_mps=self._config.min_rollback_speed_mps,
            acceleration_mps2=self._config.max_rollback_acceleration_mps2,
            slowdown_distance_m=self._config.slowdown_distance_m,
            elapsed_s=elapsed,
        )
        self._last_speed_mps = speed
        return DepartureStep(command_mps=-speed, progress_m=progress)

    def _fail(self, reason, progress=0.0):
        self._terminal = DepartureStep(progress_m=progress, failure_reason=reason)
        return self._terminal
