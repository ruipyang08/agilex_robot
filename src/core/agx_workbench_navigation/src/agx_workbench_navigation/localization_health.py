"""Continuous localization evidence for the onboard mission; no ROS dependency."""

import math


def _finite(values, size):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if len(result) == size and all(math.isfinite(x) for x in result) else None


def _pose(position, orientation, covariance):
    position = _finite(position, 3)
    orientation = _finite(orientation, 4)
    covariance = _finite(covariance, 36)
    if position is None or orientation is None or covariance is None:
        return None
    norm = sum(value * value for value in orientation)
    if not math.isfinite(norm) or abs(norm - 1.) > .001:
        return None
    if any(covariance[index * 7] < 0. for index in range(6)):
        return None
    return position, orientation


def _yaw(q):
    x, y, z, w = q
    return math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))


def _yaw_distance(a, b):
    return abs(math.atan2(math.sin(_yaw(a)-_yaw(b)), math.cos(_yaw(a)-_yaw(b))))


class LocalizationHealth:
    """Pose and odometry leases plus consistency with the current TF chain.

    Limits describe evidence freshness/continuity, not permission to move. Travel
    posture qualification and the arbiter's independent safety gates still apply.
    """

    def __init__(self, receipt_age_s=.75, source_age_s=.75, future_skew_s=.05,
                 linear_speed_mps=.5, angular_speed_radps=.8,
                 position_tolerance_m=.10, yaw_tolerance_rad=.10):
        values = (receipt_age_s, source_age_s, future_skew_s, linear_speed_mps,
                  angular_speed_radps, position_tolerance_m, yaw_tolerance_rad)
        if any(not math.isfinite(value) or value <= 0. for value in values):
            raise ValueError('localization bounds must be finite and positive')
        if receipt_age_s > 5. or source_age_s > 5.:
            raise ValueError('localization age bounds must not exceed 5 seconds')
        self.receipt_age_s, self.source_age_s = receipt_age_s, source_age_s
        self.future_skew_s = future_skew_s
        self.linear_speed_mps, self.angular_speed_radps = linear_speed_mps, angular_speed_radps
        self.position_tolerance_m, self.yaw_tolerance_rad = position_tolerance_m, yaw_tolerance_rad
        self._pose = self._odom = None
        self._last_valid_pose = None
        self._high_water = {}

    def _time(self, stream, stamp, receipt, now):
        values = _finite((stamp, receipt, now), 3)
        if values is None:
            return False
        stamp, receipt, now = values
        previous = self._high_water.get(stream)
        if (stamp <= 0. or now <= 0. or receipt < 0. or
                not -self.future_skew_s <= now-stamp <= self.source_age_s or
                (previous is not None and (stamp <= previous[0] or receipt < previous[1]))):
            return False
        self._high_water[stream] = stamp, receipt
        return True

    def observe_pose(self, *, frame, position, orientation, covariance,
                     stamp_s, receipt_s, source_now_s):
        self._pose = None
        pose = _pose(position, orientation, covariance)
        if not self._time('pose', stamp_s, receipt_s, source_now_s) or frame != 'map' or pose is None:
            return False
        previous = self._last_valid_pose
        if previous is not None:
            dt = min(self.source_age_s, stamp_s - previous[0])
            distance = math.hypot(pose[0][0]-previous[2][0][0], pose[0][1]-previous[2][0][1])
            if (distance > self.position_tolerance_m + self.linear_speed_mps * dt or
                    _yaw_distance(pose[1],previous[2][1]) > self.yaw_tolerance_rad + self.angular_speed_radps * dt):
                return False
        self._pose = stamp_s, receipt_s, pose
        self._last_valid_pose = self._pose
        return True

    def observe_odometry(self, *, frame, child_frame, position, orientation, covariance,
                         velocity, stamp_s, receipt_s, source_now_s):
        self._odom = None
        if (not self._time('odom', stamp_s, receipt_s, source_now_s) or
                frame != 'odom' or child_frame != 'rslidar' or
                _pose(position, orientation, covariance) is None or _finite(velocity, 6) is None):
            return False
        self._odom = stamp_s, receipt_s
        return True

    def pose_needs_refresh(self, receipt_now_s):
        return self._pose is None or receipt_now_s - self._pose[1] >= self.receipt_age_s * .5

    def ready(self, *, receipt_now_s, source_now_s, transform_frame, transform_child,
              transform_position, transform_orientation):
        if self._pose is None or self._odom is None:
            return False
        for sample in (self._pose, self._odom):
            if (not 0. <= receipt_now_s-sample[1] <= self.receipt_age_s or
                    not -self.future_skew_s <= source_now_s-sample[0] <= self.source_age_s):
                return False
        transform = _pose(transform_position, transform_orientation, (0.,)*36)
        if transform_frame != 'map' or transform_child != 'rslidar' or transform is None:
            return False
        pose = self._pose[2]
        age = max(0., source_now_s-self._pose[0])
        distance = math.hypot(transform[0][0]-pose[0][0], transform[0][1]-pose[0][1])
        return (distance <= self.position_tolerance_m + self.linear_speed_mps * age and
                _yaw_distance(transform[1],pose[1]) <= self.yaw_tolerance_rad + self.angular_speed_radps * age)
