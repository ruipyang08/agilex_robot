"""Signed canonical motion snapshots for source-time command shaping.

The receiver supplies monotonic time. Heartbeats cannot turn an old pose interval
into a new measurement, and feedback never grants a producer operation.
"""
from dataclasses import dataclass, replace
import math
from numbers import Real
import struct
from typing import Optional

from .contracts import BaseAuthority, Velocity2D


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('motion values must be finite numbers')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('motion values must be finite numbers')
    return value


@dataclass(frozen=True)
class MotionProgress:
    velocity: Velocity2D
    revision: int
    stamp_s: float
    receipt_s: float
    upstream_age_s: float = 0.0
    receipt_limit_s: float = 0.25
    header_limit_s: float = 0.35
    future_tolerance_s: float = 0.05
    linear_lead_mps: float = 0.002
    angular_lead_radps: float = 0.004
    stamp_ns: Optional[int] = None

    def __post_init__(self):
        if not isinstance(self.velocity, Velocity2D):
            raise ValueError('motion velocity must be Velocity2D')
        if isinstance(self.revision, bool) or not isinstance(self.revision, int) or self.revision <= 0:
            raise ValueError('motion revision must be positive')
        if self.stamp_ns is not None and (isinstance(self.stamp_ns, bool)
                or not isinstance(self.stamp_ns, int) or self.stamp_ns < 0):
            raise ValueError('invalid original ROS nanoseconds')
        for value in (self.velocity.linear_x, self.velocity.linear_y, self.velocity.angular_z):
            _finite(value)
        for name in ('stamp_s', 'receipt_s', 'upstream_age_s', 'receipt_limit_s',
                     'header_limit_s', 'future_tolerance_s', 'linear_lead_mps', 'angular_lead_radps'):
            value = _finite(getattr(self, name))
            if value < 0.0 or (name in ('receipt_limit_s', 'header_limit_s') and value == 0.0):
                raise ValueError('invalid motion age or limit')

    def fresh(self, *, now_s, receipt_now_s):
        try:
            elapsed = _finite(receipt_now_s) - self.receipt_s
            age = _finite(now_s) - self.stamp_s
        except ValueError:
            return False
        return (0.0 <= elapsed and self.upstream_age_s + elapsed <= self.receipt_limit_s
                and -self.future_tolerance_s <= age <= self.header_limit_s)


class MotionProgressCache:
    """Caller-serialized parser of the existing authority feedback message."""

    def __init__(self):
        self._session = None
        self._retired_sessions = set()
        self._policy = None
        self._policy_changed = False
        self._revision = 0
        self._payload = None
        self._sample = None
        self._poisoned_revision = None
        self._binding = None
        self._source = None
        self._selected_source = None
        self._held = True
        self._heartbeat_receipt = None
        self._heartbeat_stamp = None
        self._valid = False

    def observe(self, message, *, receipt_now_s):
        self._valid = False
        try:
            receipt = _finite(receipt_now_s)
            session = message.session_id
            if not isinstance(session, str) or not session:
                return
            if session != self._session:
                if session in self._retired_sessions:
                    return
                retired = self._retired_sessions
                if self._session is not None:
                    retired.add(self._session)
                self.__init__()
                self._retired_sessions = retired
                self._session = session
            self._binding = (session, message.grant_id, message.source_goal_id)
            self._source = BaseAuthority(message.state)
            self._selected_source = message.selected_source
            self._held = message.manipulation_hold_active
            self._heartbeat_receipt = receipt
            self._heartbeat_stamp = _finite(message.header.stamp.to_sec())
            try:
                policy = tuple(_finite(getattr(message, name)) for name in (
                    'motion_receipt_limit_s', 'motion_header_limit_s', 'motion_future_tolerance_s',
                    'linear_progress_lead_mps', 'angular_progress_lead_radps'))
            except (AttributeError, TypeError, ValueError, OverflowError):
                if self._policy is not None:
                    self._policy_changed = True
                return
            if self._policy is None:
                self._policy = policy
            elif self._policy != policy:
                self._policy_changed = True
            if self._policy_changed:
                return
            if message.motion_valid is not True:
                self._poison(message.motion_revision)
                return
            if message.motion_frame_id != 'base_link':
                raise ValueError('unsupported motion frame')
            linear, angular = message.motion.linear, message.motion.angular
            if any(_finite(v) != 0.0 for v in (linear.z, angular.x, angular.y)):
                raise ValueError('unsupported motion component')
            velocity = Velocity2D(_finite(linear.x), _finite(linear.y), _finite(angular.z))
            sample = MotionProgress(velocity=velocity, revision=message.motion_revision,
                stamp_s=_finite(message.motion_stamp.to_sec()), receipt_s=receipt,
                upstream_age_s=_finite(message.motion_receipt_age_s),
                receipt_limit_s=policy[0], header_limit_s=policy[1], future_tolerance_s=policy[2],
                linear_lead_mps=policy[3], angular_lead_radps=policy[4],
                stamp_ns=message.motion_stamp.to_nsec())
            payload = (sample.stamp_ns, velocity)
            if sample.revision < self._revision:
                return
            if sample.revision == self._revision:
                if payload != self._payload:
                    self._poisoned_revision = sample.revision
                if self._poisoned_revision == sample.revision or self._sample is None:
                    return
                # Retain first receipt; larger upstream age remains conservative.
                total_age = max(self._sample.upstream_age_s,
                    sample.upstream_age_s - (receipt - self._sample.receipt_s))
                self._sample = replace(self._sample, upstream_age_s=total_age)
            else:
                if self._sample is not None and sample.stamp_ns <= self._sample.stamp_ns:
                    self._poison(sample.revision)
                    return
                self._sample, self._payload, self._revision = sample, payload, sample.revision
            self._valid = True
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._poison(getattr(message, 'motion_revision', None))
            return

    def _poison(self, revision):
        if isinstance(revision, int) and not isinstance(revision, bool) and revision >= self._revision:
            self._revision = revision
            self._poisoned_revision = revision

    def snapshot(self, *, binding, source, now_s, receipt_now_s):
        expected_source = {BaseAuthority.DOCKING: 'docking', BaseAuthority.NAVIGATION: 'navigation'}.get(source)
        if (not self._valid or not binding or self._binding != binding or self._source != source
                or self._selected_source != expected_source or self._held is not False
                or self._sample is None):
            return None
        try:
            heartbeat_age = _finite(receipt_now_s) - self._heartbeat_receipt
            header_age = _finite(now_s) - self._heartbeat_stamp
        except (TypeError, ValueError):
            return None
        if (not 0.0 <= heartbeat_age <= .25
                or not -self._sample.future_tolerance_s <= header_age <= self._sample.header_limit_s
                or not self._sample.fresh(now_s=now_s, receipt_now_s=receipt_now_s)):
            return None
        return self._sample


def _inward_slew_endpoint(endpoint, previous, maximum_change):
    """Repair outward-rounded arithmetic by one float, also on Python 3.8."""
    if abs(endpoint - previous) <= maximum_change:
        return endpoint
    if endpoint == 0.0:
        return math.copysign(float.fromhex('0x0.0000000000001p-1022'), previous)
    bits = struct.unpack('!Q', struct.pack('!d', endpoint))[0]
    bits += 1 if (endpoint < previous) == (endpoint > 0.0) else -1
    return struct.unpack('!d', struct.pack('!Q', bits))[0]


def bound_reachable_command(previous, candidate, progress, maximum_change):
    """Nearest candidate within independent original slew and progress bounds.

    A contracting progress bound can require deceleration even when the target
    requests acceleration. An empty old-to-candidate segment does not establish
    infeasibility; only an empty reachable axis interval does. Invalid arithmetic
    also fails closed. The owner supplies the existing phase/source-time budget.
    """
    values = {}
    for axis, measured, lead in (
        ('linear_x', progress.velocity.linear_x, progress.linear_lead_mps),
        ('linear_y', progress.velocity.linear_y, progress.linear_lead_mps),
        ('angular_z', progress.velocity.angular_z, progress.angular_lead_radps),
    ):
        try:
            old = _finite(getattr(previous, axis))
            new = _finite(getattr(candidate, axis))
            change = _finite(getattr(maximum_change, axis))
        except (ValueError, OverflowError):
            return None
        if change < 0.0:
            return None
        lower, upper = min(0.0, measured) - lead, max(0.0, measured) + lead
        slew_lower, slew_upper = old - change, old + change
        if not all(math.isfinite(value) for value in (lower, upper, slew_lower, slew_upper)):
            return None
        slew_lower = _inward_slew_endpoint(slew_lower, old, change)
        slew_upper = _inward_slew_endpoint(slew_upper, old, change)
        low, high = max(lower, slew_lower), min(upper, slew_upper)
        if low > high:
            return None
        value = min(high, max(low, new))
        if not (lower <= value <= upper and abs(value - old) <= change):
            return None
        values[axis] = value
    return type(candidate)(**values)
