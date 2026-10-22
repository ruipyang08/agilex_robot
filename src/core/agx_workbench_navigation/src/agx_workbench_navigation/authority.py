"""Fail-closed arbitration for autonomous rover base commands.

The pure state machine in this module intentionally has no ROS dependency.
ROS callbacks provide both a monotonic receipt clock and the ROS header clock
used by stamped sensor evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import math
from numbers import Integral, Real
from typing import Mapping, Optional
import uuid

from .contracts import BaseAuthority, MotionDirection, SafetyHealth, Velocity2D
from .command_progress import FinalCommandLimiter
from .pose_interval import PoseIntervalConfig


_ZERO = Velocity2D(0.0, 0.0, 0.0)
_MOVING_AUTHORITIES = (BaseAuthority.NAVIGATION, BaseAuthority.DOCKING)


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


def _exact_mapping(value, expected_keys, name):
    if not isinstance(value, Mapping) or set(value) != set(expected_keys):
        raise ValueError("{0} schema is invalid".format(name))
    return value


@dataclass(frozen=True)
class MotionFeedback:
    valid: bool
    revision: int
    header_stamp_s: float
    receipt_age_s: float
    command: Velocity2D
    reason: str


@dataclass(frozen=True)
class AuthorityConfig:
    """Commissioned limits and freshness bounds for the base arbiter."""

    output_rate_hz: float
    navigation_command_receipt_s: float
    navigation_startup_grace_s: float
    docking_command_receipt_s: float
    lidar_receipt_s: float
    lidar_header_s: float
    odometry_receipt_s: float
    odometry_header_s: float
    maximum_header_future_skew_s: float
    max_linear_x_mps: float
    max_linear_y_mps: float
    max_angular_z_radps: float
    zero_epsilon: float
    stop_linear_speed_mps: float
    stop_angular_speed_radps: float
    stop_dwell_s: float
    max_linear_speed_lead_mps: float
    max_angular_speed_lead_radps: float
    min_turn_radius_m: float
    pose_feedback: PoseIntervalConfig
    departure_permit_lifetime_s: float = 0.50
    travel_readiness_receipt_s: float = 0.25
    travel_readiness_header_s: float = 0.35
    command_header_s: float = 0.25

    def __post_init__(self):
        positive_fields = (
            "output_rate_hz",
            "navigation_command_receipt_s",
            "navigation_startup_grace_s",
            "docking_command_receipt_s",
            "lidar_receipt_s",
            "lidar_header_s",
            "odometry_receipt_s",
            "odometry_header_s",
            "max_linear_x_mps",
            "max_linear_y_mps",
            "max_angular_z_radps",
            "zero_epsilon",
            "stop_linear_speed_mps",
            "stop_angular_speed_radps",
            "stop_dwell_s",
            "max_linear_speed_lead_mps", "max_angular_speed_lead_radps", "min_turn_radius_m",
        )
        for field_name in positive_fields:
            converted = _finite(getattr(self, field_name), field_name)
            if converted <= 0.0:
                raise ValueError("{0} must be positive".format(field_name))
            object.__setattr__(self, field_name, converted)
        future_skew = _finite(
            self.maximum_header_future_skew_s,
            "maximum_header_future_skew_s",
        )
        if future_skew < 0.0:
            raise ValueError("maximum_header_future_skew_s must be nonnegative")
        object.__setattr__(self, "maximum_header_future_skew_s", future_skew)
        if not isinstance(self.pose_feedback, PoseIntervalConfig) or self.pose_feedback.command_frame != "base_link":
            raise ValueError("authority pose feedback must measure the base_link command origin")
        if self.min_turn_radius_m != 0.4764:
            raise ValueError("authority supports the active Ranger Mini V2 min_turn_radius_m=0.4764")
        if self.output_rate_hz != 20.0:
            raise ValueError("output_rate_hz must be exactly 20 Hz")
        for name, maximum in (("departure_permit_lifetime_s", 2.0),
                              ("travel_readiness_receipt_s", 5.0),
                              ("travel_readiness_header_s", 5.0),
                              ("command_header_s", 1.0)):
            value = _finite(getattr(self, name), name)
            if not 0.0 < value <= maximum:
                raise ValueError("{0} must be positive and at most {1}".format(name, maximum))
            object.__setattr__(self, name, value)

    @classmethod
    def from_mapping(cls, raw):
        departure = {}
        if isinstance(raw, Mapping) and "departure" in raw:
            raw = dict(raw)
            departure = _exact_mapping(raw.pop("departure"),
                ("departure_permit_lifetime_s", "travel_readiness_receipt_s", "travel_readiness_header_s"),
                "departure")
        root = _exact_mapping(
            raw,
            ("schema_version", "output_rate_hz", "freshness", "limits", "handover", "motion", "pose_feedback"),
            "authority",
        )
        if isinstance(root["schema_version"], bool) or root["schema_version"] != 3:
            raise ValueError("unsupported authority schema_version")
        freshness = _exact_mapping(
            root["freshness"],
            (
                "navigation_command_receipt_s",
                "navigation_startup_grace_s",
                "docking_command_receipt_s",
                "lidar_receipt_s",
                "lidar_header_s",
                "odometry_receipt_s",
                "odometry_header_s",
                "maximum_header_future_skew_s",
            ),
            "freshness",
        )
        limits = _exact_mapping(
            root["limits"],
            (
                "max_linear_x_mps",
                "max_linear_y_mps",
                "max_angular_z_radps",
                "zero_epsilon",
            ),
            "limits",
        )
        handover = _exact_mapping(
            root["handover"],
            (
                "stop_linear_speed_mps",
                "stop_angular_speed_radps",
                "stop_dwell_s",
            ),
            "handover",
        )
        motion = _exact_mapping(root["motion"], (
            "max_linear_speed_lead_mps", "max_angular_speed_lead_radps", "min_turn_radius_m"), "motion")
        pose = _exact_mapping(root["pose_feedback"], (
            "parent_frame", "child_frame", "command_frame", "min_interval_s", "max_interval_s",
            "max_translation_step_m", "max_rotation_step_rad", "max_linear_speed_mps",
            "max_angular_speed_radps", "quaternion_norm_tolerance",
            "extrinsic_translation_tolerance_m", "extrinsic_rotation_tolerance_rad"), "pose_feedback")
        return cls(
            pose_feedback=PoseIntervalConfig(**pose),
            **motion,
            output_rate_hz=root["output_rate_hz"],
            navigation_command_receipt_s=freshness[
                "navigation_command_receipt_s"
            ],
            navigation_startup_grace_s=freshness[
                "navigation_startup_grace_s"
            ],
            docking_command_receipt_s=freshness[
                "docking_command_receipt_s"
            ],
            lidar_receipt_s=freshness["lidar_receipt_s"],
            lidar_header_s=freshness["lidar_header_s"],
            odometry_receipt_s=freshness["odometry_receipt_s"],
            odometry_header_s=freshness["odometry_header_s"],
            maximum_header_future_skew_s=freshness[
                "maximum_header_future_skew_s"
            ],
            max_linear_x_mps=limits["max_linear_x_mps"],
            max_linear_y_mps=limits["max_linear_y_mps"],
            max_angular_z_radps=limits["max_angular_z_radps"],
            zero_epsilon=limits["zero_epsilon"],
            stop_linear_speed_mps=handover["stop_linear_speed_mps"],
            stop_angular_speed_radps=handover["stop_angular_speed_radps"],
            stop_dwell_s=handover["stop_dwell_s"],
            **departure,
        )


@dataclass(frozen=True)
class AuthorityDecision:
    accepted: bool
    state: BaseAuthority
    reason: str


@dataclass(frozen=True)
class CommandGrantDecision(AuthorityDecision):
    session_id: str = ""
    grant_id: str = ""
    source_goal_id: str = ""


@dataclass
class _CommandGrant:
    source: BaseAuthority
    grant_id: str
    source_goal_id: str
    sequence: int = 0
    header_stamp_s: float = 0.0


@dataclass(frozen=True)
class AuthorityOutput:
    command: Velocity2D
    state: BaseAuthority
    selected_source: str
    reason: str


@dataclass(frozen=True)
class TravelReadiness:
    """Onboard guard observations; receipt time is supplied locally, never over ROS."""
    header_stamp_s: float
    run_id: str
    departure_id: str
    session_id: str
    hold_epoch: int
    sequence: int
    state_revision: int
    config_id: str
    travel_profile_id: str
    arm_ready: bool
    lift_ready: bool
    producers_stopped: bool
    clearance_verified: bool
    payload_state: int  # 0 unknown, 1 measured empty, 2 measured holding.
    payload_ref: str
    container_profile_id: str
    ready: bool
    reason: str

    def binding(self):
        """Heartbeats may advance sequence/stamp, never change the permitted state."""
        return (self.run_id, self.departure_id, self.session_id, self.hold_epoch,
                self.state_revision, self.config_id, self.travel_profile_id,
                self.arm_ready, self.lift_ready, self.producers_stopped,
                self.clearance_verified, self.payload_state, self.payload_ref,
                self.container_profile_id, self.ready)


@dataclass(frozen=True)
class GuardedDecision:
    accepted: bool
    reason: str
    session_id: str
    hold_epoch: int
    stopped_confirmed: bool = False
    permit_id: str = ""
    valid_for_s: float = 0.0


@dataclass
class _DeparturePermit:
    permit_id: str
    request: tuple
    readiness_binding: tuple
    expires_s: float
    invalid_reason: str = ""
    consumed: bool = False


def _identity(value):
    return isinstance(value, str) and bool(value.strip())


def _counter(value, minimum=0):
    return isinstance(value, Integral) and not isinstance(value, bool) and minimum <= value < 2**64


@dataclass(frozen=True)
class _CommandEvidence:
    command: object
    receipt_s: float


@dataclass(frozen=True)
class _LidarEvidence:
    health: object
    blocked_directions: object
    receipt_s: float
    header_stamp_s: float
    forward_clearance_m: float
    reverse_clearance_m: float
    left_clearance_m: float
    right_clearance_m: float
    rotation_clearance_m: float
    required_stop_distance_m: float
    reason: str
    validation_reason: Optional[str] = None


@dataclass(frozen=True)
class _OdometryEvidence:
    linear_speed_mps: float
    angular_speed_radps: float
    receipt_s: float
    header_stamp_s: float
    validation_reason: Optional[str] = None


def command_direction_mask(command, *, zero_epsilon):
    """Return the LiDAR direction mask applicable to a velocity command."""

    if not isinstance(command, Velocity2D):
        raise ValueError("command must be a Velocity2D")
    epsilon = _finite(zero_epsilon, "zero_epsilon")
    if epsilon <= 0.0:
        raise ValueError("zero_epsilon must be positive")
    mask = MotionDirection.NONE
    if command.linear_x > epsilon:
        mask |= MotionDirection.FORWARD
    elif command.linear_x < -epsilon:
        mask |= MotionDirection.REVERSE
    if command.linear_y > epsilon:
        mask |= MotionDirection.LEFT
    elif command.linear_y < -epsilon:
        mask |= MotionDirection.RIGHT
    if abs(command.angular_z) > epsilon:
        mask |= MotionDirection.ROTATION
    return mask


class AuthorityArbiter:
    """Select one autonomous command source and apply final safety gating."""

    def __init__(self, config, *, fenced_protocol=False, navigation_sweep_guard=None):
        if not isinstance(config, AuthorityConfig):
            raise ValueError("config must be an AuthorityConfig")
        self.config = config
        if navigation_sweep_guard is not None and not callable(navigation_sweep_guard):
            raise ValueError("navigation sweep guard must be callable")
        self._navigation_sweep_guard = navigation_sweep_guard
        self._state = BaseAuthority.NONE
        self._selected = None
        self._reason = "none"
        self._commands = {}
        self._lidar = None
        self._odometry = None
        self._last_receipt_s = None
        self._transition_zero_pending = False
        self._handover_target = None
        self._stopped_since_s = None
        self._fault_reason = None
        self._fault_source = None
        self._fault_direction = MotionDirection.NONE
        self._navigation_granted_s = None
        self._shutdown = False
        self._lidar_header_high_water_s = None
        self._odometry_header_high_water_s = None
        self._manipulation_hold_active = False
        self._hold_zero_since_s = None
        self._session_id = uuid.uuid4().hex
        self._hold_epoch = 0
        self._hold_requests = {}
        self._departure_requests = {}
        self._active_permit = None
        self._readiness = None
        self._readiness_receipt_s = None
        self._readiness_problem_reason = "travel_readiness_missing"
        self._readiness_high_water = {}
        self._readiness_history = deque(maxlen=128)
        self._travel_lease = None
        self._departure_required = False
        self._command_grant = None
        self._used_source_goals = set()
        self._command_headers = {}
        self._command_protocol_active = fenced_protocol
        self._last_output_command = None
        self._last_output_receipt_s = None
        self._motion_limiter = FinalCommandLimiter(config)
        self._measured_motion = None
        self._motion_revision = 0
        self._limiter_inhibit_receipt_s = None
        self._limiter_inhibit_header_s = None
        self._output_header_now_s = None

    @property
    def state(self):
        return self._state

    @property
    def fault_reason(self):
        return self._fault_reason

    @property
    def manipulation_hold_active(self):
        return self._manipulation_hold_active

    @property
    def session_id(self):
        return self._session_id

    @property
    def hold_epoch(self):
        return self._hold_epoch

    def _guarded(self, accepted, reason, **fields):
        return GuardedDecision(accepted, reason, self.session_id, self.hold_epoch, **fields)

    def _invalidate_departures(self, reason):
        """One monotonic generation covers all hold/stop/fault assertions."""
        self._hold_epoch += 1
        if self._active_permit is not None:
            self._active_permit.invalid_reason = reason
        self._active_permit = None
        self._travel_lease = None
        self._command_grant = None
        self._command_headers.clear()
        self._readiness_history.clear()

    @property
    def command_grant_id(self):
        return self._command_grant.grant_id if self._command_grant else ""

    @property
    def source_goal_id(self):
        return self._command_grant.source_goal_id if self._command_grant else ""

    def _grant_decision(self, accepted, reason):
        return CommandGrantDecision(accepted, self._state, reason, self.session_id,
                                    self.command_grant_id, self.source_goal_id)

    def request_grant(self, requested_state, expected_session_id, source_goal_id,
                      *, receipt_now_s, header_now_s):
        """ROS command admission. A grant belongs to exactly one motion operation.

        The lower-level request/update_command methods implement the original
        safety state machine; transport callers must use this fenced boundary.
        """
        self._command_protocol_active = True
        if requested_state == BaseAuthority.NONE and not isinstance(requested_state, bool):
            result = self.request(requested_state, receipt_now_s=receipt_now_s,
                                  header_now_s=header_now_s)
            return self._grant_decision(result.accepted, result.reason)
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
            if isinstance(requested_state, bool):
                raise ValueError("invalid source")
            source = self._moving_authority(requested_state)
        except ValueError:
            return self._grant_decision(False, "authority_request_invalid")
        if expected_session_id != self.session_id or not _identity(source_goal_id):
            return self._grant_decision(False, "command_grant_identity_invalid")
        if self._shutdown or self._state == BaseAuthority.FAULT or self._manipulation_hold_active:
            return self._grant_decision(False, self._fault_reason or
                ("shutdown" if self._shutdown else "manipulation_hold_active"))
        current = self._command_grant
        handover = (
            current is not None
            and current.source != source
            and (current.source, source) in (
                (BaseAuthority.NAVIGATION, BaseAuthority.DOCKING),
                (BaseAuthority.DOCKING, BaseAuthority.NAVIGATION),
            )
            and self._state == current.source
        )
        if current is not None:
            if (current.source, current.source_goal_id) == (source, source_goal_id):
                return self._grant_decision(True, self._reason)
            if not handover or not self._stored_command_is_zero(current.source):
                return self._grant_decision(False, "source_transition_requires_stop")
        if source_goal_id in self._used_source_goals:
            return self._grant_decision(False, "source_goal_reused")
        if self._state != BaseAuthority.NONE and not handover:
            return self._grant_decision(False, "source_transition_requires_stop")
        problem = self._lidar_problem(receipt, header_now, MotionDirection.NONE)
        problem = problem or self._odometry_problem(receipt, header_now)
        if problem is None and not self._odometry_stopped_for_dwell(receipt):
            problem = "odometry_not_settled"
        # Every production motion operation, including the first after boot,
        # needs a measured preparation/release. There is no standalone bypass.
        problem = problem or ("departure_release_required" if self._travel_lease is None
            else self._readiness_problem(receipt, header_now, self._travel_lease))
        if problem:
            return self._grant_decision(False, problem)
        self._commands.clear()
        self._command_headers.clear()
        self._command_grant = _CommandGrant(source, uuid.uuid4().hex, source_goal_id)
        self._used_source_goals.add(source_goal_id)
        if source == BaseAuthority.NAVIGATION:
            result = self.request(source, receipt_now_s=receipt, header_now_s=header_now)
            return self._grant_decision(result.accepted, result.reason)
        # The producer now has a grant with which to supply its zero heartbeat.
        # Actual docking selection still requires the existing measured handover.
        self._state = BaseAuthority.STOPPING_FOR_HANDOVER
        self._selected = None
        self._handover_target = source
        self._reason = "stopping_for_handover"
        self._transition_zero_pending = True
        self._stopped_since_s = None
        return self._grant_decision(True, self._reason)

    def update_authorized_command(self, source, command, *, session_id, grant_id,
                                  source_goal_id, sequence, header_stamp_s,
                                  receipt_now_s, header_now_s):
        """Accept original producer identity/time once; rejected packets refresh nothing."""
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
            stamp = _finite(header_stamp_s, "command_header_stamp_s")
            source = self._moving_authority(source)
        except ValueError:
            return AuthorityDecision(False, self._state, "command_envelope_invalid")
        grant = self._command_grant
        if (grant is None or self._shutdown or session_id != self.session_id or
                (source, grant_id, source_goal_id) !=
                (grant.source, grant.grant_id, grant.source_goal_id)):
            return AuthorityDecision(False, self._state, "command_grant_mismatch")
        if not _counter(sequence, 1) or sequence <= grant.sequence:
            return AuthorityDecision(False, self._state, "command_sequence_replayed")
        maximum_age = self.config.command_header_s
        if (stamp <= 0.0 or header_now <= 0.0 or stamp < grant.header_stamp_s or
                not -self.config.maximum_header_future_skew_s <= header_now - stamp <= maximum_age):
            return AuthorityDecision(False, self._state, "command_header_invalid")
        if not isinstance(command, Velocity2D):
            self._latch_fault("command_invalid", source)
            return AuthorityDecision(False, self._state, "command_invalid")
        grant.sequence = sequence
        grant.header_stamp_s = stamp
        self.update_command(source, command, receipt_now_s=receipt)
        self._command_headers[source] = stamp
        return AuthorityDecision(True, self._state, "command_accepted")

    def _confirm_hold(self, receipt, header_now):
        self.output(receipt_now_s=receipt, header_now_s=header_now)
        if self._shutdown or self._state == BaseAuthority.FAULT:
            return self._fault_reason or "shutdown"
        problem = self._odometry_problem(receipt, header_now)
        if problem is None and not self._odometry_stopped_for_dwell(receipt):
            problem = "odometry_not_settled"
        if problem is None and (self._hold_zero_since_s is None or
                receipt - self._hold_zero_since_s < self.config.stop_dwell_s):
            problem = "zero_output_not_settled"
        return problem

    def _assert_hold(self):
        if not self._manipulation_hold_active:
            self._manipulation_hold_active = True
            self._hold_zero_since_s = None
            if self._state != BaseAuthority.FAULT:
                self._enter_state(BaseAuthority.NONE, None, "manipulation_hold_active")

    def acquire_hold(self, run_id, operation_id, reason, *, receipt_now_s, header_now_s):
        """Idempotent typed assertion; accepted is distinct from measured stopping."""
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
        except ValueError:
            self._latch_fault("authority_time_invalid")
            return self._guarded(False, "authority_time_invalid")
        if self._shutdown:
            return self._guarded(False, "shutdown")
        if not all(_identity(value) for value in (run_id, operation_id, reason)):
            return self._guarded(False, "hold_request_invalid")
        key = (run_id, operation_id)
        previous = self._hold_requests.get(key)
        if previous is not None:
            if previous != (self.hold_epoch, reason) or not self._manipulation_hold_active:
                return self._guarded(False, "hold_operation_obsolete_or_changed")
        else:
            self._invalidate_departures("new_hold")
            self._assert_hold()
            self._hold_requests[key] = (self.hold_epoch, reason)
        problem = self._confirm_hold(receipt, header_now)
        return self._guarded(True, problem or "manipulation_hold_confirmed",
                             stopped_confirmed=problem is None)

    def hold_for_manipulation(self, *, receipt_now_s, header_now_s):
        """Legacy stop-only assertion; every call revokes departure intent.

        Repeated assertions preserve an established zero-output/stopped dwell.
        This endpoint never releases a hold or clears a fault.
        """
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
        except ValueError:
            self._latch_fault("authority_time_invalid")
            return AuthorityDecision(False, self._state, "authority_time_invalid")
        if self._shutdown:
            return AuthorityDecision(False, self._state, "shutdown")
        self._invalidate_departures("legacy_hold")
        self._assert_hold()
        problem = self._confirm_hold(receipt, header_now)
        return AuthorityDecision(problem is None, self._state,
                                 problem or "manipulation_hold_confirmed")

    def update_travel_readiness(self, readiness, *, receipt_now_s, header_now_s):
        if self._shutdown:
            return
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
        except ValueError:
            self._latch_fault("authority_time_invalid")
            return
        problem = self._validate_readiness(readiness, header_now)
        self._readiness = readiness if isinstance(readiness, TravelReadiness) else None
        self._readiness_receipt_s = receipt
        self._readiness_problem_reason = problem
        binding = readiness.binding() if isinstance(readiness, TravelReadiness) else None
        if problem or (self._readiness_history and self._readiness_history[-1][1] != binding):
            self._readiness_history.clear()
        if not problem:
            self._readiness_history.append((readiness.sequence, binding, receipt, readiness.header_stamp_s))
        if self._active_permit is not None and (problem or binding != self._active_permit.readiness_binding):
            self._active_permit.invalid_reason = problem or "travel_readiness_changed"
            self._active_permit = None
        if self._travel_lease is not None and (problem or binding != self._travel_lease):
            self._latch_fault(problem or "travel_readiness_changed")

    def _validate_readiness(self, readiness, header_now):
        if not isinstance(readiness, TravelReadiness):
            return "travel_readiness_invalid"
        if not all(_identity(value) for value in (readiness.run_id, readiness.departure_id,
                readiness.session_id, readiness.config_id, readiness.travel_profile_id)):
            return "travel_readiness_identity_invalid"
        if not (_counter(readiness.hold_epoch) and _counter(readiness.sequence, 1) and
                _counter(readiness.state_revision)):
            return "travel_readiness_counter_invalid"
        try:
            stamp = _finite(readiness.header_stamp_s, "readiness_header")
        except ValueError:
            return "travel_readiness_header_invalid"
        if header_now <= 0.0 or stamp <= 0.0 or stamp - header_now > self.config.maximum_header_future_skew_s:
            return "travel_readiness_header_invalid"
        if header_now - stamp > self.config.travel_readiness_header_s:
            return "travel_readiness_header_stale"
        if readiness.session_id != self.session_id or readiness.hold_epoch != self.hold_epoch:
            return "travel_readiness_generation_mismatch"
        stream = (readiness.run_id, readiness.departure_id, readiness.hold_epoch)
        previous = self._readiness_high_water.get(stream)
        if previous is not None and (readiness.sequence <= previous[0] or stamp <= previous[1]):
            return "travel_readiness_nonmonotonic"
        self._readiness_high_water[stream] = (readiness.sequence, stamp)
        if not all(value is True for value in (readiness.arm_ready, readiness.lift_ready,
                readiness.producers_stopped, readiness.clearance_verified, readiness.ready)):
            return "travel_readiness_not_ready"
        if not _counter(readiness.payload_state) or readiness.payload_state not in (1, 2):
            return "travel_readiness_payload_unknown"
        if readiness.payload_state == 1:
            if readiness.payload_ref != "" or readiness.container_profile_id != "":
                return "travel_readiness_payload_inconsistent"
        elif not _identity(readiness.payload_ref) or not _identity(readiness.container_profile_id):
            return "travel_readiness_payload_inconsistent"
        return None

    def _readiness_problem(self, receipt, header_now, binding=None):
        if self._readiness is None:
            return "travel_readiness_missing"
        if self._readiness_problem_reason:
            return self._readiness_problem_reason
        if not 0.0 <= receipt - self._readiness_receipt_s <= self.config.travel_readiness_receipt_s:
            return "travel_readiness_receipt_stale"
        age = header_now - self._readiness.header_stamp_s
        if header_now <= 0.0 or not -self.config.maximum_header_future_skew_s <= age <= self.config.travel_readiness_header_s:
            return "travel_readiness_header_stale"
        if self._readiness.session_id != self.session_id or self._readiness.hold_epoch != self.hold_epoch:
            return "travel_readiness_generation_mismatch"
        if binding is not None and self._readiness.binding() != binding:
            return "travel_readiness_changed"
        return None

    def _departure_problem(self, session, epoch, receipt, header_now):
        if self._shutdown:
            return "shutdown"
        if self._state == BaseAuthority.FAULT:
            return self._fault_reason
        if session != self.session_id or not _counter(epoch) or epoch != self.hold_epoch:
            return "departure_generation_mismatch"
        if not self._manipulation_hold_active or self._state != BaseAuthority.NONE:
            return "departure_requires_current_hold"
        return self._confirm_hold(receipt, header_now) or self._lidar_problem(receipt, header_now, MotionDirection.NONE)

    def prepare_departure(self, run_id, departure_id, expected_session_id, expected_hold_epoch,
                          readiness_sequence, state_revision, *, receipt_now_s, header_now_s):
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
        except ValueError:
            self._latch_fault("authority_time_invalid")
            return self._guarded(False, "authority_time_invalid")
        if not (_identity(run_id) and _identity(departure_id) and _counter(readiness_sequence, 1) and _counter(state_revision)):
            return self._guarded(False, "departure_request_invalid")
        request = (run_id, departure_id, expected_session_id, expected_hold_epoch, readiness_sequence, state_revision)
        previous = self._departure_requests.get((run_id, departure_id))
        if previous is not None:
            problem = self._permit_problem(previous, receipt)
            if previous.request != request:
                problem = "departure_id_reused"
            if problem:
                return self._guarded(False, problem)
        problem = self._departure_problem(expected_session_id, expected_hold_epoch, receipt, header_now)
        problem = problem or self._readiness_problem(receipt, header_now,
                         previous.readiness_binding if previous else None)
        if problem:
            return self._guarded(False, problem)
        readiness = self._readiness
        if (readiness.run_id, readiness.departure_id, readiness.state_revision) != (run_id, departure_id, state_revision):
            return self._guarded(False, "departure_readiness_mismatch")
        if previous is None:
            # The producer may publish its next identical heartbeat while the
            # prepare RPC is in flight. Accept only a recently OBSERVED sequence
            # from this uninterrupted full semantic binding, never arbitrary <=.
            observed = any(sequence == readiness_sequence and binding == readiness.binding() and
                0.0 <= receipt - received <= self.config.travel_readiness_receipt_s and
                -self.config.maximum_header_future_skew_s <= header_now - stamp <= self.config.travel_readiness_header_s
                for sequence, binding, received, stamp in self._readiness_history)
            if not observed:
                return self._guarded(False, "departure_readiness_mismatch")
        if previous is None:
            if self._active_permit is not None:
                return self._guarded(False, "departure_already_prepared")
            previous = _DeparturePermit(uuid.uuid4().hex, request, readiness.binding(),
                                        receipt + self.config.departure_permit_lifetime_s)
            self._departure_requests[(run_id, departure_id)] = previous
            self._active_permit = previous
        return self._guarded(True, "departure_prepared", permit_id=previous.permit_id,
                             valid_for_s=max(0.0, previous.expires_s-receipt))

    @staticmethod
    def _permit_problem(permit, receipt):
        if permit.consumed:
            return "departure_permit_consumed"
        if receipt >= permit.expires_s:
            return "departure_permit_expired"
        return permit.invalid_reason or None

    def release_hold(self, run_id, departure_id, expected_session_id, expected_hold_epoch,
                     permit_id, *, receipt_now_s, header_now_s):
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
        except ValueError:
            self._latch_fault("authority_time_invalid")
            return self._guarded(False, "authority_time_invalid")
        permit = self._departure_requests.get((run_id, departure_id))
        if permit is None or permit.permit_id != permit_id:
            return self._guarded(False, "departure_permit_unknown")
        problem = self._permit_problem(permit, receipt)
        problem = problem or self._departure_problem(expected_session_id, expected_hold_epoch, receipt, header_now)
        problem = problem or self._readiness_problem(receipt, header_now, permit.readiness_binding)
        if permit.request[:4] != (run_id, departure_id, expected_session_id, expected_hold_epoch):
            problem = "departure_generation_mismatch"
        if problem:
            return self._guarded(False, problem)
        permit.consumed = True
        self._active_permit = None
        self._travel_lease = permit.readiness_binding
        self._departure_required = True
        self._manipulation_hold_active = False
        self._hold_zero_since_s = None
        self._enter_state(BaseAuthority.NONE, None, "departure_hold_released")
        return self._guarded(True, "departure_hold_released")

    def reset_fault(self, expected_session_id, expected_hold_epoch, reason, *, receipt_now_s, header_now_s):
        """Explicit operator/qualification reset, never called by normal cleanup."""
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
        except ValueError:
            return AuthorityDecision(False, self._state, "authority_time_invalid")
        if self._shutdown or expected_session_id != self.session_id or not _counter(expected_hold_epoch) or expected_hold_epoch != self.hold_epoch or not _identity(reason):
            return AuthorityDecision(False, self._state, "fault_reset_identity_invalid")
        if self._state != BaseAuthority.FAULT:
            return AuthorityDecision(False, self._state, "fault_reset_not_faulted")
        problem = self._fault_reset_problem(receipt, header_now)
        if problem:
            return AuthorityDecision(False, self._state, problem)
        self._invalidate_departures("fault_reset")
        self._enter_state(BaseAuthority.NONE, None, "fault_reset")
        self._clear_fault()
        return AuthorityDecision(True, self._state, "fault_reset")

    def update_command(self, source, command, *, receipt_now_s):
        if self._shutdown:
            return
        # A faulted held base can collect a fresh zero heartbeat for explicit
        # measured reset. It cannot queue a nonzero command for later release.
        if self._manipulation_hold_active and not (
                self._state == BaseAuthority.FAULT and command == _ZERO):
            return
        try:
            receipt = self._receipt_time(receipt_now_s)
            source_value = self._moving_authority(source)
        except ValueError:
            self._latch_fault("authority_command_update_invalid")
            return
        self._commands[source_value] = _CommandEvidence(command, receipt)

    def update_lidar(
        self,
        *,
        health,
        blocked_directions,
        receipt_now_s,
        header_stamp_s,
        header_now_s,
        forward_clearance_m,
        reverse_clearance_m,
        left_clearance_m,
        right_clearance_m,
        rotation_clearance_m,
        required_stop_distance_m,
        reason,
    ):
        if self._shutdown:
            return
        try:
            receipt = self._receipt_time(receipt_now_s)
        except ValueError:
            self._latch_fault("authority_time_invalid")
            return
        validation_reason = None
        try:
            header_stamp = _finite(header_stamp_s, "lidar_header_stamp_s")
            header_now = _finite(header_now_s, "lidar_header_now_s")
            if header_stamp <= 0.0 or header_now <= 0.0:
                validation_reason = "lidar_header_uninitialized"
            elif (
                self._lidar_header_high_water_s is not None
                and header_stamp <= self._lidar_header_high_water_s
            ):
                validation_reason = "lidar_header_nonmonotonic"
            elif (
                header_stamp - header_now
                > self.config.maximum_header_future_skew_s
            ):
                validation_reason = "lidar_header_future"
            else:
                self._lidar_header_high_water_s = header_stamp
        except ValueError:
            header_stamp = 0.0
            validation_reason = "lidar_header_invalid"
        try:
            clearances = tuple(
                _finite(value, name)
                for value, name in (
                    (forward_clearance_m, "forward_clearance_m"),
                    (reverse_clearance_m, "reverse_clearance_m"),
                    (left_clearance_m, "left_clearance_m"),
                    (right_clearance_m, "right_clearance_m"),
                    (rotation_clearance_m, "rotation_clearance_m"),
                    (required_stop_distance_m, "required_stop_distance_m"),
                )
            )
            if any(value < 0.0 for value in clearances):
                raise ValueError("LiDAR clearances must be nonnegative")
        except ValueError:
            clearances = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            validation_reason = validation_reason or "lidar_clearance_invalid"
        try:
            health_value = self._safety_health(health)
        except ValueError:
            health_value = SafetyHealth.INVALID
            validation_reason = validation_reason or "lidar_state_invalid"
        try:
            direction_value = self._direction_mask(blocked_directions)
        except ValueError:
            direction_value = MotionDirection.ALL
            validation_reason = validation_reason or "lidar_direction_mask_invalid"
        if (
            health_value == SafetyHealth.CLEAR
            and direction_value != MotionDirection.NONE
        ) or (
            health_value == SafetyHealth.BLOCKED
            and direction_value == MotionDirection.NONE
        ):
            validation_reason = (
                validation_reason or "lidar_state_mask_inconsistent"
            )
        clearance_mask = MotionDirection.NONE
        for clearance, direction in zip(
            clearances[:4],
            (
                MotionDirection.FORWARD,
                MotionDirection.REVERSE,
                MotionDirection.LEFT,
                MotionDirection.RIGHT,
            ),
        ):
            if clearance <= clearances[5]:
                clearance_mask |= direction
        translational_mask = (
            MotionDirection.FORWARD
            | MotionDirection.REVERSE
            | MotionDirection.LEFT
            | MotionDirection.RIGHT
        )
        if (
            health_value == SafetyHealth.CLEAR
            and clearance_mask != MotionDirection.NONE
        ) or (
            health_value == SafetyHealth.BLOCKED
            and direction_value & translational_mask != clearance_mask
        ) or (
            health_value == SafetyHealth.BLOCKED
            and direction_value & MotionDirection.ROTATION
            and clearances[4] > clearances[5]
        ) or (
            health_value == SafetyHealth.BLOCKED
            and not direction_value & MotionDirection.ROTATION
            and clearances[4] <= self.config.zero_epsilon
        ):
            validation_reason = (
                validation_reason or "lidar_clearance_inconsistent"
            )
        self._lidar = _LidarEvidence(
            health=health_value,
            blocked_directions=direction_value,
            receipt_s=receipt,
            header_stamp_s=header_stamp,
            forward_clearance_m=clearances[0],
            reverse_clearance_m=clearances[1],
            left_clearance_m=clearances[2],
            right_clearance_m=clearances[3],
            rotation_clearance_m=clearances[4],
            required_stop_distance_m=clearances[5],
            reason=str(reason),
            validation_reason=validation_reason,
        )

    def update_measured_motion(self, motion, *, sample_receipt_s, receipt_now_s,
                               header_stamp_s, header_now_s, invalid_reason=""):
        valid = isinstance(motion, Velocity2D)
        self.update_odometry(
            linear_speed_mps=math.hypot(motion.linear_x, motion.linear_y) if valid else math.nan,
            angular_speed_radps=abs(motion.angular_z) if valid else math.nan,
            receipt_now_s=receipt_now_s, header_stamp_s=header_stamp_s,
            header_now_s=header_now_s, sample_receipt_s=sample_receipt_s,
            invalid_reason="odometry_pose_"+invalid_reason if not valid and invalid_reason else None)
        if valid and self._odometry and self._odometry.validation_reason is None:
            self._measured_motion = motion

    def motion_feedback(self, *, receipt_now_s, header_now_s):
        """Read-only original-observation view; publication creates no evidence."""
        evidence = self._odometry
        stamp = evidence.header_stamp_s if evidence is not None else 0.0
        try:
            now = _finite(receipt_now_s, 'motion receipt clock')
            source_now = _finite(header_now_s, 'motion source clock')
            age = now - evidence.receipt_s if evidence is not None else 0.0
            reason = self._odometry_problem(now, source_now)
            if reason is None and self._measured_motion is None:
                reason = 'motion_unavailable'
        except ValueError:
            age, reason = 0.0, 'motion_clock_invalid'
        return MotionFeedback(reason is None, self._motion_revision, stamp,
                              max(0.0, age), self._measured_motion if reason is None else _ZERO,
                              reason or 'ok')

    def update_odometry(
        self,
        *,
        linear_speed_mps,
        angular_speed_radps,
        receipt_now_s,
        header_stamp_s,
        header_now_s,
        sample_receipt_s=None,
        invalid_reason=None,
    ):
        self._measured_motion = None
        if self._shutdown:
            return
        self._motion_revision += 1
        try:
            receipt = self._receipt_time(receipt_now_s)
        except ValueError:
            self._latch_fault("authority_time_invalid")
            return
        validation_reason = invalid_reason
        operation_receipt = receipt
        try:
            if sample_receipt_s is not None:
                receipt = _finite(sample_receipt_s, "odometry_sample_receipt_s")
                if receipt < 0 or receipt > operation_receipt:
                    validation_reason = "odometry_receipt_invalid"
            linear = abs(_finite(linear_speed_mps, "linear_speed_mps"))
            angular = abs(_finite(angular_speed_radps, "angular_speed_radps"))
            header_stamp = _finite(header_stamp_s, "odometry_header_stamp_s")
            header_now = _finite(header_now_s, "odometry_header_now_s")
            if header_stamp <= 0.0 or header_now <= 0.0:
                validation_reason = "odometry_header_uninitialized"
            elif (
                self._odometry_header_high_water_s is not None
                and header_stamp <= self._odometry_header_high_water_s
            ):
                validation_reason = "odometry_header_nonmonotonic"
            elif (
                header_stamp - header_now
                > self.config.maximum_header_future_skew_s
            ):
                validation_reason = "odometry_header_future"
            else:
                self._odometry_header_high_water_s = header_stamp
        except ValueError:
            linear = math.inf
            angular = math.inf
            header_stamp = 0.0
            validation_reason = validation_reason or "odometry_invalid"
        previous = self._odometry
        if (
            previous is not None
            and receipt - previous.receipt_s
            > self.config.odometry_receipt_s
        ):
            self._stopped_since_s = None
        self._odometry = _OdometryEvidence(
            linear_speed_mps=linear,
            angular_speed_radps=angular,
            receipt_s=receipt,
            header_stamp_s=header_stamp,
            validation_reason=validation_reason,
        )
        if (validation_reason is not None or not self._speeds_stopped(linear, angular)
                or not self._odometry_after_limiter_stop(self._odometry)):
            self._stopped_since_s = None
        elif self._stopped_since_s is None:
            self._stopped_since_s = receipt

    def request(self, requested_state, *, receipt_now_s, header_now_s):
        if self._shutdown:
            return AuthorityDecision(False, self._state, "shutdown")
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
        except ValueError:
            return AuthorityDecision(False, self._state, "authority_time_invalid")

        if self._manipulation_hold_active and requested_state in _MOVING_AUTHORITIES:
            return AuthorityDecision(False, self._state, "manipulation_hold_active")

        if self._state == BaseAuthority.FAULT:
            if not isinstance(requested_state, bool) and requested_state == BaseAuthority.NONE:
                self._invalidate_departures("stop_while_faulted")
            return AuthorityDecision(False, self._state, self._fault_reason)

        try:
            if isinstance(requested_state, bool):
                raise ValueError("boolean authority is invalid")
            requested = BaseAuthority(requested_state)
        except (TypeError, ValueError):
            return AuthorityDecision(
                False,
                self._state,
                "authority_request_invalid",
            )
        if requested not in (
            BaseAuthority.NONE,
            BaseAuthority.NAVIGATION,
            BaseAuthority.DOCKING,
        ):
            return AuthorityDecision(
                False,
                self._state,
                "authority_request_invalid",
            )

        if requested == BaseAuthority.NONE:
            self._invalidate_departures("authority_stop")
            self._enter_state(BaseAuthority.NONE, None, "released")
            return AuthorityDecision(True, self._state, self._reason)

        if self._departure_required:
            problem = ("departure_release_required" if self._travel_lease is None else
                       self._readiness_problem(receipt, header_now, self._travel_lease))
            if problem:
                return AuthorityDecision(False, self._state, problem)

        if self._state == BaseAuthority.STOPPING_FOR_HANDOVER:
            if requested == self._handover_target:
                return AuthorityDecision(True, self._state, self._reason)
            return AuthorityDecision(
                False,
                self._state,
                "handover_in_progress",
            )

        lidar_problem = self._lidar_problem(
            receipt,
            header_now,
            MotionDirection.NONE,
        )
        if lidar_problem is not None:
            return AuthorityDecision(False, self._state, lidar_problem)

        if requested == BaseAuthority.DOCKING:
            command_problem, command, _ = self._command_problem(
                BaseAuthority.DOCKING,
                receipt,
                header_now,
            )
            if command_problem is not None:
                return AuthorityDecision(False, self._state, command_problem)
            mask = command_direction_mask(
                command,
                zero_epsilon=self.config.zero_epsilon,
            )
            lidar_problem = self._lidar_problem(receipt, header_now, mask)
            if lidar_problem is not None:
                return AuthorityDecision(False, self._state, lidar_problem)
            if self._state == BaseAuthority.DOCKING:
                return AuthorityDecision(True, self._state, self._reason)
            if self._state == BaseAuthority.NAVIGATION:
                self._state = BaseAuthority.STOPPING_FOR_HANDOVER
                self._selected = None
                self._handover_target = BaseAuthority.DOCKING
                self._reason = "stopping_for_handover"
                self._transition_zero_pending = True
                self._stopped_since_s = None
                return AuthorityDecision(True, self._state, self._reason)
            odom_problem = self._odometry_problem(receipt, header_now)
            if odom_problem is not None:
                return AuthorityDecision(False, self._state, odom_problem)
            if not self._odometry_stopped_for_dwell(receipt):
                return AuthorityDecision(
                    False,
                    self._state,
                    "odometry_not_settled",
                )
            self._enter_state(
                BaseAuthority.DOCKING,
                BaseAuthority.DOCKING,
                "docking_granted",
            )
            return AuthorityDecision(True, self._state, self._reason)

        if self._state == BaseAuthority.DOCKING:
            self._state = BaseAuthority.STOPPING_FOR_HANDOVER
            self._selected = None
            self._handover_target = BaseAuthority.NAVIGATION
            self._reason = "stopping_for_handover"
            self._transition_zero_pending = True
            self._stopped_since_s = None
            return AuthorityDecision(True, self._state, self._reason)
        already_navigation = self._state == BaseAuthority.NAVIGATION
        self._enter_state(
            BaseAuthority.NAVIGATION,
            BaseAuthority.NAVIGATION,
            "navigation_granted",
        )
        if not already_navigation:
            self._navigation_granted_s = receipt
        return AuthorityDecision(True, self._state, self._reason)

    def output(self, *, receipt_now_s, header_now_s):
        if self._shutdown:
            return self._current_output(_ZERO, "shutdown")
        try:
            receipt = self._receipt_time(receipt_now_s)
            header_now = _finite(header_now_s, "header_now_s")
            self._output_header_now_s = header_now
        except ValueError:
            return self._latch_fault("authority_time_invalid")

        if self._manipulation_hold_active:
            if self._hold_zero_since_s is None:
                self._hold_zero_since_s = receipt
            return self._current_output(_ZERO, self._fault_reason or "manipulation_hold_active")

        if self._state == BaseAuthority.FAULT:
            return self._current_output(_ZERO, self._fault_reason)
        if self._travel_lease is not None:
            problem = self._readiness_problem(receipt, header_now, self._travel_lease)
            if problem:
                return self._latch_fault(problem)
        if self._state == BaseAuthority.NONE:
            self._transition_zero_pending = False
            return self._current_output(_ZERO, self._reason)
        if self._state == BaseAuthority.STOPPING_FOR_HANDOVER:
            return self._handover_output(receipt, header_now)

        source = self._selected
        command_problem, command, mask = self._command_problem(source, receipt, header_now)
        if (
            source == BaseAuthority.NAVIGATION
            and command_problem == "navigation_command_missing"
            and self._navigation_granted_s is not None
            and receipt - self._navigation_granted_s
            <= self.config.navigation_startup_grace_s
        ):
            lidar_problem = self._lidar_problem(
                receipt,
                header_now,
                MotionDirection.NONE,
            )
            if lidar_problem is not None:
                return self._latch_fault(
                    lidar_problem,
                    source=source,
                    direction=MotionDirection.NONE,
                )
            if self._transition_zero_pending:
                self._transition_zero_pending = False
                return self._current_output(_ZERO, "transition_zero")
            return self._current_output(_ZERO, "navigation_command_waiting")
        if (
            source == BaseAuthority.NAVIGATION
            and command_problem in ("navigation_command_stale", "navigation_command_header_stale")
            and self._stored_command_is_zero(source)
        ):
            # move_base deliberately publishes one zero before costmap clear
            # and recovery transitions, then may remain silent for longer than
            # the nonzero command lease.  Preserve that safe zero without
            # authorizing replay or falling back to another source.  Any stale
            # nonzero command still latches a fault below.
            lidar_problem = self._lidar_problem(
                receipt,
                header_now,
                MotionDirection.NONE,
            )
            if lidar_problem is not None:
                return self._latch_fault(
                    lidar_problem,
                    source=source,
                    direction=MotionDirection.NONE,
                )
            self._reason = "navigation_command_stale_zero"
            return self._current_output(_ZERO, self._reason)
        if command_problem is not None:
            return self._latch_fault(
                command_problem,
                source=source,
                direction=MotionDirection.NONE,
            )
        lidar_problem = self._lidar_problem(receipt, header_now,
            MotionDirection.NONE if source==BaseAuthority.NAVIGATION and self._navigation_sweep_guard else mask)
        if lidar_problem is not None:
            return self._latch_fault(
                lidar_problem,
                source=source,
                direction=mask,
            )
        if self._transition_zero_pending:
            self._transition_zero_pending = False
            return self._current_output(_ZERO, "transition_zero")
        if self._command_protocol_active and command != _ZERO:
            problem = self._odometry_problem(receipt, header_now)
            if problem is None and self._measured_motion is None:
                problem = "odometry_pose_motion_missing"
            if problem:
                return self._latch_fault(problem, source=source, direction=mask)
            grant = self._command_grant
            if grant is None:
                return self._latch_fault("command_grant_missing", source=source, direction=mask)
            settled = self._odometry_stopped_for_dwell(receipt)
            try:
                limited = self._motion_limiter.limit(command, self._measured_motion,
                    source_now_s=header_now,
                    binding=(self.session_id, grant.grant_id, grant.source_goal_id),
                    measured_settled=settled)
            except ValueError:
                return self._latch_fault("command_limiter_invalid", source=source, direction=mask)
            if limited.new_inhibition:
                self._limiter_inhibit_receipt_s = receipt
                self._limiter_inhibit_header_s = header_now
                self._stopped_since_s = None
            if not limited.requires_settle:
                self._limiter_inhibit_receipt_s = None
                self._limiter_inhibit_header_s = None
            return self._sweep_guarded_output(limited.command, limited.reason, source,receipt,header_now,preserve_limiter=True)
        return self._sweep_guarded_output(command,self._reason,source,receipt,header_now)

    def _sweep_guarded_output(self,command,reason,source,receipt,header_now,*,preserve_limiter=False):
        if source==BaseAuthority.NAVIGATION and self._navigation_sweep_guard is not None:
            try:
                problem=self._navigation_sweep_guard(command,self._measured_motion,header_now,receipt)
            except Exception:
                problem='navigation_sweep_guard_exception'
            if problem:
                return self._latch_fault(problem,source=source,direction=command_direction_mask(command,
                    zero_epsilon=self.config.zero_epsilon))
        return self._current_output(command,reason,preserve_limiter=preserve_limiter)

    def cancel(self, *, receipt_now_s):
        if self._shutdown:
            return self._current_output(_ZERO, "shutdown")
        try:
            self._receipt_time(receipt_now_s)
        except ValueError:
            return self._latch_fault("authority_time_invalid")
        self._invalidate_departures("cancelled")
        if self._state == BaseAuthority.FAULT:
            return self._current_output(_ZERO, self._fault_reason)
        self._enter_state(BaseAuthority.NONE, None, "cancelled")
        return self._current_output(_ZERO, self._reason)

    def shutdown(self, *, receipt_now_s):
        if self._shutdown:
            return self._current_output(_ZERO, "shutdown")
        try:
            self._receipt_time(receipt_now_s)
        except ValueError:
            pass
        self._invalidate_departures("shutdown")
        self._enter_state(BaseAuthority.NONE, None, "shutdown")
        self._shutdown = True
        return self._current_output(_ZERO, self._reason)

    def emergency_stop(self, reason, *, receipt_now_s):
        """Latch a wrapper or process fault and return an exact-zero command."""

        if self._shutdown:
            return self._current_output(_ZERO, "shutdown")
        # This is a new explicit stop even when a previous fault is latched.
        # Revoke queued reset/departure identities while retaining the first
        # fault cause. Passive output checks must not keep advancing the epoch.
        if self._state == BaseAuthority.FAULT:
            self._invalidate_departures("new_emergency_stop")
        try:
            self._receipt_time(receipt_now_s)
        except ValueError:
            return self._latch_fault("authority_time_invalid")
        if not isinstance(reason, str) or not reason:
            reason = "authority_emergency_stop"
        return self._latch_fault(reason)

    def _handover_output(self, receipt, header_now):
        lidar_problem = self._lidar_problem(
            receipt,
            header_now,
            MotionDirection.NONE,
        )
        if lidar_problem is not None:
            return self._latch_fault(lidar_problem)
        target = self._handover_target
        command_problem, command, mask = self._command_problem(target, receipt, header_now)
        if command_problem is not None:
            self._stopped_since_s = None
            self._reason = command_problem
            return self._current_output(_ZERO, self._reason)
        lidar_problem = self._lidar_problem(receipt, header_now, mask)
        if lidar_problem is not None:
            return self._latch_fault(
                lidar_problem,
                source=target,
                direction=mask,
            )
        odom_problem = self._odometry_problem(receipt, header_now)
        if odom_problem is not None:
            self._stopped_since_s = None
            self._reason = odom_problem
            return self._current_output(_ZERO, self._reason)
        if not self._odometry_stopped_for_dwell(receipt):
            self._reason = "odometry_not_settled"
            return self._current_output(_ZERO, self._reason)
        reason = (
            "docking_granted"
            if target == BaseAuthority.DOCKING
            else "navigation_granted"
        )
        self._enter_state(target, target, reason)
        if target == BaseAuthority.NAVIGATION:
            self._navigation_granted_s = receipt
        return self._current_output(_ZERO, reason)

    def _command_problem(self, source, receipt, header_now=None):
        source_name = self._source_name(source)
        evidence = self._commands.get(source)
        if evidence is None:
            return "{0}_command_missing".format(source_name), None, MotionDirection.NONE
        timeout = (
            self.config.navigation_command_receipt_s
            if source == BaseAuthority.NAVIGATION
            else self.config.docking_command_receipt_s
        )
        stamp = self._command_headers.get(source)
        if stamp is not None and header_now is not None:
            header_age = header_now - stamp
            if not -self.config.maximum_header_future_skew_s <= header_age <= self.config.command_header_s:
                return "{0}_command_header_stale".format(source_name), None, MotionDirection.NONE
        age = receipt - evidence.receipt_s
        if age < 0.0:
            return "{0}_command_from_future".format(source_name), None, MotionDirection.NONE
        if age > timeout:
            return "{0}_command_stale".format(source_name), None, MotionDirection.NONE
        command = evidence.command
        if not isinstance(command, Velocity2D):
            return "{0}_command_invalid".format(source_name), None, MotionDirection.NONE
        if (
            abs(command.linear_x) > self.config.max_linear_x_mps
            or abs(command.linear_y) > self.config.max_linear_y_mps
            or abs(command.angular_z) > self.config.max_angular_z_radps
        ):
            return "{0}_command_invalid".format(source_name), None, MotionDirection.NONE
        if not self._command_protocol_active:
            command = Velocity2D(
                0.0
                if abs(command.linear_x) <= self.config.zero_epsilon
                else command.linear_x,
                0.0
                if abs(command.linear_y) <= self.config.zero_epsilon
                else command.linear_y,
                0.0
                if abs(command.angular_z) <= self.config.zero_epsilon
                else command.angular_z,
            )
        try:
            mask = command_direction_mask(
                command,
                zero_epsilon=self.config.zero_epsilon,
            )
        except ValueError:
            return "{0}_command_invalid".format(source_name), None, MotionDirection.NONE
        return None, command, mask

    def _stored_command_is_zero(self, source):
        evidence = self._commands.get(source)
        if evidence is None or not isinstance(evidence.command, Velocity2D):
            return False
        command = evidence.command
        if self._command_protocol_active:
            return command == _ZERO
        return (
            abs(command.linear_x) <= self.config.zero_epsilon
            and abs(command.linear_y) <= self.config.zero_epsilon
            and abs(command.angular_z) <= self.config.zero_epsilon
        )

    def _lidar_problem(self, receipt, header_now, direction):
        evidence = self._lidar
        if evidence is None:
            return "lidar_missing"
        if evidence.validation_reason is not None:
            return evidence.validation_reason
        receipt_age = receipt - evidence.receipt_s
        if receipt_age < 0.0:
            return "lidar_receipt_future"
        if receipt_age > self.config.lidar_receipt_s:
            return "lidar_receipt_stale"
        header_age = header_now - evidence.header_stamp_s
        if header_age < -self.config.maximum_header_future_skew_s:
            return "lidar_header_future"
        if header_age > self.config.lidar_header_s:
            return "lidar_header_stale"
        if evidence.health == SafetyHealth.STALE:
            return "lidar_stale"
        if evidence.health == SafetyHealth.INVALID:
            return "lidar_invalid"
        if evidence.health not in (SafetyHealth.CLEAR, SafetyHealth.BLOCKED):
            return "lidar_state_invalid"
        if (
            evidence.health == SafetyHealth.BLOCKED
            and direction != MotionDirection.NONE
            and evidence.blocked_directions & direction
        ):
            return "lidar_blocked_{0}".format(
                self._direction_reason(evidence.blocked_directions & direction)
            )
        return None

    def _odometry_problem(self, receipt, header_now):
        evidence = self._odometry
        if evidence is None:
            return "odometry_missing"
        if evidence.validation_reason is not None:
            return evidence.validation_reason
        receipt_age = receipt - evidence.receipt_s
        if receipt_age < 0.0:
            return "odometry_receipt_future"
        if receipt_age > self.config.odometry_receipt_s:
            return "odometry_receipt_stale"
        header_age = header_now - evidence.header_stamp_s
        if header_age < -self.config.maximum_header_future_skew_s:
            return "odometry_header_future"
        if header_age > self.config.odometry_header_s:
            return "odometry_header_stale"
        return None

    def _fault_reset_problem(self, receipt, header_now):
        source = self._fault_source
        direction = self._fault_direction
        if self._command_protocol_active:
            if self._command_grant is not None:
                return "fault_reset_grant_not_revoked"
            if (self._last_output_receipt_s is None or self._last_output_command != _ZERO or
                    not 0.0 <= receipt - self._last_output_receipt_s <= 2.0 / self.config.output_rate_hz):
                return "fault_reset_zero_output_not_fresh"
        elif source in _MOVING_AUTHORITIES:
            command_problem, command, current_direction = self._command_problem(
                source,
                receipt,
                header_now,
            )
            if command_problem is not None:
                return "fault_reset_{0}".format(command_problem)
            direction |= current_direction
            del command
        lidar_problem = self._lidar_problem(receipt, header_now, direction)
        if lidar_problem is not None:
            return "fault_reset_{0}".format(lidar_problem)
        odom_problem = self._odometry_problem(receipt, header_now)
        if odom_problem is not None:
            return "fault_reset_{0}".format(odom_problem)
        if not self._odometry_stopped_for_dwell(receipt):
            return "fault_reset_odometry_not_settled"
        return None

    def _odometry_after_limiter_stop(self, evidence):
        # Both source acquisition and local callback receipt must follow the
        # first zero/inhibit boundary; queued pre-stop poses earn no dwell.
        if self._limiter_inhibit_receipt_s is None:
            return True
        return (evidence is not None and self._limiter_inhibit_header_s is not None
                and evidence.receipt_s > self._limiter_inhibit_receipt_s
                and evidence.header_stamp_s > self._limiter_inhibit_header_s)

    def _odometry_stopped_for_dwell(self, receipt):
        evidence = self._odometry
        if evidence is None or not self._odometry_after_limiter_stop(evidence) or not self._speeds_stopped(
            evidence.linear_speed_mps,
            evidence.angular_speed_radps,
        ):
            self._stopped_since_s = None
            return False
        if self._stopped_since_s is None:
            self._stopped_since_s = evidence.receipt_s
        return receipt - self._stopped_since_s >= self.config.stop_dwell_s

    def _speeds_stopped(self, linear, angular):
        return (
            linear <= self.config.stop_linear_speed_mps
            and angular <= self.config.stop_angular_speed_radps
        )

    def _enter_state(self, state, selected, reason):
        changed = state != self._state or selected != self._selected
        self._state = state
        self._selected = selected
        self._reason = reason
        self._handover_target = None
        if changed:
            self._stopped_since_s = None
        if state == BaseAuthority.NONE:
            self._commands.clear()
        if state != BaseAuthority.NAVIGATION:
            self._navigation_granted_s = None
        if changed:
            self._transition_zero_pending = True

    def _latch_fault(self, reason, source=None, direction=MotionDirection.NONE):
        if self._state != BaseAuthority.FAULT:
            self._invalidate_departures("fault")
            self._fault_reason = reason
            self._fault_source = source
            self._fault_direction = direction
            self._state = BaseAuthority.FAULT
            self._selected = None
            self._handover_target = None
            self._reason = reason
            self._transition_zero_pending = True
            self._stopped_since_s = None
            self._navigation_granted_s = None
        return self._current_output(_ZERO, self._fault_reason)

    def _clear_fault(self):
        self._fault_reason = None
        self._fault_source = None
        self._fault_direction = MotionDirection.NONE

    def _current_output(self, command, reason, preserve_limiter=False):
        if command == _ZERO and not preserve_limiter:
            binding = (self.session_id, self.command_grant_id, self.source_goal_id)
            new = self._motion_limiter.stop(source_now_s=self._output_header_now_s,
                                            binding=binding if all(binding) else None)
            if new:
                self._limiter_inhibit_receipt_s = self._last_receipt_s
                self._limiter_inhibit_header_s = self._output_header_now_s
                self._stopped_since_s = None
            if not self._motion_limiter.waiting_for_settle:
                self._limiter_inhibit_receipt_s = None
                self._limiter_inhibit_header_s = None
        self._last_output_command = command
        self._last_output_receipt_s = self._last_receipt_s
        return AuthorityOutput(
            command=command,
            state=self._state,
            selected_source=self._source_name(self._selected),
            reason=reason,
        )

    def _receipt_time(self, value):
        receipt = _finite(value, "receipt_now_s")
        if self._last_receipt_s is not None and receipt < self._last_receipt_s:
            raise ValueError("receipt clock must be monotonic")
        self._last_receipt_s = receipt
        return receipt

    @staticmethod
    def _moving_authority(value):
        try:
            authority = BaseAuthority(value)
        except (TypeError, ValueError):
            raise ValueError("command source is invalid")
        if authority not in _MOVING_AUTHORITIES:
            raise ValueError("command source is invalid")
        return authority

    @staticmethod
    def _safety_health(value):
        if isinstance(value, bool):
            raise ValueError("safety health is invalid")
        return SafetyHealth(value)

    @staticmethod
    def _direction_mask(value):
        if isinstance(value, bool) or not isinstance(value, (int, MotionDirection)):
            raise ValueError("direction mask is invalid")
        integer = int(value)
        if integer < 0 or integer & ~int(MotionDirection.ALL):
            raise ValueError("direction mask is invalid")
        return MotionDirection(integer)

    @staticmethod
    def _source_name(source):
        if source == BaseAuthority.NAVIGATION:
            return "navigation"
        if source == BaseAuthority.DOCKING:
            return "docking"
        return ""

    @staticmethod
    def _direction_reason(mask):
        names = []
        for direction, name in (
            (MotionDirection.FORWARD, "forward"),
            (MotionDirection.REVERSE, "reverse"),
            (MotionDirection.LEFT, "left"),
            (MotionDirection.RIGHT, "right"),
            (MotionDirection.ROTATION, "rotation"),
        ):
            if mask & direction:
                names.append(name)
        return "_".join(names) if names else "unknown"
