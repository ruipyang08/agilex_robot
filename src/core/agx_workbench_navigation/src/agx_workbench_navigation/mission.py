"""ROS-independent coordinator for one registered workbench mission.

The coordinator deliberately depends on small duck-typed ports.  ROS callbacks,
message conversion, TF lookup, and action clients live in ``workbench_mission``
so state transitions and cleanup remain deterministic and unit testable.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real

from .contracts import (
    ActionPhase,
    ActionResultCode,
    BaseAuthority,
    FinalErrorEstimate,
    SafetyHealth,
)
from .docking import DockingOutput, DockingPhase
from .departure import DepartureStep


class EvidenceState(str, Enum):
    """State of the expected-marker observation window."""

    WAITING = "waiting"
    ACQUIRED = "acquired"
    INVALID = "invalid"
    TF_UNAVAILABLE = "tf_unavailable"


@dataclass(frozen=True)
class AuthorityReply:
    """Portable form of a SetBaseAuthority response."""

    accepted: bool
    state: BaseAuthority
    reason: str


@dataclass(frozen=True)
class AuthorityEvidence:
    """Fresh observed state from the sole base-command authority."""

    state: BaseAuthority
    fresh: bool
    reason: str


@dataclass(frozen=True)
class MissionConfig:
    """Phase budgets in seconds of the clock supplied to the coordinator.

    The ROS wrapper supplies ROS/source time; its polling remains wall-driven.
    These phase limits are separate from the workflow's steady task deadline.
    """

    localization_timeout_s: float = 10.0
    camera_timeout_s: float = 10.0
    departure_timeout_s: float = 60.0
    navigation_timeout_s: float = 120.0
    settling_timeout_s: float = 5.0
    tag_acquisition_timeout_s: float = 10.0
    authority_timeout_s: float = 3.0
    docking_timeout_s: float = 240.0
    final_settling_timeout_s: float = 3.0

    def __post_init__(self):
        for field_name in (
            "localization_timeout_s",
            "camera_timeout_s",
            "departure_timeout_s",
            "navigation_timeout_s",
            "settling_timeout_s",
            "tag_acquisition_timeout_s",
            "authority_timeout_s",
            "docking_timeout_s",
            "final_settling_timeout_s",
        ):
            value = _finite(getattr(self, field_name), field_name)
            if value <= 0.0:
                raise ValueError("{0} must be positive".format(field_name))
            object.__setattr__(self, field_name, value)


@dataclass(frozen=True)
class MissionFeedback:
    """Portable representation of NavigateToWorkbench feedback."""

    phase: ActionPhase
    phase_name: str
    detail: str
    position_error_m: float
    yaw_error_rad: float
    docking_attempt: int = 0


@dataclass(frozen=True)
class MissionResult:
    """Portable representation of NavigateToWorkbench result."""

    code: ActionResultCode
    message: str
    final_position_error_m: float
    final_yaw_error_rad: float
    final_error_is_estimated: bool
    final_error_source: str


def internal_failure_result(message="internal_configuration_error"):
    """Build a fail-closed result when coordinator construction is impossible."""

    return MissionResult(
        code=ActionResultCode.INTERNAL_CONFIGURATION_ERROR,
        message=str(message),
        final_position_error_m=0.0,
        final_yaw_error_rad=0.0,
        final_error_is_estimated=False,
        final_error_source="",
    )


_REASON_CODE = {
    "invalid_registry": ActionResultCode.INVALID_REGISTRY,
    "registry_invalidated": ActionResultCode.INVALID_REGISTRY,
    "navigation_rejected": ActionResultCode.NAVIGATION_REJECTED,
    "navigation_aborted": ActionResultCode.NAVIGATION_ABORTED,
    "navigation_timeout": ActionResultCode.NAVIGATION_TIMEOUT,
    "departure_unqualified": ActionResultCode.HANDOFF_INVALID,
    "departure_deviation": ActionResultCode.BLIND_DEVIATION,
    "departure_timeout": ActionResultCode.DOCKING_TIMEOUT,
    "localization_unavailable": ActionResultCode.LOCALIZATION_UNAVAILABLE,
    "tf_unavailable": ActionResultCode.TF_UNAVAILABLE,
    "camera_not_ready": ActionResultCode.CAMERA_NOT_READY,
    "tag_acquisition_timeout": ActionResultCode.TAG_ACQUISITION_TIMEOUT,
    "wrong_marker_id": ActionResultCode.TAG_EVIDENCE_INVALID,
    "tag_not_fresh": ActionResultCode.TAG_EVIDENCE_INVALID,
    "tag_lost_before_handoff": ActionResultCode.TAG_EVIDENCE_INVALID,
    "translation_jump": ActionResultCode.TAG_EVIDENCE_INVALID,
    "angular_jump": ActionResultCode.TAG_EVIDENCE_INVALID,
    "wrong_floor_normal": ActionResultCode.TAG_EVIDENCE_INVALID,
    "impossible_height": ActionResultCode.TAG_EVIDENCE_INVALID,
    "impossible_range": ActionResultCode.TAG_EVIDENCE_INVALID,
    "back_facing": ActionResultCode.TAG_EVIDENCE_INVALID,
    "invalid_tag_pose": ActionResultCode.TAG_EVIDENCE_INVALID,
    "unexpected_marker_id": ActionResultCode.TAG_EVIDENCE_INVALID,
    "unexpected_tag_frame": ActionResultCode.TAG_EVIDENCE_INVALID,
    "tag_outside_visual_workspace": ActionResultCode.TAG_EVIDENCE_INVALID,
    "handoff_geometry_mismatch": ActionResultCode.HANDOFF_INVALID,
    "handoff_not_settled": ActionResultCode.HANDOFF_INVALID,
    "handoff_decision_not_accepted": ActionResultCode.HANDOFF_INVALID,
    "handoff_pose_outside_bounds": ActionResultCode.HANDOFF_INVALID,
    "blind_travel_limit": ActionResultCode.BLIND_TRAVEL_LIMIT,
    "route_blocked": ActionResultCode.ROUTE_BLOCKED,
    "route_invalidated": ActionResultCode.ROUTE_INVALIDATED,
    "lidar_blocked": ActionResultCode.LIDAR_BLOCKED,
    "lidar_not_clear": ActionResultCode.LIDAR_BLOCKED,
    "lidar_not_fresh": ActionResultCode.LIDAR_STALE,
    "lidar_stale": ActionResultCode.LIDAR_STALE,
    "lidar_invalid": ActionResultCode.LIDAR_INVALID,
    "authority_denied": ActionResultCode.AUTHORITY_DENIED,
    "authority_timeout": ActionResultCode.AUTHORITY_DENIED,
    "authority_fault": ActionResultCode.AUTHORITY_FAULT,
    "visual_timeout": ActionResultCode.DOCKING_TIMEOUT,
    "visual_phase_timeout": ActionResultCode.DOCKING_TIMEOUT,
    "blind_phase_timeout": ActionResultCode.DOCKING_TIMEOUT,
    "verify_phase_timeout": ActionResultCode.DOCKING_TIMEOUT,
    "docking_timeout": ActionResultCode.DOCKING_TIMEOUT,
    "visual_phase_oscillation": ActionResultCode.DOCKING_OSCILLATION,
    "final_tolerance_lost": ActionResultCode.FINAL_TOLERANCE_NOT_REACHED,
    "final_tolerance_overshoot": ActionResultCode.FINAL_TOLERANCE_NOT_REACHED,
    "final_tolerance_not_reached": ActionResultCode.FINAL_TOLERANCE_NOT_REACHED,
    "cross_track_deviation": ActionResultCode.BLIND_DEVIATION,
    "yaw_drift": ActionResultCode.BLIND_DEVIATION,
    "reverse_motion": ActionResultCode.BLIND_DEVIATION,
    "blind_overshoot": ActionResultCode.BLIND_DEVIATION,
    "rollback_retry_exhausted": ActionResultCode.ALIGNMENT_RETRY_EXHAUSTED,
    "rollback_deviation": ActionResultCode.BLIND_DEVIATION,
    "rollback_distance_limit": ActionResultCode.BLIND_DEVIATION,
    "rollback_timeout": ActionResultCode.DOCKING_TIMEOUT,
    "tag_reacquisition_after_rollback_timeout": (
        ActionResultCode.TAG_ACQUISITION_TIMEOUT
    ),
    "odometry_not_fresh": ActionResultCode.ODOM_NOT_SETTLED,
    "odometry_not_settled": ActionResultCode.ODOM_NOT_SETTLED,
    "cancelled": ActionResultCode.CANCELLED,
    "internal_configuration_error": ActionResultCode.INTERNAL_CONFIGURATION_ERROR,
}

# These handoff failures describe evidence that can legitimately recover while
# the rover remains stopped.  Geometry, identity, and commissioning failures
# are deliberately absent: retrying those cannot change the outcome and would
# only turn a precise failure into the generic docking timeout.
_RETRYABLE_HANDOFF_REJECTION_REASONS = frozenset(
    {
        "insufficient_samples",
        "invalid_samples",
        "unexpected_marker_id",
        "unexpected_sample_frame",
        "duplicate_samples",
        "nonmonotonic_samples",
        "insufficient_sample_span",
        "invalid_sample_clock",
        "stale_samples",
        "insufficient_inlier_samples",
        "excessive_translation_dispersion",
        "excessive_yaw_dispersion",
        "lateral_error",
        "yaw_error",
        "forward_residual",
        "lateral_residual",
        "floor_height_residual",
        "floor_normal_residual",
        "approach_yaw_residual",
        "rover_not_settled",
        "odometry_not_fresh",
        "lidar_not_fresh",
        "lidar_not_clear",
    }
)


def failure_code_for_reason(reason):
    """Map a stable lower-layer reason to the public action result contract."""

    if not isinstance(reason, str):
        return ActionResultCode.INTERNAL_CONFIGURATION_ERROR
    if reason.startswith("tag_not_fresh: "):
        return ActionResultCode.TAG_EVIDENCE_INVALID
    if reason.startswith("lidar_blocked_"):
        return ActionResultCode.LIDAR_BLOCKED
    if reason.startswith("route_blocked: "):
        return ActionResultCode.ROUTE_BLOCKED
    if reason.startswith("handoff_"):
        return ActionResultCode.HANDOFF_INVALID
    return _REASON_CODE.get(reason, ActionResultCode.INTERNAL_CONFIGURATION_ERROR)


def handoff_rejection_is_retryable(reason):
    """Whether stopped sensor evidence may make this rejection recover."""

    return reason in _RETRYABLE_HANDOFF_REJECTION_REASONS


def public_docking_reason(controller_reason, handoff_rejection_reason=None, tag_detail=None):
    """Restore a trusted adapter-side rejection when the controller is generic."""

    if (
        controller_reason == "handoff_decision_not_accepted"
        and handoff_rejection_reason in _REASON_CODE
    ):
        return handoff_rejection_reason
    if controller_reason in (
        "visual_tag_loss_hold", "visual_tag_loss_after_commit",
        "waiting_for_tag_reacquisition", "waiting_for_tag_reacquisition_after_rollback",
    ) and tag_detail:
        return "{}: {}".format(controller_reason, tag_detail)
    return controller_reason


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{0} must be finite".format(name))
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError("{0} must be finite".format(name))
    return converted


def _phase_name(phase):
    return ActionPhase(phase).name


class MissionCoordinator:
    """Deterministic public mission state machine with bounded cleanup."""

    def __init__(
        self,
        config,
        *,
        registry,
        navigation,
        localization,
        camera,
        tags,
        lidar,
        authority,
        docking,
        departure,
        odometry,
    ):
        if not isinstance(config, MissionConfig):
            raise ValueError("config must be a MissionConfig")
        self.config = config
        self.registry = registry
        self.navigation = navigation
        self.localization = localization
        self.camera = camera
        self.tags = tags
        self.lidar = lidar
        self.authority = authority
        self.docking = docking
        self.departure = departure
        self.odometry = odometry
        self.phase = None
        self.phase_history = []
        self.feedback_history = []
        self.result = None
        self._workbench_id = None
        self._entry = None
        self._started_s = None
        self._phase_started_s = None
        self._last_step_s = None
        self._final_estimate = None
        self._cleanup_done = False
        self._authority_requested = False
        self._camera_wait_started_s = None
        self._departure_navigation_started = False

    @property
    def terminal(self):
        return self.result is not None

    @property
    def feedback(self):
        return None if not self.feedback_history else self.feedback_history[-1]

    @property
    def cleanup_done(self):
        return self._cleanup_done

    def start(self, workbench_id, *, now_s):
        """Start exactly one goal; lookup remains an observable phase."""

        if self.phase is not None:
            raise RuntimeError("mission already started")
        if not isinstance(workbench_id, str) or not workbench_id.strip():
            raise ValueError("workbench_id must be a non-empty string")
        now = _finite(now_s, "now_s")
        self._workbench_id = workbench_id
        self._started_s = now
        self._last_step_s = now
        self._transition(ActionPhase.LOOKUP, now, "validating_registry")

    def abort_internal(self, *, now_s):
        """Fail closed when the ROS boundary encounters an unhandled error.

        This entry point is intentionally safe before ``start`` has completed,
        so malformed goals and wrapper setup failures still execute the normal
        zero-command, cancel, and authority-release cleanup.
        """

        if self.result is not None:
            return self.result
        try:
            now = _finite(now_s, "now_s")
        except (TypeError, ValueError):
            now = self._last_step_s if self._last_step_s is not None else 0.0
        if self._last_step_s is not None and now < self._last_step_s:
            now = self._last_step_s
        if self.phase is None:
            self._workbench_id = ""
            self._started_s = now
            self._last_step_s = now
            self._transition(
                ActionPhase.LOOKUP,
                now,
                "wrapper_failure_before_start",
            )
        else:
            self._last_step_s = now
        return self._fail("internal_configuration_error")

    def step(self, *, now_s, cancelled=False):
        """Advance one control tick and return a result only when terminal."""

        if self.phase is None:
            raise RuntimeError("mission not started")
        if self.result is not None:
            return self.result
        try:
            now = _finite(now_s, "now_s")
        except ValueError:
            return self._fail("internal_configuration_error")
        if self._last_step_s is not None and now < self._last_step_s:
            return self._fail("internal_configuration_error")
        self._last_step_s = now
        if cancelled is True:
            return self._finish(
                ActionResultCode.CANCELLED,
                "cancelled",
                ActionPhase.CANCELLED,
            )
        if self.phase != ActionPhase.LOOKUP:
            try:
                if not self.registry.still_valid(self._entry):
                    return self._fail("registry_invalidated")
            except Exception:
                return self._fail("registry_invalidated")

        try:
            if self.phase == ActionPhase.LOOKUP:
                return self._step_lookup(now)
            safety_result = self._safety_failure()
            if safety_result is not None:
                return safety_result
            authority_result = self._authority_failure()
            if authority_result is not None:
                return authority_result
            if self.localization.ready() is not True:
                return self._fail("localization_unavailable")
            if self.phase == ActionPhase.COARSE_NAVIGATION:
                return self._step_navigation(now)
            if self.phase == ActionPhase.DEPARTING_DOCK:
                return self._step_departure(now)
            if self.phase == ActionPhase.DEPARTURE_AUTHORITY:
                return self._step_departure_authority(now)
            if self.phase == ActionPhase.DEPARTURE_HANDOVER:
                return self._step_departure_handover(now)
            if self.phase == ActionPhase.BASE_SETTLING:
                return self._step_base_settling(now)
            if self.phase == ActionPhase.TAG_ACQUISITION:
                return self._step_tag_acquisition(now)
            if self.phase == ActionPhase.AUTHORITY_HANDOVER:
                return self._step_authority_handover(now)
            if self.phase == ActionPhase.FINAL_DOCKING:
                return self._step_docking(now)
            if self.phase == ActionPhase.FINAL_SETTLING:
                return self._step_final_settling(now)
            return self._fail("internal_configuration_error")
        except Exception:
            return self._fail("internal_configuration_error")

    def _step_lookup(self, now):
        if self._entry is None:
            try:
                self._entry = self.registry.lookup(self._workbench_id)
            except Exception:
                return self._fail("invalid_registry")
        if self.localization.ready() is not True:
            if self._timed_out(now, self.config.localization_timeout_s):
                return self._fail("localization_unavailable")
            self._report("waiting_for_localization")
            return None
        if self.camera.ready() is not True:
            if self._camera_wait_started_s is None:
                self._camera_wait_started_s = now
            if (
                now - self._camera_wait_started_s
                > self.config.camera_timeout_s
            ):
                return self._fail("camera_not_ready")
            self._report("waiting_for_camera")
            return None
        self._camera_wait_started_s = None
        safety_result = self._safety_failure()
        if safety_result is not None:
            return safety_result
        try:
            departure_plan = self.departure.plan(self._entry)
        except Exception:
            return self._fail("departure_unqualified")
        if departure_plan is not None:
            try:
                reply = self.authority.request(BaseAuthority.DOCKING)
            except Exception:
                return self._fail("authority_denied")
            if not isinstance(reply, AuthorityReply):
                return self._fail("internal_configuration_error")
            if reply.state == BaseAuthority.FAULT:
                return self._fail("authority_fault")
            if not reply.accepted or reply.state not in (
                    BaseAuthority.DOCKING, BaseAuthority.STOPPING_FOR_HANDOVER):
                return self._fail("authority_denied")
            try:
                self.departure.start(departure_plan)
            except Exception:
                return self._fail("departure_unqualified")
            if reply.state == BaseAuthority.DOCKING:
                self._transition(ActionPhase.DEPARTING_DOCK, now,
                                 "reverse_departure_active")
            else:
                self._transition(ActionPhase.DEPARTURE_AUTHORITY, now,
                                 "waiting_for_docking_departure_grant")
            return None
        try:
            reply = self.authority.request(BaseAuthority.NAVIGATION)
        except Exception:
            return self._fail("authority_denied")
        if not isinstance(reply, AuthorityReply):
            return self._fail("internal_configuration_error")
        if reply.state == BaseAuthority.FAULT:
            return self._fail("authority_fault")
        if not reply.accepted or reply.state != BaseAuthority.NAVIGATION:
            return self._fail("authority_denied")
        try:
            accepted = self.navigation.start(self._entry.staging_pose)
        except Exception:
            accepted = False
        if accepted is not True:
            return self._fail("navigation_rejected")
        self._transition(
            ActionPhase.COARSE_NAVIGATION,
            now,
            "move_base_goal_active",
        )
        return None

    def _step_departure_authority(self, now):
        if self._timed_out(now, self.config.authority_timeout_s):
            return self._fail("authority_timeout")
        try:
            self.departure.stop()
            evidence = self.authority.status()
        except Exception:
            return self._fail("authority_fault")
        if not isinstance(evidence, AuthorityEvidence) or not evidence.fresh:
            return self._fail("authority_fault")
        if evidence.state == BaseAuthority.DOCKING:
            self._transition(ActionPhase.DEPARTING_DOCK, now,
                             "reverse_departure_active")
            return None
        if evidence.state == BaseAuthority.STOPPING_FOR_HANDOVER:
            return None
        return self._fail("authority_fault")

    def _step_departure(self, now):
        if self._timed_out(now, self.config.departure_timeout_s):
            return self._fail("departure_timeout")
        try:
            output = self.departure.step()
        except Exception:
            return self._fail("departure_unqualified")
        if not isinstance(output, DepartureStep):
            return self._fail("internal_configuration_error")
        if output.failure_reason:
            return self._fail(output.failure_reason)
        if not output.complete:
            return None
        try:
            self.departure.stop()
        except Exception:
            return self._fail("authority_fault")
        self._transition(ActionPhase.DEPARTURE_HANDOVER, now,
                         "waiting_for_navigation_handover")
        return None

    def _step_departure_handover(self, now):
        if self._timed_out(now, self.config.authority_timeout_s):
            return self._fail("authority_timeout")
        if not self._departure_navigation_started:
            try:
                self.departure.stop()
                reply = self.authority.request(BaseAuthority.NAVIGATION)
            except Exception:
                return self._fail("authority_fault")
            if not isinstance(reply, AuthorityReply):
                return self._fail("internal_configuration_error")
            if not reply.accepted:
                if reply.reason == "odometry_not_settled":
                    return None
                return self._fail("authority_denied")
            if reply.state not in (BaseAuthority.STOPPING_FOR_HANDOVER,
                                   BaseAuthority.NAVIGATION):
                return self._fail("authority_fault")
            try:
                primed = self.navigation.prepare(self._entry.staging_pose)
            except Exception:
                primed = False
            if primed is not True:
                return self._fail("navigation_rejected")
            # An accepted pending grant is not active navigation authority.
            # Keep the departure source at zero until the measured handover
            # finishes; dispatching move_base earlier exposes non-navigation
            # canonical feedback to the new local follower.
            try:
                evidence = self.authority.status()
            except Exception:
                return self._fail("authority_fault")
            if not isinstance(evidence, AuthorityEvidence) or not evidence.fresh:
                return self._fail("authority_fault")
            if evidence.state == BaseAuthority.STOPPING_FOR_HANDOVER:
                return None
            if evidence.state != BaseAuthority.NAVIGATION:
                return self._fail("authority_fault")
            try:
                accepted = self.navigation.start(self._entry.staging_pose)
            except Exception:
                accepted = False
            if accepted is not True:
                return self._fail("navigation_rejected")
            self._departure_navigation_started = True
        try:
            evidence = self.authority.status()
            navigation_status = self.navigation.status()
        except Exception:
            return self._fail("authority_fault")
        if navigation_status not in ("pending", "active", "succeeded"):
            return self._fail("navigation_aborted")
        if isinstance(evidence, AuthorityEvidence) and evidence.fresh and (
                evidence.state == BaseAuthority.NAVIGATION):
            self._transition(ActionPhase.COARSE_NAVIGATION, now,
                             "move_base_goal_active_after_departure")
            return None
        if isinstance(evidence, AuthorityEvidence) and evidence.fresh and (
                evidence.state == BaseAuthority.STOPPING_FOR_HANDOVER):
            return None
        return self._fail("authority_fault")

    def _step_navigation(self, now):
        if self._timed_out(now, self.config.navigation_timeout_s):
            return self._fail("navigation_timeout")
        try:
            status = self.navigation.status()
        except Exception:
            return self._fail("navigation_aborted")
        if status=="route_blocked":
            reason='route_blocked'
            try:
                detail=getattr(self.navigation,'failure_reason',lambda: '')()
                if isinstance(detail,str) and detail:reason+=': '+detail[:256]
            except Exception:
                pass
            return self._fail(reason)
        if status in ("pending", "active"):
            return None
        if status == "succeeded":
            # The owned goal is terminal; its producer has emitted zero. Keep
            # this leg's readiness lease through measured settling and the
            # explicit NAVIGATION -> DOCKING grant handover. Generic NONE is
            # reserved for cleanup and invalidates departure authorization.
            self._transition(
                ActionPhase.BASE_SETTLING,
                now,
                "waiting_for_measured_stop",
            )
            return None
        if status == "rejected":
            return self._fail("navigation_rejected")
        return self._fail("navigation_aborted")

    def _step_base_settling(self, now):
        if self.odometry.fresh() is not True:
            return self._fail("odometry_not_fresh")
        if self.odometry.settled() is True:
            try:
                self.tags.configure(self._entry.marker_id)
            except Exception:
                return self._fail("internal_configuration_error")
            self._transition(
                ActionPhase.TAG_ACQUISITION,
                now,
                "waiting_for_expected_tag",
            )
            return None
        if self._timed_out(now, self.config.settling_timeout_s):
            return self._fail("odometry_not_settled")
        return None

    def _step_tag_acquisition(self, now):
        if self.camera.ready() is not True:
            if self._camera_wait_started_s is None:
                self._camera_wait_started_s = now
            if (
                now - self._camera_wait_started_s
                > self.config.camera_timeout_s
                or self._timed_out(
                    now,
                    self.config.tag_acquisition_timeout_s,
                )
            ):
                return self._fail("camera_not_ready")
            self._report("waiting_for_camera")
            return None
        self._camera_wait_started_s = None
        try:
            state, detail, pose = self.tags.evidence()
        except Exception:
            return self._fail("tag_not_fresh")
        if state == EvidenceState.TF_UNAVAILABLE:
            return self._fail("tf_unavailable")
        if state == EvidenceState.INVALID:
            return self._fail(str(detail))
        if state == EvidenceState.ACQUIRED:
            if pose is None:
                return self._fail("tag_not_fresh")
            try:
                self.docking.start(self._entry)
                self.docking.publish_zero()
            except Exception:
                return self._fail("internal_configuration_error")
            self._authority_requested = False
            self._transition(
                ActionPhase.AUTHORITY_HANDOVER,
                now,
                "docking_zero_heartbeat",
            )
            return None
        if self._timed_out(now, self.config.tag_acquisition_timeout_s):
            return self._fail("tag_acquisition_timeout")
        self._report(str(detail))
        return None

    def _step_authority_handover(self, now):
        try:
            self.docking.publish_zero()
            reply = self.authority.request(BaseAuthority.DOCKING)
            self._authority_requested = True
        except Exception:
            return self._fail("authority_fault")
        if not isinstance(reply, AuthorityReply):
            return self._fail("internal_configuration_error")
        if reply.state == BaseAuthority.FAULT:
            return self._fail("authority_fault")
        if reply.accepted and reply.state == BaseAuthority.DOCKING:
            self._transition(
                ActionPhase.FINAL_DOCKING,
                now,
                "docking_authority_granted",
            )
            return None
        if reply.state not in (
            BaseAuthority.NAVIGATION,
            BaseAuthority.NONE,
            BaseAuthority.STOPPING_FOR_HANDOVER,
        ):
            return self._fail("authority_fault")
        if self._timed_out(now, self.config.authority_timeout_s):
            return self._fail("authority_timeout")
        self._report(reply.reason)
        return None

    def _step_docking(self, now):
        if self._timed_out(now, self.config.docking_timeout_s):
            return self._fail("docking_timeout")
        try:
            output = self.docking.step()
        except Exception:
            return self._fail("internal_configuration_error")
        if not isinstance(output, DockingOutput):
            return self._fail("internal_configuration_error")
        self._report_docking(output)
        if not output.terminal:
            return None
        if output.succeeded and output.phase == DockingPhase.SUCCEEDED:
            estimate = output.final_error_estimate
            if (
                not isinstance(estimate, FinalErrorEstimate)
                or estimate.is_estimated is not True
                or estimate.source != "pre_handoff_alvar_plus_odom"
            ):
                return self._fail("internal_configuration_error")
            self._final_estimate = estimate
            self._transition(
                ActionPhase.FINAL_SETTLING,
                now,
                "verifying_final_tolerance",
                estimate=estimate,
            )
            return None
        return self._fail(
            output.reason,
            docking_attempt=output.attempt_index,
        )

    def _step_final_settling(self, now):
        try:
            self.docking.publish_zero()
        except Exception:
            return self._fail("internal_configuration_error")
        if self.odometry.fresh() is not True:
            return self._fail("odometry_not_fresh")
        try:
            within_tolerance = self.odometry.final_within_tolerance(
                self._final_estimate
            )
        except Exception:
            return self._fail("internal_configuration_error")
        if within_tolerance is True:
            return self._finish(
                ActionResultCode.OK,
                "succeeded",
                ActionPhase.SUCCEEDED,
            )
        if self._timed_out(now, self.config.final_settling_timeout_s):
            return self._fail("final_tolerance_not_reached")
        return None

    def _safety_failure(self):
        try:
            health = SafetyHealth(self.lidar.health())
        except (TypeError, ValueError):
            return self._fail("lidar_invalid")
        if health == SafetyHealth.CLEAR:
            return None
        if health == SafetyHealth.BLOCKED:
            # BLOCKED is directional.  The sole command arbiter owns the
            # candidate command and therefore owns the applicability test.
            # Treating every directional block as a mission-wide fault would
            # incorrectly stop, for example, forward navigation for an
            # obstacle that is only inside the right swept volume.
            return None
        if health == SafetyHealth.STALE:
            return self._fail("lidar_stale")
        return self._fail("lidar_invalid")

    def _authority_failure(self):
        expected_by_phase = {
            ActionPhase.DEPARTING_DOCK: BaseAuthority.DOCKING,
            ActionPhase.COARSE_NAVIGATION: BaseAuthority.NAVIGATION,
            ActionPhase.BASE_SETTLING: BaseAuthority.NAVIGATION,
            ActionPhase.TAG_ACQUISITION: BaseAuthority.NAVIGATION,
            ActionPhase.FINAL_DOCKING: BaseAuthority.DOCKING,
            ActionPhase.FINAL_SETTLING: BaseAuthority.DOCKING,
        }
        expected = expected_by_phase.get(self.phase)
        if expected is None:
            return None
        try:
            evidence = self.authority.status()
        except Exception:
            return self._fail("authority_fault")
        if (
            isinstance(evidence, AuthorityEvidence)
            and evidence.fresh is True
            and evidence.state == BaseAuthority.FAULT
            and evidence.reason.startswith("lidar_blocked")
        ):
            return self._fail("lidar_blocked")
        if (
            not isinstance(evidence, AuthorityEvidence)
            or evidence.fresh is not True
            or evidence.state != expected
        ):
            return self._fail("authority_fault")
        return None

    def _timed_out(self, now, limit):
        return now - self._phase_started_s > limit

    def _transition(
        self,
        phase,
        now,
        detail,
        estimate=None,
        docking_attempt=0,
    ):
        self.phase = ActionPhase(phase)
        self.phase_history.append(self.phase)
        self._phase_started_s = now
        self._report(
            detail,
            estimate=estimate,
            docking_attempt=docking_attempt,
            force=True,
        )

    def _report_docking(self, output):
        position_error = (
            abs(output.remaining_distance_m)
            if output.remaining_distance_m is not None
            else 0.0
        )
        yaw_error = (
            abs(output.yaw_drift_rad)
            if output.yaw_drift_rad is not None
            else 0.0
        )
        self._report(
            output.reason,
            position_error_m=position_error,
            yaw_error_rad=yaw_error,
            docking_attempt=output.attempt_index,
        )

    def _report(
        self,
        detail,
        *,
        position_error_m=0.0,
        yaw_error_rad=0.0,
        estimate=None,
        docking_attempt=0,
        force=False,
    ):
        if estimate is not None:
            position_error_m = estimate.error.position_m
            yaw_error_rad = estimate.error.yaw_rad
        feedback = MissionFeedback(
            phase=self.phase,
            phase_name=_phase_name(self.phase),
            detail=str(detail),
            position_error_m=float(position_error_m),
            yaw_error_rad=float(yaw_error_rad),
            docking_attempt=int(docking_attempt),
        )
        if force or not self.feedback_history or feedback != self.feedback_history[-1]:
            self.feedback_history.append(feedback)

    def _fail(self, reason, docking_attempt=0):
        return self._finish(
            failure_code_for_reason(reason),
            str(reason),
            ActionPhase.FAILED,
            docking_attempt=docking_attempt,
        )

    def _finish(
        self,
        code,
        message,
        terminal_phase,
        docking_attempt=0,
    ):
        if self.result is not None:
            return self.result
        cleanup_problems = []
        if not self._cleanup_done:
            self._cleanup_done = True
            for label, operation in (
                ("cancel move_base", self.navigation.cancel),
                ("stop departure", self.departure.stop),
                ("stop docking", self.docking.stop),
            ):
                try:
                    operation()
                except Exception as error:
                    cleanup_problems.append(
                        "{0}: {1}".format(label, error)
                    )
            try:
                release = getattr(self.authority, "release", None)
                if callable(release):
                    reply = release()
                else:
                    reply = self.authority.request(BaseAuthority.NONE)
                if (
                    not isinstance(reply, AuthorityReply)
                    or reply.accepted is not True
                    or reply.state != BaseAuthority.NONE
                ):
                    reason = (
                        reply.reason
                        if isinstance(reply, AuthorityReply)
                        else "invalid_response"
                    )
                    raise RuntimeError(reason)
            except Exception as error:
                cleanup_problems.append(
                    "release authority: {0}".format(error)
                )
        result_message = str(message)
        if cleanup_problems:
            result_message += "; cleanup: " + "; ".join(cleanup_problems)
        now = self._last_step_s
        self._transition(
            terminal_phase,
            now,
            result_message,
            estimate=self._final_estimate,
            docking_attempt=docking_attempt,
        )
        if self._final_estimate is None:
            position_error = 0.0
            yaw_error = 0.0
            estimated = False
            source = ""
        else:
            position_error = self._final_estimate.error.position_m
            yaw_error = self._final_estimate.error.yaw_rad
            estimated = self._final_estimate.is_estimated
            source = self._final_estimate.source
        self.result = MissionResult(
            code=ActionResultCode(code),
            message=result_message,
            final_position_error_m=position_error,
            final_yaw_error_rad=yaw_error,
            final_error_is_estimated=estimated,
            final_error_source=source,
        )
        return self.result
