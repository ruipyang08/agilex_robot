"""Deterministic visual and odometric floor-tag docking controller."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from numbers import Real

from .authority import command_direction_mask
from .contracts import (
    FinalErrorEstimate,
    MotionDirection,
    OdomPose2D,
    PlanarError,
    Velocity2D,
)
from .docking_motion import (
    advance_dwell,
    bounded_speed_magnitude,
    project_pose,
)
from .docking_types import (
    DockingCommand,
    DockingConfig,
    DockingOutput,
    DockingPhase,
)
from .motion_progress import MotionProgress, bound_reachable_command
from .parking_geometry import (
    HandoffCommand,
    HandoffDecision,
    _consume_handoff_authorization,
    _handoff_authorization_status,
)
from .tag_filter import FilteredTagPose


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


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def lidar_reason_for_command(command, blocked_directions):
    """Return the stable reason for a direction blocked by LiDAR."""

    if not isinstance(command, DockingCommand):
        raise ValueError("command must be a DockingCommand")
    blocked = MotionDirection(blocked_directions)
    direction = command_direction_mask(
        Velocity2D(command.linear_x, command.linear_y, command.angular_z),
        zero_epsilon=1e-9,
    )
    applicable = blocked & direction
    for flag, label in (
        (MotionDirection.FORWARD, "forward"),
        (MotionDirection.REVERSE, "reverse"),
        (MotionDirection.LEFT, "left"),
        (MotionDirection.RIGHT, "right"),
        (MotionDirection.ROTATION, "rotation"),
    ):
        if applicable & flag:
            return "lidar_blocked_{0}".format(label)
    return None


@dataclass(frozen=True)
class _MotionProgressFailure:
    previous: DockingCommand
    candidate: DockingCommand
    maximum_change: DockingCommand
    source_now_s: float
    source_dt_s: float
    receipt_now_s: float
    phase: DockingPhase
    progress: MotionProgress


class DockingController:
    """Visual alignment followed by bounded, forward-only odometric travel."""

    def __init__(self, config=None, *, requested_marker_id, expected_frame):
        self.config = DockingConfig() if config is None else config
        if not isinstance(self.config, DockingConfig):
            raise ValueError("config must be a DockingConfig")
        if (
            isinstance(requested_marker_id, bool)
            or not isinstance(requested_marker_id, int)
        ):
            raise ValueError("requested_marker_id must be an integer")
        if (
            not isinstance(expected_frame, str)
            or not expected_frame
            or expected_frame != expected_frame.strip()
        ):
            raise ValueError("expected_frame must be a nonempty frame")
        if expected_frame != "base_link":
            raise ValueError("expected_frame must be base_link")
        self.requested_marker_id = requested_marker_id
        self.expected_frame = expected_frame
        self._phase = DockingPhase.ALIGN_YAW
        self._phase_started_s = None
        self._visual_started_s = None
        self._last_desired_visual_phase = None
        self._visual_phase_transition_times = []
        self._last_step_s = None
        self._source_now_s = None
        self._source_dt_s = 0.0
        self._wall_now_s = None
        self._progress = None
        self._motion_progress_failure = None
        self._last_command = DockingCommand()
        self._terminal_output = None
        self._handoff_start = None
        self._handoff_distance_m = None
        self._pre_handoff_lateral_m = 0.0
        self._pre_handoff_yaw_rad = 0.0
        self._handoff_evidence_ready = False
        self._handoff_stopped_since_s = None
        self._handoff_settled_odom = None
        self._verify_stopped_since_s = None
        self._tag_loss_started_s = None
        self._precommit_stopped_since_s = None
        self._commit_start = None
        self._commit_forward_axis = None
        self._visual_drift_started_s = None
        self._committed_tag_loss_started_s = None
        self._odometry_loss_started_s = None
        self._rollback_attempts_completed = 0
        self._rollback_trigger_reason = None
        self._blind_braking = False
        self._rollback_braking = False
        self._rollback_stopped_since_s = None
        self._rollback_started_s = None
        self._rollback_start_along_m = None

    @property
    def phase(self):
        return self._phase

    @property
    def motion_progress_failure(self):
        """Original immutable infeasibility inputs, when optional capture worked."""
        return self._motion_progress_failure

    @property
    def handoff_ready(self):
        """Whether stopped evidence is ready for one handoff decision."""

        return (
            self._terminal_output is None
            and self._phase == DockingPhase.HANDOFF_SETTLING
            and self._handoff_evidence_ready
        )

    def accept_handoff(
        self,
        *,
        decision,
        controller_now_s,
        evidence_now_s,
    ):
        """Capture odometry exactly once at the visual-to-blind boundary."""

        if self._terminal_output is not None:
            return
        if (
            self._phase != DockingPhase.HANDOFF_SETTLING
            or not self._handoff_evidence_ready
        ):
            self._latch_failure("handoff_not_settled")
            return
        try:
            controller_time = _finite(controller_now_s, "controller_now_s")
        except ValueError:
            self._latch_failure("invalid_handoff_controller_time")
            return
        if self._last_step_s is not None and controller_time < self._last_step_s:
            self._latch_failure("nonmonotonic_handoff_controller_time")
            return
        try:
            evidence_time = _finite(evidence_now_s, "evidence_now_s")
        except ValueError:
            self._latch_failure("invalid_handoff_evidence_time")
            return

        validated = self._validate_handoff_decision(decision, evidence_time)
        if isinstance(validated, DockingOutput):
            return
        filtered, distance = validated
        if not isinstance(self._handoff_settled_odom, OdomPose2D):
            self._latch_failure("handoff_settled_odometry_unavailable")
            return
        consumption = _consume_handoff_authorization(decision)
        consumption_failures = {
            "unauthorized": "handoff_decision_not_authorized",
            "consumed": "handoff_decision_consumed",
            "payload_mismatch": "handoff_decision_payload_mismatch",
        }
        if consumption in consumption_failures:
            self._latch_failure(consumption_failures[consumption])
            return

        self._committed_tag_loss_started_s = None
        self._handoff_start = self._handoff_settled_odom
        self._handoff_distance_m = distance
        self._pre_handoff_lateral_m = filtered.lateral_m
        self._pre_handoff_yaw_rad = _normalize_angle(
            filtered.approach_yaw_rad
        )
        self._set_phase(
            DockingPhase.ODOMETRIC_FINAL_STRAIGHT,
            now_s=self._wall_now_s if self._wall_now_s is not None else controller_time,
        )
        self._last_step_s = controller_time
        self._last_command = DockingCommand()
        self._verify_stopped_since_s = None

    def _validate_handoff_decision(self, decision, evidence_time):
        failure = self._validate_handoff_envelope(decision)
        if failure is not None:
            return failure
        filtered = decision.filtered_pose
        geometry = self._validated_handoff_geometry(
            decision,
            filtered,
            evidence_time,
        )
        if isinstance(geometry, DockingOutput):
            return geometry
        forward, lateral, yaw, desired_final = geometry
        try:
            distance = _finite(
                decision.remaining_distance_m,
                "remaining_distance_m",
            )
        except ValueError:
            return self._latch_failure("invalid_handoff_distance")
        if (
            forward < self.config.visual_min_forward_m
            or forward > self.config.visual_max_forward_m
            or abs(lateral) > self.config.lateral_tolerance_m
            or abs(yaw) > self.config.yaw_tolerance_rad
        ):
            return self._latch_failure("handoff_pose_outside_bounds")
        if not math.isclose(
            forward - desired_final,
            distance,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            return self._latch_failure("handoff_geometry_mismatch")
        if distance <= 0.0:
            return self._latch_failure("nonpositive_handoff_distance")
        if distance > self.config.max_blind_travel_m:
            return self._latch_failure("blind_travel_limit")
        return filtered, distance

    def _validate_handoff_envelope(self, decision):
        if not isinstance(decision, HandoffDecision):
            return self._latch_failure("invalid_handoff_decision")
        if decision.accepted is not True:
            return self._latch_failure("handoff_decision_not_accepted")
        if decision.reason != "accepted":
            return self._latch_failure("handoff_decision_reason_invalid")
        authorization_status = _handoff_authorization_status(decision)
        authorization_failures = {
            "unauthorized": "handoff_decision_not_authorized",
            "consumed": "handoff_decision_consumed",
        }
        if authorization_status in authorization_failures:
            return self._latch_failure(
                authorization_failures[authorization_status]
            )
        if (
            not isinstance(decision.command, HandoffCommand)
            or not decision.command.is_zero()
        ):
            return self._latch_failure("handoff_command_not_zero")
        try:
            command_values = (
                _finite(decision.command.linear_x, "handoff_command"),
                _finite(decision.command.linear_y, "handoff_command"),
                _finite(decision.command.angular_z, "handoff_command"),
            )
        except ValueError:
            return self._latch_failure("handoff_command_not_zero")
        if any(value != 0.0 for value in command_values):
            return self._latch_failure("handoff_command_not_zero")
        filtered = decision.filtered_pose
        if (
            not isinstance(filtered, FilteredTagPose)
            or filtered.marker_id != self.requested_marker_id
            or filtered.frame_id != self.expected_frame
        ):
            return self._latch_failure("handoff_identity_mismatch")
        return None

    def _validated_handoff_geometry(self, decision, filtered, evidence_time):
        try:
            forward = _finite(filtered.forward_m, "filtered_forward_m")
            lateral = _finite(filtered.lateral_m, "filtered_lateral_m")
            yaw = _normalize_angle(
                _finite(filtered.approach_yaw_rad, "filtered_yaw_rad")
            )
            evidence_stamp = _finite(filtered.stamp_s, "filtered_stamp_s")
            desired_final = _finite(
                decision.desired_final_tag_x_m,
                "desired_final_tag_x_m",
            )
        except ValueError:
            return self._latch_failure("invalid_handoff_geometry")
        evidence_age = evidence_time - evidence_stamp
        if evidence_age < 0.0:
            return self._latch_failure("handoff_decision_from_future")
        if evidence_age > self.config.handoff_evidence_max_age_s:
            return self._latch_failure("handoff_decision_stale")
        return forward, lateral, yaw, desired_final

    def step(
        self,
        *,
        tag,
        odom,
        lidar_clear,
        now_s,
        tag_fresh,
        odom_fresh,
        lidar_fresh,
        odom_linear_speed_mps,
        odom_angular_speed_radps,
        motion_progress,
        monotonic_now_s=None,
        lidar_blocked_directions=MotionDirection.NONE,
        cancelled=False,
        shutdown=False,
    ):
        """Shape one command using source time and fresh signed canonical motion.

        ``now_s`` is the command header's captured ROS generation time. The
        optional monotonic clock owns watchdogs and stopped dwell; pure callers
        may use the same deterministic clock for both. Unsigned odometry speeds
        remain input-validation compatibility fields and never earn progress or
        stopping credit. Safety zeros are exceptions to normal source slew.
        """

        if self._terminal_output is not None:
            return self._terminal_output
        try:
            now = _finite(now_s, "now_s")
        except ValueError:
            return self._latch_failure("invalid_monotonic_time")
        if self._last_step_s is not None and now < self._last_step_s:
            return self._latch_failure("nonmonotonic_time")
        self._source_now_s = now
        self._source_dt_s = (0.0 if self._last_step_s is None else min(
            self.config.nominal_control_period_s, now - self._last_step_s))
        try:
            wall_now = now if monotonic_now_s is None else _finite(monotonic_now_s, "monotonic_now_s")
        except ValueError:
            return self._latch_failure("invalid_monotonic_time")
        if self._wall_now_s is not None and wall_now < self._wall_now_s:
            return self._latch_failure("nonmonotonic_wall_time")
        self._wall_now_s = wall_now
        if cancelled is True:
            return self._latch_cancelled()
        if shutdown is True:
            return self._latch_failure("shutdown")
        validated = self._validate_step_inputs(
            odom=odom,
            odom_linear_speed_mps=odom_linear_speed_mps,
            odom_angular_speed_radps=odom_angular_speed_radps,
        )
        if isinstance(validated, DockingOutput):
            return validated
        if not isinstance(motion_progress, MotionProgress) or not motion_progress.fresh(
                now_s=now, receipt_now_s=wall_now):
            return self._latch_failure("motion_progress_not_fresh")
        self._progress = motion_progress
        linear_speed = math.hypot(motion_progress.velocity.linear_x, motion_progress.velocity.linear_y)
        angular_speed = abs(motion_progress.velocity.angular_z)
        now = wall_now
        if lidar_fresh is not True:
            return self._latch_failure("lidar_not_fresh")
        if lidar_clear is not True:
            return self._latch_failure("lidar_blocked")
        if odom_fresh is not True:
            if self._odometry_loss_started_s is None:
                self._odometry_loss_started_s = now
            if (
                now - self._odometry_loss_started_s + 1e-12
                >= self.config.odometry_loss_dwell_s
            ):
                return self._latch_failure("odometry_not_fresh")
            return self._emit(
                DockingOutput(
                    phase=self._phase,
                    command=DockingCommand(),
                    reason="odometry_freshness_hold",
                ),
                now,
            )
        self._odometry_loss_started_s = None
        try:
            if isinstance(lidar_blocked_directions, bool):
                raise ValueError
            blocked_directions = MotionDirection(lidar_blocked_directions)
            if int(blocked_directions) & ~int(MotionDirection.ALL):
                raise ValueError
        except (TypeError, ValueError):
            return self._latch_failure("lidar_invalid")

        if self._phase in (
            DockingPhase.ROLLBACK_STOPPING,
            DockingPhase.ROLLBACK,
            DockingPhase.ROLLBACK_SETTLING,
            DockingPhase.REACQUIRE_TAG,
        ):
            output = self._step_recovery(
                tag=tag,
                tag_fresh=tag_fresh,
                odom=odom,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                now_s=now,
            )
        elif self._handoff_start is not None:
            output = self._step_blind(
                odom=odom,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                now_s=now,
            )
        else:
            output = self._step_visual(
                tag=tag,
                odom=odom,
                tag_fresh=tag_fresh,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                now_s=now,
            )
        return self._apply_directional_lidar(output, blocked_directions)

    def _validate_step_inputs(
        self,
        *,
        odom,
        odom_linear_speed_mps,
        odom_angular_speed_radps,
    ):
        if not isinstance(odom, OdomPose2D):
            return self._latch_failure("invalid_odometry")
        try:
            linear_speed = abs(
                _finite(odom_linear_speed_mps, "odom_linear_speed_mps")
            )
            angular_speed = abs(
                _finite(odom_angular_speed_radps, "odom_angular_speed_radps")
            )
        except ValueError:
            return self._latch_failure("invalid_odometry")
        return linear_speed, angular_speed

    def _apply_directional_lidar(self, output, blocked_directions):
        command = output.command if output.command_generated else self._last_command
        if output.terminal or command.is_zero():
            return output
        reason = lidar_reason_for_command(command, blocked_directions)
        if reason is None:
            return output
        return self._latch_failure(
            reason,
            output.remaining_distance_m,
            output.along_track_m,
            output.cross_track_m,
            output.yaw_drift_rad,
        )

    def _step_visual(
        self,
        *,
        tag,
        odom,
        tag_fresh,
        linear_speed,
        angular_speed,
        now_s,
    ):
        if tag is None or tag_fresh is not True:
            if self._commit_start is not None:
                self._clear_handoff_settling_evidence()
                self._precommit_stopped_since_s = None
                self._visual_drift_started_s = None
                if self._committed_tag_loss_started_s is None:
                    self._committed_tag_loss_started_s = now_s
                if (now_s - self._committed_tag_loss_started_s + 1e-12
                        < self.config.visual_tag_loss_dwell_s):
                    return self._emit(
                        DockingOutput(
                            phase=self._phase,
                            command=DockingCommand(),
                            reason="visual_tag_loss_hold",
                        ),
                        now_s,
                    )
                return self._request_rollback(
                    "visual_tag_loss_after_commit",
                    odom=odom,
                    linear_speed=linear_speed,
                    angular_speed=angular_speed,
                    now_s=now_s,
                )
            failure_reason = (
                "tag_lost_before_handoff"
                if tag is None
                else "tag_not_fresh"
            )
            return self._wait_for_tag_reacquisition(now_s, failure_reason)

        self._committed_tag_loss_started_s = None
        if self._tag_loss_started_s is not None:
            tag_loss_duration_s = now_s - self._tag_loss_started_s
            self._tag_loss_started_s = None
            if self._visual_started_s is not None:
                self._visual_started_s += tag_loss_duration_s
            self._phase_started_s = None
            self._last_desired_visual_phase = None

        validated_tag = self._validated_tag_pose(tag)
        if isinstance(validated_tag, DockingOutput):
            return validated_tag
        forward_m, lateral_m, yaw_rad = validated_tag
        if self._visual_started_s is None:
            self._visual_started_s = now_s
        elif now_s - self._visual_started_s > self.config.visual_timeout_s:
            return self._latch_failure("visual_timeout")
        if self._commit_start is not None:
            return self._step_committed_visual(
                forward_m=forward_m,
                lateral_m=lateral_m,
                yaw_rad=yaw_rad,
                odom=odom,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                now_s=now_s,
            )
        return self._step_visual_alignment(
            forward_m=forward_m,
            lateral_m=lateral_m,
            yaw_rad=yaw_rad,
            odom=odom,
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            now_s=now_s,
        )

    def _validated_tag_pose(self, tag):
        if not isinstance(tag, FilteredTagPose):
            return self._latch_failure("invalid_tag_pose")
        if tag.marker_id != self.requested_marker_id:
            return self._latch_failure("unexpected_marker_id")
        if tag.frame_id != self.expected_frame:
            return self._latch_failure("unexpected_tag_frame")
        try:
            forward_m = _finite(tag.forward_m, "tag_pose")
            lateral_m = _finite(tag.lateral_m, "tag_pose")
            yaw_rad = _normalize_angle(
                _finite(tag.approach_yaw_rad, "tag_pose")
            )
        except ValueError:
            return self._latch_failure("invalid_tag_pose")
        if (
            forward_m < self.config.visual_min_forward_m
            or forward_m > self.config.visual_max_forward_m
            or abs(lateral_m) > self.config.visual_max_abs_lateral_m
            or abs(yaw_rad) > self.config.visual_max_abs_yaw_rad
        ):
            return self._latch_failure("tag_outside_visual_workspace")
        return forward_m, lateral_m, yaw_rad

    def _desired_visual_phase(self, lateral_m, yaw_rad):
        if abs(yaw_rad) > self.config.yaw_tolerance_rad:
            return DockingPhase.ALIGN_YAW
        if abs(lateral_m) > self.config.lateral_tolerance_m:
            return DockingPhase.ALIGN_LATERAL
        return DockingPhase.PRECOMMIT_SETTLING

    def _step_visual_alignment(
        self,
        *,
        forward_m,
        lateral_m,
        yaw_rad,
        odom,
        linear_speed,
        angular_speed,
        now_s,
    ):
        desired_phase = self._desired_visual_phase(lateral_m, yaw_rad)
        transition_failure = self._record_visual_phase_transition(
            desired_phase,
            now_s,
        )
        if transition_failure is not None:
            return transition_failure
        previous_phase = self._phase
        if desired_phase != previous_phase and not self._last_command.is_zero():
            self._precommit_stopped_since_s = None
            if self._phase_timed_out(
                now_s,
                self.config.visual_phase_timeout_s,
            ):
                return self._latch_failure("visual_phase_timeout")
            return self._emit(
                DockingOutput(
                    phase=previous_phase,
                    command=self._decelerated_visual_command(now_s),
                    reason="visual_phase_transition_deceleration",
                ),
                now_s,
            )

        self._set_phase(desired_phase, now_s)
        if self._phase_timed_out(now_s, self.config.visual_phase_timeout_s):
            return self._latch_failure("visual_phase_timeout")
        if desired_phase == DockingPhase.PRECOMMIT_SETTLING:
            return self._step_precommit_settling(
                forward_m=forward_m,
                lateral_m=lateral_m,
                yaw_rad=yaw_rad,
                odom=odom,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                now_s=now_s,
            )
        if desired_phase == DockingPhase.ALIGN_YAW:
            self._precommit_stopped_since_s = None
            yaw_command = self.config.yaw_gain * yaw_rad
            if abs(yaw_command) < self.config.min_yaw_speed_radps:
                yaw_command = math.copysign(
                    self.config.min_yaw_speed_radps,
                    yaw_rad,
                )
            command = DockingCommand(
                angular_z=self._bounded_visual_command(
                    yaw_command,
                    -self.config.max_yaw_speed_radps,
                    self.config.max_yaw_speed_radps,
                    self._last_command.angular_z,
                    self.config.max_visual_angular_acceleration_radps2,
                    now_s,
                )
            )
            reason = "align_yaw"
        else:
            self._precommit_stopped_since_s = None
            command = DockingCommand(
                linear_y=self._bounded_visual_command(
                    self.config.lateral_gain * lateral_m,
                    -self.config.max_lateral_speed_mps,
                    self.config.max_lateral_speed_mps,
                    self._last_command.linear_y,
                    self.config.max_visual_linear_acceleration_mps2,
                    now_s,
                )
            )
            reason = "align_lateral"
        return self._emit(
            DockingOutput(
                phase=desired_phase,
                command=command,
                reason=reason,
            ),
            now_s,
        )

    def _step_precommit_settling(
        self,
        *,
        forward_m,
        lateral_m,
        yaw_rad,
        odom,
        linear_speed,
        angular_speed,
        now_s,
    ):
        measured_stopped = (
            linear_speed <= self.config.stop_linear_speed_mps
            and angular_speed <= self.config.stop_angular_speed_radps
        )
        (
            self._precommit_stopped_since_s,
            dwell_complete,
        ) = advance_dwell(
            self._precommit_stopped_since_s,
            measured_stopped,
            now_s,
            self.config.precommit_settle_dwell_s,
        )
        if dwell_complete:
            self._commit_start = odom
            self._commit_forward_axis = (
                math.cos(odom.yaw_rad),
                math.sin(odom.yaw_rad),
            )
            self._visual_drift_started_s = None
            self._set_phase(DockingPhase.FORWARD_COMMITTED, now_s)
            return self._step_committed_visual(
                forward_m=forward_m,
                lateral_m=lateral_m,
                yaw_rad=yaw_rad,
                odom=odom,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                now_s=now_s,
            )
        return self._emit(
            DockingOutput(
                phase=DockingPhase.PRECOMMIT_SETTLING,
                command=DockingCommand(),
                reason="precommit_settling",
            ),
            now_s,
        )

    def _record_visual_phase_transition(self, desired_phase, now_s):
        if (
            self._last_desired_visual_phase is not None
            and desired_phase != self._last_desired_visual_phase
        ):
            oldest_allowed_s = now_s - self.config.visual_transition_window_s
            self._visual_phase_transition_times = [
                stamp
                for stamp in self._visual_phase_transition_times
                if stamp >= oldest_allowed_s
            ]
            self._visual_phase_transition_times.append(now_s)
            if (
                len(self._visual_phase_transition_times)
                > self.config.max_visual_phase_transitions
            ):
                return self._latch_failure("visual_phase_oscillation")
        self._last_desired_visual_phase = desired_phase
        return None

    def _step_committed_visual(
        self,
        *,
        forward_m,
        lateral_m,
        yaw_rad,
        odom,
        linear_speed,
        angular_speed,
        now_s,
    ):
        drifted = (
            abs(yaw_rad) > self.config.yaw_realign_threshold_rad
            or abs(lateral_m) > self.config.lateral_realign_threshold_m
        )
        if drifted:
            if self._visual_drift_started_s is None:
                self._visual_drift_started_s = now_s
            if (
                now_s - self._visual_drift_started_s + 1e-12
                >= self.config.visual_drift_dwell_s
            ):
                return self._request_rollback(
                    "visual_alignment_lost_after_commit",
                    odom=odom,
                    linear_speed=linear_speed,
                    angular_speed=angular_speed,
                    now_s=now_s,
                )
            return self._emit(
                DockingOutput(
                    phase=self._phase,
                    command=DockingCommand(),
                    reason="visual_drift_hold",
                ),
                now_s,
            )
        self._visual_drift_started_s = None

        handoff_geometry_ready = forward_m <= (
            self.config.visual_handoff_x_m
            + self.config.visual_handoff_capture_tolerance_m
        )
        if handoff_geometry_ready:
            return self._step_handoff_settling(
                lateral_m=lateral_m,
                yaw_rad=yaw_rad,
                odom=odom,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                now_s=now_s,
            )

        self._set_phase(DockingPhase.FORWARD_COMMITTED, now_s)
        requested_forward_mps = self.config.visual_forward_gain * max(
            forward_m - self.config.visual_handoff_x_m,
            0.0,
        )
        command = DockingCommand(
            linear_x=self._bounded_visual_command(
                requested_forward_mps,
                0.0,
                self.config.max_visual_forward_speed_mps,
                self._last_command.linear_x,
                self.config.max_visual_linear_acceleration_mps2,
                now_s,
            )
        )
        if command.linear_x > 0.0:
            self._precommit_stopped_since_s = None
        return self._emit(
            DockingOutput(
                phase=DockingPhase.FORWARD_COMMITTED,
                command=command,
                reason="forward_committed",
            ),
            now_s,
        )

    def _step_handoff_settling(
        self,
        *,
        lateral_m,
        yaw_rad,
        odom,
        linear_speed,
        angular_speed,
        now_s,
    ):
        if not self._last_command.is_zero():
            self._clear_handoff_settling_evidence()
            self._precommit_stopped_since_s = None
            braking = self._decelerated_visual_command(now_s)
            if not braking.is_zero():
                return self._emit(DockingOutput(phase=self._phase,
                    command=braking, reason="handoff_deceleration"), now_s)
        self._set_phase(DockingPhase.HANDOFF_SETTLING, now_s)
        measured_stopped = (
            linear_speed <= self.config.stop_linear_speed_mps
            and angular_speed <= self.config.stop_angular_speed_radps
        )
        strictly_aligned = (
            abs(yaw_rad) <= self.config.yaw_tolerance_rad
            and abs(lateral_m) <= self.config.lateral_tolerance_m
        )
        settling_condition = measured_stopped and strictly_aligned
        started_s = self._handoff_stopped_since_s
        if started_s is None:
            started_s = self._precommit_stopped_since_s
        (
            self._handoff_stopped_since_s,
            self._handoff_evidence_ready,
        ) = advance_dwell(
            started_s,
            settling_condition,
            now_s,
            self.config.handoff_settle_dwell_s,
        )
        if not settling_condition:
            self._clear_handoff_settling_evidence()
        self._handoff_settled_odom = (
            odom if self._handoff_evidence_ready else None
        )
        return self._emit(
            DockingOutput(
                phase=DockingPhase.HANDOFF_SETTLING,
                command=DockingCommand(),
                reason="handoff_settling",
            ),
            now_s,
        )

    def _request_rollback(
        self,
        reason,
        *,
        odom,
        linear_speed,
        angular_speed,
        now_s,
        remaining=None,
        along=None,
        cross=None,
        yaw_drift=None,
    ):
        if self._rollback_attempts_completed >= self.config.max_rollback_attempts:
            return self._latch_failure(
                "rollback_retry_exhausted",
                remaining,
                along,
                cross,
                yaw_drift,
            )
        if self._commit_start is None or self._commit_forward_axis is None:
            return self._latch_failure("rollback_checkpoint_unavailable")
        self._committed_tag_loss_started_s = None
        self._rollback_trigger_reason = reason
        self._rollback_stopped_since_s = None
        if (
            linear_speed <= self.config.stop_linear_speed_mps
            and angular_speed <= self.config.stop_angular_speed_radps
        ):
            self._rollback_stopped_since_s = now_s
        self._rollback_started_s = None
        self._rollback_start_along_m = None
        self._tag_loss_started_s = None
        self._set_phase(DockingPhase.ROLLBACK_STOPPING, now_s)
        return self._emit(
            DockingOutput(
                phase=DockingPhase.ROLLBACK_STOPPING,
                command=DockingCommand(),
                reason=reason,
                remaining_distance_m=remaining,
                along_track_m=along,
                cross_track_m=cross,
                yaw_drift_rad=yaw_drift,
            ),
            now_s,
        )

    def _step_recovery(
        self,
        *,
        tag,
        tag_fresh,
        odom,
        linear_speed,
        angular_speed,
        now_s,
    ):
        if self._phase == DockingPhase.REACQUIRE_TAG:
            return self._step_reacquire_tag(
                tag=tag,
                tag_fresh=tag_fresh,
                now_s=now_s,
            )

        try:
            along, cross, yaw_drift = self._project_from_commit(odom)
        except (OverflowError, ValueError):
            return self._latch_failure("invalid_rollback_odometry_projection")

        measured_stopped = (
            linear_speed <= self.config.stop_linear_speed_mps
            and angular_speed <= self.config.stop_angular_speed_radps
        )
        if self._phase == DockingPhase.ROLLBACK_STOPPING:
            return self._step_rollback_stopping(
                along=along,
                cross=cross,
                yaw_drift=yaw_drift,
                measured_stopped=measured_stopped,
                now_s=now_s,
            )

        if self._phase == DockingPhase.ROLLBACK:
            return self._step_rollback(
                along=along,
                cross=cross,
                yaw_drift=yaw_drift,
                measured_stopped=measured_stopped,
                now_s=now_s,
            )

        return self._step_rollback_settling(
            along=along,
            cross=cross,
            yaw_drift=yaw_drift,
            measured_stopped=measured_stopped,
            now_s=now_s,
        )

    def _step_rollback_stopping(
        self,
        *,
        along,
        cross,
        yaw_drift,
        measured_stopped,
        now_s,
    ):
        (
            self._rollback_stopped_since_s,
            dwell_complete,
        ) = advance_dwell(
            self._rollback_stopped_since_s,
            measured_stopped,
            now_s,
            self.config.stop_dwell_s,
        )
        if not dwell_complete:
            return self._emit(
                DockingOutput(
                    phase=DockingPhase.ROLLBACK_STOPPING,
                    command=DockingCommand(),
                    reason="rollback_stopping",
                    along_track_m=along,
                    cross_track_m=cross,
                    yaw_drift_rad=yaw_drift,
                ),
                now_s,
            )
        rollback_budget_m = max(0.0, along) + (
            self.config.rollback_overrun_allowance_m
        )
        if rollback_budget_m > self.config.max_rollback_travel_m:
            return self._latch_failure(
                "rollback_distance_limit",
                along=along,
                cross=cross,
                yaw_drift=yaw_drift,
            )
        self._rollback_start_along_m = along
        self._rollback_started_s = now_s
        self._set_phase(DockingPhase.ROLLBACK, now_s)
        return self._step_rollback(
            along=along,
            cross=cross,
            yaw_drift=yaw_drift,
            measured_stopped=measured_stopped,
            now_s=now_s,
        )

    def _step_rollback_settling(
        self,
        *,
        along,
        cross,
        yaw_drift,
        measured_stopped,
        now_s,
    ):
        failure = self._rollback_bounds_failure(along=along, cross=cross,
            yaw_drift=yaw_drift, now_s=now_s)
        if failure is not None:
            return failure
        (
            self._rollback_stopped_since_s,
            dwell_complete,
        ) = advance_dwell(
            self._rollback_stopped_since_s,
            measured_stopped,
            now_s,
            self.config.stop_dwell_s,
        )
        if not dwell_complete:
            return self._emit(
                DockingOutput(
                    phase=DockingPhase.ROLLBACK_SETTLING,
                    command=DockingCommand(),
                    reason="rollback_settling",
                    along_track_m=along,
                    cross_track_m=cross,
                    yaw_drift_rad=yaw_drift,
                ),
                now_s,
            )
        self._rollback_attempts_completed += 1
        self._clear_completed_attempt()
        self._tag_loss_started_s = now_s
        self._set_phase(DockingPhase.REACQUIRE_TAG, now_s)
        return self._emit(
            DockingOutput(
                phase=DockingPhase.REACQUIRE_TAG,
                command=DockingCommand(),
                reason="waiting_for_tag_reacquisition_after_rollback",
            ),
            now_s,
        )

    def _rollback_bounds_failure(self, *, along, cross, yaw_drift, now_s):
        if (
            abs(cross) > self.config.max_cross_track_m
            or abs(yaw_drift) > self.config.max_yaw_drift_rad
        ):
            return self._latch_failure(
                "rollback_deviation",
                along=along,
                cross=cross,
                yaw_drift=yaw_drift,
            )
        if self._rollback_start_along_m is None:
            return self._latch_failure("rollback_checkpoint_unavailable")
        rollback_distance_m = self._rollback_start_along_m - along
        authorized_distance_m = min(
            self.config.max_rollback_travel_m,
            max(0.0, self._rollback_start_along_m)
            + self.config.rollback_overrun_allowance_m,
        )
        if rollback_distance_m > authorized_distance_m + 1e-12:
            return self._latch_failure(
                "rollback_distance_limit",
                along=along,
                cross=cross,
                yaw_drift=yaw_drift,
            )
        if (
            self._rollback_started_s is None
            or now_s - self._rollback_started_s
            > self.config.rollback_timeout_s
        ):
            return self._latch_failure(
                "rollback_timeout",
                along=along,
                cross=cross,
                yaw_drift=yaw_drift,
            )
        if self._rollback_braking and along > self.config.rollback_completion_tolerance_m:
            return self._latch_failure("rollback_completion_lost", along=along,
                cross=cross, yaw_drift=yaw_drift)
        return None

    def _step_rollback(
        self,
        *,
        along,
        cross,
        yaw_drift,
        measured_stopped,
        now_s,
    ):
        failure = self._rollback_bounds_failure(along=along, cross=cross,
            yaw_drift=yaw_drift, now_s=now_s)
        if failure is not None:
            return failure
        if along <= self.config.rollback_completion_tolerance_m:
            self._rollback_braking = True
        if self._rollback_braking and not self._last_command.is_zero():
            braking = DockingCommand(linear_x=self._bounded_visual_command(
                0.0, -self.config.max_rollback_speed_mps, 0.0, self._last_command.linear_x,
                self.config.max_rollback_acceleration_mps2, now_s))
            if not braking.is_zero():
                return self._emit(DockingOutput(phase=DockingPhase.ROLLBACK,
                    command=braking, reason="rollback_deceleration", along_track_m=along,
                    cross_track_m=cross, yaw_drift_rad=yaw_drift), now_s)
        if self._rollback_braking:
            self._rollback_stopped_since_s = now_s if measured_stopped else None
            self._set_phase(DockingPhase.ROLLBACK_SETTLING, now_s)
            return self._emit(
                DockingOutput(
                    phase=DockingPhase.ROLLBACK_SETTLING,
                    command=DockingCommand(),
                    reason="rollback_settling",
                    along_track_m=along,
                    cross_track_m=cross,
                    yaw_drift_rad=yaw_drift,
                ),
                now_s,
            )

        return self._emit(
            DockingOutput(
                phase=DockingPhase.ROLLBACK,
                command=DockingCommand(
                    linear_x=self._bounded_rollback_speed(along, now_s)
                ),
                reason="rollback_in_progress",
                along_track_m=along,
                cross_track_m=cross,
                yaw_drift_rad=yaw_drift,
            ),
            now_s,
        )

    def _step_reacquire_tag(self, *, tag, tag_fresh, now_s):
        if self._tag_loss_started_s is None:
            self._tag_loss_started_s = now_s
        if (
            now_s - self._tag_loss_started_s
            > self.config.tag_reacquisition_timeout_s
        ):
            return self._latch_failure(
                "tag_reacquisition_after_rollback_timeout"
            )
        valid_tag = (
            tag_fresh is True
            and isinstance(tag, FilteredTagPose)
            and tag.marker_id == self.requested_marker_id
            and tag.frame_id == self.expected_frame
        )
        if valid_tag:
            try:
                forward_m = _finite(tag.forward_m, "tag_pose")
                lateral_m = _finite(tag.lateral_m, "tag_pose")
                yaw_rad = _normalize_angle(
                    _finite(tag.approach_yaw_rad, "tag_pose")
                )
            except ValueError:
                valid_tag = False
            else:
                valid_tag = (
                    self.config.visual_min_forward_m
                    <= forward_m
                    <= self.config.visual_max_forward_m
                    and abs(lateral_m)
                    <= self.config.visual_max_abs_lateral_m
                    and abs(yaw_rad)
                    <= self.config.visual_max_abs_yaw_rad
                )
        if not valid_tag:
            return self._emit(
                DockingOutput(
                    phase=DockingPhase.REACQUIRE_TAG,
                    command=DockingCommand(),
                    reason="waiting_for_tag_reacquisition_after_rollback",
                ),
                now_s,
            )

        self._tag_loss_started_s = None
        self._visual_started_s = now_s
        self._last_desired_visual_phase = DockingPhase.ALIGN_YAW
        self._set_phase(DockingPhase.ALIGN_YAW, now_s)
        return self._emit(
            DockingOutput(
                phase=DockingPhase.ALIGN_YAW,
                command=DockingCommand(),
                reason="tag_reacquired_after_rollback",
            ),
            now_s,
        )

    def _project_from_commit(self, odom):
        if self._commit_start is None or self._commit_forward_axis is None:
            raise ValueError("commit checkpoint is unavailable")
        projection = project_pose(self._commit_start, odom)
        return (
            projection.along_m,
            projection.cross_m,
            projection.yaw_drift_rad,
        )

    def _clear_completed_attempt(self):
        self._committed_tag_loss_started_s = None
        self._commit_start = None
        self._commit_forward_axis = None
        self._visual_drift_started_s = None
        self._precommit_stopped_since_s = None
        self._handoff_start = None
        self._handoff_distance_m = None
        self._pre_handoff_lateral_m = 0.0
        self._pre_handoff_yaw_rad = 0.0
        self._clear_handoff_settling_evidence()
        self._verify_stopped_since_s = None
        self._rollback_trigger_reason = None
        self._blind_braking = False
        self._rollback_braking = False
        self._rollback_stopped_since_s = None
        self._rollback_started_s = None
        self._rollback_start_along_m = None
        self._last_desired_visual_phase = None
        self._visual_phase_transition_times = []
        self._visual_started_s = None

    def _bounded_rollback_speed(self, along, now_s):
        distance_to_checkpoint_m = max(
            0.0,
            along - 0.5 * self.config.rollback_completion_tolerance_m,
        )
        elapsed_s = self._source_dt_s
        previous_mps = max(0.0, -self._last_command.linear_x)
        return -bounded_speed_magnitude(
            distance_to_stop_m=distance_to_checkpoint_m,
            previous_mps=previous_mps,
            maximum_mps=self.config.max_rollback_speed_mps,
            minimum_mps=0.0,
            acceleration_mps2=self.config.max_rollback_acceleration_mps2,
            slowdown_distance_m=self.config.slowdown_distance_m,
            elapsed_s=elapsed_s,
        )

    def _wait_for_tag_reacquisition(self, now_s, failure_reason):
        if self._tag_loss_started_s is None:
            self._tag_loss_started_s = now_s
        elif (
            now_s - self._tag_loss_started_s
            > self.config.tag_reacquisition_timeout_s
        ):
            return self._latch_failure(failure_reason)
        self._phase_started_s = None
        self._last_desired_visual_phase = None
        self._clear_handoff_settling_evidence()
        return self._emit(
            DockingOutput(
                phase=self._phase,
                command=DockingCommand(),
                reason="waiting_for_tag_reacquisition",
            ),
            now_s,
        )

    def _step_blind(self, *, odom, linear_speed, angular_speed, now_s):
        if self._phase_started_s is None:
            self._phase_started_s = now_s
        if self._phase == DockingPhase.ODOMETRIC_FINAL_STRAIGHT:
            if self._phase_timed_out(now_s, self.config.blind_timeout_s):
                return self._latch_failure("blind_phase_timeout")
        elif self._phase == DockingPhase.VERIFY:
            if self._phase_timed_out(now_s, self.config.verify_timeout_s):
                return self._latch_failure("verify_phase_timeout")

        try:
            along, cross, yaw_drift, remaining = self._project_odometry(
                odom
            )
        except (OverflowError, ValueError):
            return self._latch_failure("invalid_odometry_projection")
        if abs(cross) > self.config.max_cross_track_m:
            return self._request_rollback(
                "odometric_alignment_lost_after_commit",
                odom=odom,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                now_s=now_s,
                remaining=remaining,
                along=along,
                cross=cross,
                yaw_drift=yaw_drift,
            )
        if abs(yaw_drift) > self.config.max_yaw_drift_rad:
            return self._request_rollback(
                "odometric_alignment_lost_after_commit",
                odom=odom,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                now_s=now_s,
                remaining=remaining,
                along=along,
                cross=cross,
                yaw_drift=yaw_drift,
            )
        if remaining < -self.config.max_overshoot_m:
            return self._latch_failure(
                "blind_overshoot",
                remaining,
                along,
                cross,
                yaw_drift,
            )
        if remaining < -self.config.position_tolerance_m:
            return self._latch_failure(
                "final_tolerance_overshoot",
                remaining,
                along,
                cross,
                yaw_drift,
            )
        if (
            (self._phase == DockingPhase.VERIFY or self._blind_braking)
            and abs(remaining) > self.config.position_tolerance_m
        ):
            return self._latch_failure(
                "final_tolerance_lost",
                remaining,
                along,
                cross,
                yaw_drift,
            )

        # Enter verification well inside the commissioned outer tolerance.
        # The unused band is a deterministic reserve for measured chassis
        # settling after the command becomes zero.
        verification_entry_tolerance_m = (
            0.25 * self.config.position_tolerance_m
        )
        if abs(remaining) <= verification_entry_tolerance_m:
            self._blind_braking = True
        if self._blind_braking and not self._last_command.is_zero():
            braking = DockingCommand(linear_x=self._bounded_visual_command(
                0.0, 0.0, self.config.max_blind_speed_mps, self._last_command.linear_x,
                self.config.max_blind_acceleration_mps2, now_s))
            if not braking.is_zero():
                return self._emit(DockingOutput(phase=DockingPhase.ODOMETRIC_FINAL_STRAIGHT,
                    command=braking, reason="blind_deceleration", remaining_distance_m=remaining,
                    along_track_m=along, cross_track_m=cross, yaw_drift_rad=yaw_drift), now_s)
        if self._phase == DockingPhase.VERIFY or self._blind_braking:
            self._set_phase(DockingPhase.VERIFY, now_s)
            return self._verify_stop(
                now_s=now_s,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                remaining=remaining,
                along=along,
                cross=cross,
                yaw_drift=yaw_drift,
            )

        forward_speed = self._bounded_blind_speed(remaining, now_s)
        command = DockingCommand(linear_x=forward_speed)
        return self._emit(
            DockingOutput(
                phase=DockingPhase.ODOMETRIC_FINAL_STRAIGHT,
                command=command,
                reason="odometric_final",
                remaining_distance_m=remaining,
                along_track_m=along,
                cross_track_m=cross,
                yaw_drift_rad=yaw_drift,
            ),
            now_s,
        )

    def _project_odometry(self, odom):
        start = self._handoff_start
        projection = project_pose(start, odom)
        along = projection.along_m
        cross = projection.cross_m
        yaw_drift = projection.yaw_drift_rad
        remaining = _finite(
            self._handoff_distance_m - along,
            "odometry_projection_remaining",
        )
        return along, cross, yaw_drift, remaining

    def _bounded_visual_command(
        self,
        target,
        lower,
        upper,
        previous,
        acceleration_limit,
        now_s,
    ):
        target = _clamp(target, lower, upper)
        elapsed = self._source_dt_s
        maximum_change = acceleration_limit * elapsed
        return _clamp(
            target,
            previous - maximum_change,
            previous + maximum_change,
        )

    def _decelerated_visual_command(self, now_s):
        elapsed = self._source_dt_s

        def toward_zero(value, acceleration_limit):
            maximum_change = acceleration_limit * elapsed
            if abs(value) <= maximum_change + 1e-12:
                return 0.0
            return value - math.copysign(maximum_change, value)

        return DockingCommand(
            linear_x=toward_zero(
                self._last_command.linear_x,
                self.config.max_visual_linear_acceleration_mps2,
            ),
            linear_y=toward_zero(
                self._last_command.linear_y,
                self.config.max_visual_linear_acceleration_mps2,
            ),
            angular_z=toward_zero(
                self._last_command.angular_z,
                self.config.max_visual_angular_acceleration_radps2,
            ),
        )

    def _bounded_blind_speed(self, remaining, now_s):
        verification_entry_tolerance_m = (
            0.25 * self.config.position_tolerance_m
        )
        elapsed = self._source_dt_s
        previous = max(0.0, self._last_command.linear_x)
        return bounded_speed_magnitude(
            distance_to_stop_m=max(
                0.0,
                remaining - 0.5 * verification_entry_tolerance_m,
            ),
            previous_mps=previous,
            maximum_mps=self.config.max_blind_speed_mps,
            minimum_mps=0.0,
            acceleration_mps2=self.config.max_blind_acceleration_mps2,
            slowdown_distance_m=self.config.slowdown_distance_m,
            elapsed_s=elapsed,
        )

    def _verify_stop(
        self,
        *,
        now_s,
        linear_speed,
        angular_speed,
        remaining,
        along,
        cross,
        yaw_drift,
    ):
        measured_stopped = (
            linear_speed <= self.config.stop_linear_speed_mps
            and angular_speed <= self.config.stop_angular_speed_radps
        )
        (
            self._verify_stopped_since_s,
            dwell_complete,
        ) = advance_dwell(
            self._verify_stopped_since_s,
            measured_stopped,
            now_s,
            self.config.stop_dwell_s,
        )
        if not dwell_complete:
            return self._emit(
                DockingOutput(
                    phase=DockingPhase.VERIFY,
                    command=DockingCommand(),
                    reason="verify_stop_dwell",
                    remaining_distance_m=remaining,
                    along_track_m=along,
                    cross_track_m=cross,
                    yaw_drift_rad=yaw_drift,
                ),
                now_s,
            )

        lateral_error = self._pre_handoff_lateral_m + cross
        yaw_error = _normalize_angle(self._pre_handoff_yaw_rad + yaw_drift)
        estimate = FinalErrorEstimate(
            error=PlanarError(
                position_m=math.hypot(remaining, lateral_error),
                yaw_rad=yaw_error,
            ),
            is_estimated=True,
            source="pre_handoff_alvar_plus_odom",
        )
        output = DockingOutput(
            phase=DockingPhase.SUCCEEDED,
            command=DockingCommand(),
            reason="succeeded",
            terminal=True,
            succeeded=True,
            remaining_distance_m=remaining,
            along_track_m=along,
            cross_track_m=cross,
            yaw_drift_rad=yaw_drift,
            final_error_estimate=estimate,
            attempt_index=1 + self._rollback_attempts_completed,
        )
        self._phase = DockingPhase.SUCCEEDED
        self._committed_tag_loss_started_s = None
        self._terminal_output = output
        self._last_command = DockingCommand()
        self._last_step_s = self._source_now_s
        return output

    def _phase_timed_out(self, now_s, timeout_s):
        return (
            self._phase_started_s is not None
            and now_s - self._phase_started_s > timeout_s
        )

    def _set_phase(self, phase, now_s):
        if phase == self._phase:
            if self._phase_started_s is None and now_s is not None:
                self._phase_started_s = now_s
            return
        self._phase = phase
        self._phase_started_s = now_s
        if phase != DockingPhase.HANDOFF_SETTLING:
            self._clear_handoff_settling_evidence()
        if phase != DockingPhase.VERIFY:
            self._verify_stopped_since_s = None

    def _clear_handoff_settling_evidence(self):
        self._handoff_evidence_ready = False
        self._handoff_stopped_since_s = None
        self._handoff_settled_odom = None

    def _emit(self, output, now_s):
        if not output.command.is_zero():
            if self._last_step_s is not None and self._source_now_s == self._last_step_s:
                # This safety/phase poll has no new physical time for normal
                # generation. Keep the original tuple/time and command lease;
                # the retained tuple still receives directional LiDAR checks.
                return replace(output, command=DockingCommand(), command_generated=False,
                    attempt_index=1 + self._rollback_attempts_completed)
            if output.phase == DockingPhase.ODOMETRIC_FINAL_STRAIGHT:
                change = DockingCommand(
                    linear_x=self.config.max_blind_acceleration_mps2 * self._source_dt_s)
            elif output.phase == DockingPhase.ROLLBACK:
                change = DockingCommand(
                    linear_x=self.config.max_rollback_acceleration_mps2 * self._source_dt_s)
            else:
                change = DockingCommand(
                    linear_x=self.config.max_visual_linear_acceleration_mps2 * self._source_dt_s,
                    linear_y=self.config.max_visual_linear_acceleration_mps2 * self._source_dt_s,
                    angular_z=self.config.max_visual_angular_acceleration_radps2 * self._source_dt_s)
            shaped = bound_reachable_command(
                self._last_command, output.command, self._progress, change)
            if shaped is None:
                previous, source_now, source_dt = self._last_command, self._source_now_s, self._source_dt_s
                failed = self._latch_failure("motion_progress_infeasible",
                    output.remaining_distance_m, output.along_track_m,
                    output.cross_track_m, output.yaw_drift_rad)
                try:
                    self._motion_progress_failure = _MotionProgressFailure(
                        previous, output.command, change, source_now, source_dt,
                        self._wall_now_s, output.phase, self._progress)
                except Exception:
                    pass  # Optional evidence cannot replace the terminal zero.
                return failed
            output = replace(output, command=shaped)
        output = replace(
            output,
            attempt_index=1 + self._rollback_attempts_completed,
        )
        self._last_command = output.command
        self._last_step_s = self._source_now_s
        return output

    def _latch_cancelled(self):
        output = DockingOutput(
            phase=DockingPhase.CANCELLED,
            command=DockingCommand(),
            reason="cancelled",
            terminal=True,
            attempt_index=1 + self._rollback_attempts_completed,
        )
        self._phase = DockingPhase.CANCELLED
        self._committed_tag_loss_started_s = None
        self._terminal_output = output
        self._last_command = DockingCommand()
        self._last_step_s = None
        self._source_dt_s = 0.0
        self._clear_handoff_settling_evidence()
        return output

    def _latch_failure(
        self,
        reason,
        remaining=None,
        along=None,
        cross=None,
        yaw_drift=None,
    ):
        output = DockingOutput(
            phase=DockingPhase.FAILED,
            command=DockingCommand(),
            reason=reason,
            terminal=True,
            remaining_distance_m=remaining,
            along_track_m=along,
            cross_track_m=cross,
            yaw_drift_rad=yaw_drift,
            attempt_index=1 + self._rollback_attempts_completed,
        )
        self._phase = DockingPhase.FAILED
        self._committed_tag_loss_started_s = None
        self._terminal_output = output
        self._last_command = DockingCommand()
        self._last_step_s = None
        self._source_dt_s = 0.0
        self._clear_handoff_settling_evidence()
        return output
