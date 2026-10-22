"""Validated contracts shared by the deployable docking controller."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Optional

from .contracts import FinalErrorEstimate


class DockingPhase(str, Enum):
    """Exclusive controller phases, including latched terminal states."""

    ALIGN_YAW = "ALIGN_YAW"
    ALIGN_LATERAL = "ALIGN_LATERAL"
    PRECOMMIT_SETTLING = "PRECOMMIT_SETTLING"
    FORWARD_COMMITTED = "FORWARD_COMMITTED"
    HANDOFF_SETTLING = "HANDOFF_SETTLING"
    ODOMETRIC_FINAL_STRAIGHT = "ODOMETRIC_FINAL_STRAIGHT"
    ROLLBACK_STOPPING = "ROLLBACK_STOPPING"
    ROLLBACK = "ROLLBACK"
    ROLLBACK_SETTLING = "ROLLBACK_SETTLING"
    REACQUIRE_TAG = "REACQUIRE_TAG"
    VERIFY = "VERIFY"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class DockingCommand:
    """ROS-independent body-frame velocity command."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    def is_zero(self):
        return (
            self.linear_x == 0.0
            and self.linear_y == 0.0
            and self.angular_z == 0.0
        )


@dataclass(frozen=True)
class DockingConfig:
    """Commissioned limits for visual alignment and blind travel."""

    yaw_tolerance_rad: float = 0.035
    lateral_tolerance_m: float = 0.010
    yaw_realign_threshold_rad: float = 0.070
    lateral_realign_threshold_m: float = 0.020
    visual_handoff_x_m: float = 2.05
    visual_handoff_capture_tolerance_m: float = 0.02
    visual_min_forward_m: float = 0.40
    visual_max_forward_m: float = 2.20
    visual_max_abs_lateral_m: float = 0.65
    visual_max_abs_yaw_rad: float = 0.60
    max_blind_travel_m: float = 1.70
    max_blind_speed_mps: float = 0.20
    min_blind_speed_mps: float = 0.01
    max_blind_acceleration_mps2: float = 0.15
    precommit_settle_dwell_s: float = 0.50
    visual_tag_loss_dwell_s: float = 0.25
    visual_drift_dwell_s: float = 0.25
    odometry_loss_dwell_s: float = 0.40
    max_rollback_attempts: int = 2
    rollback_completion_tolerance_m: float = 0.020
    rollback_overrun_allowance_m: float = 0.050
    max_rollback_travel_m: float = 2.0
    max_rollback_speed_mps: float = 0.05
    min_rollback_speed_mps: float = 0.01
    max_rollback_acceleration_mps2: float = 0.10
    rollback_timeout_s: float = 50.0
    position_tolerance_m: float = 0.015
    max_cross_track_m: float = 0.025
    max_yaw_drift_rad: float = 0.035
    handoff_settle_dwell_s: float = 0.50
    handoff_evidence_max_age_s: float = 0.35
    tag_reacquisition_timeout_s: float = 5.0
    stop_dwell_s: float = 0.50
    yaw_gain: float = 0.80
    lateral_gain: float = 0.80
    visual_forward_gain: float = 0.80
    min_yaw_speed_radps: float = 0.05
    max_yaw_speed_radps: float = 0.20
    max_lateral_speed_mps: float = 0.08
    max_visual_forward_speed_mps: float = 0.08
    max_visual_linear_acceleration_mps2: float = 0.15
    max_visual_angular_acceleration_radps2: float = 0.40
    slowdown_distance_m: float = 0.25
    nominal_control_period_s: float = 0.10
    max_overshoot_m: float = 0.020
    stop_linear_speed_mps: float = 0.020
    stop_angular_speed_radps: float = 0.010
    visual_phase_timeout_s: float = 5.0
    visual_timeout_s: float = 15.0
    max_visual_phase_transitions: int = 6
    visual_transition_window_s: float = 3.0
    blind_timeout_s: float = 45.0
    verify_timeout_s: float = 5.0

    def __post_init__(self):
        positive_fields = (
            "yaw_tolerance_rad",
            "lateral_tolerance_m",
            "yaw_realign_threshold_rad",
            "lateral_realign_threshold_m",
            "visual_handoff_x_m",
            "visual_handoff_capture_tolerance_m",
            "visual_min_forward_m",
            "visual_max_forward_m",
            "visual_max_abs_lateral_m",
            "visual_max_abs_yaw_rad",
            "max_blind_travel_m",
            "max_blind_speed_mps",
            "min_blind_speed_mps",
            "max_blind_acceleration_mps2",
            "precommit_settle_dwell_s",
            "visual_tag_loss_dwell_s",
            "visual_drift_dwell_s",
            "odometry_loss_dwell_s",
            "rollback_completion_tolerance_m",
            "rollback_overrun_allowance_m",
            "max_rollback_travel_m",
            "max_rollback_speed_mps",
            "min_rollback_speed_mps",
            "max_rollback_acceleration_mps2",
            "rollback_timeout_s",
            "position_tolerance_m",
            "max_cross_track_m",
            "max_yaw_drift_rad",
            "handoff_settle_dwell_s",
            "handoff_evidence_max_age_s",
            "tag_reacquisition_timeout_s",
            "yaw_gain",
            "lateral_gain",
            "visual_forward_gain",
            "min_yaw_speed_radps",
            "max_yaw_speed_radps",
            "max_lateral_speed_mps",
            "max_visual_forward_speed_mps",
            "max_visual_linear_acceleration_mps2",
            "max_visual_angular_acceleration_radps2",
            "slowdown_distance_m",
            "nominal_control_period_s",
            "stop_dwell_s",
            "visual_phase_timeout_s",
            "visual_timeout_s",
            "visual_transition_window_s",
            "blind_timeout_s",
            "verify_timeout_s",
        )
        nonnegative_fields = (
            "max_overshoot_m",
            "stop_linear_speed_mps",
            "stop_angular_speed_radps",
        )
        for field_name in positive_fields:
            value = _finite(getattr(self, field_name), field_name)
            if value <= 0.0:
                raise ValueError("{0} must be positive".format(field_name))
            object.__setattr__(self, field_name, value)
        for field_name in nonnegative_fields:
            value = _finite(getattr(self, field_name), field_name)
            if value < 0.0:
                raise ValueError("{0} must be nonnegative".format(field_name))
            object.__setattr__(self, field_name, value)
        if self.position_tolerance_m >= self.max_blind_travel_m:
            raise ValueError(
                "position_tolerance_m must be less than max_blind_travel_m"
            )
        if not (
            self.yaw_tolerance_rad
            < self.yaw_realign_threshold_rad
            <= self.visual_max_abs_yaw_rad
        ):
            raise ValueError(
                "yaw realign threshold must exceed yaw tolerance and fit "
                "inside the visual workspace"
            )
        if not (
            self.lateral_tolerance_m
            < self.lateral_realign_threshold_m
            <= self.visual_max_abs_lateral_m
        ):
            raise ValueError(
                "lateral realign threshold must exceed lateral tolerance "
                "and fit inside the visual workspace"
            )
        if self.min_yaw_speed_radps > self.max_yaw_speed_radps:
            raise ValueError(
                "min_yaw_speed_radps must not exceed max_yaw_speed_radps"
            )
        if self.min_blind_speed_mps > self.max_blind_speed_mps:
            raise ValueError(
                "min_blind_speed_mps must not exceed max_blind_speed_mps"
            )
        if self.min_rollback_speed_mps > self.max_rollback_speed_mps:
            raise ValueError(
                "min_rollback_speed_mps must not exceed "
                "max_rollback_speed_mps"
            )
        verification_entry_tolerance_m = 0.25 * self.position_tolerance_m
        if math.hypot(
            self.lateral_tolerance_m,
            verification_entry_tolerance_m,
        ) > self.position_tolerance_m:
            raise ValueError(
                "lateral tolerance must fit inside the final planar budget"
            )
        if self.visual_min_forward_m >= self.visual_max_forward_m:
            raise ValueError(
                "visual_min_forward_m must be less than visual_max_forward_m"
            )
        if not (
            self.visual_min_forward_m
            <= self.visual_handoff_x_m
            <= self.visual_max_forward_m
        ):
            raise ValueError(
                "visual_handoff_x_m must be inside the visual forward bounds"
            )
        if (
            self.visual_handoff_x_m
            + self.visual_handoff_capture_tolerance_m
            > self.visual_max_forward_m
        ):
            raise ValueError(
                "visual handoff capture band must be inside the visual "
                "forward bounds"
            )
        if (
            isinstance(self.max_rollback_attempts, bool)
            or not isinstance(self.max_rollback_attempts, int)
            or self.max_rollback_attempts < 0
        ):
            raise ValueError(
                "max_rollback_attempts must be a nonnegative integer"
            )
        minimum_rollback_budget_m = (
            self.visual_max_forward_m
            - self.visual_handoff_x_m
            + self.rollback_overrun_allowance_m
        )
        if self.max_rollback_travel_m < minimum_rollback_budget_m:
            raise ValueError(
                "max_rollback_travel_m must cover the full recoverable "
                "approach"
            )
        if self.rollback_completion_tolerance_m >= self.max_rollback_travel_m:
            raise ValueError(
                "rollback_completion_tolerance_m must be less than "
                "max_rollback_travel_m"
            )
        if (
            isinstance(self.max_visual_phase_transitions, bool)
            or not isinstance(self.max_visual_phase_transitions, int)
            or self.max_visual_phase_transitions <= 0
        ):
            raise ValueError("max_visual_phase_transitions must be positive")


@dataclass(frozen=True)
class DockingOutput:
    """One deterministic controller decision.

    When command_generated is false, command is a zero placeholder, not an
    emitted stop or stopped evidence. Publishers must not renew a proposal.
    """

    phase: DockingPhase
    command: DockingCommand
    reason: str
    terminal: bool = False
    succeeded: bool = False
    remaining_distance_m: Optional[float] = None
    along_track_m: Optional[float] = None
    cross_track_m: Optional[float] = None
    yaw_drift_rad: Optional[float] = None
    final_error_estimate: Optional[FinalErrorEstimate] = None
    attempt_index: int = 1
    command_generated: bool = True

    @property
    def final_error_is_estimated(self):
        return (
            self.final_error_estimate is not None
            and self.final_error_estimate.is_estimated
        )

    @property
    def final_error_source(self):
        if self.final_error_estimate is None:
            return None
        return self.final_error_estimate.source


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
