"""Robust, ROS-independent floor-tag to odometry handoff geometry."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from numbers import Real
from statistics import median
import threading
from typing import Optional, Sequence
import weakref

from .tag_filter import FilteredTagPose
from .parking_target import final_tag_distance


@dataclass(frozen=True)
class HandoffCommand:
    """A command-shaped value that is always zero at the handoff boundary."""

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
class HandoffUncertainty:
    """Median absolute deviations and sample accounting for the final window."""

    forward_mad_m: float
    lateral_mad_m: float
    floor_height_mad_m: float
    yaw_mad_rad: float
    inlier_count: int
    rejected_count: int
    sample_span_s: float


_HANDOFF_DECISION_CONSTRUCTION_SEAL = object()
_HANDOFF_FRAME = "base_link"
_HANDOFF_AUTHORIZATION_LOCK = threading.Lock()
_HANDOFF_AUTHORIZATIONS = weakref.WeakKeyDictionary()


class _HandoffAuthorization:
    __slots__ = ("__weakref__",)

    def __init__(self, construction_seal):
        if construction_seal is not _HANDOFF_DECISION_CONSTRUCTION_SEAL:
            raise TypeError("handoff authorization is internal")


@dataclass(frozen=True, init=False)
class HandoffDecision:
    """Fail-closed result used to authorize the odometric final approach."""

    accepted: bool
    reason: str
    filtered_pose: Optional[FilteredTagPose]
    desired_final_tag_x_m: Optional[float]
    remaining_distance_m: Optional[float]
    uncertainty: HandoffUncertainty
    command: HandoffCommand = HandoffCommand()
    _authorization: Optional[_HandoffAuthorization] = field(
        default=None,
        repr=False,
        compare=False,
    )

    def __init__(self, *, _construction_seal=None):
        if _construction_seal is not _HANDOFF_DECISION_CONSTRUCTION_SEAL:
            raise TypeError(
                "HandoffDecision values are created by parking geometry"
            )


def _decision_payload_fingerprint(decision):
    filtered = decision.filtered_pose
    if filtered is None:
        filtered_fingerprint = None
    elif not isinstance(filtered, FilteredTagPose):
        filtered_fingerprint = (
            "invalid_filtered_pose",
            type(filtered).__module__,
            type(filtered).__qualname__,
            id(filtered),
        )
    else:
        filtered_fingerprint = (
            filtered.marker_id,
            filtered.stamp_s,
            filtered.frame_id,
            filtered.forward_m,
            filtered.lateral_m,
            filtered.floor_height_m,
            filtered.approach_yaw_rad,
            filtered.forward_residual_m,
            filtered.lateral_residual_m,
            filtered.floor_height_residual_m,
            filtered.floor_normal_residual_rad,
            filtered.approach_yaw_residual_rad,
        )
    uncertainty = decision.uncertainty
    command = decision.command
    uncertainty_fingerprint = (
        (
            "invalid_uncertainty",
            type(uncertainty).__module__,
            type(uncertainty).__qualname__,
            id(uncertainty),
        )
        if not isinstance(uncertainty, HandoffUncertainty)
        else (
            uncertainty.forward_mad_m,
            uncertainty.lateral_mad_m,
            uncertainty.floor_height_mad_m,
            uncertainty.yaw_mad_rad,
            uncertainty.inlier_count,
            uncertainty.rejected_count,
            uncertainty.sample_span_s,
        )
    )
    command_fingerprint = (
        (
            "invalid_command",
            type(command).__module__,
            type(command).__qualname__,
            id(command),
        )
        if not isinstance(command, HandoffCommand)
        else (command.linear_x, command.linear_y, command.angular_z)
    )
    return (
        decision.accepted,
        decision.reason,
        filtered_fingerprint,
        decision.desired_final_tag_x_m,
        decision.remaining_distance_m,
        uncertainty_fingerprint,
        command_fingerprint,
    )


def _make_decision(
    *,
    accepted,
    reason,
    filtered_pose,
    desired_final_tag_x_m,
    remaining_distance_m,
    uncertainty,
    command=None,
    authorize=False,
):
    decision = HandoffDecision(
        _construction_seal=_HANDOFF_DECISION_CONSTRUCTION_SEAL
    )
    values = {
        "accepted": accepted,
        "reason": reason,
        "filtered_pose": filtered_pose,
        "desired_final_tag_x_m": desired_final_tag_x_m,
        "remaining_distance_m": remaining_distance_m,
        "uncertainty": uncertainty,
        "command": HandoffCommand() if command is None else command,
        "_authorization": None,
    }
    for field_name, value in values.items():
        object.__setattr__(decision, field_name, value)
    if authorize:
        authorization = _HandoffAuthorization(
            _HANDOFF_DECISION_CONSTRUCTION_SEAL
        )
        object.__setattr__(decision, "_authorization", authorization)
        with _HANDOFF_AUTHORIZATION_LOCK:
            _HANDOFF_AUTHORIZATIONS[authorization] = (
                _decision_payload_fingerprint(decision),
                False,
            )
    return decision


def _handoff_authorization_status(decision):
    if not isinstance(decision, HandoffDecision):
        return "unauthorized"
    authorization = getattr(decision, "_authorization", None)
    if not isinstance(authorization, _HandoffAuthorization):
        return "unauthorized"
    with _HANDOFF_AUTHORIZATION_LOCK:
        record = _HANDOFF_AUTHORIZATIONS.get(authorization)
        if record is None:
            return "unauthorized"
        return "consumed" if record[1] else "authorized"


def _consume_handoff_authorization(decision):
    authorization = getattr(decision, "_authorization", None)
    if not isinstance(authorization, _HandoffAuthorization):
        return "unauthorized"
    with _HANDOFF_AUTHORIZATION_LOCK:
        record = _HANDOFF_AUTHORIZATIONS.get(authorization)
        if record is None:
            return "unauthorized"
        if _decision_payload_fingerprint(decision) != record[0]:
            return "payload_mismatch"
        if record[1]:
            return "consumed"
        _HANDOFF_AUTHORIZATIONS[authorization] = (record[0], True)
        return "consumed_now"


_ZERO_UNCERTAINTY = HandoffUncertainty(
    forward_mad_m=0.0,
    lateral_mad_m=0.0,
    floor_height_mad_m=0.0,
    yaw_mad_rad=0.0,
    inlier_count=0,
    rejected_count=0,
    sample_span_s=0.0,
)


def _rejected(reason, uncertainty=_ZERO_UNCERTAINTY):
    return _make_decision(
        accepted=False,
        reason=reason,
        filtered_pose=None,
        desired_final_tag_x_m=None,
        remaining_distance_m=None,
        uncertainty=uncertainty,
    )


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        converted = float(value)
    except (OverflowError, TypeError, ValueError):
        return None
    if not math.isfinite(converted):
        return None
    return converted


def _angle_normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_distance(left, right):
    return abs(_angle_normalize(left - right))


def _circular_mean(values):
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    if math.isclose(sine, 0.0, abs_tol=1e-15) and math.isclose(
        cosine, 0.0, abs_tol=1e-15
    ):
        return _angle_normalize(float(median(values)))
    return math.atan2(sine, cosine)


def _mad(values, center):
    return float(median(abs(value - center) for value in values))


def _yaw_mad(values, center):
    return float(median(_angle_distance(value, center) for value in values))


def _valid_sample(sample):
    if not isinstance(sample, FilteredTagPose):
        return False
    if (
        isinstance(sample.marker_id, bool)
        or not isinstance(sample.marker_id, int)
        or not isinstance(sample.frame_id, str)
        or not sample.frame_id.strip()
    ):
        return False
    numeric_fields = (
        "stamp_s",
        "forward_m",
        "lateral_m",
        "floor_height_m",
        "approach_yaw_rad",
        "forward_residual_m",
        "lateral_residual_m",
        "floor_height_residual_m",
        "floor_normal_residual_rad",
        "approach_yaw_residual_rad",
    )
    for field_name in numeric_fields:
        converted = _finite(getattr(sample, field_name))
        if converted is None:
            return False
        if "residual" in field_name and converted < 0.0:
            return False
    return True


def _robust_inliers(
    samples,
    outlier_mad_scale,
    outlier_translation_floor_m,
    outlier_yaw_floor_rad,
):
    forward_center = float(median(sample.forward_m for sample in samples))
    lateral_center = float(median(sample.lateral_m for sample in samples))
    floor_center = float(median(sample.floor_height_m for sample in samples))
    yaw_center = _circular_mean(
        [sample.approach_yaw_rad for sample in samples]
    )
    forward_mad = _mad(
        [sample.forward_m for sample in samples], forward_center
    )
    lateral_mad = _mad(
        [sample.lateral_m for sample in samples], lateral_center
    )
    floor_mad = _mad(
        [sample.floor_height_m for sample in samples], floor_center
    )
    yaw_mad = _yaw_mad(
        [sample.approach_yaw_rad for sample in samples], yaw_center
    )

    def within(value, center, value_mad):
        return abs(value - center) <= outlier_mad_scale * max(
            value_mad,
            outlier_translation_floor_m,
        )

    inliers = []
    for sample in samples:
        if not within(sample.forward_m, forward_center, forward_mad):
            continue
        if not within(sample.lateral_m, lateral_center, lateral_mad):
            continue
        if not within(sample.floor_height_m, floor_center, floor_mad):
            continue
        if (
            _angle_distance(sample.approach_yaw_rad, yaw_center)
            > outlier_mad_scale * max(yaw_mad, outlier_yaw_floor_rad)
        ):
            continue
        inliers.append(sample)
    return tuple(inliers)


def _combine(inliers, total_count, sample_span_s):
    forward = float(median(sample.forward_m for sample in inliers))
    lateral = float(median(sample.lateral_m for sample in inliers))
    floor_height = float(median(sample.floor_height_m for sample in inliers))
    yaw = _circular_mean([sample.approach_yaw_rad for sample in inliers])
    uncertainty = HandoffUncertainty(
        forward_mad_m=_mad(
            [sample.forward_m for sample in inliers], forward
        ),
        lateral_mad_m=_mad(
            [sample.lateral_m for sample in inliers], lateral
        ),
        floor_height_mad_m=_mad(
            [sample.floor_height_m for sample in inliers], floor_height
        ),
        yaw_mad_rad=_yaw_mad(
            [sample.approach_yaw_rad for sample in inliers], yaw
        ),
        inlier_count=len(inliers),
        rejected_count=total_count - len(inliers),
        sample_span_s=sample_span_s,
    )
    filtered = FilteredTagPose(
        marker_id=inliers[0].marker_id,
        stamp_s=max(sample.stamp_s for sample in inliers),
        frame_id=inliers[0].frame_id,
        forward_m=forward,
        lateral_m=lateral,
        floor_height_m=floor_height,
        approach_yaw_rad=yaw,
        forward_residual_m=max(
            sample.forward_residual_m for sample in inliers
        ),
        lateral_residual_m=max(
            sample.lateral_residual_m for sample in inliers
        ),
        floor_height_residual_m=max(
            sample.floor_height_residual_m for sample in inliers
        ),
        floor_normal_residual_rad=max(
            sample.floor_normal_residual_rad for sample in inliers
        ),
        approach_yaw_residual_rad=max(
            sample.approach_yaw_residual_rad for sample in inliers
        ),
    )
    return filtered, uncertainty


def handoff_for_remaining_distance(
    remaining_distance_m,
    max_blind_travel_m=0.60,
    absolute_max_blind_travel_m=0.75,
):
    """Validate a precomputed blind distance without authorizing any command."""

    remaining = _finite(remaining_distance_m)
    maximum = _finite(max_blind_travel_m)
    absolute = _finite(absolute_max_blind_travel_m)
    if absolute is None or absolute <= 0.0:
        return _rejected("invalid_absolute_blind_travel_limit")
    if maximum is None or maximum <= 0.0 or maximum > absolute:
        return _rejected("blind_travel_limit_not_commissioned")
    if remaining is None or remaining <= 0.0:
        return _rejected("nonpositive_remaining_distance")
    if remaining > maximum:
        return _rejected("blind_travel_limit")
    return _make_decision(
        accepted=True,
        reason="accepted",
        filtered_pose=None,
        desired_final_tag_x_m=None,
        remaining_distance_m=remaining,
        uncertainty=_ZERO_UNCERTAINTY,
    )


def build_handoff(
    *,
    samples,
    requested_marker_id,
    expected_frame,
    front_reference_x_m,
    front_reference_state,
    operation_mode,
    parking_safety_distance_m,
    max_blind_travel_m,
    now_s,
    rover_settled,
    odom_fresh,
    lidar_fresh,
    lidar_forward_clear,
    longitudinal_docking_offset_m=None,
    marker_size_m=None,
    parking_target=None,
    absolute_max_blind_travel_m=0.75,
    commissioning=False,
    required_samples=8,
    min_span_s=0.35,
    max_sample_age_s=0.25,
    outlier_mad_scale=3.5,
    outlier_translation_floor_m=0.0005,
    outlier_yaw_floor_rad=0.001,
    max_translation_mad_m=0.010,
    max_yaw_mad_rad=0.015,
    max_lateral_error_m=0.010,
    max_yaw_error_rad=0.035,
    max_forward_residual_m=0.025,
    max_lateral_residual_m=0.025,
    max_floor_height_residual_m=0.015,
    max_floor_normal_residual_rad=0.080,
    max_approach_yaw_residual_rad=0.040,
):
    """Combine the final accepted tag window and gate an odometric handoff."""

    if (
        isinstance(requested_marker_id, bool)
        or not isinstance(requested_marker_id, int)
    ):
        return _rejected("invalid_requested_marker_id")
    if (
        not isinstance(expected_frame, str)
        or not expected_frame
        or expected_frame != expected_frame.strip()
    ):
        return _rejected("invalid_expected_frame")
    if expected_frame != _HANDOFF_FRAME:
        return _rejected("expected_frame_must_be_base_link")

    front_reference = _finite(front_reference_x_m)
    if front_reference is None:
        return _rejected("missing_front_reference")
    safety_distance = _finite(parking_safety_distance_m)
    if safety_distance is None or safety_distance < 0.0:
        return _rejected("invalid_parking_safety_distance")
    absolute = _finite(absolute_max_blind_travel_m)
    if absolute is None or absolute <= 0.0:
        return _rejected("invalid_absolute_blind_travel_limit")
    maximum = _finite(max_blind_travel_m)
    if maximum is None or maximum <= 0.0 or maximum > absolute:
        return _rejected("blind_travel_limit_not_commissioned")

    if operation_mode not in ("simulation", "hardware", "commissioning"):
        return _rejected("invalid_operation_mode")
    reference_allowed = front_reference_state == "verified"
    if operation_mode == "simulation" and front_reference_state == "simulation_only":
        reference_allowed = True
    if (
        operation_mode == "commissioning"
        and commissioning is True
        and front_reference_state == "simulation_only"
    ):
        reference_allowed = True
    if not reference_allowed:
        return _rejected("front_reference_not_verified")

    if isinstance(required_samples, bool) or not isinstance(required_samples, int):
        return _rejected("invalid_required_samples")
    if required_samples < 2:
        return _rejected("invalid_required_samples")
    numeric_limits = {
        "invalid_minimum_sample_span": (min_span_s, False),
        "invalid_sample_age": (max_sample_age_s, False),
        "invalid_outlier_scale": (outlier_mad_scale, False),
        "invalid_outlier_translation_floor": (
            outlier_translation_floor_m,
            False,
        ),
        "invalid_outlier_yaw_floor": (outlier_yaw_floor_rad, False),
        "invalid_translation_dispersion_limit": (
            max_translation_mad_m,
            True,
        ),
        "invalid_yaw_dispersion_limit": (max_yaw_mad_rad, True),
        "invalid_lateral_limit": (max_lateral_error_m, True),
        "invalid_yaw_limit": (max_yaw_error_rad, True),
        "invalid_forward_residual_limit": (max_forward_residual_m, True),
        "invalid_lateral_residual_limit": (max_lateral_residual_m, True),
        "invalid_floor_height_residual_limit": (
            max_floor_height_residual_m,
            True,
        ),
        "invalid_floor_normal_residual_limit": (
            max_floor_normal_residual_rad,
            True,
        ),
        "invalid_yaw_residual_limit": (
            max_approach_yaw_residual_rad,
            True,
        ),
    }
    converted_limits = {}
    for reason, (value, allow_zero) in numeric_limits.items():
        converted = _finite(value)
        if converted is None or converted < 0.0 or (
            not allow_zero and converted == 0.0
        ):
            return _rejected(reason)
        converted_limits[reason] = converted

    if isinstance(samples, (str, bytes)) or not isinstance(samples, Sequence):
        return _rejected("invalid_samples")
    sample_window = tuple(samples)
    if len(sample_window) < required_samples:
        return _rejected("insufficient_samples")
    if not all(_valid_sample(sample) for sample in sample_window):
        return _rejected("invalid_samples")
    if any(sample.marker_id != requested_marker_id for sample in sample_window):
        return _rejected("unexpected_marker_id")
    if any(sample.frame_id != expected_frame for sample in sample_window):
        return _rejected("unexpected_sample_frame")
    stamps = [float(sample.stamp_s) for sample in sample_window]
    if len(stamps) != len(set(stamps)):
        return _rejected("duplicate_samples")
    if any(current <= previous for previous, current in zip(stamps, stamps[1:])):
        return _rejected("nonmonotonic_samples")
    span = stamps[-1] - stamps[0]
    if span < converted_limits["invalid_minimum_sample_span"]:
        return _rejected("insufficient_sample_span")
    now = _finite(now_s)
    if now is None or stamps[-1] > now:
        return _rejected("invalid_sample_clock")
    if now - stamps[-1] > converted_limits["invalid_sample_age"]:
        return _rejected("stale_samples")

    inliers = _robust_inliers(
        sample_window,
        converted_limits["invalid_outlier_scale"],
        converted_limits["invalid_outlier_translation_floor"],
        converted_limits["invalid_outlier_yaw_floor"],
    )
    if len(inliers) < required_samples:
        uncertainty = HandoffUncertainty(
            forward_mad_m=0.0,
            lateral_mad_m=0.0,
            floor_height_mad_m=0.0,
            yaw_mad_rad=0.0,
            inlier_count=len(inliers),
            rejected_count=len(sample_window) - len(inliers),
            sample_span_s=span,
        )
        return _rejected("insufficient_inlier_samples", uncertainty)
    filtered, uncertainty = _combine(inliers, len(sample_window), span)
    if max(
        uncertainty.forward_mad_m,
        uncertainty.lateral_mad_m,
        uncertainty.floor_height_mad_m,
    ) > converted_limits["invalid_translation_dispersion_limit"]:
        return _rejected("excessive_translation_dispersion", uncertainty)
    if uncertainty.yaw_mad_rad > converted_limits["invalid_yaw_dispersion_limit"]:
        return _rejected("excessive_yaw_dispersion", uncertainty)
    if abs(filtered.lateral_m) > converted_limits["invalid_lateral_limit"]:
        return _rejected("lateral_error", uncertainty)
    if (
        _angle_distance(filtered.approach_yaw_rad, 0.0)
        > converted_limits["invalid_yaw_limit"]
    ):
        return _rejected("yaw_error", uncertainty)
    residual_gates = (
        (
            filtered.forward_residual_m,
            converted_limits["invalid_forward_residual_limit"],
            "forward_residual",
        ),
        (
            filtered.lateral_residual_m,
            converted_limits["invalid_lateral_residual_limit"],
            "lateral_residual",
        ),
        (
            filtered.floor_height_residual_m,
            converted_limits["invalid_floor_height_residual_limit"],
            "floor_height_residual",
        ),
        (
            filtered.floor_normal_residual_rad,
            converted_limits["invalid_floor_normal_residual_limit"],
            "floor_normal_residual",
        ),
        (
            filtered.approach_yaw_residual_rad,
            converted_limits["invalid_yaw_residual_limit"],
            "approach_yaw_residual",
        ),
    )
    for value, limit, reason in residual_gates:
        if value > limit:
            return _rejected(reason, uncertainty)
    if rover_settled is not True:
        return _rejected("rover_not_settled", uncertainty)
    if odom_fresh is not True:
        return _rejected("odometry_not_fresh", uncertainty)
    if lidar_fresh is not True:
        return _rejected("lidar_not_fresh", uncertainty)
    if lidar_forward_clear is not True:
        return _rejected("lidar_not_clear", uncertainty)

    if longitudinal_docking_offset_m is not None and _finite(longitudinal_docking_offset_m) is None:
        return _rejected("invalid_longitudinal_docking_offset")
    try:
        desired_final_tag_x = final_tag_distance(
            front_reference_x_m=front_reference,
            parking_safety_distance_m=safety_distance,
            longitudinal_docking_offset_m=longitudinal_docking_offset_m,
            marker_size_m=marker_size_m, parking_target=parking_target)
    except ValueError:
        return _rejected("invalid_parking_target" if parking_target is not None
                         else "final_tag_behind_base")
    remaining = filtered.forward_m - desired_final_tag_x
    distance_decision = handoff_for_remaining_distance(
        remaining,
        max_blind_travel_m=maximum,
        absolute_max_blind_travel_m=absolute,
    )
    if not distance_decision.accepted:
        return _rejected(distance_decision.reason, uncertainty)
    return _make_decision(
        accepted=True,
        reason="accepted",
        filtered_pose=filtered,
        desired_final_tag_x_m=desired_final_tag_x,
        remaining_distance_m=remaining,
        uncertainty=uncertainty,
        authorize=True,
    )
