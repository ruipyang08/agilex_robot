"""ROS-independent robust floor-marker registration."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from statistics import median
from typing import Iterable, Mapping, Optional, Sequence, Tuple

from .contracts import PlanarPose
from .parking_target import final_tag_distance


class RegistrationError(ValueError):
    """Registration evidence is insufficient or violates its limits."""


@dataclass(frozen=True)
class Pose3D:
    """A rigid transform represented by translation and x-y-z-w quaternion."""

    translation_m: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]


@dataclass(frozen=True)
class RegistrationSample:
    """Synchronized map-to-base and base-to-marker evidence."""

    stamp_s: float
    map_to_base: Pose3D
    base_to_marker: Pose3D


@dataclass(frozen=True)
class RegistrationConfig:
    now_s: float
    max_sample_age_s: float
    floor_z_m: float
    max_floor_height_error_m: float
    max_floor_normal_error_rad: float
    minimum_samples: int
    minimum_base_poses: int
    base_pose_translation_separation_m: float
    base_pose_yaw_separation_rad: float
    mad_scale: float
    translation_mad_floor_m: float
    yaw_mad_floor_rad: float
    max_translation_residual_m: float
    max_yaw_residual_rad: float
    marker_to_staging_offset: PlanarPose
    alignment_lateral_m: float
    alignment_yaw_rad: float
    front_reference_x_m: float
    parking_safety_distance_m: float
    marker_size_m: Optional[float] = None
    parking_target: Optional[Mapping] = None


@dataclass(frozen=True)
class ResidualQuantiles:
    p50: float
    p95: float
    maximum: float


@dataclass(frozen=True)
class RegistrationResiduals:
    translation_m: ResidualQuantiles
    floor_normal_rad: ResidualQuantiles
    yaw_rad: ResidualQuantiles


@dataclass(frozen=True)
class RegistrationResult:
    marker_pose: PlanarPose
    marker_height_m: float
    staging_pose: PlanarPose
    alignment_offset: PlanarPose
    final_tag_x_in_base_m: float
    sample_count: int
    rejected_count: int
    base_pose_count: int
    residuals: RegistrationResiduals


@dataclass(frozen=True)
class _Candidate:
    stamp_s: float
    map_to_base: Pose3D
    marker_xyz: Tuple[float, float, float]
    marker_yaw: float
    floor_normal_error: float


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise RegistrationError("{0} is non-finite".format(name))
    try:
        converted = float(value)
    except (OverflowError, ValueError):
        raise RegistrationError("{0} is non-finite".format(name))
    if not math.isfinite(converted):
        raise RegistrationError("{0} is non-finite".format(name))
    return converted


def _positive(value, name):
    converted = _finite(value, name)
    if converted <= 0.0:
        raise RegistrationError("{0} must be positive".format(name))
    return converted


def _nonnegative(value, name):
    converted = _finite(value, name)
    if converted < 0.0:
        raise RegistrationError("{0} must be nonnegative".format(name))
    return converted


def _validated_vector(value, length, name):
    if (
        isinstance(value, (str, bytes))
        or not isinstance(value, Sequence)
        or len(value) != length
    ):
        raise RegistrationError(
            "{0} must contain {1} finite values".format(name, length)
        )
    return tuple(
        _finite(component, "{0}[{1}]".format(name, index))
        for index, component in enumerate(value)
    )


def _validated_quaternion(value, name):
    quaternion = _validated_vector(value, 4, name)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise RegistrationError("{0} must be a unit quaternion".format(name))
    return quaternion


def _validate_config(config):
    if not isinstance(config, RegistrationConfig):
        raise RegistrationError("config must be a RegistrationConfig")
    _finite(config.now_s, "now_s")
    _positive(config.max_sample_age_s, "max_sample_age_s")
    _finite(config.floor_z_m, "floor_z_m")
    _positive(config.max_floor_height_error_m, "max_floor_height_error_m")
    _positive(config.max_floor_normal_error_rad, "max_floor_normal_error_rad")
    if (
        isinstance(config.minimum_samples, bool)
        or not isinstance(config.minimum_samples, int)
        or config.minimum_samples < 12
    ):
        raise RegistrationError("minimum_samples must be at least 12")
    if (
        isinstance(config.minimum_base_poses, bool)
        or not isinstance(config.minimum_base_poses, int)
        or config.minimum_base_poses < 3
    ):
        raise RegistrationError("minimum_base_poses must be at least 3")
    _positive(
        config.base_pose_translation_separation_m,
        "base_pose_translation_separation_m",
    )
    _positive(
        config.base_pose_yaw_separation_rad,
        "base_pose_yaw_separation_rad",
    )
    _positive(config.mad_scale, "mad_scale")
    _positive(config.translation_mad_floor_m, "translation_mad_floor_m")
    _positive(config.yaw_mad_floor_rad, "yaw_mad_floor_rad")
    _positive(
        config.max_translation_residual_m,
        "max_translation_residual_m",
    )
    _positive(config.max_yaw_residual_rad, "max_yaw_residual_rad")
    if not isinstance(config.marker_to_staging_offset, PlanarPose):
        raise RegistrationError(
            "marker_to_staging_offset must be a PlanarPose"
        )
    _finite(config.alignment_lateral_m, "alignment_lateral_m")
    _finite(config.alignment_yaw_rad, "alignment_yaw_rad")
    _finite(config.front_reference_x_m, "front_reference_x_m")
    _nonnegative(
        config.parking_safety_distance_m,
        "parking_safety_distance_m",
    )
    try:
        final_tag_distance(
            front_reference_x_m=config.front_reference_x_m,
            parking_safety_distance_m=config.parking_safety_distance_m,
            marker_size_m=config.marker_size_m, parking_target=config.parking_target)
    except ValueError as exc:
        raise RegistrationError(str(exc)) from exc


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_conjugate(quaternion):
    return (
        -quaternion[0],
        -quaternion[1],
        -quaternion[2],
        quaternion[3],
    )


def _rotate_vector(quaternion, vector):
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, vector_quaternion),
        _quaternion_conjugate(quaternion),
    )
    return (rotated[0], rotated[1], rotated[2])


def _compose(left, right):
    left_translation, left_quaternion = left
    right_translation, right_quaternion = right
    rotated_translation = _rotate_vector(left_quaternion, right_translation)
    return (
        tuple(
            left_translation[index] + rotated_translation[index]
            for index in range(3)
        ),
        _quaternion_multiply(left_quaternion, right_quaternion),
    )


def _wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _angle_distance(left, right):
    return abs(_wrap_angle(left - right))


def _pose_values(pose, name):
    if not isinstance(pose, Pose3D):
        raise RegistrationError("{0} must be a Pose3D".format(name))
    translation = _validated_vector(
        pose.translation_m,
        3,
        "{0}.translation_m".format(name),
    )
    quaternion = _validated_quaternion(
        pose.quaternion_xyzw,
        "{0}.quaternion_xyzw".format(name),
    )
    return translation, quaternion


def _sample_candidate(sample, config):
    if not isinstance(sample, RegistrationSample):
        raise RegistrationError("sample must be a RegistrationSample")
    stamp = _finite(sample.stamp_s, "sample.stamp_s")
    age = config.now_s - stamp
    if age < 0.0 or age > config.max_sample_age_s:
        raise RegistrationError("stale sample")
    map_to_base = _pose_values(sample.map_to_base, "map_to_base")
    base_to_marker = _pose_values(sample.base_to_marker, "base_to_marker")
    marker_translation, marker_quaternion = _compose(
        map_to_base,
        base_to_marker,
    )
    height_error = abs(marker_translation[2] - config.floor_z_m)
    if height_error > config.max_floor_height_error_m:
        raise RegistrationError("floor height mismatch")
    normal = _rotate_vector(marker_quaternion, (0.0, 0.0, 1.0))
    normal_z = min(max(normal[2], -1.0), 1.0)
    normal_error = math.acos(normal_z)
    if normal_error > config.max_floor_normal_error_rad:
        raise RegistrationError("floor normal mismatch")
    arrow = _rotate_vector(marker_quaternion, (1.0, 0.0, 0.0))
    arrow_length = math.hypot(arrow[0], arrow[1])
    if arrow_length <= 1e-9:
        raise RegistrationError("marker arrow cannot be projected to map plane")
    yaw = math.atan2(arrow[1], arrow[0])
    return _Candidate(
        stamp_s=stamp,
        map_to_base=sample.map_to_base,
        marker_xyz=marker_translation,
        marker_yaw=yaw,
        floor_normal_error=normal_error,
    )


def _sample_key(sample):
    return (
        sample.stamp_s,
        tuple(sample.map_to_base.translation_m),
        tuple(sample.map_to_base.quaternion_xyzw),
        tuple(sample.base_to_marker.translation_m),
        tuple(sample.base_to_marker.quaternion_xyzw),
    )


def _circular_median(values):
    if not values:
        raise RegistrationError("cannot estimate yaw from no samples")
    candidates = sorted(_wrap_angle(value) for value in values)
    return min(
        candidates,
        key=lambda candidate: (
            sum(_angle_distance(candidate, value) for value in candidates),
            candidate,
        ),
    )


def _marker_center(candidates):
    return (
        median(candidate.marker_xyz[0] for candidate in candidates),
        median(candidate.marker_xyz[1] for candidate in candidates),
        median(candidate.marker_xyz[2] for candidate in candidates),
        _circular_median([candidate.marker_yaw for candidate in candidates]),
    )


def _candidate_residual(candidate, center):
    translation = math.sqrt(
        sum(
            (candidate.marker_xyz[index] - center[index]) ** 2
            for index in range(3)
        )
    )
    yaw = _angle_distance(candidate.marker_yaw, center[3])
    return translation, yaw


def _mad_threshold(values, scale, floor):
    center = median(values)
    absolute_deviations = [abs(value - center) for value in values]
    robust_sigma = 1.4826 * median(absolute_deviations)
    return center + scale * max(robust_sigma, floor)


def _quantile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        raise RegistrationError("cannot report empty residuals")
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _quantiles(values):
    return ResidualQuantiles(
        p50=_quantile(values, 0.50),
        p95=_quantile(values, 0.95),
        maximum=max(values),
    )


def _base_planar_pose(candidate):
    translation, quaternion = _pose_values(
        candidate.map_to_base,
        "map_to_base",
    )
    arrow = _rotate_vector(quaternion, (1.0, 0.0, 0.0))
    if math.hypot(arrow[0], arrow[1]) <= 1e-9:
        raise RegistrationError("base heading cannot be projected")
    return PlanarPose(
        x_m=translation[0],
        y_m=translation[1],
        yaw_rad=math.atan2(arrow[1], arrow[0]),
    )


def _distinct_base_poses(candidates, config):
    representatives = []
    ordered = sorted(
        (_base_planar_pose(candidate) for candidate in candidates),
        key=lambda pose: (pose.x_m, pose.y_m, pose.yaw_rad),
    )
    for pose in ordered:
        matches_existing = any(
            math.hypot(
                pose.x_m - existing.x_m,
                pose.y_m - existing.y_m,
            )
            < config.base_pose_translation_separation_m
            and _angle_distance(pose.yaw_rad, existing.yaw_rad)
            < config.base_pose_yaw_separation_rad
            for existing in representatives
        )
        if not matches_existing:
            representatives.append(pose)
    return len(representatives)


def _compose_planar(left, right):
    cosine = math.cos(left.yaw_rad)
    sine = math.sin(left.yaw_rad)
    return PlanarPose(
        x_m=left.x_m + cosine * right.x_m - sine * right.y_m,
        y_m=left.y_m + sine * right.x_m + cosine * right.y_m,
        yaw_rad=_wrap_angle(left.yaw_rad + right.yaw_rad),
    )


def _rejection_summary(rejections):
    if not rejections:
        return ""
    counts = {}
    for reason in rejections:
        counts[reason] = counts.get(reason, 0) + 1
    return "; rejected: {0}".format(
        ", ".join(
            "{0} ({1})".format(reason, counts[reason])
            for reason in sorted(counts)
        )
    )


def estimate_registration(
    samples: Iterable[RegistrationSample],
    config: RegistrationConfig,
) -> RegistrationResult:
    """Estimate ``map -> marker`` from synchronized full-pose evidence."""

    _validate_config(config)
    unique_keys = set()
    candidates = []
    rejections = []
    supplied_samples = list(samples)
    for sample in supplied_samples:
        try:
            candidate = _sample_candidate(sample, config)
            key = _sample_key(sample)
            if key in unique_keys:
                raise RegistrationError("duplicate pose observation")
            unique_keys.add(key)
            candidates.append(candidate)
        except (AttributeError, RegistrationError, TypeError) as exc:
            rejections.append(str(exc))
    candidates.sort(
        key=lambda candidate: (
            candidate.stamp_s,
            candidate.marker_xyz,
            candidate.marker_yaw,
        )
    )
    if len(candidates) < config.minimum_samples:
        raise RegistrationError(
            "registration requires at least {0} accepted samples{1}".format(
                config.minimum_samples,
                _rejection_summary(rejections),
            )
        )

    initial_center = _marker_center(candidates)
    initial_residuals = [
        _candidate_residual(candidate, initial_center)
        for candidate in candidates
    ]
    translation_threshold = _mad_threshold(
        [value[0] for value in initial_residuals],
        config.mad_scale,
        config.translation_mad_floor_m,
    )
    yaw_threshold = _mad_threshold(
        [value[1] for value in initial_residuals],
        config.mad_scale,
        config.yaw_mad_floor_rad,
    )
    accepted = []
    for candidate, residual in zip(candidates, initial_residuals):
        if residual[0] > translation_threshold or residual[1] > yaw_threshold:
            rejections.append("median/MAD pose outlier")
        else:
            accepted.append(candidate)
    if len(accepted) < config.minimum_samples:
        raise RegistrationError(
            "registration requires at least {0} accepted samples after "
            "median/MAD rejection{1}".format(
                config.minimum_samples,
                _rejection_summary(rejections),
            )
        )
    base_pose_count = _distinct_base_poses(accepted, config)
    if base_pose_count < config.minimum_base_poses:
        raise RegistrationError(
            "registration requires at least three distinct base poses"
        )

    center = _marker_center(accepted)
    translation_residuals = []
    yaw_residuals = []
    floor_normal_residuals = []
    for candidate in accepted:
        translation, yaw = _candidate_residual(candidate, center)
        translation_residuals.append(translation)
        yaw_residuals.append(yaw)
        floor_normal_residuals.append(candidate.floor_normal_error)
    if max(translation_residuals) > config.max_translation_residual_m:
        raise RegistrationError("maximum translation residual exceeds limit")
    if max(yaw_residuals) > config.max_yaw_residual_rad:
        raise RegistrationError("maximum yaw residual exceeds limit")

    marker_pose = PlanarPose(
        x_m=center[0],
        y_m=center[1],
        yaw_rad=center[3],
    )
    return RegistrationResult(
        marker_pose=marker_pose,
        marker_height_m=center[2],
        staging_pose=_compose_planar(
            marker_pose,
            config.marker_to_staging_offset,
        ),
        alignment_offset=PlanarPose(
            x_m=0.0,
            y_m=config.alignment_lateral_m,
            yaw_rad=config.alignment_yaw_rad,
        ),
        final_tag_x_in_base_m=final_tag_distance(
            front_reference_x_m=config.front_reference_x_m,
            parking_safety_distance_m=config.parking_safety_distance_m,
            marker_size_m=config.marker_size_m, parking_target=config.parking_target),
        sample_count=len(accepted),
        rejected_count=len(supplied_samples) - len(accepted),
        base_pose_count=base_pose_count,
        residuals=RegistrationResiduals(
            translation_m=_quantiles(translation_residuals),
            floor_normal_rad=_quantiles(floor_normal_residuals),
            yaw_rad=_quantiles(yaw_residuals),
        ),
    )
