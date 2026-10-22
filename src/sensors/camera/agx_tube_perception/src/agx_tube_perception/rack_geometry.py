"""Hue-independent cap-surface extraction and configured-pitch lattice fitting."""

from dataclasses import dataclass
from itertools import combinations
from math import pi
from numbers import Integral, Real
from typing import Tuple

import cv2
import numpy as np

from .detector import DetectorConfig, candidate_contours, roi_mask


def _finite_scalar(value, category):
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value):
        raise ValueError("{0} must be a finite numeric value".format(category))
    return float(value)


def _positive_scalar(value, category, allow_zero=False):
    numeric = _finite_scalar(value, category)
    if numeric < 0.0 if allow_zero else numeric <= 0.0:
        qualifier = "nonnegative" if allow_zero else "positive"
        raise ValueError("{0} must be {1}".format(category, qualifier))
    return numeric


def _positive_integer(value, category):
    if isinstance(value, bool) or not isinstance(value, Integral) or value <= 0:
        raise LayoutError("{0} must be a positive integer".format(category))
    return int(value)


def _numeric_tuple(values, length, category):
    if isinstance(values, (str, bytes)):
        raise ValueError("{0} must contain {1} numeric values".format(category, length))
    try:
        result = tuple(values)
    except TypeError:
        raise ValueError("{0} must contain {1} numeric values".format(category, length))
    if len(result) != length:
        raise ValueError("{0} must contain {1} numeric values".format(category, length))
    return tuple(_finite_scalar(value, category) for value in result)


def _integer_tuple(values, category):
    if isinstance(values, (str, bytes)):
        raise ValueError("{0} must contain integers".format(category))
    try:
        result = tuple(values)
    except TypeError:
        raise ValueError("{0} must contain integers".format(category))
    if any(isinstance(value, bool) or not isinstance(value, Integral) for value in result):
        raise ValueError("{0} must contain integers".format(category))
    return tuple(int(value) for value in result)


@dataclass(frozen=True)
class RgbCapSurface:
    """One color-independent circular cap candidate extracted from RGB."""

    center_uv: Tuple[float, float]
    contour: np.ndarray
    ellipse: Tuple[Tuple[float, float], Tuple[float, float], float]
    circularity: float

    def __post_init__(self):
        center_uv = _numeric_tuple(self.center_uv, 2, "center")
        try:
            ellipse_center, ellipse_axes, ellipse_angle = self.ellipse
        except (TypeError, ValueError):
            raise ValueError("ellipse must contain center, axes, and angle")
        ellipse_center = _numeric_tuple(ellipse_center, 2, "ellipse center")
        ellipse_axes = _numeric_tuple(ellipse_axes, 2, "ellipse axes")
        if min(ellipse_axes) <= 0.0:
            raise ValueError("ellipse axes must be positive")
        ellipse_angle = _finite_scalar(ellipse_angle, "ellipse angle")
        circularity = _finite_scalar(self.circularity, "circularity")
        if not 0.0 < circularity <= 1.0:
            raise ValueError("circularity must be in (0, 1]")
        if not isinstance(self.contour, np.ndarray):
            raise ValueError("contour must be an array")
        if (
            self.contour.ndim != 3
            or self.contour.shape[0] < 5
            or self.contour.shape[1:] != (1, 2)
            or not np.issubdtype(self.contour.dtype, np.number)
            or not np.all(np.isfinite(self.contour))
        ):
            raise ValueError("contour must be a finite Nx1x2 array with five points")
        contour_copy = np.array(self.contour, copy=True)
        contour = np.frombuffer(
            contour_copy.tobytes(), dtype=contour_copy.dtype
        ).reshape(contour_copy.shape)
        object.__setattr__(self, "center_uv", center_uv)
        object.__setattr__(
            self,
            "ellipse",
            (ellipse_center, ellipse_axes, ellipse_angle),
        )
        object.__setattr__(self, "circularity", circularity)
        object.__setattr__(self, "contour", contour)


@dataclass(frozen=True)
class LatticeFit:
    """Immutable target-anchored assignment of observations to lattice slots."""

    axis_u: Tuple[float, float]
    axis_v: Tuple[float, float]
    grid_u: Tuple[int, ...]
    grid_v: Tuple[int, ...]
    inlier_indices: Tuple[int, ...]
    inlier_count: int
    rms_error_m: float

    def __post_init__(self):
        axis_u = _numeric_tuple(self.axis_u, 2, "axis u")
        axis_v = _numeric_tuple(self.axis_v, 2, "axis v")
        grid_u = _integer_tuple(self.grid_u, "grid u")
        grid_v = _integer_tuple(self.grid_v, "grid v")
        inlier_indices = _integer_tuple(self.inlier_indices, "inlier indices")
        if (
            isinstance(self.inlier_count, bool)
            or not isinstance(self.inlier_count, Integral)
            or self.inlier_count <= 0
        ):
            raise ValueError("inlier count must be a positive integer")
        inlier_count = int(self.inlier_count)
        if not (
            len(grid_u)
            == len(grid_v)
            == len(inlier_indices)
            == inlier_count
        ):
            raise ValueError("lattice result member lengths must match inlier count")
        if len(set(inlier_indices)) != inlier_count or min(inlier_indices) < 0:
            raise ValueError("inlier indices must be unique and nonnegative")
        if len(set(zip(grid_u, grid_v))) != inlier_count:
            raise ValueError("lattice grid assignments must be unique")
        rms_error_m = _positive_scalar(
            self.rms_error_m, "RMS error", allow_zero=True
        )
        object.__setattr__(self, "axis_u", axis_u)
        object.__setattr__(self, "axis_v", axis_v)
        object.__setattr__(self, "grid_u", grid_u)
        object.__setattr__(self, "grid_v", grid_v)
        object.__setattr__(self, "inlier_indices", inlier_indices)
        object.__setattr__(self, "inlier_count", inlier_count)
        object.__setattr__(self, "rms_error_m", rms_error_m)


class LayoutError(RuntimeError):
    """The observations cannot support the configured fixture layout."""


def _score_axis(points, target_index, axis_u, rows, columns, pitch_m, tolerance_m):
    axis_u = np.asarray(axis_u, dtype=np.float64)
    axis_u = axis_u / np.linalg.norm(axis_u)
    axis_v = np.asarray((-axis_u[1], axis_u[0]), dtype=np.float64)
    relative = points - points[target_index]
    continuous = np.column_stack((relative @ axis_u, relative @ axis_v)) / pitch_m
    integer = np.rint(continuous).astype(np.int32)
    predicted = (
        integer[:, :1] * axis_u * pitch_m
        + integer[:, 1:] * axis_v * pitch_m
    )
    residual = np.linalg.norm(relative - predicted, axis=1)

    slots = {}
    for index in np.flatnonzero(residual <= tolerance_m):
        index = int(index)
        slot = (int(integer[index, 0]), int(integer[index, 1]))
        previous = slots.get(slot)
        if (
            previous is None
            or index == target_index
            or (
                previous != target_index
                and residual[index] < residual[previous]
            )
        ):
            slots[slot] = index
    indices = tuple(sorted(slots.values()))
    if not indices or target_index not in indices:
        return None
    assigned = integer[list(indices)]
    if np.ptp(assigned[:, 0]) >= columns or np.ptp(assigned[:, 1]) >= rows:
        return None
    rms = float(np.sqrt(np.mean(np.square(residual[list(indices)]))))
    return indices, integer, axis_u, axis_v, rms


def fit_square_lattice(
    points_xy,
    target_index,
    rows,
    columns,
    pitch_m,
    pitch_tolerance_m,
    minimum_inliers,
    maximum_rms_error_m,
):
    """Fit a target-anchored square lattice at the configured physical pitch."""
    try:
        points = np.asarray(points_xy, dtype=np.float64)
    except (TypeError, ValueError):
        raise LayoutError("layout points must be a finite Nx2 array")
    if (
        points.ndim != 2
        or points.shape[0] == 0
        or points.shape[1] != 2
        or not np.all(np.isfinite(points))
    ):
        raise LayoutError("layout points must be a nonempty finite Nx2 array")
    if isinstance(target_index, bool) or not isinstance(target_index, Integral):
        raise LayoutError("target index must be an integer")
    target_index = int(target_index)
    if not 0 <= target_index < len(points):
        raise LayoutError("target index is outside layout points")

    rows = _positive_integer(rows, "rows")
    columns = _positive_integer(columns, "columns")
    minimum_inliers = _positive_integer(minimum_inliers, "minimum inliers")
    try:
        pitch_m = _positive_scalar(pitch_m, "pitch m")
        pitch_tolerance_m = _positive_scalar(
            pitch_tolerance_m, "pitch tolerance m"
        )
        maximum_rms_error_m = _positive_scalar(
            maximum_rms_error_m, "maximum rms error m"
        )
    except ValueError as error:
        raise LayoutError(str(error))
    if minimum_inliers > len(points) or minimum_inliers > rows * columns:
        raise LayoutError("minimum inliers exceed observations or lattice capacity")

    candidates = []
    for first, second in combinations(range(len(points)), 2):
        delta = points[second] - points[first]
        length = float(np.linalg.norm(delta))
        if length > 0.0 and abs(length - pitch_m) <= pitch_tolerance_m:
            for direction in (delta / length, -delta / length):
                scored = _score_axis(
                    points,
                    target_index,
                    direction,
                    rows,
                    columns,
                    pitch_m,
                    pitch_tolerance_m,
                )
                if scored is not None:
                    candidates.append(scored)
    if not candidates:
        raise LayoutError("no configured-pitch square lattice")

    best = min(
        candidates,
        key=lambda item: (
            -len(item[0]),
            item[4],
            (float(item[2][0]), float(item[2][1])),
        ),
    )
    indices, integer, axis_u, axis_v, rms = best
    if len(indices) < minimum_inliers:
        raise LayoutError("layout has insufficient lattice inliers")
    if rms > maximum_rms_error_m:
        raise LayoutError("lattice RMS error exceeds configured maximum")
    return LatticeFit(
        axis_u=(float(axis_u[0]), float(axis_u[1])),
        axis_v=(float(axis_v[0]), float(axis_v[1])),
        grid_u=tuple(int(integer[index, 0]) for index in indices),
        grid_v=tuple(int(integer[index, 1]) for index in indices),
        inlier_indices=indices,
        inlier_count=len(indices),
        rms_error_m=rms,
    )


def detect_rgb_cap_surfaces(rgb, config):
    """Extract saturated circular cap candidates without assigning hue labels."""

    if not isinstance(config, DetectorConfig):
        raise TypeError("config must be a DetectorConfig")
    if (
        not isinstance(rgb, np.ndarray)
        or rgb.ndim != 3
        or rgb.shape[2] != 3
        or rgb.dtype != np.uint8
        or rgb.shape[0] == 0
        or rgb.shape[1] == 0
    ):
        raise ValueError("RGB must be a nonempty uint8 HxWx3 array")

    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    proposal_mask = cv2.inRange(
        hsv,
        np.asarray(
            (0, config.minimum_layout_saturation, 0),
            dtype=np.uint8,
        ),
        np.asarray((179, 255, 255), dtype=np.uint8),
    )
    proposal_mask = cv2.bitwise_and(
        proposal_mask,
        roi_mask(rgb.shape, config),
    )
    kernel = np.ones(
        (config.morphology_kernel_px, config.morphology_kernel_px),
        dtype=np.uint8,
    )
    proposal_mask = cv2.morphologyEx(
        proposal_mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=config.morphology_iterations,
    )
    proposal_mask = cv2.morphologyEx(
        proposal_mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=config.morphology_iterations,
    )

    results = []
    for contour, area, center_uv in candidate_contours(proposal_mask, config):
        perimeter = cv2.arcLength(contour, True)
        circularity = 4.0 * pi * area / (perimeter * perimeter)
        results.append(
            RgbCapSurface(
                center_uv=(float(center_uv[0]), float(center_uv[1])),
                contour=contour,
                ellipse=cv2.fitEllipse(contour),
                circularity=float(circularity),
            )
        )
    return tuple(sorted(results, key=lambda item: item.center_uv))
