"""Conservative target-colored tube-cap detection from RGB images."""

from dataclasses import dataclass
from itertools import combinations
from math import pi
from numbers import Integral, Real
from typing import Tuple

import cv2
import numpy as np


def _finite_scalar(value, category):
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value):
        raise ValueError("{0} must be a finite numeric value".format(category))


def _integer_scalar(value, category):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError("{0} must be an integer".format(category))


def _numeric_vector(values, arity, category, require_integer=False):
    if isinstance(values, (str, bytes)):
        raise ValueError("{0} must contain {1} numeric values".format(category, arity))
    try:
        vector = tuple(values)
    except TypeError:
        raise ValueError("{0} must contain {1} numeric values".format(category, arity))
    if len(vector) != arity:
        raise ValueError("{0} must contain {1} numeric values".format(category, arity))
    for value in vector:
        if require_integer:
            _integer_scalar(value, category)
        else:
            _finite_scalar(value, category)
    return vector


@dataclass(frozen=True)
class DetectorConfig:
    """Validated parameters shared with the test-tube grasp configuration."""

    roi_normalized: Tuple[float, float, float, float] = (0.2, 0.2, 0.8, 0.8)
    minimum_layout_saturation: int = 70
    green_hsv_lower: Tuple[int, int, int] = (40, 80, 50)
    green_hsv_upper: Tuple[int, int, int] = (85, 255, 255)
    morphology_kernel_px: int = 3
    morphology_iterations: int = 1
    min_area_px: float = 80
    max_area_px: float = 5000
    min_circularity: float = 0.70
    max_axis_ratio: float = 1.35
    max_interframe_spread_m: float = 0.004

    def __post_init__(self):
        roi = _numeric_vector(self.roi_normalized, 4, "ROI")
        if any(value < 0.0 or value > 1.0 for value in roi):
            raise ValueError("ROI values must be in [0, 1]")
        if roi[0] >= roi[2] or roi[1] >= roi[3]:
            raise ValueError("ROI must have increasing x and y bounds")

        _integer_scalar(
            self.minimum_layout_saturation,
            "minimum layout saturation",
        )
        if not 1 <= self.minimum_layout_saturation <= 255:
            raise ValueError("minimum layout saturation must be in [1, 255]")

        lower = _numeric_vector(self.green_hsv_lower, 3, "HSV", require_integer=True)
        upper = _numeric_vector(self.green_hsv_upper, 3, "HSV", require_integer=True)
        hsv_maximums = (179, 255, 255)
        if any(
            value < 0 or value > maximum
            for value, maximum in zip(lower + upper, hsv_maximums + hsv_maximums)
        ) or any(low > high for low, high in zip(lower, upper)):
            raise ValueError("HSV lower bounds must not exceed upper bounds")

        _integer_scalar(self.morphology_kernel_px, "morphology kernel")
        _integer_scalar(self.morphology_iterations, "morphology iterations")
        if self.morphology_kernel_px <= 0 or self.morphology_kernel_px % 2 == 0:
            raise ValueError("morphology kernel must be positive and odd")
        if self.morphology_iterations <= 0:
            raise ValueError("morphology iterations must be positive")
        _finite_scalar(self.min_area_px, "area")
        _finite_scalar(self.max_area_px, "area")
        if self.min_area_px <= 0 or self.max_area_px < self.min_area_px:
            raise ValueError("area bounds must be positive and ordered")
        _finite_scalar(self.min_circularity, "circularity")
        if not 0.0 < self.min_circularity <= 1.0:
            raise ValueError("circularity must be in (0, 1]")
        _finite_scalar(self.max_axis_ratio, "axis ratio")
        if self.max_axis_ratio < 1.0:
            raise ValueError("axis ratio must be at least one")
        _finite_scalar(self.max_interframe_spread_m, "maximum interframe spread")
        if self.max_interframe_spread_m <= 0:
            raise ValueError("maximum interframe spread must be positive")


class DetectionError(RuntimeError):
    """A fail-closed detection result with a stable machine-readable code."""

    def __init__(self, code, detail, candidate_count=0):
        self.code = code
        self.detail = detail
        self.candidate_count = candidate_count
        super().__init__("{0}: {1}".format(code, detail))


def roi_mask(image_shape, config):
    height, width = image_shape[:2]
    x0 = int(np.floor(config.roi_normalized[0] * width))
    y0 = int(np.floor(config.roi_normalized[1] * height))
    x1 = int(np.ceil(config.roi_normalized[2] * width))
    y1 = int(np.ceil(config.roi_normalized[3] * height))
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[y0:y1, x0:x1] = 255
    return mask


def candidate_contours(binary_mask, config):
    contour_result = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = contour_result[-2]
    accepted = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < config.min_area_px or area > config.max_area_px:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        circularity = 4.0 * pi * area / (perimeter * perimeter)
        if circularity < config.min_circularity or len(contour) < 5:
            continue
        ellipse = cv2.fitEllipse(contour)
        axis_a, axis_b = ellipse[1]
        if min(axis_a, axis_b) <= 0:
            continue
        if max(axis_a, axis_b) / min(axis_a, axis_b) > config.max_axis_ratio:
            continue
        accepted.append((contour, area, ellipse[0]))
    return accepted


def aggregate_points(points, max_spread_m):
    """Return a componentwise median only when all observations agree.

    Raises ValueError when ``max_spread_m`` is not a finite positive real scalar.
    """
    try:
        array = np.asarray(points, dtype=np.float64)
    except (TypeError, ValueError):
        raise DetectionError("inconsistent", "points must be numeric 3D coordinates")
    if array.ndim != 2 or array.shape[0] == 0 or array.shape[1] != 3:
        raise DetectionError("inconsistent", "points must be a nonempty Nx3 collection")
    if not np.all(np.isfinite(array)):
        raise DetectionError("inconsistent", "points must be finite")
    _finite_scalar(max_spread_m, "maximum spread")
    if max_spread_m <= 0:
        raise ValueError("maximum spread must be positive")
    for first, second in combinations(array, 2):
        if np.linalg.norm(first - second) > max_spread_m:
            raise DetectionError("inconsistent", "point spread exceeds configured maximum")
    median = np.median(array, axis=0)
    return (float(median[0]), float(median[1]), float(median[2]))
