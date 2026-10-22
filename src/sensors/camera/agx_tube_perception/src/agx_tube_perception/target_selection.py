"""ROS-independent 2D cap candidates, target selection, and stability."""

from dataclasses import dataclass
from itertools import combinations
from math import hypot, isfinite, pi
from numbers import Integral, Real
from typing import Tuple

import cv2
import numpy as np

from .detector import DetectorConfig, candidate_contours, roi_mask
from .rack_geometry import RgbCapSurface, detect_rgb_cap_surfaces


PixelCoordinate = Tuple[float, float]
BoundingBox = Tuple[int, int, int, int]


def _finite_pixel_pair(value, name):
    if isinstance(value, (str, bytes)):
        raise ValueError("{0} must contain two finite pixels".format(name))
    try:
        pair = tuple(value)
    except TypeError:
        raise ValueError("{0} must contain two finite pixels".format(name))
    if len(pair) != 2 or any(
        isinstance(component, bool)
        or not isinstance(component, Real)
        or not isfinite(float(component))
        for component in pair
    ):
        raise ValueError("{0} must contain two finite pixels".format(name))
    return (float(pair[0]), float(pair[1]))


@dataclass(frozen=True)
class CapCandidate2D:
    """One shape-valid cap proposal with its target-color evidence."""

    surface: RgbCapSurface
    bounding_box_xywh: BoundingBox
    area_px: float
    target_overlap_px: int

    def __post_init__(self):
        if not isinstance(self.surface, RgbCapSurface):
            raise ValueError("surface must be an RgbCapSurface")
        if (
            not isinstance(self.bounding_box_xywh, tuple)
            or len(self.bounding_box_xywh) != 4
            or any(
                isinstance(value, bool) or not isinstance(value, Integral)
                for value in self.bounding_box_xywh
            )
        ):
            raise ValueError("bounding box must contain four integers")
        x, y, width, height = (
            int(value) for value in self.bounding_box_xywh
        )
        if x < 0 or y < 0 or width <= 0 or height <= 0:
            raise ValueError("bounding box must have nonnegative origin and size")
        if (
            isinstance(self.area_px, bool)
            or not isinstance(self.area_px, Real)
            or not isfinite(float(self.area_px))
            or float(self.area_px) <= 0.0
        ):
            raise ValueError("candidate area must be finite and positive")
        if (
            isinstance(self.target_overlap_px, bool)
            or not isinstance(self.target_overlap_px, Integral)
            or int(self.target_overlap_px) < 0
        ):
            raise ValueError("target overlap must be a nonnegative integer")
        object.__setattr__(self, "bounding_box_xywh", (x, y, width, height))
        object.__setattr__(self, "area_px", float(self.area_px))
        object.__setattr__(
            self, "target_overlap_px", int(self.target_overlap_px)
        )

    @property
    def center_uv(self):
        return self.surface.center_uv


class TargetSelectionError(RuntimeError):
    """A fail-closed image target outcome with a stable error code."""

    def __init__(self, code, detail, candidate_count=0):
        if not isinstance(code, str) or not code:
            raise ValueError("selection error code must be nonempty")
        if not isinstance(detail, str) or not detail:
            raise ValueError("selection error detail must be nonempty")
        if (
            isinstance(candidate_count, bool)
            or not isinstance(candidate_count, Integral)
            or candidate_count < 0
        ):
            raise ValueError("selection candidate count must be nonnegative")
        self.code = code
        self.detail = detail
        self.candidate_count = int(candidate_count)
        super().__init__("{0}: {1}".format(code, detail))


def detect_cap_candidates(rgb, config):
    """Extract shape-valid cap candidates and green-mask overlap evidence."""

    if not isinstance(config, DetectorConfig):
        raise TypeError("config must be a DetectorConfig")
    surfaces = detect_rgb_cap_surfaces(rgb, config)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    target_mask = cv2.inRange(
        hsv,
        np.asarray(config.green_hsv_lower, dtype=np.uint8),
        np.asarray(config.green_hsv_upper, dtype=np.uint8),
    )
    candidates = []
    for surface in surfaces:
        contour_mask = np.zeros(target_mask.shape, dtype=np.uint8)
        cv2.drawContours(
            contour_mask,
            [surface.contour],
            -1,
            255,
            thickness=-1,
        )
        overlap = cv2.countNonZero(
            cv2.bitwise_and(target_mask, contour_mask)
        )
        candidates.append(
            CapCandidate2D(
                surface=surface,
                bounding_box_xywh=tuple(
                    int(value) for value in cv2.boundingRect(surface.contour)
                ),
                area_px=float(cv2.contourArea(surface.contour)),
                target_overlap_px=int(overlap),
            )
        )
    return tuple(candidates)


def _detect_target_candidates(rgb, config):
    """Extract target-colored cap shapes independently of the layout mask."""

    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    target_mask = cv2.inRange(
        hsv,
        np.asarray(config.green_hsv_lower, dtype=np.uint8),
        np.asarray(config.green_hsv_upper, dtype=np.uint8),
    )
    target_mask = cv2.bitwise_and(
        target_mask,
        roi_mask(rgb.shape, config),
    )
    kernel = np.ones(
        (config.morphology_kernel_px, config.morphology_kernel_px),
        dtype=np.uint8,
    )
    target_mask = cv2.morphologyEx(
        target_mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=config.morphology_iterations,
    )
    target_mask = cv2.morphologyEx(
        target_mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=config.morphology_iterations,
    )

    candidates = []
    for contour, area, center_uv in candidate_contours(target_mask, config):
        perimeter = cv2.arcLength(contour, True)
        surface = RgbCapSurface(
            center_uv=(float(center_uv[0]), float(center_uv[1])),
            contour=contour,
            ellipse=cv2.fitEllipse(contour),
            circularity=float(4.0 * pi * area / (perimeter * perimeter)),
        )
        candidates.append(
            CapCandidate2D(
                surface=surface,
                bounding_box_xywh=tuple(
                    int(value) for value in cv2.boundingRect(contour)
                ),
                area_px=float(area),
                target_overlap_px=int(cv2.countNonZero(target_mask)),
            )
        )
    return tuple(candidates)


def select_unique_green_target(candidates):
    """Return the sole shape-valid candidate overlapping configured green."""

    if isinstance(candidates, (str, bytes)):
        raise TypeError("candidates must contain CapCandidate2D values")
    try:
        values = tuple(candidates)
    except TypeError:
        raise TypeError("candidates must contain CapCandidate2D values")
    if any(not isinstance(candidate, CapCandidate2D) for candidate in values):
        raise TypeError("candidates must contain CapCandidate2D values")
    targets = tuple(
        candidate for candidate in values if candidate.target_overlap_px > 0
    )
    if not targets:
        raise TargetSelectionError(
            "target_absent",
            "no valid green target cap",
            candidate_count=0,
        )
    if len(targets) > 1:
        raise TargetSelectionError(
            "ambiguous_target",
            "multiple valid green target caps",
            candidate_count=len(targets),
        )
    return targets[0]


@dataclass(frozen=True)
class TargetFrameSelection:
    """All shape-valid candidates and the selected target from one frame."""

    candidates: Tuple[CapCandidate2D, ...]
    selected_index: int

    def __post_init__(self):
        if not isinstance(self.candidates, tuple) or not self.candidates:
            raise ValueError("selection candidates must be a nonempty tuple")
        if any(
            not isinstance(candidate, CapCandidate2D)
            for candidate in self.candidates
        ):
            raise ValueError("selection candidates must be CapCandidate2D values")
        if (
            isinstance(self.selected_index, bool)
            or not isinstance(self.selected_index, Integral)
            or not 0 <= int(self.selected_index) < len(self.candidates)
        ):
            raise ValueError("selected candidate index is out of bounds")
        object.__setattr__(self, "selected_index", int(self.selected_index))

    @property
    def target(self):
        return self.candidates[self.selected_index]


def recognize_unique_green_cap(rgb, config):
    """Recognize one green target independently and retain layout candidates."""

    layout_candidates = detect_cap_candidates(rgb, config)
    target = select_unique_green_target(
        _detect_target_candidates(rgb, config)
    )
    candidates = tuple(
        candidate
        for candidate in layout_candidates
        if candidate.target_overlap_px == 0
    ) + (target,)
    return TargetFrameSelection(candidates, len(candidates) - 1)


@dataclass(frozen=True)
class TargetSample2D:
    """One validated, source-stamped target in an image coordinate domain."""

    stamp_ns: int
    frame_id: str
    image_width: int
    image_height: int
    target_uv: PixelCoordinate
    reference_uv: PixelCoordinate

    def __post_init__(self):
        if (
            isinstance(self.stamp_ns, bool)
            or not isinstance(self.stamp_ns, Integral)
            or self.stamp_ns <= 0
        ):
            raise ValueError("target stamp must be a positive integer")
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("target frame must be a nonempty string")
        if (
            isinstance(self.image_width, bool)
            or not isinstance(self.image_width, Integral)
            or self.image_width <= 0
            or isinstance(self.image_height, bool)
            or not isinstance(self.image_height, Integral)
            or self.image_height <= 0
        ):
            raise ValueError("image dimensions must be positive integers")
        target = _finite_pixel_pair(self.target_uv, "target pixels")
        reference = _finite_pixel_pair(
            self.reference_uv, "reference pixels"
        )
        width = int(self.image_width)
        height = int(self.image_height)
        if not (0.0 <= target[0] < width and 0.0 <= target[1] < height):
            raise ValueError("target pixels must lie inside the image")
        if not (
            0.0 <= reference[0] < width
            and 0.0 <= reference[1] < height
        ):
            raise ValueError("reference pixels must lie inside the image")
        object.__setattr__(self, "stamp_ns", int(self.stamp_ns))
        object.__setattr__(self, "frame_id", self.frame_id.strip())
        object.__setattr__(self, "image_width", width)
        object.__setattr__(self, "image_height", height)
        object.__setattr__(self, "target_uv", target)
        object.__setattr__(self, "reference_uv", reference)


def newest_consistent_target(samples, maximum_spread_px):
    """Return the newest target only when the image-space batch agrees."""

    if (
        isinstance(maximum_spread_px, bool)
        or not isinstance(maximum_spread_px, Real)
        or not isfinite(float(maximum_spread_px))
        or float(maximum_spread_px) <= 0.0
    ):
        raise ValueError("maximum target spread must be finite and positive")
    if isinstance(samples, (str, bytes)):
        values = ()
    else:
        try:
            values = tuple(samples)
        except TypeError:
            values = ()
    if not values or any(
        not isinstance(sample, TargetSample2D) for sample in values
    ):
        raise TargetSelectionError(
            "inconsistent_target",
            "target samples must be a nonempty TargetSample2D collection",
        )
    stamps = tuple(sample.stamp_ns for sample in values)
    if any(
        current <= previous
        for previous, current in zip(stamps, stamps[1:])
    ):
        raise TargetSelectionError(
            "inconsistent_target",
            "target sample stamps must be strictly increasing",
        )
    first = values[0]
    if any(
        sample.frame_id != first.frame_id
        or sample.image_width != first.image_width
        or sample.image_height != first.image_height
        or sample.reference_uv != first.reference_uv
        for sample in values[1:]
    ):
        raise TargetSelectionError(
            "inconsistent_target",
            "target samples use inconsistent image metadata",
        )
    limit = float(maximum_spread_px)
    for first_sample, second_sample in combinations(values, 2):
        if hypot(
            first_sample.target_uv[0] - second_sample.target_uv[0],
            first_sample.target_uv[1] - second_sample.target_uv[1],
        ) > limit:
            raise TargetSelectionError(
                "inconsistent_target",
                "target centroid spread exceeds configured maximum",
            )
    return values[-1]
