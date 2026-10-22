"""ROS-independent RGB tube-cap perception primitives."""

from .detector import (
    DetectionError,
    DetectorConfig,
    aggregate_points,
)
from .fixture_profile import FixtureProfile, load_profile_registry
from .rack_geometry import (
    LatticeFit,
    LayoutError,
    RgbCapSurface,
    detect_rgb_cap_surfaces,
    fit_square_lattice,
)
from .service_logic import (
    ObservationEpoch,
    batch_is_stale,
    classify_batch_status,
    classify_lift_feedback,
    classify_sensor_timing,
    compute_lift_target,
    detection_status,
    lift_height_batch_is_consistent,
    resolve_lift_relative_z,
    three_vector,
)
from .target_selection import (
    CapCandidate2D,
    TargetFrameSelection,
    TargetSample2D,
    TargetSelectionError,
    detect_cap_candidates,
    newest_consistent_target,
    recognize_unique_green_cap,
    select_unique_green_target,
)

__all__ = [
    "DetectionError",
    "DetectorConfig",
    "aggregate_points",
    "FixtureProfile",
    "load_profile_registry",
    "LatticeFit",
    "LayoutError",
    "RgbCapSurface",
    "detect_rgb_cap_surfaces",
    "fit_square_lattice",
    "ObservationEpoch",
    "batch_is_stale",
    "classify_batch_status",
    "classify_lift_feedback",
    "classify_sensor_timing",
    "compute_lift_target",
    "detection_status",
    "lift_height_batch_is_consistent",
    "resolve_lift_relative_z",
    "three_vector",
    "CapCandidate2D",
    "TargetFrameSelection",
    "TargetSample2D",
    "TargetSelectionError",
    "detect_cap_candidates",
    "newest_consistent_target",
    "recognize_unique_green_cap",
    "select_unique_green_target",
]
