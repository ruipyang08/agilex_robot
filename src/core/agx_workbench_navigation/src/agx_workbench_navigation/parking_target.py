"""ROS-free parking references shared by registration and docking.

Coordinates are along the aligned registered approach, not world X. The tag
size is the detector's code square; label/rendering borders are irrelevant.
"""
import math
from numbers import Real
from typing import Mapping


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(name + " must be finite numeric")
    try:
        value = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(name + " must be finite numeric") from exc
    if not math.isfinite(value):
        raise ValueError(name + " must be finite numeric")
    return value


def normalize_parking_target(value):
    """Validate the opt-in reference without reinterpreting legacy registries."""
    if not isinstance(value, Mapping):
        raise ValueError("parking_target must be a mapping")
    if set(value) - {"schema_version", "reference", "backoff_distance_m"}:
        raise ValueError("parking_target has unknown fields")
    if type(value.get("schema_version")) is not int or value["schema_version"] != 1:
        raise ValueError("parking_target schema_version must be 1")
    if value.get("reference") != "tag_table_side_edge":
        raise ValueError("unsupported parking_target reference")
    backoff = _number(value.get("backoff_distance_m", 0.0), "backoff_distance_m")
    if backoff < 0.0:
        raise ValueError("backoff_distance_m must be nonnegative")
    return {"schema_version": 1, "reference": "tag_table_side_edge",
            "backoff_distance_m": backoff}


def final_tag_distance(*, front_reference_x_m, parking_safety_distance_m,
                       longitudinal_docking_offset_m=None, marker_size_m=None,
                       parking_target=None):
    """Return tag-centre X in the aligned base frame at the requested stop.

The shared legacy parking identity must have zero safety distance in edge mode;
the explicit backoff is its sole longitudinal tuning parameter. An explicitly
supplied legacy longitudinal offset (even zero) is conflicting in edge mode.
"""
    front = _number(front_reference_x_m, "front_reference_x_m")
    distance = _number(parking_safety_distance_m, "parking_safety_distance_m")
    if distance < 0.0:
        raise ValueError("parking_safety_distance_m must be nonnegative")
    if parking_target is None:
        offset = (0.0 if longitudinal_docking_offset_m is None else
                  _number(longitudinal_docking_offset_m, "longitudinal_docking_offset_m"))
        target = front + distance + offset
    else:
        config = normalize_parking_target(parking_target)
        if longitudinal_docking_offset_m is not None:
            raise ValueError("parking_target conflicts with longitudinal offset")
        if distance != 0.0:
            raise ValueError("parking_target requires zero legacy parking_safety_distance_m")
        size = _number(marker_size_m, "marker_size_m")
        if size <= 0.0:
            raise ValueError("marker_size_m must be positive")
        target = front - size / 2.0 + config["backoff_distance_m"]
    if not math.isfinite(target) or target < 0.0:
        raise ValueError("parking target places the tag behind base_link or is nonfinite")
    return target
