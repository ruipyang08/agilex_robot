"""Strict, immutable loading for commissioned test-tube fixture profiles."""

from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence, Tuple

import numpy as np
import yaml


CommandGapSample = Tuple[float, float]
CommandGapSamples = Tuple[CommandGapSample, ...]
HsvTriplet = Tuple[int, int, int]
NormalizedRoi = Tuple[float, float, float, float]


def _require_mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError("{} must be a mapping".format(context))
    return value


def _require_exact_keys(
    value: Mapping[str, Any], expected: Sequence[str], context: str
) -> None:
    expected_keys = set(expected)
    actual_keys = set(value)
    missing = sorted(expected_keys - actual_keys)
    unexpected = sorted(actual_keys - expected_keys)
    if missing or unexpected:
        raise ValueError(
            "{} keys are invalid (missing={}, unexpected={})".format(
                context, missing, unexpected
            )
        )


def _string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("{} must be a non-empty string".format(context))
    return value


def _integer(value: Any, context: str, minimum: int = 0) -> int:
    if type(value) is not int:
        raise ValueError("{} must be an integer".format(context))
    if value < minimum:
        raise ValueError("{} must be at least {}".format(context, minimum))
    return value


def _real(
    value: Any, context: str, positive: bool = False, nonnegative: bool = False
) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError("{} must be numeric".format(context))
    result = float(value)
    if not np.isfinite(result):
        raise ValueError("{} must be finite".format(context))
    if positive and result <= 0.0:
        raise ValueError("{} must be greater than zero".format(context))
    if nonnegative and result < 0.0:
        raise ValueError("{} must be nonnegative".format(context))
    return result


def _fixed_vector(
    value: Any,
    context: str,
    length: int,
    positive: bool = False,
    unit_interval: bool = False,
) -> Tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("{} must contain exactly {} values".format(context, length))
    vector = tuple(
        _real(component, "{}[{}]".format(context, index), positive=positive)
        for index, component in enumerate(value)
    )
    if unit_interval and any(component < 0.0 or component > 1.0 for component in vector):
        raise ValueError("{} values must be in [0, 1]".format(context))
    return vector


def _hsv_triplet(value: Any, context: str) -> HsvTriplet:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("{} must contain exactly three integers".format(context))
    channels = tuple(
        _integer(component, "{}[{}]".format(context, index))
        for index, component in enumerate(value)
    )
    limits = (179, 255, 255)
    if any(channel > limit for channel, limit in zip(channels, limits)):
        raise ValueError("{} contains an out-of-range channel".format(context))
    return channels  # type: ignore[return-value]


def _normalized_roi(value: Any, context: str) -> NormalizedRoi:
    roi = _fixed_vector(
        value,
        context,
        length=4,
        unit_interval=True,
    )
    if roi[0] >= roi[2] or roi[1] >= roi[3]:
        raise ValueError("{} ROI bounds must be strictly increasing".format(context))
    return roi  # type: ignore[return-value]


def _command_gap_samples(value: Any, context: str) -> CommandGapSamples:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise ValueError("{} must contain at least two samples".format(context))
    samples = []
    for index, sample in enumerate(value):
        if not isinstance(sample, (list, tuple)) or len(sample) != 2:
            raise ValueError(
                "{}[{}] must contain command and gap".format(context, index)
            )
        samples.append(
            (
                _real(sample[0], "{}[{}].command".format(context, index)),
                _real(
                    sample[1],
                    "{}[{}].gap".format(context, index),
                    nonnegative=True,
                ),
            )
        )
    result = tuple(samples)
    if any(
        current[0] >= following[0] or current[1] >= following[1]
        for current, following in zip(result, result[1:])
    ):
        raise ValueError(
            "{} command and gap columns must be strictly increasing".format(context)
        )
    return result


@dataclass(frozen=True)
class FixtureProfile:
    """Validated physical and behavioral priors for one fixture version."""

    task_type: str
    target_cap_color: str

    table_surface_height_above_floor_m: float
    table_height_tolerance_m: float

    body_diameter_m: float
    body_height_m: float
    cap_diameter_m: float
    cap_height_m: float
    diameter_tolerance_m: float
    height_tolerance_m: float

    rows: int
    columns: int
    center_pitch_m: float
    rack_height_above_surface_m: float
    pitch_tolerance_m: float
    maximum_missing_slots: int
    minimum_layout_inliers: int
    maximum_lattice_rms_error_m: float

    color_frame: str
    roi_normalized: NormalizedRoi
    minimum_layout_saturation: int
    target_hsv_lower: HsvTriplet
    target_hsv_upper: HsvTriplet
    cap_top_height_band_m: float
    minimum_cap_circularity: float
    maximum_cap_axis_ratio: float
    maximum_target_temporal_spread_px: float
    maximum_target_center_error_px: float
    maximum_recentering_iterations: int

    fully_open_command: float
    close_command: float
    insertion_radial_clearance_m: float
    finger_thickness_m: float
    finger_width_m: float
    tube_clearance_margin_m: float
    command_gap_samples: CommandGapSamples

    preparation_clearance_above_cap_m: float
    pregrasp_clearance_m: float
    grasp_offset_below_cap_top_m: float
    lift_clearance_m: float
    downward_axis_tolerance_deg: float

    def __post_init__(self) -> None:
        _string(self.task_type, "task_type")
        _string(self.target_cap_color, "target_cap_color")
        _string(self.color_frame, "color_frame")

        if not isinstance(self.roi_normalized, tuple):
            raise ValueError("roi_normalized must be an immutable tuple")
        _normalized_roi(self.roi_normalized, "roi_normalized")
        _integer(
            self.minimum_layout_saturation,
            "minimum_layout_saturation",
            minimum=1,
        )
        if self.minimum_layout_saturation > 255:
            raise ValueError("minimum_layout_saturation must be at most 255")

        positive_fields = (
            "table_surface_height_above_floor_m",
            "table_height_tolerance_m",
            "body_diameter_m",
            "body_height_m",
            "cap_diameter_m",
            "cap_height_m",
            "diameter_tolerance_m",
            "height_tolerance_m",
            "center_pitch_m",
            "pitch_tolerance_m",
            "maximum_lattice_rms_error_m",
            "cap_top_height_band_m",
            "maximum_target_temporal_spread_px",
            "maximum_target_center_error_px",
            "insertion_radial_clearance_m",
            "finger_thickness_m",
            "finger_width_m",
            "tube_clearance_margin_m",
            "preparation_clearance_above_cap_m",
            "pregrasp_clearance_m",
            "grasp_offset_below_cap_top_m",
            "lift_clearance_m",
            "downward_axis_tolerance_deg",
        )
        for field_name in positive_fields:
            try:
                _real(getattr(self, field_name), field_name, positive=True)
            except ValueError as exc:
                if field_name == "insertion_radial_clearance_m":
                    raise ValueError("insertion gap must be greater than zero") from exc
                raise
        rack_height = _real(
            self.rack_height_above_surface_m,
            "rack_height_above_surface_m",
        )
        if rack_height < 0.0:
            raise ValueError("rack height must be nonnegative")

        _integer(self.rows, "rows", minimum=1)
        _integer(self.columns, "columns", minimum=1)
        _integer(self.maximum_missing_slots, "maximum_missing_slots")
        _integer(self.minimum_layout_inliers, "minimum_layout_inliers", minimum=1)
        _integer(
            self.maximum_recentering_iterations,
            "maximum_recentering_iterations",
            minimum=1,
        )

        if self.cap_diameter_m <= self.body_diameter_m:
            raise ValueError("cap_diameter_m must exceed body_diameter_m")
        if self.center_pitch_m - self.cap_diameter_m <= 0.0:
            raise ValueError("cap surface gap must be greater than zero")

        slot_count = self.rows * self.columns
        if self.maximum_missing_slots >= slot_count:
            raise ValueError("maximum_missing_slots must leave at least one layout slot")
        expected_inliers = slot_count - self.maximum_missing_slots
        if self.minimum_layout_inliers != expected_inliers:
            raise ValueError(
                "layout inliers must equal rows * columns - maximum_missing_slots"
            )

        if not isinstance(self.target_hsv_lower, tuple) or not isinstance(
            self.target_hsv_upper, tuple
        ):
            raise ValueError("HSV bounds must be immutable tuples")
        lower = _hsv_triplet(self.target_hsv_lower, "target_hsv_lower")
        upper = _hsv_triplet(self.target_hsv_upper, "target_hsv_upper")
        if any(low > high for low, high in zip(lower, upper)):
            raise ValueError("target HSV lower bounds must not exceed upper bounds")

        circularity = _real(
            self.minimum_cap_circularity,
            "minimum_cap_circularity",
            positive=True,
        )
        if circularity > 1.0:
            raise ValueError("minimum_cap_circularity must be at most one")
        axis_ratio = _real(
            self.maximum_cap_axis_ratio,
            "maximum_cap_axis_ratio",
            positive=True,
        )
        if axis_ratio < 1.0:
            raise ValueError("maximum_cap_axis_ratio must be at least one")

        close_command = _real(self.close_command, "close_command")
        fully_open_command = _real(self.fully_open_command, "fully_open_command")
        if fully_open_command <= close_command:
            raise ValueError("fully_open_command must exceed close_command")
        if not isinstance(self.command_gap_samples, tuple) or any(
            not isinstance(sample, tuple) for sample in self.command_gap_samples
        ):
            raise ValueError(
                "command_gap_samples must be an immutable tuple of tuples"
            )
        samples = _command_gap_samples(self.command_gap_samples, "command_gap_samples")
        if samples[0][0] < close_command or samples[-1][0] > fully_open_command:
            raise ValueError("command gap samples must lie within command limits")
        if self.grasp_offset_below_cap_top_m >= self.cap_height_m:
            raise ValueError("grasp offset must remain within the cap height")
        if self.downward_axis_tolerance_deg > 180.0:
            raise ValueError("downward_axis_tolerance_deg must be at most 180")

    @property
    def expected_cap_top_height_above_floor_m(self) -> float:
        return (
            self.table_surface_height_above_floor_m
            + self.rack_height_above_surface_m
            + self.body_height_m
            + self.cap_height_m
        )


_PROFILE_KEYS = (
    "task_type",
    "target_cap_color",
    "table",
    "tube",
    "layout",
    "perception",
    "gripper",
    "motion",
    "simulation",
)
_SECTION_KEYS = {
    "table": ("surface_height_above_floor_m", "height_tolerance_m"),
    "tube": (
        "body_diameter_m",
        "body_height_m",
        "cap_diameter_m",
        "cap_height_m",
        "diameter_tolerance_m",
        "height_tolerance_m",
    ),
    "layout": (
        "rows",
        "columns",
        "center_pitch_m",
        "rack_height_above_surface_m",
        "pitch_tolerance_m",
        "maximum_missing_slots",
        "minimum_layout_inliers",
        "maximum_lattice_rms_error_m",
    ),
    "perception": (
        "color_frame",
        "roi_normalized",
        "minimum_layout_saturation",
        "target_hsv_lower",
        "target_hsv_upper",
        "cap_top_height_band_m",
        "minimum_cap_circularity",
        "maximum_cap_axis_ratio",
        "maximum_target_temporal_spread_px",
        "maximum_target_center_error_px",
        "maximum_recentering_iterations",
    ),
    "gripper": (
        "fully_open_command",
        "close_command",
        "insertion_radial_clearance_m",
        "finger_thickness_m",
        "finger_width_m",
        "tube_clearance_margin_m",
        "command_gap_samples",
    ),
    "motion": (
        "preparation_clearance_above_cap_m",
        "pregrasp_clearance_m",
        "grasp_offset_below_cap_top_m",
        "lift_clearance_m",
        "downward_axis_tolerance_deg",
    ),
    "simulation": (
        "target_row",
        "target_column",
        "tube_mass_kg",
        "table_dimensions_m",
        "table_center_world_xyz_m",
        "target_cap_rgb",
        "non_target_cap_colors",
        "settling_steps",
        "maximum_settling_translation_m",
        "maximum_settling_tilt_deg",
    ),
}


def _validate_simulation(
    simulation: Mapping[str, Any], rows: int, columns: int, context: str
) -> None:
    target_row = _integer(simulation["target_row"], context + ".target_row")
    target_column = _integer(simulation["target_column"], context + ".target_column")
    if target_row >= rows or target_column >= columns:
        raise ValueError("{} target slot is outside the fixture".format(context))
    _real(simulation["tube_mass_kg"], context + ".tube_mass_kg", positive=True)
    _fixed_vector(
        simulation["table_dimensions_m"],
        context + ".table_dimensions_m",
        3,
        positive=True,
    )
    table_center = _fixed_vector(
        simulation["table_center_world_xyz_m"],
        context + ".table_center_world_xyz_m",
        3,
    )
    if table_center[2] <= 0.0:
        raise ValueError("{} table center z must be positive".format(context))
    _fixed_vector(
        simulation["target_cap_rgb"],
        context + ".target_cap_rgb",
        3,
        unit_interval=True,
    )

    colors = simulation["non_target_cap_colors"]
    if not isinstance(colors, list) or not colors:
        raise ValueError("{} non-target colors must be a non-empty list".format(context))
    names = set()
    for index, raw_color in enumerate(colors):
        color_context = "{}.non_target_cap_colors[{}]".format(context, index)
        color = _require_mapping(raw_color, color_context)
        _require_exact_keys(color, ("name", "rgb"), color_context)
        name = _string(color["name"], color_context + ".name")
        if name in names:
            raise ValueError("{} color names must be unique".format(context))
        names.add(name)
        _fixed_vector(
            color["rgb"], color_context + ".rgb", 3, unit_interval=True
        )

    _integer(simulation["settling_steps"], context + ".settling_steps", minimum=1)
    _real(
        simulation["maximum_settling_translation_m"],
        context + ".maximum_settling_translation_m",
        positive=True,
    )
    tilt = _real(
        simulation["maximum_settling_tilt_deg"],
        context + ".maximum_settling_tilt_deg",
        positive=True,
    )
    if tilt > 180.0:
        raise ValueError("{} settling tilt must be at most 180".format(context))


def _load_profile(raw_profile: Any, context: str) -> FixtureProfile:
    profile = _require_mapping(raw_profile, context)
    _require_exact_keys(profile, _PROFILE_KEYS, context)

    sections = {}
    for section_name, expected_keys in _SECTION_KEYS.items():
        section = _require_mapping(profile[section_name], context + "." + section_name)
        _require_exact_keys(section, expected_keys, context + "." + section_name)
        sections[section_name] = section

    table = sections["table"]
    tube = sections["tube"]
    layout = sections["layout"]
    perception = sections["perception"]
    gripper = sections["gripper"]
    motion = sections["motion"]

    rows = _integer(layout["rows"], context + ".layout.rows", minimum=1)
    columns = _integer(layout["columns"], context + ".layout.columns", minimum=1)
    _validate_simulation(sections["simulation"], rows, columns, context + ".simulation")

    return FixtureProfile(
        task_type=_string(profile["task_type"], context + ".task_type"),
        target_cap_color=_string(
            profile["target_cap_color"], context + ".target_cap_color"
        ),
        table_surface_height_above_floor_m=_real(
            table["surface_height_above_floor_m"],
            context + ".table.surface_height_above_floor_m",
            positive=True,
        ),
        table_height_tolerance_m=_real(
            table["height_tolerance_m"],
            context + ".table.height_tolerance_m",
            positive=True,
        ),
        body_diameter_m=_real(
            tube["body_diameter_m"],
            context + ".tube.body_diameter_m",
            positive=True,
        ),
        body_height_m=_real(
            tube["body_height_m"], context + ".tube.body_height_m", positive=True
        ),
        cap_diameter_m=_real(
            tube["cap_diameter_m"], context + ".tube.cap_diameter_m", positive=True
        ),
        cap_height_m=_real(
            tube["cap_height_m"], context + ".tube.cap_height_m", positive=True
        ),
        diameter_tolerance_m=_real(
            tube["diameter_tolerance_m"],
            context + ".tube.diameter_tolerance_m",
            positive=True,
        ),
        height_tolerance_m=_real(
            tube["height_tolerance_m"],
            context + ".tube.height_tolerance_m",
            positive=True,
        ),
        rows=rows,
        columns=columns,
        center_pitch_m=_real(
            layout["center_pitch_m"],
            context + ".layout.center_pitch_m",
            positive=True,
        ),
        rack_height_above_surface_m=_real(
            layout["rack_height_above_surface_m"],
            context + ".layout.rack_height_above_surface_m",
        ),
        pitch_tolerance_m=_real(
            layout["pitch_tolerance_m"],
            context + ".layout.pitch_tolerance_m",
            positive=True,
        ),
        maximum_missing_slots=_integer(
            layout["maximum_missing_slots"],
            context + ".layout.maximum_missing_slots",
        ),
        minimum_layout_inliers=_integer(
            layout["minimum_layout_inliers"],
            context + ".layout.minimum_layout_inliers",
            minimum=1,
        ),
        maximum_lattice_rms_error_m=_real(
            layout["maximum_lattice_rms_error_m"],
            context + ".layout.maximum_lattice_rms_error_m",
            positive=True,
        ),
        color_frame=_string(
            perception["color_frame"], context + ".perception.color_frame"
        ),
        roi_normalized=_normalized_roi(
            perception["roi_normalized"],
            context + ".perception.roi_normalized",
        ),
        minimum_layout_saturation=_integer(
            perception["minimum_layout_saturation"],
            context + ".perception.minimum_layout_saturation",
            minimum=1,
        ),
        target_hsv_lower=_hsv_triplet(
            perception["target_hsv_lower"],
            context + ".perception.target_hsv_lower",
        ),
        target_hsv_upper=_hsv_triplet(
            perception["target_hsv_upper"],
            context + ".perception.target_hsv_upper",
        ),
        cap_top_height_band_m=_real(
            perception["cap_top_height_band_m"],
            context + ".perception.cap_top_height_band_m",
            positive=True,
        ),
        minimum_cap_circularity=_real(
            perception["minimum_cap_circularity"],
            context + ".perception.minimum_cap_circularity",
            positive=True,
        ),
        maximum_cap_axis_ratio=_real(
            perception["maximum_cap_axis_ratio"],
            context + ".perception.maximum_cap_axis_ratio",
            positive=True,
        ),
        maximum_target_temporal_spread_px=_real(
            perception["maximum_target_temporal_spread_px"],
            context + ".perception.maximum_target_temporal_spread_px",
            positive=True,
        ),
        maximum_target_center_error_px=_real(
            perception["maximum_target_center_error_px"],
            context + ".perception.maximum_target_center_error_px",
            positive=True,
        ),
        maximum_recentering_iterations=_integer(
            perception["maximum_recentering_iterations"],
            context + ".perception.maximum_recentering_iterations",
            minimum=1,
        ),
        fully_open_command=_real(
            gripper["fully_open_command"], context + ".gripper.fully_open_command"
        ),
        close_command=_real(
            gripper["close_command"], context + ".gripper.close_command"
        ),
        insertion_radial_clearance_m=_real(
            gripper["insertion_radial_clearance_m"],
            context + ".gripper.insertion_radial_clearance_m",
        ),
        finger_thickness_m=_real(
            gripper["finger_thickness_m"],
            context + ".gripper.finger_thickness_m",
            positive=True,
        ),
        finger_width_m=_real(
            gripper["finger_width_m"],
            context + ".gripper.finger_width_m",
            positive=True,
        ),
        tube_clearance_margin_m=_real(
            gripper["tube_clearance_margin_m"],
            context + ".gripper.tube_clearance_margin_m",
            positive=True,
        ),
        command_gap_samples=_command_gap_samples(
            gripper["command_gap_samples"],
            context + ".gripper.command_gap_samples",
        ),
        preparation_clearance_above_cap_m=_real(
            motion["preparation_clearance_above_cap_m"],
            context + ".motion.preparation_clearance_above_cap_m",
            positive=True,
        ),
        pregrasp_clearance_m=_real(
            motion["pregrasp_clearance_m"],
            context + ".motion.pregrasp_clearance_m",
            positive=True,
        ),
        grasp_offset_below_cap_top_m=_real(
            motion["grasp_offset_below_cap_top_m"],
            context + ".motion.grasp_offset_below_cap_top_m",
            positive=True,
        ),
        lift_clearance_m=_real(
            motion["lift_clearance_m"],
            context + ".motion.lift_clearance_m",
            positive=True,
        ),
        downward_axis_tolerance_deg=_real(
            motion["downward_axis_tolerance_deg"],
            context + ".motion.downward_axis_tolerance_deg",
            positive=True,
        ),
    )


def load_profile_registry(path: Path) -> Mapping[str, FixtureProfile]:
    """Load a profile registry, rejecting all unreviewed schema variation."""

    registry_path = Path(path)
    raw = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    root = _require_mapping(raw, "fixture profile registry")
    _require_exact_keys(root, ("tube_fixture_profiles",), "fixture profile registry")
    raw_profiles = _require_mapping(
        root["tube_fixture_profiles"], "tube_fixture_profiles"
    )

    profiles = {}
    for raw_name, raw_profile in raw_profiles.items():
        name = _string(raw_name, "profile name")
        profiles[name] = profile_from_mapping(name, raw_profile)
    return MappingProxyType(profiles)


def profile_from_mapping(
    profile_id: str, raw_profile: Mapping[str, Any]
) -> FixtureProfile:
    """Validate one ROS-parameter profile through the file-loader path."""

    name = _string(profile_id, "profile id")
    profile = _require_mapping(
        raw_profile, "tube_fixture_profiles.{}".format(name)
    )
    return _load_profile(profile, "tube_fixture_profiles.{}".format(name))
