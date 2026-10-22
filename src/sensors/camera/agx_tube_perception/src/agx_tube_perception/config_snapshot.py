"""Pure immutable perception snapshots and bounded, content-bound admission.

The full graph config_id is supplied by the trusted onboard workflow. This module
verifies the wire projection digest and prevents rebinding that graph identity;
it does not claim to reconstruct the full graph from its perception subset.
"""
from dataclasses import dataclass
import hashlib
from math import floor, isfinite
from numbers import Integral, Real
import re
import struct
import threading
from typing import Tuple

from .detector import DetectorConfig
from .service_logic import compute_lift_target, validate_mission_context

# Message declaration order is a versioned cross-language byte contract.
PROJECTION_FIELDS = (
    ('schema_version', 'uint32'),
    ('planning_frame', 'string'),
    ('color_frame', 'string'),
    ('lift_state_topic', 'string'),
    ('table_surface_height_above_floor_m', 'float64'),
    ('table_height_tolerance_m', 'float64'),
    ('arm_base_height_at_zero_lift_above_floor_m', 'float64'),
    ('lift_minimum_position_m', 'float64'),
    ('lift_maximum_position_m', 'float64'),
    ('lift_target_tolerance_m', 'float64'),
    ('lift_state_max_age_sec', 'float64'),
    ('body_diameter_m', 'float64'),
    ('body_height_m', 'float64'),
    ('cap_diameter_m', 'float64'),
    ('cap_height_m', 'float64'),
    ('diameter_tolerance_m', 'float64'),
    ('height_tolerance_m', 'float64'),
    ('rack_height_above_surface_m', 'float64'),
    ('rows', 'int32'),
    ('columns', 'int32'),
    ('maximum_missing_slots', 'int32'),
    ('minimum_layout_inliers', 'int32'),
    ('center_pitch_m', 'float64'),
    ('pitch_tolerance_m', 'float64'),
    ('maximum_lattice_rms_error_m', 'float64'),
    ('roi_normalized', 'float64[4]'),
    ('minimum_layout_saturation', 'int32'),
    ('target_hsv_lower', 'int32[3]'),
    ('target_hsv_upper', 'int32[3]'),
    ('cap_top_height_band_m', 'float64'),
    ('minimum_cap_circularity', 'float64'),
    ('maximum_cap_axis_ratio', 'float64'),
    ('maximum_target_temporal_spread_px', 'float64'),
    ('maximum_target_center_error_px', 'float64'),
    ('maximum_recentering_iterations', 'int32'),
    ('required_consistent_frames', 'int32'),
    ('synchronizer_queue_size', 'int32'),
    ('max_sync_skew_sec', 'float64'),
    ('detection_timeout_sec', 'float64'),
    ('maximum_frame_age_sec', 'float64'),
    ('max_interframe_spread_m', 'float64'),
    ('workspace_min_xyz_m', 'float64[3]'),
    ('workspace_max_xyz_m', 'float64[3]'),
)


@dataclass(frozen=True)
class PerceptionConfig:
    """Narrow geometry and perception facts, with no control permissions."""
    schema_version: int
    planning_frame: str
    color_frame: str
    lift_state_topic: str
    table_surface_height_above_floor_m: float
    table_height_tolerance_m: float
    arm_base_height_at_zero_lift_above_floor_m: float
    lift_minimum_position_m: float
    lift_maximum_position_m: float
    lift_target_tolerance_m: float
    lift_state_max_age_sec: float
    body_diameter_m: float
    body_height_m: float
    cap_diameter_m: float
    cap_height_m: float
    diameter_tolerance_m: float
    height_tolerance_m: float
    rack_height_above_surface_m: float
    rows: int
    columns: int
    maximum_missing_slots: int
    minimum_layout_inliers: int
    center_pitch_m: float
    pitch_tolerance_m: float
    maximum_lattice_rms_error_m: float
    roi_normalized: Tuple[float, float, float, float]
    minimum_layout_saturation: int
    target_hsv_lower: Tuple[int, int, int]
    target_hsv_upper: Tuple[int, int, int]
    cap_top_height_band_m: float
    minimum_cap_circularity: float
    maximum_cap_axis_ratio: float
    maximum_target_temporal_spread_px: float
    maximum_target_center_error_px: float
    maximum_recentering_iterations: int
    required_consistent_frames: int
    synchronizer_queue_size: int
    max_sync_skew_sec: float
    detection_timeout_sec: float
    maximum_frame_age_sec: float
    max_interframe_spread_m: float
    workspace_min_xyz_m: Tuple[float, float, float]
    workspace_max_xyz_m: Tuple[float, float, float]

    def __post_init__(self):
        # Normalize caller-owned lists and numpy scalar values before freezing.
        for name, kind in PROJECTION_FIELDS:
            value = getattr(self, name)
            if '[' in kind:
                count = int(kind.split('[')[1][:-1])
                if isinstance(value, (str, bytes)):
                    raise ValueError(name + ' requires a fixed numeric array')
                try:
                    value = tuple(value)
                except TypeError:
                    raise ValueError(name + ' requires a fixed numeric array')
                if len(value) != count:
                    raise ValueError(name + ' has invalid array length')
                value = tuple(_scalar(item, kind.split('[')[0], name) for item in value)
            else:
                value = _scalar(value, kind, name)
            object.__setattr__(self, name, value)
        if self.schema_version != 1:
            raise ValueError('unsupported perception schema_version')
        if not self.lift_state_topic.startswith('/'):
            raise ValueError('lift_state_topic must be absolute')
        positive = (
            'table_surface_height_above_floor_m', 'table_height_tolerance_m',
            'arm_base_height_at_zero_lift_above_floor_m', 'lift_state_max_age_sec',
            'body_diameter_m', 'body_height_m', 'cap_diameter_m', 'cap_height_m',
            'diameter_tolerance_m', 'height_tolerance_m', 'center_pitch_m',
            'pitch_tolerance_m', 'maximum_lattice_rms_error_m', 'cap_top_height_band_m',
            'maximum_target_temporal_spread_px', 'maximum_target_center_error_px',
            'maximum_recentering_iterations', 'required_consistent_frames',
            'synchronizer_queue_size', 'max_sync_skew_sec', 'detection_timeout_sec',
            'maximum_frame_age_sec', 'max_interframe_spread_m', 'rows', 'columns',
            'minimum_layout_inliers',
        )
        if any(getattr(self, name) <= 0 for name in positive):
            raise ValueError('perception dimensions, counts and tolerances must be positive')
        if self.required_consistent_frames > 128 or self.synchronizer_queue_size > 128:
            raise ValueError('perception history counts exceed capacity 128')
        if self.detection_timeout_sec > 3600:
            raise ValueError('perception wait exceeds maximum 3600 seconds')
        if self.rows * self.columns > 4096:
            raise ValueError('rack layout exceeds 4096 slot capacity')
        if any(getattr(self, name) < 0 for name in (
            'lift_minimum_position_m', 'lift_maximum_position_m',
            'lift_target_tolerance_m', 'rack_height_above_surface_m', 'maximum_missing_slots',
        )):
            raise ValueError('perception bounds and clearances must be nonnegative')
        if (self.cap_diameter_m <= self.body_diameter_m or
                self.center_pitch_m <= self.cap_diameter_m or
                self.pitch_tolerance_m >= self.center_pitch_m or
                self.maximum_lattice_rms_error_m > self.pitch_tolerance_m):
            raise ValueError('inconsistent perception geometry')
        if self.minimum_layout_inliers != self.rows * self.columns - self.maximum_missing_slots:
            raise ValueError('layout inliers disagree with missing slots')
        if any(low >= high for low, high in zip(self.workspace_min_xyz_m, self.workspace_max_xyz_m)):
            raise ValueError('workspace bounds must be increasing')
        # Reuse the actual detector's HSV, ROI, saturation, and shape validation.
        self.detector_config()
        self.lift_target()
        if self.lift_target_tolerance_m * 1000 >= (1 << 63) - 1:
            raise ValueError('lift tolerance exceeds native integer range')

    def detector_config(self):
        defaults = DetectorConfig()
        return DetectorConfig(
            roi_normalized=self.roi_normalized,
            minimum_layout_saturation=self.minimum_layout_saturation,
            green_hsv_lower=self.target_hsv_lower, green_hsv_upper=self.target_hsv_upper,
            morphology_kernel_px=defaults.morphology_kernel_px,
            morphology_iterations=defaults.morphology_iterations,
            min_area_px=defaults.min_area_px, max_area_px=defaults.max_area_px,
            min_circularity=self.minimum_cap_circularity, max_axis_ratio=self.maximum_cap_axis_ratio,
            max_interframe_spread_m=self.max_interframe_spread_m,
        )

    def lift_target(self):
        return compute_lift_target(self.table_surface_height_above_floor_m,
                                   self.arm_base_height_at_zero_lift_above_floor_m,
                                   self.lift_minimum_position_m, self.lift_maximum_position_m)

    @property
    def lift_tolerance_mm(self):
        return int(floor(self.lift_target_tolerance_m * 1000.0 + 0.5))


def _scalar(value, kind, name):
    if kind == 'string':
        if not isinstance(value, str) or not value.strip():
            raise ValueError(name + ' must be a nonempty string')
        try:
            encoded = value.encode('utf-8')
        except UnicodeError as error:
            raise ValueError(name + ' is not valid UTF8') from error
        if len(encoded) > 4096 or '\x00' in value:
            raise ValueError(name + ' exceeds string limit or contains NUL')
        return value
    if isinstance(value, bool):
        raise ValueError(name + ' cannot be a boolean')
    if kind in ('int32', 'uint32'):
        if not isinstance(value, Integral):
            raise ValueError(name + ' must be an integer')
        low, high = (0, (1 << 32) - 1) if kind == 'uint32' else (-(1 << 31), (1 << 31) - 1)
        if not low <= value <= high:
            raise ValueError(name + ' is outside wire integer range')
        return int(value)
    if not isinstance(value, Real) or not isfinite(float(value)):
        raise ValueError(name + ' must be a finite real')
    # IEEE negative zero must not create a second identity for the same fact.
    return float(value) if value else 0.0


def _string_bytes(value):
    encoded = _scalar(value, 'string', 'identity').encode('utf-8')
    return struct.pack('<I', len(encoded)) + encoded


def projection_bytes(perception, fixture_instance_id, fixture_profile_id, target_cap_color):
    if not isinstance(perception, PerceptionConfig):
        raise ValueError('projection requires validated PerceptionConfig')
    content = b'agx_tube_perception_projection_v1\x00'
    for value in (fixture_instance_id, fixture_profile_id, target_cap_color):
        content += _string_bytes(value)
    for name, kind in PROJECTION_FIELDS:
        value = getattr(perception, name)
        if kind == 'string':
            content += _string_bytes(value)
        else:
            count = int(kind.split('[')[1][:-1]) if '[' in kind else 1
            base = kind.split('[')[0]
            values = value if '[' in kind else (value,)
            content += struct.pack('<' + {'float64': 'd', 'int32': 'i', 'uint32': 'I'}[base] * count, *values)
    return content


def projection_digest(perception, fixture_instance_id, fixture_profile_id, target_cap_color):
    return 'sha256:' + hashlib.sha256(projection_bytes(
        perception, fixture_instance_id, fixture_profile_id, target_cap_color)).hexdigest()


@dataclass(frozen=True)
class PerceptionRequestContext:
    config_id: str
    fixture_instance_id: str
    fixture_profile_id: str
    projection_sha256: str
    target_cap_color: str
    perception: PerceptionConfig

    @property
    def history_key(self):
        return (self.config_id, self.fixture_instance_id, self.fixture_profile_id,
                self.projection_sha256, self.target_cap_color)


def _from_wire(value):
    try:
        return PerceptionConfig(**{name: getattr(value, name) for name, _ in PROJECTION_FIELDS})
    except (AttributeError, TypeError) as error:
        raise ValueError('malformed typed perception projection') from error


class ContextRegistry:
    """No eviction: capacity exhaustion fails closed, preserving identity bindings."""
    def __init__(self, maximum_contexts=16):
        if type(maximum_contexts) is not int or not 1 <= maximum_contexts <= 128:
            raise ValueError('context capacity must be in [1, 128]')
        self._maximum_contexts = maximum_contexts
        self._contexts = {}
        self._graph_bindings = {}
        self._lock = threading.Lock()

    def admit(self, request, validate_transport=None):
        config_id = _scalar(request.config_id, 'string', 'config_id')
        if re.fullmatch(r'sha256:[0-9a-f]{64}', config_id) is None:
            raise ValueError('config_id must be a SHA256 identity')
        instance = _scalar(request.fixture_instance_id, 'string', 'fixture_instance_id')
        profile = _scalar(request.fixture_profile_id, 'string', 'fixture_profile_id')
        color = _scalar(request.target_cap_color, 'string', 'target_cap_color')
        perception = _from_wire(request.perception)
        digest = projection_digest(perception, instance, profile, color)
        if digest != request.projection_sha256:
            raise ValueError('perception projection digest mismatch')
        if validate_transport is not None:
            validate_transport(perception)
        context = PerceptionRequestContext(config_id, instance, profile, digest, color, perception)
        # Same full graph may select another classifier. Geometry, frames and all
        # non-classifier facts must remain identical across those selections.
        invariant = (instance, profile) + tuple(
            getattr(perception, name) for name, _ in PROJECTION_FIELDS
            if name not in ('target_hsv_lower', 'target_hsv_upper'))
        color_key = (config_id, color)
        with self._lock:
            if config_id in self._graph_bindings and self._graph_bindings[config_id] != invariant:
                raise ValueError('config_id already bound to different perception geometry or fixture')
            if color_key in self._contexts:
                existing = self._contexts[color_key]
                if existing != context:
                    raise ValueError('config_id/color already bound to a different projection')
                return existing
            if len(self._contexts) >= self._maximum_contexts:
                raise ValueError('configured perception context capacity exhausted')
            self._graph_bindings[config_id] = invariant
            self._contexts[color_key] = context
            return context


def legacy_request_context(profile_id, profile, mission):
    """Validate the legacy contract, then freeze the same narrow projection."""
    context = validate_mission_context(mission, profile_id, profile.target_cap_color)
    if profile.target_cap_color != 'green' or mission['target_cap_color'] != 'green':
        raise ValueError('commissioned target_cap_color must be green')
    values = {}
    for name, _ in PROJECTION_FIELDS:
        if hasattr(profile, name):
            values[name] = getattr(profile, name)
        elif name in mission['perception']:
            values[name] = mission['perception'][name]
    values.update(schema_version=1, planning_frame=context['planning_frame'],
                  lift_state_topic=context['lift']['state_topic'],
                  arm_base_height_at_zero_lift_above_floor_m=context['lift']['arm_base_height_at_zero_lift_above_floor_m'],
                  lift_minimum_position_m=context['lift']['minimum_position_m'],
                  lift_maximum_position_m=context['lift']['maximum_position_m'],
                  lift_target_tolerance_m=context['lift']['target_tolerance_m'],
                  lift_state_max_age_sec=context['lift']['state_max_age_sec'])
    perception = PerceptionConfig(**values)
    instance = 'legacy:' + profile_id
    digest = projection_digest(perception, instance, profile_id, profile.target_cap_color)
    return PerceptionRequestContext('legacy:' + digest, instance, profile_id, digest,
                                    profile.target_cap_color, perception)
