"""Installed geometry/limits shared by previews and operation-bound execution.

Explicit experimental execution is restricted to simulation by each host. A
policy never supplies a route token, command grant or hardware qualification.
"""
from dataclasses import dataclass
from hashlib import sha256
import json
import math
from numbers import Real
from pathlib import PurePosixPath
from typing import Tuple


def _keys(value, keys, name):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError("{} requires exactly {}".format(name, sorted(keys)))


def _number(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{} must be numeric".format(name))
    result = float(value)
    if not math.isfinite(result) or result < 0 or (positive and result == 0):
        raise ValueError("{} must be finite and {}".format(name, "positive" if positive else "nonnegative"))
    return result


@dataclass(frozen=True)
class MotionPolicy:
    model_id: str
    frame_id: str
    execution_enabled: bool
    footprint_xy_m: Tuple[Tuple[float, float], ...]
    error_budget_m: Tuple[Tuple[str, float], ...]
    soft_clearance_m: float
    max_translation_step_m: float
    max_rotation_step_rad: float
    motion_limits: Tuple[Tuple[str, float], ...]
    evidence_ids: Tuple[str, ...]
    digest: str
    footprint_sources: Tuple[Tuple[str, str], ...] = ()
    qualification: str = "unqualified"

    @property
    def hard_padding_m(self):
        return sum(value for _, value in self.error_budget_m)

    def require_execution(self, mode=None):
        if not self.execution_enabled:
            raise ValueError("motion policy execution is disabled")
        if self.qualification != 'experimental_simulation' or mode != 'simulation':
            raise ValueError("experimental policy execution requires simulation mode")


def _validate_candidate_polygon(points):
    # Keep the one-metre body-radius bound assumed by primitive integration.
    if any(math.hypot(x, y) > 1.0 for x, y in points):
        raise ValueError('candidate exceeds supported one-metre radius')
    orientation = None
    for i, (x, y) in enumerate(points):
        u, v = points[(i+1) % len(points)]
        a, b = points[(i+2) % len(points)]
        turn = (u-x)*(b-v) - (v-y)*(a-u)
        if abs(turn) < 1e-12:
            raise ValueError('candidate has duplicate or collinear vertices')
        if orientation is None:
            orientation = 1 if turn > 0 else -1
        if turn*orientation <= 0 or any(
                ((u-x)*(py-y)-(v-y)*(px-x))*orientation < -1e-12
                for px, py in points + ((0., 0.),)):
            raise ValueError('candidate must be convex, ordered and contain base origin')


def parse_motion_policy(raw):
    if not isinstance(raw, dict) or type(raw.get('schema_version')) is not int or raw['schema_version'] not in (1, 2):
        raise ValueError("unsupported policy schema")
    version = raw['schema_version']
    keys = ("schema_version", "model_id", "frame_id", "execution_enabled",
            "qualification", "footprint_xy_m", "error_budget_m", "soft_clearance_m",
            "sampling", "motion_limits", "evidence_ids")
    _keys(raw, keys + (("footprint_provenance",) if version == 2 else ()), "policy")
    experimental = (version == 2 and raw['execution_enabled'] is True and
                    raw['qualification'] == 'experimental_simulation')
    if not experimental and (raw['execution_enabled'] is not False or raw['qualification'] != 'unqualified'):
        raise ValueError("preview policy cannot authorize execution or qualification")
    if raw['frame_id'] != 'base_link':
        raise ValueError("unsupported footprint frame")
    if not isinstance(raw['model_id'], str) or not raw['model_id'].strip():
        raise ValueError("model_id required")
    # Version one stays byte-compatible. Version two permits diagnostic candidates.
    points = raw['footprint_xy_m']
    expected = ((-.5, -.33), (.5, -.33), (.5, .33), (-.5, .33))
    if not isinstance(points, (list, tuple)) or not 3 <= len(points) <= 64:
        raise ValueError("footprint requires 3 to 64 vertices")
    normalized = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2 or any(
                isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v) for v in point):
            raise ValueError("finite footprint XY pairs required")
        normalized.append(tuple(float(v) for v in point))
    if version == 1 and tuple(normalized) != expected:
        raise ValueError("footprint changes require a separately qualified schema")
    sources = ()
    if version == 2:
        expected = tuple(normalized)
        _validate_candidate_polygon(expected)
        provenance = raw['footprint_provenance']
        _keys(provenance, ('scope', 'source_sha256'), 'footprint provenance')
        if provenance['scope'] != 'diagnostic_candidate':
            raise ValueError('footprint provenance cannot claim qualification')
        hashes = provenance['source_sha256']
        if not isinstance(hashes, dict) or not hashes:
            raise ValueError('footprint source hashes required')
        for path, digest in hashes.items():
            if (not isinstance(path, str) or not path or PurePosixPath(path).is_absolute() or
                    '..' in PurePosixPath(path).parts or '\\' in path or
                    not isinstance(digest, str) or len(digest) != 64 or
                    any(c not in '0123456789abcdef' for c in digest)):
                raise ValueError('portable footprint source paths and SHA256 digests required')
        sources = tuple(sorted(hashes.items()))
    _keys(raw['error_budget_m'], ('geometry', 'localization', 'tracking', 'reserve'), 'error budget')
    budget = tuple((key, _number(value, key)) for key, value in sorted(raw['error_budget_m'].items()))
    if not math.isfinite(sum(value for _, value in budget)):
        raise ValueError('combined error budget must be finite')
    _keys(raw['sampling'], ('max_translation_step_m', 'max_rotation_step_rad'), 'sampling')
    sampling = {k: _number(v, k, True) for k, v in raw['sampling'].items()}
    ceilings = {'max_speed_mps': .5, 'acceleration_mps2': .25, 'deceleration_mps2': .3,
                'max_yaw_rate_radps': .55, 'angular_acceleration_radps2': .4,
                'curvature_rate_per_m_s': .5, 'lateral_acceleration_mps2': .2,
                'min_turn_radius_m': .4764, 'steering_settle_timeout_s': 2., 'stop_dwell_s': .12}
    _keys(raw['motion_limits'], ceilings, 'motion limits')
    limits = {k: _number(v, k, True) for k, v in raw['motion_limits'].items()}
    for key, ceiling in ceilings.items():
        if key in ('min_turn_radius_m', 'stop_dwell_s'):
            if limits[key] < ceiling:
                raise ValueError('{} below supported minimum'.format(key))
        elif limits[key] > ceiling:
            raise ValueError('{} exceeds supported ceiling'.format(key))
    evidence = raw['evidence_ids']
    if not isinstance(evidence, list) or not evidence or any(
            not isinstance(v, str) or not v.strip() for v in evidence) or len(set(evidence)) != len(evidence):
        raise ValueError('unique, nonempty evidence IDs required')
    soft = _number(raw['soft_clearance_m'], 'soft clearance')
    canonical = dict(raw, footprint_xy_m=expected, error_budget_m=dict(budget),
                     sampling=sampling, motion_limits=limits, soft_clearance_m=soft,
                     evidence_ids=sorted(evidence))
    digest = sha256(json.dumps(canonical, sort_keys=True, separators=(',', ':'),
                               allow_nan=False).encode('utf-8')).hexdigest()
    return MotionPolicy(raw['model_id'], raw['frame_id'], raw['execution_enabled'], expected, budget,
                        soft, sampling['max_translation_step_m'], sampling['max_rotation_step_rad'],
                        tuple(sorted(limits.items())), tuple(sorted(evidence)), digest, sources, raw["qualification"])
