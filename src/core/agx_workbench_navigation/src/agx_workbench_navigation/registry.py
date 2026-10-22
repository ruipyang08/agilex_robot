"""Validated, map-bound workbench registry contracts.

This module is deliberately ROS-free so that the same bytes and policy gates
can be validated in unit tests, a ROS1 container, and the onboard computer.
"""

from __future__ import annotations

from dataclasses import dataclass
from copy import deepcopy
import hashlib
import json
import math
from numbers import Integral, Real
import os
from pathlib import Path
import re
import tempfile
from types import MappingProxyType
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple, Union

import yaml

from .parking_target import final_tag_distance, normalize_parking_target


SCHEMA_VERSION = 1
EXPECTED_CAMERA_FRAME = "cam2_color_optical_frame"
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class RegistryError(ValueError):
    """A registry or one of its bound identities is invalid."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise RegistryError("duplicate YAML key: {0}".format(key))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _load_yaml(source: Union[Path, str]) -> Mapping[str, Any]:
    path = Path(source)
    try:
        loaded = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    except RegistryError:
        raise
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise RegistryError("cannot load YAML {0}: {1}".format(path, exc))
    if not isinstance(loaded, Mapping):
        raise RegistryError("{0} must contain a YAML mapping".format(path))
    return loaded


def _as_mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise RegistryError("{0} must be a mapping".format(name))
    return value


def _strict_keys(
    value: Mapping[str, Any],
    name: str,
    required: Iterable[str],
    optional: Iterable[str] = (),
) -> None:
    required_set = set(required)
    allowed = required_set | set(optional)
    unknown = set(value) - allowed
    missing = required_set - set(value)
    if unknown:
        raise RegistryError(
            "{0} has unknown fields: {1}".format(name, ", ".join(sorted(unknown)))
        )
    if missing:
        raise RegistryError(
            "{0} is missing fields: {1}".format(name, ", ".join(sorted(missing)))
        )


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RegistryError("{0} must be a non-empty string".format(name))
    return value


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise RegistryError("{0} must be a finite numeric value".format(name))
    try:
        converted = float(value)
    except (OverflowError, ValueError):
        raise RegistryError("{0} must be a finite numeric value".format(name))
    if not math.isfinite(converted):
        raise RegistryError("{0} must be a finite numeric value".format(name))
    return converted


def _positive(value: Any, name: str) -> float:
    converted = _number(value, name)
    if converted <= 0.0:
        raise RegistryError("{0} must be positive".format(name))
    return converted


def _nonnegative(value: Any, name: str) -> float:
    converted = _number(value, name)
    if converted < 0.0:
        raise RegistryError("{0} must be nonnegative".format(name))
    return converted


def _integer(value: Any, name: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise RegistryError("{0} must be an integer".format(name))
    converted = int(value)
    if converted < minimum:
        raise RegistryError(
            "{0} must be at least {1}".format(name, minimum)
        )
    return converted


def _numeric_sequence(
    value: Any,
    name: str,
    length: int,
) -> Tuple[float, ...]:
    if (
        isinstance(value, (str, bytes))
        or not isinstance(value, Sequence)
        or len(value) != length
    ):
        raise RegistryError(
            "{0} must contain exactly {1} numeric values".format(name, length)
        )
    return tuple(_number(item, "{0}[{1}]".format(name, index)) for index, item in enumerate(value))


def _sha256(value: Any, name: str) -> str:
    text = _string(value, name)
    if SHA256_PATTERN.fullmatch(text) is None:
        raise RegistryError("{0} must be a lowercase SHA-256 digest".format(name))
    return text


def _file_sha256(path: Path, name: str) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as exc:
        raise RegistryError("cannot hash {0} {1}: {2}".format(name, path, exc))


def _canonical_bytes(value: Any) -> bytes:
    try:
        return json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise RegistryError("identity is not canonical JSON: {0}".format(exc))


def canonical_sha256(value: Any) -> str:
    """Return the SHA-256 of canonical, sorted JSON bytes."""

    return hashlib.sha256(_canonical_bytes(value)).hexdigest()


def _portable_path(path: Path, base: Path, name: str) -> str:
    try:
        relative = os.path.relpath(str(path.resolve()), str(base.resolve()))
    except (OSError, ValueError) as exc:
        raise RegistryError("cannot make {0} portable: {1}".format(name, exc))
    if Path(relative).is_absolute():
        raise RegistryError("{0} must be stored as a relative path".format(name))
    return Path(relative).as_posix()


def _canonical_payload_json(payload: Mapping[str, Any]) -> str:
    return _canonical_bytes(payload).decode("utf-8")


def _deep_freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType(
            {key: _deep_freeze(item) for key, item in value.items()}
        )
    if isinstance(value, (list, tuple)):
        return tuple(_deep_freeze(item) for item in value)
    return value


@dataclass(frozen=True)
class CameraIdentity:
    identity_sha256: str
    serial_no: str
    expected_optical_frame: str
    mount_state: str
    _payload_json: str

    @property
    def payload(self) -> Dict[str, Any]:
        return json.loads(self._payload_json)


@dataclass(frozen=True)
class ParkingIdentity:
    identity_sha256: str
    front_reference_x_m: float
    parking_safety_distance_m: float
    max_blind_travel_m: float
    absolute_max_blind_travel_m: float
    calibration_state: str
    _payload_json: str

    @property
    def payload(self) -> Dict[str, Any]:
        return json.loads(self._payload_json)


@dataclass(frozen=True)
class LidarIdentity:
    profile_identity_sha256: str
    envelope_identity_sha256: str
    raw_configuration_sha256: str
    _payload_json: str

    @property
    def payload(self) -> Dict[str, Any]:
        return json.loads(self._payload_json)


@dataclass(frozen=True)
class MapIdentity:
    map_id: str
    yaml_path: str
    image_path: str
    yaml_sha256: str
    image_sha256: str


@dataclass(frozen=True)
class Workbench:
    workbench_id: str
    marker_id: int
    marker_size_m: float
    map_pose: Mapping[str, Any]
    staging_pose: Mapping[str, Any]
    docking_offset: Mapping[str, float]
    parking_geometry: Mapping[str, Any]
    registration: Mapping[str, Any]
    parking_target: Optional[Mapping[str, Any]] = None

    @property
    def final_tag_x_in_base_m(self) -> float:
        return final_tag_distance(
            front_reference_x_m=self.parking_geometry["front_reference"]["x_m"],
            parking_safety_distance_m=self.parking_geometry["parking_safety_distance_m"],
            longitudinal_docking_offset_m=self.docking_offset.get("longitudinal_m"),
            marker_size_m=self.marker_size_m, parking_target=self.parking_target)


@dataclass(frozen=True)
class WorkbenchRegistry:
    path: Path
    map_identity: MapIdentity
    camera: CameraIdentity
    parking: ParkingIdentity
    lidar: LidarIdentity
    workbenches: Tuple[Workbench, ...]

    def workbench(self, workbench_id: str) -> Workbench:
        for entry in self.workbenches:
            if entry.workbench_id == workbench_id:
                return entry
        raise RegistryError("unknown workbench id: {0}".format(workbench_id))


IdentitySource = Union[Path, str, Mapping[str, Any]]


def _identity_source(source: IdentitySource, name: str) -> Tuple[Mapping[str, Any], Optional[bytes]]:
    if isinstance(source, Mapping):
        return source, None
    path = Path(source)
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise RegistryError("cannot read {0} config {1}: {2}".format(name, path, exc))
    return _load_yaml(path), raw


def _camera_info(camera: Mapping[str, Any]) -> Mapping[str, Any]:
    if "camera_info" in camera:
        return _as_mapping(camera["camera_info"], "camera.camera_info")
    color = _as_mapping(camera.get("color"), "camera.color")
    return _as_mapping(color.get("calibration"), "camera.color.calibration")


def _camera_info_payload(info: Mapping[str, Any]) -> Dict[str, Any]:
    def field(lower: str, upper: str) -> Any:
        if lower in info:
            return info[lower]
        return info.get(upper)

    width = _integer(info.get("width"), "camera_info.width", 1)
    height = _integer(info.get("height"), "camera_info.height", 1)
    distortion_model = _string(
        info.get("distortion_model"), "camera_info.distortion_model"
    )
    d_value = field("d", "D")
    if (
        isinstance(d_value, (str, bytes))
        or not isinstance(d_value, Sequence)
        or not d_value
    ):
        raise RegistryError("camera_info.d must contain numeric values")
    d = [
        _number(item, "camera_info.d[{0}]".format(index))
        for index, item in enumerate(d_value)
    ]
    return {
        "width": width,
        "height": height,
        "distortion_model": distortion_model,
        "d": d,
        "k": list(_numeric_sequence(field("k", "K"), "camera_info.k", 9)),
        "r": list(_numeric_sequence(field("r", "R"), "camera_info.r", 9)),
        "p": list(_numeric_sequence(field("p", "P"), "camera_info.p", 12)),
    }


def load_camera_identity(source: Union[CameraIdentity, IdentitySource]) -> CameraIdentity:
    """Load the canonical camera stream and mechanical-mount identity."""

    if isinstance(source, CameraIdentity):
        return source
    camera, _ = _identity_source(source, "camera")
    profile = _as_mapping(camera.get("profile"), "camera.profile")
    normalized_profile = {
        "width": _integer(profile.get("width"), "camera.profile.width", 1),
        "height": _integer(profile.get("height"), "camera.profile.height", 1),
        "fps": _positive(profile.get("fps"), "camera.profile.fps"),
    }
    info = _camera_info_payload(_camera_info(camera))
    if (
        info["width"] != normalized_profile["width"]
        or info["height"] != normalized_profile["height"]
    ):
        raise RegistryError("camera profile and CameraInfo dimensions differ")
    mount = _as_mapping(camera.get("mount"), "camera.mount")
    mount_state = _string(mount.get("state"), "camera.mount.state")
    if mount_state not in ("cad_nominal", "vendor_nominal", "verified"):
        raise RegistryError(
            "camera mount state must be cad_nominal, vendor_nominal, or verified"
        )
    parent_frame = _string(
        mount.get("parent_frame"), "camera.mount.parent_frame"
    )
    if parent_frame != "base_link":
        raise RegistryError("camera mount parent frame must be base_link")
    normalized_mount = {
        "parent_frame": parent_frame,
        "xyz_m": list(
            _numeric_sequence(mount.get("xyz_m"), "camera.mount.xyz_m", 3)
        ),
        "rpy_rad": list(
            _numeric_sequence(mount.get("rpy_rad"), "camera.mount.rpy_rad", 3)
        ),
        "state": mount_state,
        "source": _string(
            mount.get("source", mount.get("source_sha256")),
            "camera.mount.source",
        ),
        "source_sha256": (
            _sha256(mount["source_sha256"], "camera.mount.source_sha256")
            if mount.get("source_sha256") is not None
            else None
        ),
        "date": mount.get("date"),
        "residual_evidence": mount.get("residual_evidence"),
    }
    frames = camera.get("frames", {})
    if not isinstance(frames, Mapping):
        raise RegistryError("camera.frames must be a mapping")
    expected_frame = _string(
        camera.get("expected_optical_frame", frames.get("color_optical")),
        "camera expected optical frame",
    )
    if expected_frame != EXPECTED_CAMERA_FRAME:
        raise RegistryError(
            "unsupported camera frame: {0}".format(expected_frame)
        )
    payload = {
        "camera_name": _string(
            camera.get("camera_name", camera.get("instance_name")),
            "camera.camera_name",
        ),
        "model": _string(camera.get("model", "Intel RealSense D435"), "camera.model"),
        "serial_no": _string(camera.get("serial_no"), "camera.serial_no"),
        "profile": normalized_profile,
        "camera_info": info,
        "mount": normalized_mount,
        "expected_optical_frame": expected_frame,
    }
    identity_sha256 = canonical_sha256(payload)
    return CameraIdentity(
        identity_sha256=identity_sha256,
        serial_no=payload["serial_no"],
        expected_optical_frame=expected_frame,
        mount_state=mount_state,
        _payload_json=_canonical_payload_json(payload),
    )


def load_parking_identity(
    source: Union[ParkingIdentity, IdentitySource],
) -> ParkingIdentity:
    """Load the physical-front and bounded-blind-travel identity."""

    if isinstance(source, ParkingIdentity):
        return source
    parking, _ = _identity_source(source, "parking")
    front = _as_mapping(parking.get("front_reference"), "parking.front_reference")
    state = _string(front.get("state"), "parking.front_reference.state")
    if state not in ("simulation_only", "verified"):
        raise RegistryError(
            "front-reference state must be simulation_only or verified"
        )
    frame = _string(front.get("frame"), "parking.front_reference.frame")
    if frame != "base_link":
        raise RegistryError("front-reference frame must be base_link")
    maximum = _positive(
        parking.get("max_blind_travel_m"), "max_blind_travel_m"
    )
    absolute = _positive(
        parking.get("absolute_max_blind_travel_m"),
        "absolute_max_blind_travel_m",
    )
    if maximum > absolute:
        raise RegistryError(
            "max_blind_travel_m exceeds absolute_max_blind_travel_m"
        )
    payload = {
        "front_reference": {
            "frame": frame,
            "x_m": _number(front.get("x_m"), "front_reference_x_m"),
            "state": state,
            "source": _string(front.get("source"), "front_reference.source"),
        },
        "parking_safety_distance_m": _nonnegative(
            parking.get("parking_safety_distance_m"),
            "parking_safety_distance_m",
        ),
        "max_blind_travel_m": maximum,
        "absolute_max_blind_travel_m": absolute,
    }
    return ParkingIdentity(
        identity_sha256=canonical_sha256(payload),
        front_reference_x_m=payload["front_reference"]["x_m"],
        parking_safety_distance_m=payload["parking_safety_distance_m"],
        max_blind_travel_m=maximum,
        absolute_max_blind_travel_m=absolute,
        calibration_state=state,
        _payload_json=_canonical_payload_json(payload),
    )


def load_lidar_identity(source: Union[LidarIdentity, IdentitySource]) -> LidarIdentity:
    """Load independent LiDAR profile and collision-envelope identities."""

    if isinstance(source, LidarIdentity):
        return source
    lidar, raw_bytes = _identity_source(source, "LiDAR")
    raw_digest = (
        hashlib.sha256(raw_bytes).hexdigest()
        if raw_bytes is not None
        else None
    )
    footprint = _as_mapping(lidar.get("footprint"), "lidar.footprint")
    filter_config = _as_mapping(lidar.get("filter"), "lidar.filter")
    latency = _as_mapping(lidar.get("latency"), "lidar.latency")
    normalized_footprint = {
        key: _number(footprint.get(key), "lidar.footprint.{0}".format(key))
        for key in ("front_x_m", "rear_x_m", "left_y_m", "right_y_m")
    }
    if (
        normalized_footprint["front_x_m"]
        <= normalized_footprint["rear_x_m"]
        or normalized_footprint["left_y_m"]
        <= normalized_footprint["right_y_m"]
    ):
        raise RegistryError("LiDAR footprint bounds are invalid")
    normalized_filter = {
        "min_obstacle_z_m": _number(
            filter_config.get("min_obstacle_z_m"),
            "lidar.filter.min_obstacle_z_m",
        ),
        "max_obstacle_z_m": _number(
            filter_config.get("max_obstacle_z_m"),
            "lidar.filter.max_obstacle_z_m",
        ),
        "self_padding_m": _nonnegative(
            filter_config.get("self_padding_m"), "lidar.filter.self_padding_m"
        ),
        "minimum_points_per_zone": _integer(
            filter_config.get("minimum_points_per_zone"),
            "lidar.filter.minimum_points_per_zone",
            1,
        ),
    }
    if (
        normalized_filter["min_obstacle_z_m"]
        >= normalized_filter["max_obstacle_z_m"]
    ):
        raise RegistryError("LiDAR obstacle height filter is invalid")
    normalized_latency = {
        key: _nonnegative(latency.get(key), "lidar.latency.{0}".format(key))
        for key in ("sensor_s", "command_s", "brake_s")
    }
    if raw_digest is None:
        raw_digest = canonical_sha256(lidar)
    profile = {
        "model": _string(lidar.get("model"), "lidar.model"),
        "topic": _string(lidar.get("topic"), "lidar.topic"),
        "expected_frame": _string(
            lidar.get("expected_frame"), "lidar.expected_frame"
        ),
        "target_frame": _string(lidar.get("target_frame"), "lidar.target_frame"),
        "max_age_s": _positive(lidar.get("max_age_s"), "lidar.max_age_s"),
        "minimum_rate_hz": _positive(
            lidar.get("minimum_rate_hz"), "lidar.minimum_rate_hz"
        ),
        "minimum_valid_points_per_cloud": _integer(
            lidar.get("minimum_valid_points_per_cloud"),
            "lidar.minimum_valid_points_per_cloud",
            1,
        ),
        "sensor_range_max_m": _positive(
            lidar.get("sensor_range_max_m"), "lidar.sensor_range_max_m"
        ),
        "raw_configuration_sha256": raw_digest,
    }
    if profile["topic"] != "/rslidar_points":
        raise RegistryError("unsupported LiDAR topic")
    if profile["expected_frame"] != "rslidar":
        raise RegistryError("unsupported LiDAR sensor frame")
    if profile["target_frame"] != "base_link":
        raise RegistryError("unsupported LiDAR target frame")
    envelope = {
        "footprint": normalized_footprint,
        "filter": normalized_filter,
        "static_clearance_m": _positive(
            lidar.get("static_clearance_m"), "lidar.static_clearance_m"
        ),
        "latency": normalized_latency,
        "minimum_braking_deceleration_mps2": _positive(
            lidar.get("minimum_braking_deceleration_mps2"),
            "lidar.minimum_braking_deceleration_mps2",
        ),
        "raw_configuration_sha256": raw_digest,
    }
    payload = {"profile": profile, "envelope": envelope}
    return LidarIdentity(
        profile_identity_sha256=canonical_sha256(profile),
        envelope_identity_sha256=canonical_sha256(envelope),
        raw_configuration_sha256=raw_digest,
        _payload_json=_canonical_payload_json(payload),
    )


def _map_identity(
    map_id: str,
    map_yaml: Path,
    registry_dir: Path,
) -> MapIdentity:
    map_path = map_yaml.resolve()
    map_data = _load_yaml(map_path)
    image_value = _string(map_data.get("image"), "map image")
    image_relative = Path(image_value)
    if image_relative.is_absolute():
        raise RegistryError("map image must use a portable relative path")
    image_path = (map_path.parent / image_relative).resolve()
    return MapIdentity(
        map_id=_string(map_id, "map.id"),
        yaml_path=_portable_path(map_path, registry_dir, "map YAML"),
        image_path=_portable_path(image_path, registry_dir, "map image"),
        yaml_sha256=_file_sha256(map_path, "map YAML"),
        image_sha256=_file_sha256(image_path, "map image"),
    )


def _map_document(identity: MapIdentity) -> Dict[str, Any]:
    return {
        "id": identity.map_id,
        "yaml": identity.yaml_path,
        "image": identity.image_path,
        "yaml_sha256": identity.yaml_sha256,
        "image_sha256": identity.image_sha256,
    }


def build_registry_document(
    *,
    map_id: str,
    map_yaml: Union[Path, str],
    camera: Union[CameraIdentity, IdentitySource],
    parking: Union[ParkingIdentity, IdentitySource],
    lidar: Union[LidarIdentity, IdentitySource],
    workbenches: Sequence[Mapping[str, Any]],
    registry_dir: Union[Path, str],
) -> Dict[str, Any]:
    """Build a portable document with canonical identity snapshots."""

    camera_identity = load_camera_identity(camera)
    parking_identity = load_parking_identity(parking)
    lidar_identity = load_lidar_identity(lidar)
    map_identity = _map_identity(
        map_id,
        Path(map_yaml),
        Path(registry_dir),
    )
    entries = []
    for supplied in workbenches:
        entry = deepcopy(dict(supplied))
        entry["parking_geometry"] = {
            **parking_identity.payload,
            "identity_sha256": parking_identity.identity_sha256,
        }
        entry["lidar_safety"] = {
            "profile_identity_sha256": lidar_identity.profile_identity_sha256,
            "envelope_identity_sha256": lidar_identity.envelope_identity_sha256,
        }
        entry["expected_camera_frame"] = camera_identity.expected_optical_frame
        entries.append(entry)
    return {
        "schema_version": SCHEMA_VERSION,
        "map": _map_document(map_identity),
        "camera_calibration": {
            "identity_sha256": camera_identity.identity_sha256,
            "payload": camera_identity.payload,
        },
        "parking_geometry": {
            "identity_sha256": parking_identity.identity_sha256,
            "payload": parking_identity.payload,
        },
        "lidar_safety": {
            "profile_identity_sha256": lidar_identity.profile_identity_sha256,
            "envelope_identity_sha256": lidar_identity.envelope_identity_sha256,
            "raw_configuration_sha256": lidar_identity.raw_configuration_sha256,
            "payload": lidar_identity.payload,
        },
        "workbenches": entries,
    }


def _validate_pose(value: Any, name: str) -> Dict[str, Any]:
    pose = _as_mapping(value, name)
    _strict_keys(
        pose,
        name,
        ("frame", "position_m", "orientation_xyzw"),
    )
    frame = _string(pose["frame"], "{0}.frame".format(name))
    if frame != "map":
        raise RegistryError("{0} frame must be map".format(name))
    position = _numeric_sequence(
        pose["position_m"], "{0}.position_m".format(name), 3
    )
    quaternion = _numeric_sequence(
        pose["orientation_xyzw"],
        "{0}.orientation_xyzw".format(name),
        4,
    )
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise RegistryError("{0} must contain a unit quaternion".format(name))
    return {
        "frame": frame,
        "position_m": list(position),
        "orientation_xyzw": list(quaternion),
    }


def _validate_residual_group(value: Any, name: str) -> Dict[str, float]:
    group = _as_mapping(value, name)
    _strict_keys(group, name, ("p50", "p95", "max"))
    normalized = {
        key: _nonnegative(group[key], "{0}.{1}".format(name, key))
        for key in ("p50", "p95", "max")
    }
    if not (
        normalized["p50"] <= normalized["p95"] <= normalized["max"]
    ):
        raise RegistryError("{0} residual quantiles must be ordered".format(name))
    return normalized


def _validate_registration(value: Any, name: str) -> Dict[str, Any]:
    registration = _as_mapping(value, name)
    _strict_keys(
        registration,
        name,
        ("sample_count", "base_pose_count", "residuals", "created_at"),
    )
    residuals = _as_mapping(
        registration["residuals"], "{0}.residuals".format(name)
    )
    _strict_keys(
        residuals,
        "{0}.residuals".format(name),
        ("translation_m", "floor_normal_rad", "yaw_rad"),
    )
    return {
        "sample_count": _integer(
            registration["sample_count"],
            "{0}.sample_count".format(name),
            12,
        ),
        "base_pose_count": _integer(
            registration["base_pose_count"],
            "{0}.base_pose_count".format(name),
            3,
        ),
        "residuals": {
            key: _validate_residual_group(
                residuals[key],
                "{0}.residuals.{1}".format(name, key),
            )
            for key in ("translation_m", "floor_normal_rad", "yaw_rad")
        },
        "created_at": _string(
            registration["created_at"], "{0}.created_at".format(name)
        ),
    }


def _validate_parking_policy(
    parking: ParkingIdentity,
    camera: CameraIdentity,
    mode: str,
    commissioning: bool,
) -> None:
    if mode not in ("hardware", "simulation"):
        raise RegistryError("mode must be hardware or simulation")
    if not isinstance(commissioning, bool):
        raise RegistryError("commissioning must be a bool")
    if (
        parking.calibration_state == "simulation_only"
        and mode != "simulation"
        and not commissioning
    ):
        raise RegistryError(
            "hardware operation requires a verified front reference"
        )
    if mode == "hardware" and not commissioning:
        if parking.calibration_state != "verified":
            raise RegistryError(
                "hardware operation requires a verified front reference"
            )
        if camera.mount_state != "verified":
            raise RegistryError(
                "hardware operation requires a verified camera mount"
            )


def _identity_from_embedded_camera(value: Any) -> CameraIdentity:
    section = _as_mapping(value, "camera_calibration")
    _strict_keys(section, "camera_calibration", ("identity_sha256", "payload"))
    stored = _sha256(
        section["identity_sha256"], "camera_calibration.identity_sha256"
    )
    payload = _as_mapping(section["payload"], "camera payload")
    identity = load_camera_identity(payload)
    if _canonical_bytes(payload) != _canonical_bytes(identity.payload):
        raise RegistryError(
            "camera payload contains unknown or noncanonical fields"
        )
    if stored != identity.identity_sha256:
        raise RegistryError("camera identity digest mismatch")
    return identity


def _identity_from_embedded_parking(value: Any) -> ParkingIdentity:
    section = _as_mapping(value, "parking_geometry")
    _strict_keys(section, "parking_geometry", ("identity_sha256", "payload"))
    stored = _sha256(section["identity_sha256"], "parking identity_sha256")
    payload = _as_mapping(section["payload"], "parking payload")
    identity = load_parking_identity(payload)
    if _canonical_bytes(payload) != _canonical_bytes(identity.payload):
        raise RegistryError(
            "parking payload contains unknown or noncanonical fields"
        )
    if stored != identity.identity_sha256:
        raise RegistryError("parking identity digest mismatch")
    return identity


def _identity_from_embedded_lidar(value: Any) -> LidarIdentity:
    section = _as_mapping(value, "lidar_safety")
    _strict_keys(
        section,
        "lidar_safety",
        (
            "profile_identity_sha256",
            "envelope_identity_sha256",
            "raw_configuration_sha256",
            "payload",
        ),
    )
    payload = _as_mapping(section["payload"], "LiDAR payload")
    _strict_keys(payload, "LiDAR payload", ("profile", "envelope"))
    profile = dict(_as_mapping(payload["profile"], "LiDAR profile"))
    envelope = dict(_as_mapping(payload["envelope"], "LiDAR envelope"))
    _strict_keys(
        profile,
        "LiDAR profile",
        (
            "model",
            "topic",
            "expected_frame",
            "target_frame",
            "max_age_s",
            "minimum_rate_hz",
            "minimum_valid_points_per_cloud",
            "sensor_range_max_m",
            "raw_configuration_sha256",
        ),
    )
    _strict_keys(
        envelope,
        "LiDAR envelope",
        (
            "footprint",
            "filter",
            "static_clearance_m",
            "latency",
            "minimum_braking_deceleration_mps2",
            "raw_configuration_sha256",
        ),
    )
    footprint = _as_mapping(envelope["footprint"], "LiDAR footprint")
    filter_config = _as_mapping(envelope["filter"], "LiDAR filter")
    latency = _as_mapping(envelope["latency"], "LiDAR latency")
    _strict_keys(
        footprint,
        "LiDAR footprint",
        ("front_x_m", "rear_x_m", "left_y_m", "right_y_m"),
    )
    _strict_keys(
        filter_config,
        "LiDAR filter",
        (
            "min_obstacle_z_m",
            "max_obstacle_z_m",
            "self_padding_m",
            "minimum_points_per_zone",
        ),
    )
    _strict_keys(
        latency,
        "LiDAR latency",
        ("sensor_s", "command_s", "brake_s"),
    )
    raw = _sha256(
        section["raw_configuration_sha256"],
        "LiDAR raw configuration digest",
    )
    if _sha256(
        profile["raw_configuration_sha256"],
        "LiDAR profile raw configuration digest",
    ) != raw:
        raise RegistryError("LiDAR profile raw digest mismatch")
    if _sha256(
        envelope["raw_configuration_sha256"],
        "LiDAR envelope raw configuration digest",
    ) != raw:
        raise RegistryError("LiDAR envelope raw digest mismatch")

    flat_configuration = {
        "model": profile["model"],
        "topic": profile["topic"],
        "expected_frame": profile["expected_frame"],
        "target_frame": profile["target_frame"],
        "max_age_s": profile["max_age_s"],
        "minimum_rate_hz": profile["minimum_rate_hz"],
        "minimum_valid_points_per_cloud": profile[
            "minimum_valid_points_per_cloud"
        ],
        "sensor_range_max_m": profile["sensor_range_max_m"],
        "footprint": footprint,
        "filter": filter_config,
        "static_clearance_m": envelope["static_clearance_m"],
        "latency": latency,
        "minimum_braking_deceleration_mps2": envelope[
            "minimum_braking_deceleration_mps2"
        ],
    }
    normalized = load_lidar_identity(flat_configuration).payload
    normalized["profile"]["raw_configuration_sha256"] = raw
    normalized["envelope"]["raw_configuration_sha256"] = raw
    if _canonical_bytes(payload) != _canonical_bytes(normalized):
        raise RegistryError(
            "LiDAR payload contains unknown or noncanonical fields"
        )

    profile_digest = canonical_sha256(profile)
    envelope_digest = canonical_sha256(envelope)
    if profile_digest != _sha256(
        section["profile_identity_sha256"], "LiDAR profile identity"
    ):
        raise RegistryError("LiDAR profile identity digest mismatch")
    if envelope_digest != _sha256(
        section["envelope_identity_sha256"], "LiDAR envelope identity"
    ):
        raise RegistryError("LiDAR envelope identity digest mismatch")
    return LidarIdentity(
        profile_identity_sha256=profile_digest,
        envelope_identity_sha256=envelope_digest,
        raw_configuration_sha256=raw,
        _payload_json=_canonical_payload_json(payload),
    )


def _validate_map_section(
    value: Any,
    registry_path: Path,
    map_yaml: Path,
) -> MapIdentity:
    section = _as_mapping(value, "map")
    _strict_keys(
        section,
        "map",
        ("id", "yaml", "image", "yaml_sha256", "image_sha256"),
    )
    yaml_value = _string(section["yaml"], "map.yaml")
    image_value = _string(section["image"], "map.image")
    if Path(yaml_value).is_absolute() or Path(image_value).is_absolute():
        raise RegistryError("map paths must be portable relative paths")
    bound_map = (registry_path.parent / yaml_value).resolve()
    supplied_map = map_yaml.resolve()
    if bound_map != supplied_map:
        raise RegistryError("map YAML path does not match registry binding")
    map_data = _load_yaml(supplied_map)
    referenced_image = _string(map_data.get("image"), "map image")
    if Path(referenced_image).is_absolute():
        raise RegistryError("map image must use a portable relative path")
    actual_image = (supplied_map.parent / referenced_image).resolve()
    bound_image = (registry_path.parent / image_value).resolve()
    if actual_image != bound_image:
        raise RegistryError("map image path does not match registry binding")
    yaml_digest = _file_sha256(supplied_map, "map YAML")
    image_digest = _file_sha256(actual_image, "map image")
    if yaml_digest != _sha256(section["yaml_sha256"], "map YAML digest"):
        raise RegistryError("map YAML digest mismatch")
    if image_digest != _sha256(section["image_sha256"], "map image digest"):
        raise RegistryError("map image digest mismatch")
    return MapIdentity(
        map_id=_string(section["id"], "map.id"),
        yaml_path=yaml_value,
        image_path=image_value,
        yaml_sha256=yaml_digest,
        image_sha256=image_digest,
    )


def _validate_workbenches(
    values: Any,
    camera: CameraIdentity,
    parking: ParkingIdentity,
    lidar: LidarIdentity,
) -> Tuple[Workbench, ...]:
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise RegistryError("workbenches must be a sequence")
    entries = []
    ids = set()
    marker_ids = set()
    for index, value in enumerate(values):
        name = "workbenches[{0}]".format(index)
        entry = _as_mapping(value, name)
        _strict_keys(
            entry,
            name,
            (
                "id",
                "marker",
                "map_pose",
                "staging_pose",
                "docking_offset",
                "parking_geometry",
                "lidar_safety",
                "expected_camera_frame",
                "registration",
            ),
            optional=("parking_target",),
        )
        workbench_id = _string(entry["id"], "{0}.id".format(name))
        if workbench_id in ids:
            raise RegistryError("duplicate workbench id: {0}".format(workbench_id))
        ids.add(workbench_id)
        marker = _as_mapping(entry["marker"], "{0}.marker".format(name))
        _strict_keys(
            marker,
            "{0}.marker".format(name),
            ("family", "id", "size_m", "artifact_sha256"),
        )
        family = _string(marker["family"], "{0}.marker.family".format(name))
        if family not in ("ALVAR", "ALVAR MarkerData"):
            raise RegistryError("unsupported marker family")
        marker_id = _integer(marker["id"], "{0}.marker.id".format(name), 100)
        if marker_id > 199:
            raise RegistryError("workbench marker ID must be within 100-199")
        if marker_id in marker_ids:
            raise RegistryError("duplicate marker id: {0}".format(marker_id))
        marker_ids.add(marker_id)
        marker_size = _positive(
            marker["size_m"], "{0}.marker.size_m".format(name)
        )
        _sha256(
            marker["artifact_sha256"],
            "{0}.marker.artifact_sha256".format(name),
        )
        expected_frame = _string(
            entry["expected_camera_frame"],
            "{0}.expected_camera_frame".format(name),
        )
        if expected_frame != EXPECTED_CAMERA_FRAME:
            raise RegistryError("unsupported camera frame")
        if expected_frame != camera.expected_optical_frame:
            raise RegistryError("camera frame identity mismatch")
        docking = _as_mapping(
            entry["docking_offset"], "{0}.docking_offset".format(name)
        )
        _strict_keys(
            docking,
            "{0}.docking_offset".format(name),
            ("lateral_m", "yaw_rad"),
            optional=("longitudinal_m",),
        )
        normalized_docking = {
            "lateral_m": _number(
                docking["lateral_m"], "{0}.docking_offset.lateral_m".format(name)
            ),
            "yaw_rad": _number(
                docking["yaw_rad"], "{0}.docking_offset.yaw_rad".format(name)
            ),
        }
        if "longitudinal_m" in docking:
            normalized_docking["longitudinal_m"] = _number(
                docking["longitudinal_m"], "docking_offset.longitudinal_m")
        try:
            parking_target = (normalize_parking_target(entry["parking_target"])
                              if "parking_target" in entry else None)
            final_tag_distance(
                front_reference_x_m=parking.front_reference_x_m,
                parking_safety_distance_m=parking.parking_safety_distance_m,
                longitudinal_docking_offset_m=normalized_docking.get("longitudinal_m"),
                marker_size_m=marker_size, parking_target=parking_target)
        except ValueError as exc:
            raise RegistryError(str(exc)) from exc
        entry_parking = _as_mapping(
            entry["parking_geometry"], "{0}.parking_geometry".format(name)
        )
        entry_parking_digest = _sha256(
            entry_parking.get("identity_sha256"),
            "{0}.parking_geometry.identity_sha256".format(name),
        )
        entry_parking_payload = {
            key: value
            for key, value in entry_parking.items()
            if key != "identity_sha256"
        }
        if (
            entry_parking_digest != parking.identity_sha256
            or canonical_sha256(entry_parking_payload) != parking.identity_sha256
        ):
            raise RegistryError("workbench parking identity digest mismatch")
        entry_lidar = _as_mapping(
            entry["lidar_safety"], "{0}.lidar_safety".format(name)
        )
        _strict_keys(
            entry_lidar,
            "{0}.lidar_safety".format(name),
            ("profile_identity_sha256", "envelope_identity_sha256"),
        )
        if _sha256(
            entry_lidar["profile_identity_sha256"],
            "{0}.LiDAR profile identity".format(name),
        ) != lidar.profile_identity_sha256:
            raise RegistryError("LiDAR profile identity digest mismatch")
        if _sha256(
            entry_lidar["envelope_identity_sha256"],
            "{0}.LiDAR envelope identity".format(name),
        ) != lidar.envelope_identity_sha256:
            raise RegistryError("LiDAR envelope identity digest mismatch")
        entries.append(
            Workbench(
                workbench_id=workbench_id,
                marker_id=marker_id,
                marker_size_m=marker_size,
                map_pose=_deep_freeze(
                    _validate_pose(
                        entry["map_pose"], "{0}.map_pose".format(name)
                    )
                ),
                staging_pose=_deep_freeze(
                    _validate_pose(
                        entry["staging_pose"], "{0}.staging_pose".format(name)
                    )
                ),
                docking_offset=_deep_freeze(normalized_docking),
                parking_target=(_deep_freeze(parking_target)
                                if parking_target is not None else None),
                parking_geometry=_deep_freeze(entry_parking_payload),
                registration=_deep_freeze(
                    _validate_registration(
                        entry["registration"], "{0}.registration".format(name)
                    )
                ),
            )
        )
    if not entries:
        raise RegistryError("registry must contain at least one workbench")
    return tuple(entries)


def load_registry(
    path: Union[Path, str],
    *,
    map_yaml: Union[Path, str],
    camera: Optional[Union[CameraIdentity, IdentitySource]] = None,
    parking: Optional[Union[ParkingIdentity, IdentitySource]] = None,
    lidar: Optional[Union[LidarIdentity, IdentitySource]] = None,
    mode: str = "hardware",
    commissioning: bool = False,
) -> WorkbenchRegistry:
    """Validate a registry and every material identity against current inputs."""

    if mode not in ("hardware", "simulation"):
        raise RegistryError("mode must be hardware or simulation")
    if mode == "hardware":
        missing_identities = [
            name
            for name, source in (
                ("camera", camera),
                ("parking", parking),
                ("LiDAR", lidar),
            )
            if source is None
        ]
        if missing_identities:
            raise RegistryError(
                "hardware requires explicit current identity inputs; missing: "
                + ", ".join(missing_identities)
            )
    registry_path = Path(path)
    document = _load_yaml(registry_path)
    _strict_keys(
        document,
        "registry",
        (
            "schema_version",
            "map",
            "camera_calibration",
            "parking_geometry",
            "lidar_safety",
            "workbenches",
        ),
    )
    if (
        isinstance(document["schema_version"], bool)
        or not isinstance(document["schema_version"], Integral)
        or int(document["schema_version"]) != SCHEMA_VERSION
    ):
        raise RegistryError("unsupported registry schema_version")
    map_identity = _validate_map_section(
        document["map"], registry_path, Path(map_yaml)
    )
    embedded_camera = _identity_from_embedded_camera(
        document["camera_calibration"]
    )
    embedded_parking = _identity_from_embedded_parking(
        document["parking_geometry"]
    )
    embedded_lidar = _identity_from_embedded_lidar(document["lidar_safety"])
    current_camera = (
        embedded_camera if camera is None else load_camera_identity(camera)
    )
    current_parking = (
        embedded_parking if parking is None else load_parking_identity(parking)
    )
    current_lidar = (
        embedded_lidar if lidar is None else load_lidar_identity(lidar)
    )
    if current_camera.identity_sha256 != embedded_camera.identity_sha256:
        raise RegistryError("camera identity digest mismatch")
    if current_parking.identity_sha256 != embedded_parking.identity_sha256:
        raise RegistryError("parking identity digest mismatch")
    if (
        current_lidar.profile_identity_sha256
        != embedded_lidar.profile_identity_sha256
    ):
        raise RegistryError("LiDAR profile identity digest mismatch")
    if (
        current_lidar.envelope_identity_sha256
        != embedded_lidar.envelope_identity_sha256
    ):
        raise RegistryError("LiDAR envelope identity digest mismatch")
    _validate_parking_policy(
        current_parking,
        current_camera,
        mode,
        commissioning,
    )
    workbenches = _validate_workbenches(
        document["workbenches"],
        current_camera,
        current_parking,
        current_lidar,
    )
    return WorkbenchRegistry(
        path=registry_path.resolve(),
        map_identity=map_identity,
        camera=current_camera,
        parking=current_parking,
        lidar=current_lidar,
        workbenches=workbenches,
    )


def write_registry(
    path: Union[Path, str],
    document: Mapping[str, Any],
    *,
    map_yaml: Union[Path, str],
    camera: Optional[Union[CameraIdentity, IdentitySource]] = None,
    parking: Optional[Union[ParkingIdentity, IdentitySource]] = None,
    lidar: Optional[Union[LidarIdentity, IdentitySource]] = None,
    mode: str = "hardware",
    commissioning: bool = False,
) -> WorkbenchRegistry:
    """Atomically write only a document that reloads and fully validates."""

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=".{0}.".format(target.name),
        suffix=".tmp",
        dir=str(target.parent),
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                deepcopy(dict(document)),
                stream,
                allow_unicode=True,
                sort_keys=False,
            )
            stream.flush()
            os.fsync(stream.fileno())
        validated = load_registry(
            temporary,
            map_yaml=map_yaml,
            camera=camera,
            parking=parking,
            lidar=lidar,
            mode=mode,
            commissioning=commissioning,
        )
        os.replace(str(temporary), str(target))
        directory_descriptor = os.open(str(target.parent), os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
        return WorkbenchRegistry(
            path=target.resolve(),
            map_identity=validated.map_identity,
            camera=validated.camera,
            parking=validated.parking,
            lidar=validated.lidar,
            workbenches=validated.workbenches,
        )
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise
