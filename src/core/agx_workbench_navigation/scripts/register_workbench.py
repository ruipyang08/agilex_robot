#!/usr/bin/env python3
"""Collect rendered ALVAR evidence and atomically register one workbench."""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import fcntl
import math
from pathlib import Path
import sys
import time

import yaml

from agx_workbench_navigation.contracts import PlanarPose
from agx_workbench_navigation.parking_target import final_tag_distance, normalize_parking_target
from agx_workbench_navigation.registration import (
    Pose3D,
    RegistrationConfig,
    RegistrationError,
    RegistrationSample,
    estimate_registration,
)
from agx_workbench_navigation.registry import (
    RegistryError,
    build_registry_document,
    load_camera_identity,
    load_lidar_identity,
    load_parking_identity,
    load_registry,
    write_registry,
)


MARKER_TOPIC = "/cam2/ar_pose_marker"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TEMPLATE = PACKAGE_ROOT / "config/workbench_registry.example.yaml"
DEFAULT_LIDAR_CONFIG = PACKAGE_ROOT / "config/lidar_safety.yaml"


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Estimate a map-bound floor marker from timestamped ALVAR and TF "
            "evidence. The marker's global pose is never entered manually."
        )
    )
    parser.add_argument("--map", dest="map_path", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workbench-id", required=True)
    parser.add_argument("--marker-id", type=int, required=True)
    parser.add_argument("--map-id")
    parser.add_argument("--identity-template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--camera-config", type=Path)
    parser.add_argument("--parking-config", type=Path)
    parser.add_argument(
        "--lidar-config",
        type=Path,
        default=DEFAULT_LIDAR_CONFIG,
    )
    parser.add_argument("--mode", choices=("simulation", "hardware"), default="simulation")
    parser.add_argument("--commissioning", action="store_true")
    parser.add_argument("--accept", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--max-sample-age-s", type=float, default=120.0)
    parser.add_argument("--floor-height-m", type=float, default=0.001)
    parser.add_argument("--max-floor-height-error-m", type=float, default=0.03)
    parser.add_argument("--max-floor-normal-error-rad", type=float, default=0.10)
    parser.add_argument("--maximum-translation-residual-m", type=float, default=0.03)
    parser.add_argument("--maximum-yaw-residual-rad", type=float, default=0.05)
    parser.add_argument("--marker-to-staging-x-m", type=float, default=-2.00)
    parser.add_argument("--marker-to-staging-y-m", type=float, default=0.0)
    parser.add_argument("--marker-to-staging-yaw-rad", type=float, default=0.0)
    parser.add_argument("--alignment-lateral-m", type=float, default=0.0)
    reference = parser.add_mutually_exclusive_group()
    reference.add_argument(
        "--alignment-longitudinal-m", type=float, default=0.0,
        help="Docking distance offset in metres; does not change the measured marker pose.",
    )
    reference.add_argument("--parking-reference", choices=("tag_table_side_edge",),
                           help="Align the physical rover front with the tag edge nearest the table.")
    parser.add_argument("--backoff-distance-m", type=float,
                        help="Nonnegative backoff for tag_table_side_edge; defaults to zero.")
    parser.add_argument("--alignment-yaw-rad", type=float, default=0.0)
    return parser


def _load_template(path):
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise RegistryError("cannot read identity template: {0}".format(exc))
    if not isinstance(value, dict):
        raise RegistryError("identity template must contain a mapping")
    return value


def _validate_output_location(map_path, output_path):
    if map_path.resolve().parent != output_path.resolve().parent:
        raise RegistryError(
            "registry output must be written beside the selected map"
        )


def _identity_sources(arguments):
    if arguments.mode == "hardware" and arguments.camera_config is None:
        raise RegistryError(
            "hardware registration requires --camera-config with captured "
            "driver CameraInfo"
        )
    template = _load_template(arguments.identity_template)
    try:
        camera_source = (
            arguments.camera_config
            if arguments.camera_config is not None
            else template["camera_calibration"]["payload"]
        )
        parking_source = (
            arguments.parking_config
            if arguments.parking_config is not None
            else template["parking_geometry"]["payload"]
        )
    except (KeyError, TypeError) as exc:
        raise RegistryError(
            "identity template is missing a camera or parking payload: "
            "{0}".format(exc)
        )
    return (
        load_camera_identity(camera_source),
        load_parking_identity(parking_source),
        load_lidar_identity(arguments.lidar_config),
        template,
    )


def _pose3d_from_transform(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return Pose3D(
        translation_m=(translation.x, translation.y, translation.z),
        quaternion_xyzw=(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def _pose3d_from_pose(pose):
    position = pose.pose.position
    orientation = pose.pose.orientation
    return Pose3D(
        translation_m=(position.x, position.y, position.z),
        quaternion_xyzw=(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ),
    )


def _pose_document(pose, height_m):
    return {
        "frame": "map",
        "position_m": [pose.x_m, pose.y_m, height_m],
        "orientation_xyzw": [
            0.0,
            0.0,
            math.sin(pose.yaw_rad / 2.0),
            math.cos(pose.yaw_rad / 2.0),
        ],
    }


def _residual_document(group):
    return {"p50": group.p50, "p95": group.p95, "max": group.maximum}


def _registration_document(result):
    return {
        "sample_count": result.sample_count,
        "base_pose_count": result.base_pose_count,
        "residuals": {
            "translation_m": _residual_document(
                result.residuals.translation_m
            ),
            "floor_normal_rad": _residual_document(
                result.residuals.floor_normal_rad
            ),
            "yaw_rad": _residual_document(result.residuals.yaw_rad),
        },
        "created_at": datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z"),
    }


def _result_summary(result, longitudinal=0.0):
    return {
        "accepted_samples": result.sample_count,
        "rejected_samples": result.rejected_count,
        "base_poses": result.base_pose_count,
        "marker_pose": {
            "x_m": result.marker_pose.x_m,
            "y_m": result.marker_pose.y_m,
            "z_m": result.marker_height_m,
            "yaw_rad": result.marker_pose.yaw_rad,
        },
        "staging_pose": {
            "x_m": result.staging_pose.x_m,
            "y_m": result.staging_pose.y_m,
            "yaw_rad": result.staging_pose.yaw_rad,
        },
        "final_tag_x_in_base_m": result.final_tag_x_in_base_m + longitudinal,
        "residuals": _registration_document(result)["residuals"],
    }


def _registration_config(arguments, parking, now_s):
    target = _parking_target_argument(arguments)
    marker_size = (_marker_template(_load_template(arguments.identity_template),
                                   arguments.marker_id)["size_m"] if target else None)
    return RegistrationConfig(
        now_s=now_s,
        max_sample_age_s=arguments.max_sample_age_s,
        floor_z_m=arguments.floor_height_m,
        max_floor_height_error_m=arguments.max_floor_height_error_m,
        max_floor_normal_error_rad=arguments.max_floor_normal_error_rad,
        minimum_samples=12,
        minimum_base_poses=3,
        base_pose_translation_separation_m=0.15,
        base_pose_yaw_separation_rad=0.15,
        mad_scale=3.5,
        translation_mad_floor_m=0.004,
        yaw_mad_floor_rad=0.008,
        max_translation_residual_m=arguments.maximum_translation_residual_m,
        max_yaw_residual_rad=arguments.maximum_yaw_residual_rad,
        marker_to_staging_offset=PlanarPose(
            arguments.marker_to_staging_x_m,
            arguments.marker_to_staging_y_m,
            arguments.marker_to_staging_yaw_rad,
        ),
        alignment_lateral_m=arguments.alignment_lateral_m,
        alignment_yaw_rad=arguments.alignment_yaw_rad,
        front_reference_x_m=parking.front_reference_x_m,
        parking_safety_distance_m=parking.parking_safety_distance_m,
        marker_size_m=marker_size,
        parking_target=target,
    )


class _StationaryPoseGate:
    """Admit evidence only after map-to-base remains inside a tight dwell."""

    def __init__(
        self,
        dwell_s=0.50,
        max_translation_m=0.01,
        max_yaw_rad=0.02,
    ):
        self._dwell_s = float(dwell_s)
        self._max_translation_m = float(max_translation_m)
        self._max_yaw_rad = float(max_yaw_rad)
        if (
            not math.isfinite(self._dwell_s)
            or self._dwell_s <= 0.0
            or not math.isfinite(self._max_translation_m)
            or self._max_translation_m <= 0.0
            or not math.isfinite(self._max_yaw_rad)
            or self._max_yaw_rad <= 0.0
        ):
            raise ValueError("stationary pose gate limits must be positive")
        self._reference = None
        self._dwell_started_s = None
        self._last_stamp_s = None

    @staticmethod
    def _planar(pose):
        x_m, y_m, _z_m = (float(value) for value in pose.translation_m)
        qx, qy, qz, qw = (
            float(value) for value in pose.quaternion_xyzw
        )
        values = (x_m, y_m, qx, qy, qz, qw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("stationary pose evidence must be finite")
        yaw_rad = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        return x_m, y_m, yaw_rad

    def update(self, pose, stamp_s):
        try:
            stamp = float(stamp_s)
            current = self._planar(pose)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._reference = None
            self._dwell_started_s = None
            return False
        if not math.isfinite(stamp):
            self._reference = None
            self._dwell_started_s = None
            return False
        if self._last_stamp_s is not None and stamp <= self._last_stamp_s:
            self._reference = current
            self._dwell_started_s = self._last_stamp_s
            return False
        self._last_stamp_s = stamp
        if self._reference is None:
            self._reference = current
            self._dwell_started_s = stamp
            return False
        translation = math.hypot(
            current[0] - self._reference[0],
            current[1] - self._reference[1],
        )
        yaw_delta = math.atan2(
            math.sin(current[2] - self._reference[2]),
            math.cos(current[2] - self._reference[2]),
        )
        if (
            translation > self._max_translation_m
            or abs(yaw_delta) > self._max_yaw_rad
        ):
            self._reference = current
            self._dwell_started_s = stamp
            return False
        return stamp - self._dwell_started_s >= self._dwell_s


class _Collector:
    def __init__(
        self,
        marker_id,
        expected_frame,
        tf_buffer,
        tf_listener,
        rospy,
        transform_pose,
        stationary_gate=None,
    ):
        self._marker_id = marker_id
        self._expected_frame = expected_frame
        self._tf_buffer = tf_buffer
        self._tf_listener = tf_listener
        self._rospy = rospy
        self._transform_pose = transform_pose
        self._stationary_gate = (
            _StationaryPoseGate()
            if stationary_gate is None
            else stationary_gate
        )
        self.samples = []
        self.diagnostics = []

    def callback(self, message):
        desired = [marker for marker in message.markers if marker.id == self._marker_id]
        if len(desired) != 1:
            if desired:
                self.diagnostics.append("ambiguous duplicate requested marker")
            return
        if message.header.frame_id != self._expected_frame:
            self.diagnostics.append("unexpected marker frame")
            return
        marker = desired[0]
        marker_header = getattr(marker, "header", None)
        if marker_header is not None and (
            marker_header.frame_id != message.header.frame_id
            or marker_header.stamp != message.header.stamp
        ):
            self.diagnostics.append("marker header does not match message")
            return
        pose_header = marker.pose.header
        try:
            pose_header_unset = (
                not pose_header.frame_id
                and float(pose_header.stamp.to_sec()) == 0.0
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.diagnostics.append("marker pose header does not match message")
            return
        if pose_header_unset:
            # ar_track_alvar fills AlvarMarker.header but leaves the nested
            # PoseStamped header at its ROS default. Accept that driver format
            # only when the authoritative per-marker header above is present
            # and agrees with the enclosing message.
            if marker_header is None:
                self.diagnostics.append("marker pose header is unset")
                return
        elif (
            pose_header.frame_id != message.header.frame_id
            or pose_header.stamp != message.header.stamp
        ):
            self.diagnostics.append("marker pose header does not match message")
            return
        try:
            map_to_base = self._tf_buffer.lookup_transform(
                "map",
                "base_link",
                message.header.stamp,
                self._rospy.Duration(0.10),
            )
            map_to_base_pose = _pose3d_from_transform(map_to_base)
            if not self._stationary_gate.update(
                map_to_base_pose,
                message.header.stamp.to_sec(),
            ):
                return
            base_to_camera = self._tf_buffer.lookup_transform(
                "base_link",
                message.header.frame_id,
                message.header.stamp,
                self._rospy.Duration(0.10),
            )
            base_to_marker = self._transform_pose(
                marker.pose,
                base_to_camera,
            )
        except Exception as exc:
            self.diagnostics.append("timestamped TF rejected: {0}".format(exc))
            return
        self.samples.append(
            RegistrationSample(
                stamp_s=message.header.stamp.to_sec(),
                map_to_base=map_to_base_pose,
                base_to_marker=_pose3d_from_pose(base_to_marker),
            )
        )


def _wait_for_registration(
    collector,
    arguments,
    parking,
    rospy,
    monotonic=None,
    wall_sleep=None,
):
    monotonic = time.monotonic if monotonic is None else monotonic
    wall_sleep = time.sleep if wall_sleep is None else wall_sleep
    try:
        timeout_s = float(arguments.timeout_s)
    except (TypeError, ValueError, OverflowError):
        raise RegistrationError("timeout_s must be finite and positive")
    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise RegistrationError("timeout_s must be finite and positive")
    deadline = monotonic() + timeout_s
    last_error = None
    while not rospy.is_shutdown() and monotonic() < deadline:
        if len(collector.samples) >= 12:
            config = _registration_config(
                arguments,
                parking,
                rospy.Time.now().to_sec(),
            )
            try:
                return estimate_registration(collector.samples, config)
            except RegistrationError as exc:
                last_error = exc
        wall_sleep(0.05)
    details = " ({0})".format(last_error) if last_error is not None else ""
    raise RegistrationError(
        "registration timed out without sufficient multi-pose evidence{0}".format(
            details
        )
    )


def _collect(arguments, camera, parking):
    import rospy
    import tf2_geometry_msgs
    import tf2_ros
    from ar_track_alvar_msgs.msg import AlvarMarkers

    rospy.init_node("register_workbench", anonymous=False)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(arguments.timeout_s))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    collector = _Collector(
        arguments.marker_id,
        camera.expected_optical_frame,
        tf_buffer,
        tf_listener,
        rospy,
        tf2_geometry_msgs.do_transform_pose,
    )
    subscriber = rospy.Subscriber(
        MARKER_TOPIC,
        AlvarMarkers,
        collector.callback,
        queue_size=50,
    )
    try:
        return _wait_for_registration(
            collector,
            arguments,
            parking,
            rospy,
        )
    finally:
        subscriber.unregister()


def _marker_template(template, marker_id):
    if isinstance(marker_id, bool) or not isinstance(marker_id, int):
        raise RegistryError("workbench marker ID must be an integer within 100-199")
    if marker_id < 100 or marker_id > 199:
        raise RegistryError("workbench marker ID must be within 100-199")
    try:
        workbenches = template["workbenches"]
        markers = [
            workbench["marker"]
            for workbench in workbenches
            if workbench["marker"]["id"] == marker_id
        ]
    except (KeyError, TypeError) as exc:
        raise RegistryError(
            "identity template has malformed marker entries: {0}".format(exc)
        )
    if len(markers) != 1:
        if not markers:
            raise RegistryError(
                "marker {0} is absent from identity template".format(marker_id)
            )
        raise RegistryError(
            "marker {0} is duplicated in identity template".format(marker_id)
        )
    marker = markers[0]
    try:
        size_m = marker["size_m"]
        digest = marker["artifact_sha256"]
    except (KeyError, TypeError) as exc:
        raise RegistryError(
            "marker {0} has incomplete template evidence: {1}".format(
                marker_id,
                exc,
            )
        )
    if (
        isinstance(size_m, bool)
        or not isinstance(size_m, (int, float))
        or not math.isfinite(float(size_m))
        or float(size_m) <= 0.0
    ):
        raise RegistryError(
            "marker {0} template size_m must be finite and positive".format(
                marker_id
            )
        )
    if (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(character not in "0123456789abcdef" for character in digest)
    ):
        raise RegistryError(
            "marker {0} template artifact digest is invalid".format(marker_id)
        )
    return marker


def _parking_target_argument(arguments):
    reference = getattr(arguments, "parking_reference", None)
    backoff = getattr(arguments, "backoff_distance_m", None)
    if reference is None:
        if backoff is not None:
            raise RegistryError("backoff requires --parking-reference")
        return None
    try:
        return normalize_parking_target({
            "schema_version": 1, "reference": reference,
            "backoff_distance_m": 0.0 if backoff is None else backoff})
    except ValueError as exc:
        raise RegistryError(str(exc)) from exc


def _workbench_document(arguments, result, template):
    marker_template = _marker_template(template, arguments.marker_id)
    target = _parking_target_argument(arguments)
    document = {
        "id": arguments.workbench_id,
        "marker": {
            "family": "ALVAR",
            "id": arguments.marker_id,
            "size_m": marker_template["size_m"],
            "artifact_sha256": marker_template["artifact_sha256"],
        },
        "map_pose": _pose_document(result.marker_pose, result.marker_height_m),
        "staging_pose": _pose_document(result.staging_pose, 0.0),
        "docking_offset": {
            "lateral_m": result.alignment_offset.y_m,
            "longitudinal_m": getattr(arguments, "alignment_longitudinal_m", 0.0),
            "yaw_rad": result.alignment_offset.yaw_rad,
        },
        "registration": _registration_document(result),
    }
    if target is not None:
        document["parking_target"] = target
        del document["docking_offset"]["longitudinal_m"]
    return document


def _append_registration(existing, candidate):
    """Preserve measured registrations; a new estimate cannot rewrite identity."""
    if existing is None:
        return deepcopy(candidate)
    for section in ("schema_version", "map", "camera_calibration", "parking_geometry", "lidar_safety"):
        if existing.get(section) != candidate.get(section):
            raise RegistryError("existing registry identity differs: " + section)
    ids = {entry["id"] for entry in existing["workbenches"]}
    tags = {entry["marker"]["id"] for entry in existing["workbenches"]}
    for entry in candidate["workbenches"]:
        if entry["id"] in ids or entry["marker"]["id"] in tags:
            raise RegistryError("workbench ID or marker is already registered")
        ids.add(entry["id"])
        tags.add(entry["marker"]["id"])
    merged = deepcopy(existing)
    merged["workbenches"].extend(deepcopy(candidate["workbenches"]))
    return merged


def main(argv=None):
    arguments = build_parser().parse_args(argv)
    try:
        _validate_output_location(arguments.map_path, arguments.output)
        if arguments.marker_id < 100 or arguments.marker_id > 199:
            raise RegistryError("workbench marker ID must be within 100-199")
        camera, parking, lidar, template = _identity_sources(arguments)
        marker = _marker_template(template, arguments.marker_id)
        target = _parking_target_argument(arguments)
        longitudinal = arguments.alignment_longitudinal_m if target is None else None
        try:
            final_tag_distance(
                front_reference_x_m=parking.front_reference_x_m,
                parking_safety_distance_m=parking.parking_safety_distance_m,
                longitudinal_docking_offset_m=longitudinal,
                marker_size_m=marker["size_m"], parking_target=target)
        except ValueError as exc:
            raise RegistryError(str(exc)) from exc
        result = _collect(arguments, camera, parking)
        print(yaml.safe_dump(_result_summary(result, longitudinal or 0.0), sort_keys=False), end="")
        if not arguments.accept:
            print("Not written: pass --accept after reviewing the estimate.")
            return 0
        document = build_registry_document(
            map_id=arguments.map_id or arguments.map_path.stem,
            map_yaml=arguments.map_path,
            camera=camera,
            parking=parking,
            lidar=lidar,
            workbenches=[_workbench_document(arguments, result, template)],
            registry_dir=arguments.output.parent,
        )
        # Serialize cooperating registration invocations across read/merge/write.
        # The registry writer still validates and atomically replaces its output.
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        with (arguments.output.parent / ("." + arguments.output.name + ".lock")).open("a") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            existing = None
            if arguments.output.exists():
                load_registry(arguments.output, map_yaml=arguments.map_path,
                    camera=camera, parking=parking, lidar=lidar, mode=arguments.mode,
                    commissioning=arguments.commissioning)
                existing = _load_template(arguments.output)
            write_registry(
                arguments.output, _append_registration(existing, document),
                map_yaml=arguments.map_path, camera=camera, parking=parking,
                lidar=lidar, mode=arguments.mode, commissioning=arguments.commissioning,
            )
        print("Wrote validated registry: {0}".format(arguments.output))
        return 0
    except (RegistryError, RegistrationError) as exc:
        print("registration failed: {0}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
