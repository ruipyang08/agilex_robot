#!/usr/bin/env python3
"""Expose conservative target and sparse-layout RGB perception over ROS1."""

import collections
from dataclasses import dataclass
from types import MappingProxyType
import threading
import time

from agx_pick_msg.msg import VisualTarget2D
from agx_pick_msg.srv import (
    DetectTubeCap,
    DetectTubeCapRequest,
    DetectTubeCapResponse,
    SelectTubeTarget2D,
    SelectTubeTarget2DResponse,
    DetectTubeCapConfigured,
    DetectTubeCapConfiguredResponse,
    SelectTubeTarget2DConfigured,
    SelectTubeTarget2DConfiguredResponse,
)
from bt_task_msgs.msg import LiftMotorMsg
from agx_tube_perception import (
    DetectorConfig,
    LayoutError,
    ObservationEpoch,
    batch_is_stale,
    classify_batch_status,
    classify_sensor_timing,
    fit_square_lattice,
    newest_consistent_target,
    recognize_unique_green_cap,
    TargetSample2D,
    TargetSelectionError,
    three_vector,
)
from agx_tube_perception.fixture_profile import profile_from_mapping
from agx_tube_perception.config_snapshot import ContextRegistry, legacy_request_context
from agx_tube_perception.service_logic import (
    classify_lift_feedback,
    compute_lift_target,
    intersect_pixel_ray_with_horizontal_plane,
    lift_height_batch_is_consistent,
    newest_coherent_observation,
    projected_ellipse_diameter_m,
    resolve_lift_relative_z,
    validate_cap_diameter,
    validate_mission_context,
)
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros


@dataclass(frozen=True)
class _Observation:
    stamp: object
    stamp_ns: int
    status: int
    detail: str
    candidate_count: int = 0
    lift_back_height_mm: int = 0
    planning_xyz: object = None
    observed_cap_diameter_m: float = 0.0
    target_uv: object = None
    principal_uv: object = None
    layout_status: int = DetectTubeCapResponse.LAYOUT_UNDERCONSTRAINED
    layout_detail: str = ""
    layout_candidate_count: int = 0
    observed_cap_tops: object = ()
    grid_u: object = ()
    grid_v: object = ()
    grid_axis_u: object = (0.0, 0.0)
    grid_axis_v: object = (0.0, 0.0)
    lattice_inlier_count: int = 0
    lattice_rms_error_m: float = 0.0


@dataclass(frozen=True)
class _TargetObservation:
    stamp_ns: int
    status: int
    detail: str
    candidate_count: int = 0
    sample: object = None


class TubePerceptionSession:
    """One immutable configuration plus its exclusively owned observation history."""

    def __init__(self, context, bridge, tf_buffer, lift_snapshot):
        self._request_context = context
        self._fixture_profile_id = context.fixture_profile_id
        self._profile = context.perception
        self._context = MappingProxyType({
            "planning_frame": context.perception.planning_frame,
            "lift": MappingProxyType({
                "state_topic": context.perception.lift_state_topic,
                "arm_base_height_at_zero_lift_above_floor_m": context.perception.arm_base_height_at_zero_lift_above_floor_m,
                "minimum_position_m": context.perception.lift_minimum_position_m,
                "maximum_position_m": context.perception.lift_maximum_position_m,
                "target_tolerance_m": context.perception.lift_target_tolerance_m,
                "state_max_age_sec": context.perception.lift_state_max_age_sec,
            }),
        })
        perception = vars(context.perception)
        self._target_cap_color = context.target_cap_color
        self._planning_frame = self._context["planning_frame"]
        self._color_frame = self._profile.color_frame
        self._lift_target = compute_lift_target(
            self._profile.table_surface_height_above_floor_m,
            self._context["lift"][
                "arm_base_height_at_zero_lift_above_floor_m"
            ],
            self._context["lift"]["minimum_position_m"],
            self._context["lift"]["maximum_position_m"],
        )
        self._zero_lift_cap_top_z_m = (
            self._lift_target.metres
            + self._profile.rack_height_above_surface_m
            + self._profile.body_height_m
            + self._profile.cap_height_m
        )
        self._lift_tolerance_mm = int(
            np.floor(
                self._context["lift"]["target_tolerance_m"] * 1000.0
                + 0.5
            )
        )
        self._workspace_min = np.asarray(
            three_vector(
                perception["workspace_min_xyz_m"], "workspace_min_xyz_m"
            ),
            dtype=np.float64,
        )
        self._workspace_max = np.asarray(
            three_vector(
                perception["workspace_max_xyz_m"], "workspace_max_xyz_m"
            ),
            dtype=np.float64,
        )

        defaults = DetectorConfig()
        self._detector_config = DetectorConfig(
            roi_normalized=self._profile.roi_normalized,
            minimum_layout_saturation=self._profile.minimum_layout_saturation,
            green_hsv_lower=self._profile.target_hsv_lower,
            green_hsv_upper=self._profile.target_hsv_upper,
            morphology_kernel_px=defaults.morphology_kernel_px,
            morphology_iterations=defaults.morphology_iterations,
            min_area_px=defaults.min_area_px,
            max_area_px=defaults.max_area_px,
            min_circularity=self._profile.minimum_cap_circularity,
            max_axis_ratio=self._profile.maximum_cap_axis_ratio,
            max_interframe_spread_m=perception["max_interframe_spread_m"],
        )
        self._required_consistent_frames = int(
            perception["required_consistent_frames"]
        )
        synchronizer_queue_size = int(perception["synchronizer_queue_size"])
        self._max_sync_skew_sec = float(perception["max_sync_skew_sec"])
        self._detection_timeout_sec = float(perception["detection_timeout_sec"])
        self._maximum_frame_age_sec = float(
            perception["maximum_frame_age_sec"]
        )
        self._maximum_target_temporal_spread_px = (
            self._profile.maximum_target_temporal_spread_px
        )

        self._bridge = bridge
        self._tf_buffer = tf_buffer
        self._lift_snapshot = lift_snapshot
        observation_capacity = max(
            synchronizer_queue_size * 2,
            self._required_consistent_frames * 4,
        )
        self._observations = collections.deque(maxlen=observation_capacity)
        self._observation_epoch = ObservationEpoch(self._observations)
        self._target_observations = collections.deque(
            maxlen=observation_capacity
        )
        self._target_observation_epoch = ObservationEpoch(
            self._target_observations
        )
        self._condition = threading.Condition()


    @staticmethod
    def _lift_faulted(message):
        if message is None:
            return False
        return bool(
            message.overflowI
            or message.overflowV
            or message.encodeErr
            or message.posBiasOver
            or message.underVol
            or message.overLoad
            or False  # externalCtrlMode = control source (lift CLI), not a fault
            # stopFlag is a commanded-state bit: the vendor driver reports it true
            # whenever the column is at rest, so treating it as a fault makes the
            # detector permanently unavailable (measured 2026-10-22: lift idle at
            # 0 mm -> stopFlag True -> /detect_tube_cap returned status 4). Reads only
            # measured height here, issues no motion, so it cannot mask a real stop.
            # Real faults stay covered by the flags above and fpsError below.
            or message.fpsError != 0
        )

    def _validated_lift_height(self):
        message, receipt_time = self._lift_snapshot()
        now_wall = time.monotonic()
        failure = classify_lift_feedback(
            has_feedback=message is not None,
            initialized=bool(message.initState) if message is not None else False,
            back_height_mm=message.backHeight if message is not None else 0,
            target_height_mm=message.targetHeight if message is not None else 0,
            expected_target_height_mm=self._lift_target.millimetres,
            stamp_ns=(
                message.header.stamp.to_nsec()
                if message is not None
                else 0
            ),
            now_ns=rospy.Time.now().to_nsec(),
            wall_receipt_age_sec=(
                now_wall - receipt_time
                if receipt_time is not None
                else float("inf")
            ),
            maximum_age_sec=self._context["lift"]["state_max_age_sec"],
            tolerance_mm=self._lift_tolerance_mm,
            faulted=self._lift_faulted(message),
        )
        return (
            failure,
            int(message.backHeight)
            if failure is None and message is not None
            else None,
        )

    def _append_observation(self, observation):
        with self._condition:
            if not self._observation_epoch.begin(observation.stamp_ns):
                return
            self._observation_epoch.append(observation)
            self._condition.notify_all()

    def _append_target_observation(self, observation):
        with self._condition:
            if not self._target_observation_epoch.begin(
                observation.stamp_ns
            ):
                return
            self._target_observation_epoch.append(observation)
            self._condition.notify_all()

    def _target_failure(self, stamp, status, detail, candidate_count=0):
        self._append_target_observation(
            _TargetObservation(
                stamp_ns=stamp.to_nsec(),
                status=status,
                detail=detail,
                candidate_count=candidate_count,
            )
        )

    def _failure(self, stamp, status, detail, candidate_count=0):
        self._append_observation(
            _Observation(
                stamp=stamp,
                stamp_ns=stamp.to_nsec(),
                status=status,
                detail=detail,
                candidate_count=candidate_count,
                layout_status=status,
                layout_detail=detail,
            )
        )

    def _in_workspace(self, point):
        values = np.asarray(point, dtype=np.float64)
        return bool(
            np.all(np.isfinite(values))
            and np.all(values >= self._workspace_min)
            and np.all(values <= self._workspace_max)
        )

    def _height_is_consistent(self, point, expected_cap_top_z_m):
        tolerance = (
            self._profile.cap_top_height_band_m
            + self._profile.height_tolerance_m
        )
        return abs(float(point[2]) - expected_cap_top_z_m) <= tolerance

    def _lookup_transform(self, stamp):
        point = PointStamped()
        point.header.frame_id = self._color_frame
        point.header.stamp = stamp
        return self._tf_buffer.lookup_transform(
            self._planning_frame,
            point.header.frame_id,
            point.header.stamp,
            rospy.Duration(0.10),
        )

    def _layout_outcome(
        self,
        selection,
        camera_info,
        transform,
        expected_cap_top_z_m,
    ):
        projected = []
        projected_index_by_candidate = {}
        for candidate_index, candidate in enumerate(selection.candidates):
            surface = candidate.surface
            try:
                point = intersect_pixel_ray_with_horizontal_plane(
                    surface.center_uv,
                    expected_cap_top_z_m,
                    camera_info.K,
                    transform,
                )
                diameter_m = projected_ellipse_diameter_m(
                    surface.ellipse,
                    expected_cap_top_z_m,
                    camera_info.K,
                    transform,
                )
                validate_cap_diameter(
                    diameter_m,
                    self._profile.cap_diameter_m,
                    self._profile.diameter_tolerance_m,
                )
                if not self._in_workspace(point):
                    raise ValueError("candidate is outside the workspace")
            except ValueError as error:
                if candidate_index == selection.selected_index:
                    return {
                        "status": DetectTubeCapResponse.INCONSISTENT_TARGET,
                        "detail": (
                            "selected target failed metric validation: {0}"
                        ).format(error),
                        "candidate_count": len(projected),
                    }
                continue
            projected_index_by_candidate[candidate_index] = len(projected)
            projected.append((surface, point))
        surfaces = tuple(item[0] for item in projected)
        planning_points = tuple(item[1] for item in projected)
        target_index = projected_index_by_candidate.get(
            selection.selected_index
        )
        if target_index is None:
            return {
                "status": DetectTubeCapResponse.INCONSISTENT_TARGET,
                "detail": "selected target is not a metric cap candidate",
                "candidate_count": len(surfaces),
            }

        try:
            fit = fit_square_lattice(
                np.asarray(
                    [(point[0], point[1]) for point in planning_points],
                    dtype=np.float64,
                ),
                target_index=target_index,
                rows=self._profile.rows,
                columns=self._profile.columns,
                pitch_m=self._profile.center_pitch_m,
                pitch_tolerance_m=self._profile.pitch_tolerance_m,
                minimum_inliers=self._profile.minimum_layout_inliers,
                maximum_rms_error_m=self._profile.maximum_lattice_rms_error_m,
            )
        except LayoutError as error:
            return {
                "status": DetectTubeCapResponse.LAYOUT_UNDERCONSTRAINED,
                "detail": str(error),
                "candidate_count": len(surfaces),
            }
        if target_index not in fit.inlier_indices:
            return {
                "status": DetectTubeCapResponse.INCONSISTENT_TARGET,
                "detail": "target cap is not a fitted lattice inlier",
                "candidate_count": len(surfaces),
            }
        inlier_points = tuple(
            planning_points[index] for index in fit.inlier_indices
        )
        return {
            "status": DetectTubeCapResponse.SUCCESS,
            "detail": "target and sparse layout detected",
            "candidate_count": len(surfaces),
            "points": inlier_points,
            "grid_u": fit.grid_u,
            "grid_v": fit.grid_v,
            "axis_u": fit.axis_u,
            "axis_v": fit.axis_v,
            "inlier_count": fit.inlier_count,
            "rms_error_m": fit.rms_error_m,
        }

    def _record_target_observation(self, stamp, camera_info, selected):
        if len(camera_info.K) != 9:
            raise ValueError("camera matrix must contain nine values")
        sample = TargetSample2D(
            stamp_ns=stamp.to_nsec(),
            frame_id=self._color_frame,
            image_width=int(camera_info.width),
            image_height=int(camera_info.height),
            target_uv=selected.center_uv,
            reference_uv=(
                float(camera_info.K[2]),
                float(camera_info.K[5]),
            ),
        )
        self._append_target_observation(
            _TargetObservation(
                stamp_ns=sample.stamp_ns,
                status=SelectTubeTarget2DResponse.SUCCESS,
                detail="target cap selected in RGB",
                candidate_count=1,
                sample=sample,
            )
        )

    def _synchronized_callback(self, color_message, camera_info):
        stamp = color_message.header.stamp
        timing_failure = classify_sensor_timing(
            color_stamp_ns=stamp.to_nsec(),
            camera_info_stamp_ns=camera_info.header.stamp.to_nsec(),
            now_stamp_ns=rospy.Time.now().to_nsec(),
            maximum_age_sec=self._maximum_frame_age_sec,
        )
        if timing_failure is not None:
            self._target_failure(
                stamp,
                SelectTubeTarget2DResponse.STALE_FRAME,
                timing_failure.detail,
            )
            self._failure(stamp, timing_failure.status, timing_failure.detail)
            return
        frame_detail = (
            "color image and camera info require the configured frame"
        )
        if (
            color_message.header.frame_id != self._color_frame
            or camera_info.header.frame_id != self._color_frame
        ):
            self._failure(
                stamp,
                DetectTubeCapResponse.TF_UNAVAILABLE,
                frame_detail,
            )
            self._target_failure(
                stamp,
                SelectTubeTarget2DResponse.INVALID_OBSERVATION,
                frame_detail,
            )
            return
        try:
            rgb = self._bridge.imgmsg_to_cv2(
                color_message, desired_encoding="rgb8"
            )
        except CvBridgeError as error:
            detail = "RGB conversion failed: {0}".format(error)
            self._target_failure(
                stamp,
                SelectTubeTarget2DResponse.INVALID_OBSERVATION,
                detail,
            )
            self._failure(
                stamp,
                DetectTubeCapResponse.TF_UNAVAILABLE,
                detail,
            )
            return

        try:
            selection = recognize_unique_green_cap(
                rgb, self._detector_config
            )
            self._record_target_observation(
                stamp, camera_info, selection.target
            )
        except TargetSelectionError as error:
            target_statuses = {
                "target_absent": SelectTubeTarget2DResponse.TARGET_ABSENT,
                "ambiguous_target": (
                    SelectTubeTarget2DResponse.AMBIGUOUS_TARGET
                ),
            }
            metric_statuses = {
                "target_absent": DetectTubeCapResponse.TARGET_ABSENT,
                "ambiguous_target": DetectTubeCapResponse.AMBIGUOUS_TARGET,
            }
            if error.code not in target_statuses:
                raise RuntimeError(
                    "unsupported target-selection error: {0}".format(
                        error.code
                    )
                )
            self._target_failure(
                stamp,
                target_statuses[error.code],
                error.detail,
                error.candidate_count,
            )
            self._failure(
                stamp,
                metric_statuses[error.code],
                error.detail,
                error.candidate_count,
            )
            return
        except (cv2.error, TypeError, ValueError) as error:
            detail = "image target observation is invalid: {0}".format(error)
            self._target_failure(
                stamp,
                SelectTubeTarget2DResponse.INVALID_OBSERVATION,
                detail,
            )
            self._failure(
                stamp,
                DetectTubeCapResponse.INCONSISTENT_TARGET,
                detail,
            )
            return

        lift_failure, lift_back_height_mm = self._validated_lift_height()
        if lift_failure is not None:
            self._failure(
                stamp,
                lift_failure.status,
                lift_failure.detail,
            )
            return
        expected_cap_top_z_m = resolve_lift_relative_z(
            self._zero_lift_cap_top_z_m,
            lift_back_height_mm,
        )
        try:
            transform = self._lookup_transform(stamp)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            ValueError,
        ) as error:
            self._failure(
                stamp,
                DetectTubeCapResponse.TF_UNAVAILABLE,
                "stamped camera TF unavailable: {0}".format(error),
            )
            return

        target_surface = selection.target.surface
        try:
            target_planning = intersect_pixel_ray_with_horizontal_plane(
                target_surface.center_uv,
                expected_cap_top_z_m,
                camera_info.K,
                transform,
            )
            observed_cap_diameter_m = projected_ellipse_diameter_m(
                target_surface.ellipse,
                expected_cap_top_z_m,
                camera_info.K,
                transform,
            )
            validate_cap_diameter(
                observed_cap_diameter_m,
                self._profile.cap_diameter_m,
                self._profile.diameter_tolerance_m,
            )
            principal_uv = (camera_info.K[2], camera_info.K[5])
        except ValueError as error:
            self._failure(
                stamp,
                DetectTubeCapResponse.INCONSISTENT_TARGET,
                "target plane projection failed: {0}".format(error),
                1,
            )
            return

        if not self._in_workspace(target_planning):
            self._failure(
                stamp,
                DetectTubeCapResponse.OUT_OF_WORKSPACE,
                "detected cap top is outside the configured workspace",
                1,
            )
            return
        if not self._height_is_consistent(
            target_planning, expected_cap_top_z_m
        ):
            self._failure(
                stamp,
                DetectTubeCapResponse.OUT_OF_WORKSPACE,
                "detected cap top is outside the expected height band",
                1,
            )
            return

        try:
            layout = self._layout_outcome(
                selection,
                camera_info,
                transform,
                expected_cap_top_z_m,
            )
        except ValueError as error:
            layout = {
                "status": DetectTubeCapResponse.LAYOUT_UNDERCONSTRAINED,
                "detail": "layout extraction failed: {0}".format(error),
                "candidate_count": 0,
            }
        self._append_observation(
            _Observation(
                stamp=stamp,
                stamp_ns=stamp.to_nsec(),
                status=DetectTubeCapResponse.SUCCESS,
                detail="target cap detected",
                candidate_count=1,
                lift_back_height_mm=lift_back_height_mm,
                planning_xyz=target_planning,
                observed_cap_diameter_m=observed_cap_diameter_m,
                target_uv=target_surface.center_uv,
                principal_uv=principal_uv,
                layout_status=layout["status"],
                layout_detail=layout["detail"],
                layout_candidate_count=layout["candidate_count"],
                observed_cap_tops=layout.get("points", ()),
                grid_u=layout.get("grid_u", ()),
                grid_v=layout.get("grid_v", ()),
                grid_axis_u=layout.get("axis_u", (0.0, 0.0)),
                grid_axis_v=layout.get("axis_v", (0.0, 0.0)),
                lattice_inlier_count=layout.get("inlier_count", 0),
                lattice_rms_error_m=layout.get("rms_error_m", 0.0),
            )
        )

    @staticmethod
    def _target_response_for_failure(status, detail, candidate_count):
        response = SelectTubeTarget2DResponse()
        response.status = status
        response.detail = detail
        response.target_candidate_count = candidate_count
        return response

    @staticmethod
    def _response_for_failure(
        status, detail, candidate_count, layout_candidate_count=0
    ):
        response = DetectTubeCapResponse()
        response.status = status
        response.detail = detail
        response.target_candidate_count = candidate_count
        response.layout_candidate_count = layout_candidate_count
        return response

    def _complete_response(self, observations, mode):
        full_layout = mode == DetectTubeCapRequest.FULL_LAYOUT
        statuses = (
            [observation.layout_status for observation in observations]
            if full_layout
            else [observation.status for observation in observations]
        )
        status = classify_batch_status(statuses)
        candidate_count = max(
            observation.candidate_count for observation in observations
        )
        layout_candidate_count = (
            max(
                observation.layout_candidate_count
                for observation in observations
            )
            if full_layout
            else 0
        )
        if status == DetectTubeCapResponse.INCONSISTENT_TARGET and any(
            value != DetectTubeCapResponse.INCONSISTENT_TARGET
            for value in statuses
        ):
            return self._response_for_failure(
                DetectTubeCapResponse.INCONSISTENT_TARGET,
                "fresh observations contain mixed outcomes",
                candidate_count,
                layout_candidate_count,
            )
        if status != DetectTubeCapResponse.SUCCESS:
            detail = (
                observations[-1].layout_detail
                if full_layout
                else observations[-1].detail
            )
            return self._response_for_failure(
                status,
                detail,
                candidate_count,
                layout_candidate_count,
            )
        try:
            newest, response_cap_top = newest_coherent_observation(
                observations,
                self._detector_config.max_interframe_spread_m,
                require_layout=full_layout,
            )
        except (RuntimeError, ValueError) as error:
            return self._response_for_failure(
                DetectTubeCapResponse.INCONSISTENT_TARGET,
                getattr(error, "detail", str(error)),
                candidate_count,
                layout_candidate_count,
            )

        response = DetectTubeCapResponse()
        response.status = DetectTubeCapResponse.SUCCESS
        response.detail = (
            "consistent target and sparse layout detected"
            if full_layout
            else "consistent target cap detected"
        )
        response.cap_top.header.frame_id = self._planning_frame
        response.cap_top.header.stamp = newest.stamp
        (
            response.cap_top.point.x,
            response.cap_top.point.y,
            response.cap_top.point.z,
        ) = response_cap_top
        response.target_candidate_count = newest.candidate_count
        response.observed_cap_diameter_m = float(
            newest.observed_cap_diameter_m
        )
        response.target_u_px = float(newest.target_uv[0])
        response.target_v_px = float(newest.target_uv[1])
        response.principal_u_px = float(newest.principal_uv[0])
        response.principal_v_px = float(newest.principal_uv[1])
        if full_layout:
            response.layout_candidate_count = newest.layout_candidate_count
            response.lattice_inlier_count = newest.lattice_inlier_count
            response.lattice_rms_error_m = newest.lattice_rms_error_m
            response.grid_u = newest.grid_u
            response.grid_v = newest.grid_v
            (
                response.grid_axis_u.x,
                response.grid_axis_u.y,
                response.grid_axis_u.z,
            ) = (newest.grid_axis_u[0], newest.grid_axis_u[1], 0.0)
            (
                response.grid_axis_v.x,
                response.grid_axis_v.y,
                response.grid_axis_v.z,
            ) = (newest.grid_axis_v[0], newest.grid_axis_v[1], 0.0)
            response.observed_cap_tops = [
                Point(x=point_xyz[0], y=point_xyz[1], z=point_xyz[2])
                for point_xyz in newest.observed_cap_tops
            ]
        return response

    def _handle_target_request(self, request):
        requested_profile_id = request.fixture_profile_id
        if requested_profile_id != self._fixture_profile_id:
            return self._target_response_for_failure(
                SelectTubeTarget2DResponse.PROFILE_MISMATCH,
                "requested fixture profile is not active",
                0,
            )

        deadline = self._detection_timeout_sec + time.monotonic()
        while not rospy.is_shutdown():
            with self._condition:
                batch = self._target_observation_epoch.select_batch(
                    not_before_ns=request.not_before.to_nsec(),
                    required_count=self._required_consistent_frames,
                )
                if batch is not None:
                    candidate_count = max(
                        observation.candidate_count
                        for observation in batch
                    )
                    if batch_is_stale(
                        batch,
                        now_stamp_ns=rospy.Time.now().to_nsec(),
                        maximum_age_sec=self._maximum_frame_age_sec,
                    ):
                        return self._target_response_for_failure(
                            SelectTubeTarget2DResponse.STALE_FRAME,
                            "fresh target batch exceeded frame age limit",
                            candidate_count,
                        )
                    statuses = tuple(
                        observation.status for observation in batch
                    )
                    if len(set(statuses)) != 1:
                        return self._target_response_for_failure(
                            SelectTubeTarget2DResponse.INCONSISTENT_TARGET,
                            "fresh target observations contain mixed outcomes",
                            candidate_count,
                        )
                    status = statuses[0]
                    if status != SelectTubeTarget2DResponse.SUCCESS:
                        return self._target_response_for_failure(
                            status,
                            batch[-1].detail,
                            candidate_count,
                        )
                    try:
                        newest = newest_consistent_target(
                            tuple(
                                observation.sample for observation in batch
                            ),
                            self._maximum_target_temporal_spread_px,
                        )
                    except (TargetSelectionError, ValueError) as error:
                        return self._target_response_for_failure(
                            SelectTubeTarget2DResponse.INCONSISTENT_TARGET,
                            getattr(error, "detail", str(error)),
                            candidate_count,
                        )

                    response = SelectTubeTarget2DResponse()
                    response.status = SelectTubeTarget2DResponse.SUCCESS
                    response.detail = "consistent 2D target selected"
                    response.target = VisualTarget2D()
                    seconds, nanoseconds = divmod(
                        newest.stamp_ns, 1_000_000_000
                    )
                    response.target.header.stamp = rospy.Time(
                        seconds, nanoseconds
                    )
                    response.target.header.frame_id = newest.frame_id
                    response.target.u_px = newest.target_uv[0]
                    response.target.v_px = newest.target_uv[1]
                    response.target.reference_u_px = newest.reference_uv[0]
                    response.target.reference_v_px = newest.reference_uv[1]
                    response.target.image_width = newest.image_width
                    response.target.image_height = newest.image_height
                    response.target_candidate_count = 1
                    return response
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(timeout=remaining)
        return self._target_response_for_failure(
            SelectTubeTarget2DResponse.STALE_FRAME,
            "timed out waiting for a complete fresh target batch",
            0,
        )

    def _handle_request(self, request):
        if request.fixture_profile_id != self._fixture_profile_id:
            return self._response_for_failure(
                DetectTubeCapResponse.PROFILE_MISMATCH,
                "requested fixture profile is not active",
                0,
            )
        if request.mode not in (request.TARGET_ONLY, request.FULL_LAYOUT):
            return self._response_for_failure(
                DetectTubeCapResponse.INVALID_REQUEST,
                "unsupported detection mode",
                0,
            )
        if request.target_cap_color != self._target_cap_color:
            return self._response_for_failure(
                DetectTubeCapResponse.INVALID_REQUEST,
                "request target color does not match the selected profile",
                0,
            )

        lift_failure, request_lift_back_height_mm = (
            self._validated_lift_height()
        )
        if lift_failure is not None:
            return self._response_for_failure(
                DetectTubeCapResponse.STALE_FRAME,
                lift_failure.detail,
                0,
            )
        deadline = time.monotonic() + self._detection_timeout_sec
        while not rospy.is_shutdown():
            with self._condition:
                batch = self._observation_epoch.select_batch(
                    not_before_ns=request.not_before.to_nsec(),
                    required_count=self._required_consistent_frames,
                )
                if batch is not None:
                    lift_failure, current_lift_back_height_mm = (
                        self._validated_lift_height()
                    )
                    lift_consistent = (
                        lift_failure is None
                        and lift_height_batch_is_consistent(
                            request_height_mm=request_lift_back_height_mm,
                            current_height_mm=current_lift_back_height_mm,
                            observation_heights_mm=(
                                observation.lift_back_height_mm
                                for observation in batch
                                if observation.status
                                == DetectTubeCapResponse.SUCCESS
                            ),
                            target_height_mm=self._lift_target.millimetres,
                            tolerance_mm=self._lift_tolerance_mm,
                        )
                    )
                    if not lift_consistent:
                        return self._response_for_failure(
                            DetectTubeCapResponse.STALE_FRAME,
                            (
                                lift_failure.detail
                                if lift_failure is not None
                                else "observation batch lift height changed"
                            ),
                            max(
                                observation.candidate_count
                                for observation in batch
                            ),
                        )
                    if batch_is_stale(
                        batch,
                        now_stamp_ns=rospy.Time.now().to_nsec(),
                        maximum_age_sec=self._maximum_frame_age_sec,
                    ):
                        return self._response_for_failure(
                            DetectTubeCapResponse.STALE_FRAME,
                            "fresh observation batch exceeded frame age limit",
                            max(
                                observation.candidate_count
                                for observation in batch
                            ),
                            (
                                max(
                                    observation.layout_candidate_count
                                    for observation in batch
                                )
                                if request.mode
                                == DetectTubeCapRequest.FULL_LAYOUT
                                else 0
                            ),
                        )
                    return self._complete_response(batch, request.mode)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(timeout=remaining)
        return self._response_for_failure(
            DetectTubeCapResponse.STALE_FRAME,
            "timed out waiting for a complete fresh observation batch",
            0,
        )



class TubeCapDetectorNode(TubePerceptionSession):
    """Share sensor transport while isolating all legacy/configured histories."""

    def __init__(self):
        registry = self._required_mapping("/tube_fixture_profiles")
        mission = self._required_mapping("/tube_grasp")
        profile_id = mission.get("active_profile_id")
        if not isinstance(profile_id, str) or not profile_id.strip():
            raise ValueError("active_profile_id must be a nonempty string")
        if profile_id not in registry:
            raise ValueError("active fixture profile is not present in registry")
        self._fixture_profile_id = profile_id
        self._profile = profile_from_mapping(
            self._fixture_profile_id,
            registry[self._fixture_profile_id],
        )
        self._context = validate_mission_context(
            mission,
            expected_profile_id=self._fixture_profile_id,
            expected_color=self._profile.target_cap_color,
        )
        if (
            self._profile.target_cap_color != "green"
            or mission["target_cap_color"] != "green"
        ):
            raise ValueError("commissioned target_cap_color must be green")

        context = legacy_request_context(self._fixture_profile_id, self._profile, mission)
        self._lift_lock = threading.Lock()
        self._latest_lift_state = None
        self._latest_lift_wall_time = None
        bridge = CvBridge()
        tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(tf_buffer)
        super().__init__(context, bridge, tf_buffer, self._lift_snapshot)
        self._configured_contexts = ContextRegistry(
            rospy.get_param("~maximum_configured_contexts", 16))
        self._sessions_lock = threading.Lock()
        self._configured_sessions = {}
        synchronizer_queue_size = context.perception.synchronizer_queue_size
        subscriber_queue_size = max(1, min(synchronizer_queue_size, 5))
        self._lift_state_subscriber = rospy.Subscriber(
            self._context["lift"]["state_topic"],
            LiftMotorMsg,
            self._on_lift_state,
            queue_size=5,
        )
        self._color_subscriber = message_filters.Subscriber(
            "color", Image, queue_size=subscriber_queue_size, buff_size=2 ** 24
        )
        self._camera_info_subscriber = message_filters.Subscriber(
            "camera_info", CameraInfo, queue_size=subscriber_queue_size
        )
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (
                self._color_subscriber,
                self._camera_info_subscriber,
            ),
            queue_size=synchronizer_queue_size,
            slop=self._max_sync_skew_sec,
            allow_headerless=False,
        )
        self._synchronizer.registerCallback(self._dispatch_synchronized)
        self._target_service = rospy.Service(
            "/select_tube_target_2d",
            SelectTubeTarget2D,
            self._handle_target_request
        )
        self._service = rospy.Service("/detect_tube_cap", DetectTubeCap, self._handle_request)

        self._configured_target_service = rospy.Service(
            "/select_tube_target_2d_configured", SelectTubeTarget2DConfigured,
            self._handle_configured_target_request)
        self._configured_service = rospy.Service(
            "/detect_tube_cap_configured", DetectTubeCapConfigured,
            self._handle_configured_request)

    @staticmethod
    def _required_mapping(name):
        value = rospy.get_param(name)
        if not isinstance(value, dict):
            raise ValueError("{0} must be a mapping".format(name))
        return value

    def _on_lift_state(self, message):
        with self._lift_lock:
            self._latest_lift_state = message
            self._latest_lift_wall_time = time.monotonic()

    def _lift_snapshot(self):
        with self._lift_lock:
            return self._latest_lift_state, self._latest_lift_wall_time

    def _dispatch_synchronized(self, color_message, camera_info):
        self._synchronized_callback(color_message, camera_info)
        with self._sessions_lock:
            sessions = tuple(self._configured_sessions.values())
        for session in sessions:
            session._synchronized_callback(color_message, camera_info)

    def _validate_transport(self, perception):
        active = self._request_context.perception
        # Transport parameters belong to node startup, not request mutation.
        # Reject unsupported transport instead of accepting ignored fields.
        for name in ("planning_frame", "color_frame", "lift_state_topic",
                     "synchronizer_queue_size", "max_sync_skew_sec"):
            if getattr(perception, name) != getattr(active, name):
                raise ValueError("configured perception requires unsupported transport: " + name)

    def _configured_session(self, request):
        context = self._configured_contexts.admit(request, self._validate_transport)
        with self._sessions_lock:
            if context.history_key not in self._configured_sessions:
                self._configured_sessions[context.history_key] = TubePerceptionSession(
                    context, self._bridge, self._tf_buffer, self._lift_snapshot)
            return context, self._configured_sessions[context.history_key]

    @staticmethod
    def _echo_identity(response, identity):
        for name in ("config_id", "fixture_instance_id", "fixture_profile_id", "projection_sha256"):
            setattr(response, name, getattr(identity, name))
        return response

    def _configured_response(self, request, response_type, target_only):
        response = response_type()
        try:
            stamp_ns = request.not_before.to_nsec()
            if stamp_ns < 0:
                raise ValueError("not_before must be nonnegative")
            mode = DetectTubeCapRequest.TARGET_ONLY if target_only else request.mode
            if mode not in (DetectTubeCapRequest.TARGET_ONLY, DetectTubeCapRequest.FULL_LAYOUT):
                raise ValueError("unsupported detection mode")
            context, session = self._configured_session(request)
        except (AttributeError, TypeError, ValueError) as error:
            response.status = response.INVALID_REQUEST
            response.detail = str(error)
            return self._echo_identity(response, request)
        seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
        frozen_request = _SnapshotRequest(
            context.fixture_profile_id, context.target_cap_color,
            rospy.Time(seconds, nanoseconds), mode)
        legacy = (session._handle_target_request(frozen_request) if target_only
                  else session._handle_request(frozen_request))
        for name in legacy.__slots__:
            setattr(response, name, getattr(legacy, name))
        return self._echo_identity(response, context)

    def _handle_configured_target_request(self, request):
        return self._configured_response(request, SelectTubeTarget2DConfiguredResponse, True)

    def _handle_configured_request(self, request):
        return self._configured_response(request, DetectTubeCapConfiguredResponse, False)


@dataclass(frozen=True)
class _SnapshotRequest:
    fixture_profile_id: str
    target_cap_color: str
    not_before: object
    mode: int
    TARGET_ONLY = 0
    FULL_LAYOUT = 1


def main():
    rospy.init_node("tube_cap_detector", anonymous=False)
    TubeCapDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
