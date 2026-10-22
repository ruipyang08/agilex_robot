#!/usr/bin/env python3
"""End-to-end synthetic RGB contract for the ROS1 detector service."""

import math
import time
import unittest

from agx_pick_msg.srv import (
    DetectTubeCap,
    DetectTubeCapRequest,
    SelectTubeTarget2D,
    SelectTubeTarget2DRequest,
)
from agx_tube_perception.service_logic import (
    camera_point_from_planning,
    commissioned_search_camera_transform,
)
from bt_task_msgs.msg import LiftMotorMsg
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import TransformStamped
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros


class DetectorLayoutServiceTest(unittest.TestCase):
    WIDTH = 640
    HEIGHT = 480
    FX = 400.0
    FY = 300.0
    CX = 320.0
    CY = 240.0
    FRAME = "cam1_color_optical_frame"
    PROFILE = "table_grid_3x3_v1"
    LIFT_HEIGHT_M = 0.100
    ZERO_LIFT_CAP_Z = 0.194
    CAP_Z = ZERO_LIFT_CAP_Z - LIFT_HEIGHT_M
    CAMERA_CLEARANCE_M = 0.100

    @classmethod
    def setUpClass(cls):
        cls.bridge = CvBridge()
        cls.color_publisher = rospy.Publisher(
            "/test_tube/color", Image, queue_size=5
        )
        cls.info_publisher = rospy.Publisher(
            "/test_tube/camera_info", CameraInfo, queue_size=5
        )
        cls.lift_publisher = rospy.Publisher(
            "/lifter_1/LiftMotorStatePub",
            LiftMotorMsg,
            queue_size=1,
            latch=True,
        )
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer)
        cls.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        mission = rospy.get_param("/tube_grasp")
        profile = rospy.get_param(
            "/tube_fixture_profiles/{0}".format(cls.PROFILE)
        )
        fixture_center = mission["context"][
            "nominal_fixture_center_xyz_m"
        ]
        configured_zero_lift_cap_z = (
            profile["table"]["surface_height_above_floor_m"]
            - mission["lift"][
                "arm_base_height_at_zero_lift_above_floor_m"
            ]
            + profile["layout"]["rack_height_above_surface_m"]
            + profile["tube"]["body_height_m"]
            + profile["tube"]["cap_height_m"]
        )
        if not math.isclose(
            configured_zero_lift_cap_z,
            cls.ZERO_LIFT_CAP_Z,
            abs_tol=1.0e-9,
        ):
            raise RuntimeError("configured zero-lift cap height changed")
        camera_offset = mission["transforms"]["link6_to_camera_xyz_m"]
        cls.CAMERA_TRANSFORM = commissioned_search_camera_transform(
            search_tcp_xyz=(
                fixture_center[0] - camera_offset[0],
                fixture_center[1] + camera_offset[1],
                cls.CAP_Z + cls.CAMERA_CLEARANCE_M,
            ),
            link6_to_tcp_xyz=mission["transforms"][
                "link6_to_grasp_tcp_xyz_m"
            ],
            link6_to_tcp_quaternion_xyzw=mission["transforms"][
                "link6_to_grasp_tcp_quaternion_xyzw"
            ],
            link6_to_camera_xyz=mission["transforms"][
                "link6_to_camera_xyz_m"
            ],
            link6_to_camera_quaternion_xyzw=mission["transforms"][
                "link6_to_camera_quaternion_xyzw"
            ],
        )
        cls.FIXTURE_CENTER = (
            float(fixture_center[0]),
            float(fixture_center[1]),
            float(fixture_center[2]) - cls.LIFT_HEIGHT_M,
        )
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "link_base"
        transform.child_frame_id = cls.FRAME
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = cls.CAMERA_TRANSFORM.translation_xyz
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = cls.CAMERA_TRANSFORM.quaternion_xyzw
        cls.tf_broadcaster.sendTransform(transform)
        rospy.wait_for_service("/detect_tube_cap", timeout=20.0)
        rospy.wait_for_service("/select_tube_target_2d", timeout=20.0)
        cls.detect = rospy.ServiceProxy("/detect_tube_cap", DetectTubeCap)
        cls.select_target = rospy.ServiceProxy(
            "/select_tube_target_2d", SelectTubeTarget2D
        )
        cls._wait_for_readiness()

    @classmethod
    def _wait_for_readiness(cls):
        """Bound startup on both sensor subscriptions and latched static TF."""
        rospy.sleep(0.25)
        deadline = time.monotonic() + 10.0
        while (
            cls.color_publisher.get_num_connections() == 0
            or cls.info_publisher.get_num_connections() == 0
            or cls.lift_publisher.get_num_connections() == 0
            or not cls.tf_buffer.can_transform(
                "link_base",
                cls.FRAME,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "detector subscriptions or static TF were not ready"
                )
            rospy.sleep(0.05)

    @classmethod
    def _grid_centers(cls, pitch_m=0.040):
        angle = math.radians(17.0)
        axis_u = np.asarray((math.cos(angle), math.sin(angle)))
        axis_v = np.asarray((-math.sin(angle), math.cos(angle)))
        result = {}
        for row in range(3):
            for column in range(3):
                offset = (
                    (column - 1) * pitch_m * axis_u
                    + (row - 1) * pitch_m * axis_v
                )
                planning = (
                    cls.FIXTURE_CENTER[0] + offset[0],
                    cls.FIXTURE_CENTER[1] + offset[1],
                    cls.CAP_Z,
                )
                camera = camera_point_from_planning(
                    planning, cls.CAMERA_TRANSFORM
                )
                if camera[2] <= 0.0:
                    raise RuntimeError("cap is behind commissioned camera")
                result[(row, column)] = (
                    (
                        int(round(cls.CX + cls.FX * camera[0] / camera[2])),
                        int(round(cls.CY + cls.FY * camera[1] / camera[2])),
                    ),
                    camera[2],
                )
        return result

    @classmethod
    def _messages(
        cls,
        target,
        omitted,
        pitch_m=0.040,
        target_diameter_m=0.020,
    ):
        rgb = np.zeros((cls.HEIGHT, cls.WIDTH, 3), dtype=np.uint8)
        palette = ((25, 64, 217), (217, 140, 25), (140, 51, 191))
        palette_index = 0
        for slot, geometry in sorted(cls._grid_centers(pitch_m).items()):
            if slot in omitted:
                continue
            center, cap_depth = geometry
            color = (25, 204, 51) if slot == target else palette[palette_index % 3]
            if slot != target:
                palette_index += 1
            diameter_m = (
                target_diameter_m if slot == target else 0.020
            )
            radius = int(
                round(
                    ((cls.FX + cls.FY) / 2.0)
                    * diameter_m
                    / (2.0 * cap_depth)
                )
            )
            cv2.circle(rgb, center, radius, color, thickness=-1)

        color = cls.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        info = CameraInfo()
        info.width = cls.WIDTH
        info.height = cls.HEIGHT
        info.K = (
            cls.FX,
            0.0,
            cls.CX,
            0.0,
            cls.FY,
            cls.CY,
            0.0,
            0.0,
            1.0,
        )
        return color, info

    @classmethod
    def _publish_batch(
        cls,
        target,
        omitted=frozenset(),
        pitch_m=0.040,
        target_diameter_m=0.020,
    ):
        color, info = cls._messages(
            target, omitted, pitch_m, target_diameter_m
        )
        first_stamp = None
        for _ in range(3):
            stamp = rospy.Time.now()
            if first_stamp is None:
                first_stamp = stamp
            cls._publish_frame(color, info, stamp)
            cls.last_published_stamp = stamp
            rospy.sleep(0.08)
        rospy.sleep(0.15)
        return first_stamp

    @classmethod
    def _publish_frame(cls, color, info, stamp):
        cls._publish_lift_state()
        color.header.stamp = stamp
        info.header.stamp = stamp
        color.header.frame_id = cls.FRAME
        info.header.frame_id = cls.FRAME
        cls.color_publisher.publish(color)
        cls.info_publisher.publish(info)

    @classmethod
    def _publish_lift_state(
        cls,
        initialized=True,
        back_height_mm=100,
        target_height_mm=100,
    ):
        state = LiftMotorMsg()
        state.header.stamp = rospy.Time.now()
        state.initState = initialized
        state.backHeight = back_height_mm
        state.targetHeight = target_height_mm
        state.overflowI = False
        state.overflowV = False
        state.encodeErr = False
        state.posBiasOver = False
        state.underVol = False
        state.overLoad = False
        state.externalCtrlMode = False
        state.stopFlag = False
        state.fpsError = 0
        cls.lift_publisher.publish(state)

    @classmethod
    def _request(cls, mode, not_before):
        request = DetectTubeCapRequest()
        request.fixture_profile_id = cls.PROFILE
        request.target_cap_color = "green"
        request.mode = mode
        request.not_before = not_before
        return cls.detect(request)

    @classmethod
    def _request_target_2d(cls, not_before):
        request = SelectTubeTarget2DRequest()
        request.fixture_profile_id = cls.PROFILE
        request.not_before = not_before
        return cls.select_target(request)

    def test_sparse_layout_and_off_center_target_pixels(self):
        first_stamp = self._publish_batch(
            target=(0, 0),
            omitted=frozenset(((0, 2), (1, 2), (2, 2))),
        )
        target = self._request(DetectTubeCapRequest.TARGET_ONLY, first_stamp)
        self.assertEqual(target.status, target.SUCCESS, target.detail)
        self.assertGreater(
            math.hypot(
                target.target_u_px - target.principal_u_px,
                target.target_v_px - target.principal_v_px,
            ),
            1.0,
        )

        layout = self._request(DetectTubeCapRequest.FULL_LAYOUT, first_stamp)
        self.assertEqual(layout.status, layout.SUCCESS, layout.detail)
        self.assertEqual(layout.target_candidate_count, 1)
        self.assertEqual(layout.layout_candidate_count, 6)
        self.assertEqual(layout.lattice_inlier_count, 6)
        self.assertEqual(len(layout.observed_cap_tops), 6)
        self.assertEqual(len(layout.grid_u), 6)
        self.assertEqual(len(layout.grid_v), 6)
        self.assertLessEqual(layout.lattice_rms_error_m, 0.004)
        target_index = next(
            index
            for index, key in enumerate(zip(layout.grid_u, layout.grid_v))
            if key == (0, 0)
        )
        layout_target = layout.observed_cap_tops[target_index]
        self.assertEqual(layout.cap_top.header.stamp, self.last_published_stamp)
        self.assertAlmostEqual(layout.cap_top.point.x, layout_target.x)
        self.assertAlmostEqual(layout.cap_top.point.y, layout_target.y)
        self.assertAlmostEqual(layout.cap_top.point.z, layout_target.z)

    def test_every_target_slot_supports_target_then_full_layout(self):
        for row in range(3):
            for column in range(3):
                first_stamp = self._publish_batch((row, column))
                target = self._request(
                    DetectTubeCapRequest.TARGET_ONLY, first_stamp
                )
                self.assertEqual(target.status, target.SUCCESS, target.detail)
                layout = self._request(
                    DetectTubeCapRequest.FULL_LAYOUT, first_stamp
                )
                self.assertEqual(layout.status, layout.SUCCESS, layout.detail)
                self.assertEqual(layout.layout_candidate_count, 9)
                self.assertEqual(layout.lattice_inlier_count, 9)

    def test_ambiguous_green_fails_closed(self):
        rgb_message, info = self._messages((1, 1), frozenset())
        rgb_array = self.bridge.imgmsg_to_cv2(
            rgb_message, desired_encoding="rgb8"
        )
        second, second_depth = self._grid_centers()[(0, 0)]
        radius = int(
            round(
                ((self.FX + self.FY) / 2.0)
                * 0.010
                / second_depth
            )
        )
        cv2.circle(
            rgb_array, second, radius, (25, 204, 51), thickness=-1
        )
        rgb_message = self.bridge.cv2_to_imgmsg(
            rgb_array, encoding="rgb8"
        )
        # Exercise the service through the regular publisher path with the
        # malformed frame by publishing the modified messages three times.
        first_stamp = None
        for _ in range(3):
            stamp = rospy.Time.now()
            first_stamp = first_stamp or stamp
            self._publish_frame(rgb_message, info, stamp)
            rospy.sleep(0.08)
        response = self._request(
            DetectTubeCapRequest.FULL_LAYOUT, first_stamp
        )
        self.assertEqual(response.status, response.AMBIGUOUS_TARGET)

    def test_target_only_rejects_wrong_physical_diameter(self):
        for diameter_m in (0.012, 0.028):
            with self.subTest(diameter_m=diameter_m):
                first_stamp = self._publish_batch(
                    (1, 1), target_diameter_m=diameter_m
                )
                target_2d = self._request_target_2d(first_stamp)
                self.assertEqual(
                    target_2d.status, target_2d.SUCCESS, target_2d.detail
                )
                response = self._request(
                    DetectTubeCapRequest.TARGET_ONLY, first_stamp
                )
                self.assertEqual(
                    response.status, response.INCONSISTENT_TARGET
                )
                self.assertIn("diameter", response.detail)

    def test_rotated_seven_inlier_layout(self):
        first_stamp = self._publish_batch(
            (1, 1), omitted=frozenset(((0, 0), (2, 2)))
        )
        response = self._request(
            DetectTubeCapRequest.FULL_LAYOUT, first_stamp
        )
        self.assertEqual(response.status, response.SUCCESS, response.detail)
        self.assertEqual(response.layout_candidate_count, 7)
        self.assertEqual(response.lattice_inlier_count, 7)

    def test_underconstrained_and_wrong_pitch_layouts_fail_closed(self):
        first_stamp = self._publish_batch(
            (1, 1),
            omitted=frozenset(((0, 0), (0, 1), (0, 2), (2, 2))),
        )
        response = self._request(
            DetectTubeCapRequest.FULL_LAYOUT, first_stamp
        )
        self.assertEqual(response.status, response.LAYOUT_UNDERCONSTRAINED)
        self.assertEqual(response.layout_candidate_count, 5)

        first_stamp = self._publish_batch((1, 1), pitch_m=0.056)
        response = self._request(
            DetectTubeCapRequest.FULL_LAYOUT, first_stamp
        )
        self.assertEqual(response.status, response.LAYOUT_UNDERCONSTRAINED)
        self.assertEqual(response.layout_candidate_count, 8)

    def test_future_request_cutoff_times_out_stale(self):
        response = self._request(
            DetectTubeCapRequest.TARGET_ONLY,
            rospy.Time.now() + rospy.Duration(30.0),
        )
        self.assertEqual(response.status, response.STALE_FRAME)

    def test_uninitialized_lift_feedback_fails_closed(self):
        first_stamp = self._publish_batch((1, 1))
        self._publish_lift_state(initialized=False)
        try:
            rospy.sleep(0.05)
            response = self._request(
                DetectTubeCapRequest.TARGET_ONLY, first_stamp
            )
            self.assertEqual(response.status, response.STALE_FRAME)
        finally:
            self._publish_lift_state()

    def test_out_of_tolerance_lift_feedback_fails_closed(self):
        first_stamp = self._publish_batch((1, 1))
        self._publish_lift_state(back_height_mm=90)
        try:
            rospy.sleep(0.05)
            response = self._request(
                DetectTubeCapRequest.TARGET_ONLY, first_stamp
            )
            self.assertEqual(response.status, response.STALE_FRAME)
        finally:
            self._publish_lift_state()


if __name__ == "__main__":
    rospy.init_node("test_detector_layout_service")
    import rostest

    rostest.rosrun(
        "agx_tube_perception",
        "detector_layout_service",
        DetectorLayoutServiceTest,
    )
