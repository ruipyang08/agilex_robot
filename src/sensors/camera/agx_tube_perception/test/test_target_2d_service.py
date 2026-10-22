#!/usr/bin/env python3
"""Prove the 2D target service succeeds without lift feedback or TF."""

import time
import unittest

from agx_pick_msg.srv import (
    SelectTubeTarget2D,
    SelectTubeTarget2DRequest,
)
from cv_bridge import CvBridge
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image


class Target2DServiceTest(unittest.TestCase):
    WIDTH = 640
    HEIGHT = 480
    FRAME = "cam1_color_optical_frame"
    PROFILE = "table_grid_3x3_v1"
    TARGET = (365, 215)
    REFERENCE = (320.0, 240.0)

    @classmethod
    def setUpClass(cls):
        cls.bridge = CvBridge()
        cls.color_publisher = rospy.Publisher(
            "/test_target_2d/color", Image, queue_size=5
        )
        cls.info_publisher = rospy.Publisher(
            "/test_target_2d/camera_info", CameraInfo, queue_size=5
        )
        rospy.wait_for_service("/select_tube_target_2d", timeout=20.0)
        cls.select_target = rospy.ServiceProxy(
            "/select_tube_target_2d", SelectTubeTarget2D
        )
        deadline = time.monotonic() + 10.0
        while (
            cls.color_publisher.get_num_connections() == 0
            or cls.info_publisher.get_num_connections() == 0
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError("detector image subscriptions were not ready")
            rospy.sleep(0.05)

    @classmethod
    def _messages(cls):
        rgb = np.full((cls.HEIGHT, cls.WIDTH, 3), 24, dtype=np.uint8)
        for center, color in (
            ((260, 215), (165, 55, 220)),
            (cls.TARGET, (25, 204, 51)),
            ((470, 215), (35, 95, 230)),
        ):
            cv2.circle(rgb, center, 14, color, thickness=-1)
        color = cls.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        info = CameraInfo()
        info.width = cls.WIDTH
        info.height = cls.HEIGHT
        info.K = (
            400.0,
            0.0,
            cls.REFERENCE[0],
            0.0,
            400.0,
            cls.REFERENCE[1],
            0.0,
            0.0,
            1.0,
        )
        return color, info

    def test_selects_latest_stable_green_cap_without_geometry_inputs(self):
        color, info = self._messages()
        stamps = []
        for _ in range(3):
            stamp = rospy.Time.now()
            stamps.append(stamp)
            color.header.stamp = stamp
            info.header.stamp = stamp
            color.header.frame_id = self.FRAME
            info.header.frame_id = self.FRAME
            self.color_publisher.publish(color)
            self.info_publisher.publish(info)
            rospy.sleep(0.06)

        request = SelectTubeTarget2DRequest()
        request.fixture_profile_id = self.PROFILE
        request.not_before = stamps[0]
        response = self.select_target(request)

        self.assertEqual(response.status, response.SUCCESS, response.detail)
        self.assertEqual(response.target_candidate_count, 1)
        self.assertAlmostEqual(response.target.u_px, self.TARGET[0], places=1)
        self.assertAlmostEqual(response.target.v_px, self.TARGET[1], places=1)
        self.assertEqual(
            response.target.reference_u_px, self.REFERENCE[0]
        )
        self.assertEqual(
            response.target.reference_v_px, self.REFERENCE[1]
        )
        self.assertEqual(response.target.image_width, self.WIDTH)
        self.assertEqual(response.target.image_height, self.HEIGHT)
        self.assertEqual(response.target.header.frame_id, self.FRAME)
        self.assertEqual(response.target.header.stamp, stamps[-1])


if __name__ == "__main__":
    rospy.init_node("test_target_2d_service")
    import rostest

    rostest.rosrun(
        "agx_tube_perception",
        "target_2d_service",
        Target2DServiceTest,
    )
