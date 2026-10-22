#!/usr/bin/env python3
"""Fail-closed readiness check for the physical rover D435 camera."""

import math
import threading
import time

import message_filters
import rospy
import tf2_ros
from sensor_msgs.msg import CameraInfo, Image

from agx_workbench_navigation.camera_readiness import (
    CameraReadinessSample,
    MAXIMUM_SAMPLE_WINDOW,
    SynchronizedCameraSampleWindow,
    camera_readiness_observation,
    camera_readiness_errors,
)


def _message_stamp_s(message):
    if message is None:
        return None
    return message.header.stamp.to_sec()


def _message_frame_id(message):
    if message is None:
        return None
    return message.header.frame_id


def _message_value(message, attribute):
    if message is None:
        return None
    return getattr(message, attribute)


def _camera_tf_available(tf_buffer):
    try:
        return tf_buffer.can_transform(
            "base_link",
            "cam2_color_optical_frame",
            rospy.Time(0),
            rospy.Duration(0),
        )
    except tf2_ros.TransformException:
        return False


def _readiness_sample(image, camera_info, arrival_monotonic_s):
    return CameraReadinessSample(
        image_width=_message_value(image, "width"),
        image_height=_message_value(image, "height"),
        image_frame_id=_message_frame_id(image),
        image_stamp_s=_message_stamp_s(image),
        camera_info_width=_message_value(camera_info, "width"),
        camera_info_height=_message_value(camera_info, "height"),
        camera_info_frame_id=_message_frame_id(camera_info),
        camera_info_stamp_s=_message_stamp_s(camera_info),
        camera_matrix_k=_message_value(camera_info, "K"),
        arrival_monotonic_s=arrival_monotonic_s,
    )


def main():
    rospy.init_node("check_cam2_ready")
    timeout_sec = rospy.get_param("~timeout_sec", 5.0)
    try:
        timeout_sec = float(timeout_sec)
    except (TypeError, ValueError):
        rospy.logwarn("Invalid ~timeout_sec; using 5.0 seconds")
        timeout_sec = 5.0
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        rospy.logwarn("Invalid ~timeout_sec; using 5.0 seconds")
        timeout_sec = 5.0

    sample_window = SynchronizedCameraSampleWindow(MAXIMUM_SAMPLE_WINDOW)
    sample_lock = threading.Lock()

    def synchronized_callback(image, camera_info):
        sample = _readiness_sample(image, camera_info, time.monotonic())
        with sample_lock:
            sample_window.append(sample)

    def sample_snapshot():
        with sample_lock:
            return sample_window.snapshot()

    tf_buffer = tf2_ros.Buffer()
    _tf_listener = tf2_ros.TransformListener(tf_buffer)
    _image_subscriber = message_filters.Subscriber(
        "/cam2/color/image_raw", Image, queue_size=MAXIMUM_SAMPLE_WINDOW
    )
    _camera_info_subscriber = message_filters.Subscriber(
        "/cam2/color/camera_info", CameraInfo, queue_size=MAXIMUM_SAMPLE_WINDOW
    )
    synchronizer = message_filters.TimeSynchronizer(
        [_image_subscriber, _camera_info_subscriber], MAXIMUM_SAMPLE_WINDOW
    )
    synchronizer.registerCallback(synchronized_callback)

    ready_observation = None
    deadline = time.monotonic() + timeout_sec
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        observation = camera_readiness_observation(
            sample_snapshot(),
            _camera_tf_available(tf_buffer),
        )
        collection_errors = camera_readiness_errors(
            observation, ros_now_s=rospy.Time.now().to_sec()
        )
        if not collection_errors:
            ready_observation = observation
            break
        time.sleep(0.05)

    if ready_observation is None:
        observation = camera_readiness_observation(
            sample_snapshot(),
            _camera_tf_available(tf_buffer),
        )
        errors = camera_readiness_errors(
            observation, ros_now_s=rospy.Time.now().to_sec()
        )
    else:
        observation = ready_observation
        errors = ()
    if errors:
        for error in errors:
            rospy.logerr(error)
        return 1
    rospy.loginfo("cam2 readiness contract satisfied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
