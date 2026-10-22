#!/usr/bin/env python3
"""Sensor provenance in isolated ROS; fake inputs cannot qualify robot motion."""
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32, read_points
from std_msgs.msg import Header
from agx_workbench_navigation.msg import LidarObstacleSnapshot, LidarSafetyState


class LidarSnapshotRos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.clouds, cls.snapshots, cls.health = [], [], []
        cls.base_x, cls.frame, cls.publish_cloud = 1., 'rslidar', True
        cls.cloud_pub = rospy.Publisher('/rslidar_points', PointCloud2, queue_size=1)
        cls.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        cls.subscribers = [rospy.Subscriber('/rslidar_points', PointCloud2, cls.raw),
            rospy.Subscriber('/workbench_nav/lidar_obstacle_snapshot', LidarObstacleSnapshot, cls.snapshot),
            rospy.Subscriber('/workbench_nav/lidar_safety_state', LidarSafetyState, cls.safety)]
        cls.static = tf2_ros.StaticTransformBroadcaster()
        transform = TransformStamped(); transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'base_link'; transform.child_frame_id = 'rslidar'
        transform.transform.rotation.w = 1.; cls.static.sendTransform(transform)
        cls.dynamic = tf2_ros.TransformBroadcaster()
        cls.timer = rospy.Timer(rospy.Duration(.05), cls.tick)

    @classmethod
    def tearDownClass(cls):
        cls.timer.shutdown()

    @classmethod
    def raw(cls, message):
        with cls.lock: cls.clouds.append(message)

    @classmethod
    def snapshot(cls, message):
        with cls.lock: cls.snapshots.append(message)

    @classmethod
    def safety(cls, message):
        with cls.lock: cls.health.append(message)

    @classmethod
    def tick(cls, _event):
        stamp = rospy.Time.now()
        with cls.lock: x, frame, publish = cls.base_x, cls.frame, cls.publish_cloud
        transform = TransformStamped(); transform.header.stamp = stamp
        transform.header.frame_id = 'odom'; transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x; transform.transform.rotation.w = 1.
        cls.dynamic.sendTransform(transform)
        odom = Odometry(); odom.header.stamp = stamp
        odom.header.frame_id = 'odom'; odom.child_frame_id = 'rslidar'
        odom.pose.pose.position.x = x; odom.pose.pose.orientation.w = 1.
        cls.odom_pub.publish(odom)
        if publish:
            # Finite self returns satisfy raw sensor health. One isolated real
            # obstacle survives the exact height/self filter in the snapshot.
            points = [(0., 0., .1)]*100+[(.8, 0., .1)]
            cls.cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), points))

    def wait_for(self, predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with self.lock:
                result = predicate()
            if result: return result
            time.sleep(.01)
        self.fail('sensor evidence did not reach the expected state')

    def test_original_cloud_identity_pose_and_expiration(self):
        received = self.wait_for(lambda: next((s for s in reversed(self.snapshots)
            if s.state in (s.CLEAR, s.BLOCKED) and s.capture_pose.header.frame_id == 'odom'), None))
        with self.lock:
            type(self).publish_cloud = False
        # Let any in-flight callback finish before changing the newest TF pose.
        time.sleep(.1)
        received = self.wait_for(lambda: self.snapshots[-1] if self.snapshots and
            self.snapshots[-1].state in (0, 1) else None)
        with self.lock:
            type(self).base_x = 2.
        newer = self.wait_for(lambda: next((s for s in reversed(self.snapshots)
            if s.header.seq > received.header.seq), None))
        self.assertEqual(newer.header.stamp, received.header.stamp)
        self.assertEqual(newer.cloud_sequence, received.cloud_sequence)
        self.assertEqual(newer.receipt_steady_s, received.receipt_steady_s)
        self.assertAlmostEqual(newer.capture_pose.pose.position.x, 1.)
        # tf2 may stamp an explicitly timed static lookup with the query time.
        # Preserve the returned stamp across republication in either case.
        self.assertEqual(newer.transform_stamp, received.transform_stamp)
        self.assertIn(newer.transform_stamp.to_sec(), (0., newer.header.stamp.to_sec()))
        points = tuple(read_points(newer.obstacle_points, field_names=('x', 'y', 'z')))
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], .8, places=6)
        with self.lock:
            matching = [c for c in self.clouds if c.header.stamp == received.header.stamp]
        self.assertTrue(matching)
        self.assertEqual(matching[-1].header.seq, received.cloud_sequence)
        stale = self.wait_for(lambda: next((s for s in reversed(self.snapshots)
            if s.state == s.STALE and s.header.stamp == received.header.stamp), None))
        self.assertEqual(stale.receipt_steady_s, received.receipt_steady_s)
        # Malformed new input invalidates the snapshot rather than retaining a
        # previous clear cloud under a newly published timestamp.
        with self.lock:
            type(self).frame, type(self).publish_cloud = 'wrong-frame', True
        invalid = self.wait_for(lambda: next((s for s in reversed(self.snapshots)
            if s.state == s.INVALID and s.source_frame == 'wrong-frame'), None))
        self.assertEqual(invalid.obstacle_points.width, 0)


if __name__ == '__main__':
    rospy.init_node('lidar_snapshot_ros')
    rostest.rosrun('agx_workbench_navigation', 'lidar_snapshot', LidarSnapshotRos)
