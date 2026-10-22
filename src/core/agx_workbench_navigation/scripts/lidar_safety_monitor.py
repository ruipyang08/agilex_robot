#!/usr/bin/env python3
"""Publish fail-closed directional safety from the raw rover LiDAR cloud."""

import math
from pathlib import Path
import threading
import time
from hashlib import sha256

from nav_msgs.msg import Odometry
import rospkg
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
import tf2_ros
import yaml

from agx_workbench_navigation.contracts import SafetyHealth, Velocity2D
from agx_workbench_navigation.effective_config import lidar_configuration, start_ros_report
from agx_workbench_navigation.lidar_safety import (
    ClockAlignmentWait,
    DuplicateStampFilter,
    LidarEvidenceTracker,
    LidarSafetyEvaluator,
    Point3D,
    SafetyConfig,
    decode_xyz_points,
    evidence_decision,
    resolve_lidar_config_path,
)
from agx_workbench_navigation.msg import LidarSafetyState, LidarObstacleSnapshot
from agx_workbench_navigation.lidar_snapshot import CapturePose, ObstacleCapture, snapshot_health


def _load_config(path):
    return _load_config_source(path)[0]


def _load_config_source(path):
    try:
        with Path(path).open('rb') as stream:
            source = stream.read(1024*1024+1)
        if not 0 < len(source) <= 1024*1024:
            raise ValueError('LiDAR configuration exceeds size bound')
        raw = yaml.safe_load(source.decode('utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError("cannot read LiDAR safety configuration: {0}".format(exc))
    return SafetyConfig.from_mapping(raw), 'sha256:'+sha256(source).hexdigest()


def _finite_transform_values(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    values = (
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )
    try:
        converted = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("TF contains nonnumeric values")
    if not all(math.isfinite(value) for value in converted):
        raise ValueError("TF contains nonfinite values")
    norm = math.sqrt(sum(value * value for value in converted[3:]))
    if norm <= 1e-12:
        raise ValueError("TF quaternion has zero norm")
    return converted[:3], tuple(value / norm for value in converted[3:])


def _transform_points(points, transform):
    translation, quaternion = _finite_transform_values(transform)
    qx, qy, qz, qw = quaternion
    rotation = (
        (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ),
        (
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ),
        (
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ),
    )
    transformed = []
    for point in points:
        if not (
            math.isfinite(point.x)
            and math.isfinite(point.y)
            and math.isfinite(point.z)
        ):
            transformed.append(point)
            continue
        transformed.append(
            Point3D(
                rotation[0][0] * point.x
                + rotation[0][1] * point.y
                + rotation[0][2] * point.z
                + translation[0],
                rotation[1][0] * point.x
                + rotation[1][1] * point.y
                + rotation[1][2] * point.z
                + translation[1],
                rotation[2][0] * point.x
                + rotation[2][1] * point.y
                + rotation[2][2] * point.z
                + translation[2],
            )
        )
    return tuple(transformed)


class LidarSafetyMonitor:
    """Thin ROS boundary around the deterministic safety evaluator."""

    def __init__(self, config, filter_digest=None):
        self._config = config
        self._lock = threading.Lock()
        self._publish_lock = threading.Lock()
        self._publish_stamp_filter = DuplicateStampFilter()
        self._evidence = LidarEvidenceTracker()
        self._clock_alignment = ClockAlignmentWait(config.max_age_s)
        self._capture = self._capture_points = None
        self._last_cloud_header = Header()
        self._filter_digest = filter_digest or ''
        package = Path(rospkg.RosPack().get_path('agx_workbench_navigation'))
        policy_path = Path(rospy.get_param('~motion_policy_file', str(package/'config/navigation_motion_policy.yaml')))
        policy_source=policy_path.read_bytes()
        self._policy_digest = 'sha256:'+sha256(policy_source).hexdigest()
        from agx_workbench_navigation.motion_policy import parse_motion_policy
        policy=parse_motion_policy(yaml.safe_load(policy_source.decode('utf-8')))
        self._evaluator=LidarSafetyEvaluator(config,
            self_footprint_xy_m=policy.footprint_xy_m if policy.footprint_sources else None)
        self._snapshot_publisher = rospy.Publisher('/workbench_nav/lidar_obstacle_snapshot',
            LidarObstacleSnapshot, queue_size=1, latch=True)

        self._publisher = rospy.Publisher(
            "/workbench_nav/lidar_safety_state",
            LidarSafetyState,
            queue_size=1,
            latch=True,
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._odometry_subscriber = rospy.Subscriber(
            "/odom",
            Odometry,
            self._odometry_callback,
            queue_size=1,
        )
        self._cloud_subscriber = rospy.Subscriber(
            "/rslidar_points",
            PointCloud2,
            self._cloud_callback,
            queue_size=1,
        )
        self._timer = rospy.Timer(rospy.Duration(0.05), self._timer_callback)
        self._effective_report = start_ros_report("lidar", lidar_configuration(config,self._evaluator.self_exclusion_xy_m))

    def _odometry_callback(self, odometry):
        try:
            velocity = Velocity2D(
                linear_x=odometry.twist.twist.linear.x,
                linear_y=odometry.twist.twist.linear.y,
                angular_z=odometry.twist.twist.angular.z,
            )
            stamp_s = odometry.header.stamp.to_sec()
            if not math.isfinite(stamp_s) or stamp_s < 0.0:
                raise ValueError("odometry stamp is invalid")
        except (AttributeError, TypeError, ValueError, OverflowError):
            with self._lock:
                self._evidence.record_odometry_failure("odometry_invalid")
            self._publish_current()
            return
        with self._lock:
            self._evidence.record_valid_odometry(velocity, stamp_s=stamp_s)

    def _cloud_callback(self, cloud):
        receipt = time.monotonic()
        cloud_header = Header(seq=cloud.header.seq, stamp=cloud.header.stamp, frame_id=cloud.header.frame_id)
        if cloud.header.frame_id != self._config.expected_frame:
            self._set_cloud_invalid("cloud_frame_invalid", cloud_header)
            return
        decoded = decode_xyz_points(
            data=cloud.data,
            width=cloud.width,
            height=cloud.height,
            fields=cloud.fields,
            point_step=cloud.point_step,
            row_step=cloud.row_step,
            is_bigendian=cloud.is_bigendian,
        )
        if decoded.state == SafetyHealth.INVALID:
            self._set_cloud_invalid("cloud_schema:{0}".format(decoded.reason), cloud_header)
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._config.target_frame,
                self._config.expected_frame,
                cloud.header.stamp,
                rospy.Duration(0.05),
            )
            points = _transform_points(decoded.points, transform)
            cloud_stamp_s = cloud.header.stamp.to_sec()
            transform_stamp_s = transform.header.stamp.to_sec()
            original_transform_stamp_s = transform_stamp_s
            if transform_stamp_s == 0.0:
                # A zero-stamped static transform is valid for every cloud stamp.
                transform_stamp_s = cloud_stamp_s
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            AttributeError,
            TypeError,
            ValueError,
            OverflowError,
        ):
            self._set_cloud_invalid("tf_unavailable", cloud_header)
            return
        capture = capture_points = None
        try:
            # Relative stopping checks use odom, so a map correction cannot
            # relocate the captured cloud. The global owner transforms this
            # original pose to map separately when assembling its snapshot.
            base = self._tf_buffer.lookup_transform('odom', self._config.target_frame,
                                                     cloud.header.stamp, rospy.Duration(0.05))
            p, q = base.transform.translation, base.transform.rotation
            pose = CapturePose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            capture = ObstacleCapture(cloud.header.seq, cloud_stamp_s, original_transform_stamp_s,
                original_transform_stamp_s == 0., receipt, cloud.header.frame_id, pose,
                self._evaluator.filtered_obstacles(points), self._policy_digest, self._filter_digest)
            header = Header(seq=cloud.header.seq, stamp=cloud.header.stamp, frame_id=self._config.target_frame)
            capture_points = create_cloud_xyz32(header, [(p.x, p.y, p.z) for p in capture.points])
        except Exception:
            # Missing capture-pose evidence inhibits the new geometric path.
            # Legacy directional docking does not depend on map/odom TF here.
            pass
        with self._lock:
            accepted = self._evidence.record_valid_cloud(
                points=points,
                cloud_stamp_s=cloud_stamp_s,
                tf_stamp_s=transform_stamp_s,
            )
            if accepted:
                self._capture, self._capture_points = capture, capture_points
            else:
                self._capture = self._capture_points = None
            self._last_cloud_header = cloud_header
        self._publish_current()

    def _set_cloud_invalid(self, reason, cloud_header=None):
        with self._lock:
            self._evidence.record_cloud_failure(reason)
            self._capture = self._capture_points = None
            if cloud_header is not None:
                self._last_cloud_header = cloud_header
        self._publish_current()

    def _publish_current(self):
        with self._publish_lock:
            with self._lock:
                evidence = self._evidence.snapshot()
                capture, capture_points, cloud_header = self._capture, self._capture_points, self._last_cloud_header
                # An odometry callback must not replace this snapshot after
                # its decision clock was sampled.
                now = rospy.Time.now()
            decision = evidence_decision(self._evaluator, evidence, now_s=now.to_sec())
            # ROS transports /clock and sensor state independently. Wait for
            # matching time without publishing future evidence or freshening
            # the last safety proof. Other invalid states fail immediately.
            if self._clock_alignment.defer(
                decision, evidence, now_s=now.to_sec(), receipt_s=time.monotonic()
            ):
                return
            if not self._publish_stamp_filter.accept(now.to_sec()):
                return
            message = LidarSafetyState()
            message.header.stamp = now
            message.header.frame_id = self._config.target_frame
            message.state = int(decision.state)
            message.blocked_directions = int(decision.blocked_directions)
            message.forward_clearance_m = decision.forward_clearance_m
            message.reverse_clearance_m = decision.reverse_clearance_m
            message.left_clearance_m = decision.left_clearance_m
            message.right_clearance_m = decision.right_clearance_m
            message.rotation_clearance_m = decision.rotation_clearance_m
            message.required_stop_distance_m = decision.required_stop_distance_m
            message.reason = decision.reason
            self._publisher.publish(message)
            snapshot = LidarObstacleSnapshot()
            snapshot.header.seq, snapshot.header.stamp = cloud_header.seq, cloud_header.stamp
            snapshot.cloud_sequence = cloud_header.seq
            snapshot.header.frame_id = self._config.target_frame
            snapshot.source_frame = cloud_header.frame_id
            snapshot.policy_digest, snapshot.filter_digest = self._policy_digest, self._filter_digest
            snapshot.state = int(snapshot_health(capture, decision.state, now.to_sec(), time.monotonic(),
                                                 self._config.max_age_s, 1.0))
            snapshot.reason = decision.reason if capture is not None else 'capture_pose_or_policy_unavailable'
            if capture is not None and capture.cloud_stamp_s == evidence.cloud_stamp_s:
                snapshot.transform_stamp = rospy.Time.from_sec(capture.transform_stamp_s)
                snapshot.transform_static = capture.transform_static
                snapshot.receipt_steady_s = capture.receipt_steady_s
                snapshot.capture_pose.header.frame_id = 'odom'
                snapshot.capture_pose.header.stamp = snapshot.header.stamp
                p, q = snapshot.capture_pose.pose.position, snapshot.capture_pose.pose.orientation
                p.x, p.y, p.z = capture.capture_pose.position_xyz_m
                q.x, q.y, q.z, q.w = capture.capture_pose.orientation_xyzw
                snapshot.obstacle_points = capture_points
            else:
                snapshot.state = snapshot.INVALID
            if self._cloud_subscriber.get_num_connections() != 1:
                snapshot.state, snapshot.reason = snapshot.INVALID, 'cloud_source_owner_count'
            self._snapshot_publisher.publish(snapshot)

    def _timer_callback(self, _event):
        self._publish_current()


def main():
    rospy.init_node("lidar_safety_monitor")
    try:
        configured_path = rospy.get_param("~config_file", None)
        config_path = (
            Path(configured_path)
            if configured_path is not None
            else resolve_lidar_config_path(rospkg.RosPack().get_path)
        )
        config, filter_digest = _load_config_source(config_path)
    except ValueError as exc:
        rospy.logfatal(str(exc))
        return 1
    LidarSafetyMonitor(config, filter_digest)
    rospy.loginfo(
        "LiDAR safety monitor consuming %s in frame %s",
        config.topic,
        config.expected_frame,
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
