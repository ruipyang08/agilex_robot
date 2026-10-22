#!/usr/bin/env python3
"""ROS1 wrapper for the fail-closed autonomous base-command arbiter."""

import os
import math
import threading
import time
from collections import deque
from pathlib import Path
from hashlib import sha256

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospkg
import rospy
import tf2_ros
import yaml
from std_srvs.srv import Trigger, TriggerResponse

from agx_workbench_navigation.authority import (
    AuthorityArbiter, AuthorityConfig, TravelReadiness as ReadinessEvidence,
)
from agx_workbench_navigation.contracts import BaseAuthority, Velocity2D, SafetyHealth
from agx_workbench_navigation.command_sweep_safety import CommandSweepGuard
from agx_workbench_navigation.lidar_snapshot import CapturePose, ObstacleCapture
from agx_workbench_navigation.lidar_safety import decode_xyz_points
from agx_workbench_navigation.motion_policy import parse_motion_policy
from agx_workbench_navigation.pose_interval import PoseIntervalEstimator
from agx_workbench_navigation.effective_config import base_configuration, start_ros_report
from agx_workbench_navigation.msg import (
    AuthorizedBaseCommand, BaseAuthorityState, LidarSafetyState, TravelReadiness, LidarObstacleSnapshot,
)
from agx_workbench_navigation.srv import (
    SetBaseAuthority,
    SetBaseAuthorityResponse,
    AcquireBaseHold, AcquireBaseHoldResponse,
    PrepareBaseDeparture, PrepareBaseDepartureResponse,
    ReleaseBaseHold, ReleaseBaseHoldResponse,
    ResetBaseFault, ResetBaseFaultResponse,
)


def _load_config():
    configured_path = rospy.get_param("~config_file", None)
    if configured_path is None:
        package_path = rospkg.RosPack().get_path("agx_workbench_navigation")
        config_path = os.path.join(package_path, "config/authority.yaml")
    else:
        config_path = os.path.abspath(os.path.expanduser(configured_path))
    with open(config_path, "r") as stream:
        return AuthorityConfig.from_mapping(yaml.safe_load(stream))


def _twist(command):
    message = Twist()
    message.linear.x = command.linear_x
    message.linear.y = command.linear_y
    message.angular.z = command.angular_z
    return message


class BaseCommandArbiterNode:
    def __init__(self):
        self._lock = threading.RLock()
        config = _load_config()
        self._pose_estimator = PoseIntervalEstimator(config.pose_feedback)
        self._sweep_guard=None
        policy_path=rospy.get_param('~motion_policy_file','')
        if policy_path:
            source=Path(policy_path).read_bytes()
            if not 0 < len(source) <= 1024*1024:
                raise ValueError('installed motion policy exceeds bound')
            policy=parse_motion_policy(yaml.safe_load(source.decode('utf-8')))
            if policy.execution_enabled:
                policy.require_execution(rospy.get_param('~operation_mode','hardware'))
                filter_path=Path(rospy.get_param('~lidar_config_file'))
                filter_source=filter_path.read_bytes()
                if not 0 < len(filter_source) <= 1024*1024:
                    raise ValueError('installed LiDAR policy exceeds bound')
                self._sweep_guard=CommandSweepGuard(policy,'sha256:'+sha256(source).hexdigest(),
                    'sha256:'+sha256(filter_source).hexdigest(),
                    receipt_max_age_s=config.lidar_receipt_s,source_max_age_s=config.odometry_header_s,
                    max_evaluation_s=min(.04,.8/config.output_rate_hz))
        self._arbiter = AuthorityArbiter(config, fenced_protocol=True,
            navigation_sweep_guard=self._check_navigation_sweep if self._sweep_guard else None)
        self._pending_odometry = deque()
        self._draining_odometry = False
        self._odometry_source_highwater = None
        self._odometry_receipt_highwater = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._stop_event = threading.Event()
        self._output_period_s = 1.0 / config.output_rate_hz
        self._command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._state_pub = rospy.Publisher(
            "/workbench_nav/authority_state",
            BaseAuthorityState,
            queue_size=1,
            latch=True,
        )
        self._navigation_sub = rospy.Subscriber(
            "/workbench_nav/cmd_vel/navigation",
            AuthorizedBaseCommand,
            self._on_navigation,
            queue_size=1,
        )
        self._docking_sub = rospy.Subscriber(
            "/workbench_nav/cmd_vel/docking",
            AuthorizedBaseCommand,
            self._on_docking,
            queue_size=1,
        )
        self._lidar_sub = rospy.Subscriber(
            "/workbench_nav/lidar_safety_state",
            LidarSafetyState,
            self._on_lidar,
            queue_size=1,
        )
        if self._sweep_guard:
            self._obstacle_sub=rospy.Subscriber('/workbench_nav/lidar_obstacle_snapshot',
                LidarObstacleSnapshot,self._on_obstacle_capture,queue_size=1)
        self._odom_sub = rospy.Subscriber(
            "/odom",
            Odometry,
            self._on_odometry,
            queue_size=1,
        )
        self._service = rospy.Service(
            "/workbench_nav/set_authority",
            SetBaseAuthority,
            self._on_authority_request,
        )
        self._hold_service = rospy.Service(
            "/workbench_nav/hold_for_manipulation", Trigger,
            self._on_manipulation_hold)
        self._acquire_service = rospy.Service(
            "/workbench_nav/acquire_base_hold", AcquireBaseHold, self._on_acquire_hold)
        self._prepare_service = rospy.Service(
            "/workbench_nav/prepare_base_departure", PrepareBaseDeparture, self._on_prepare_departure)
        self._release_service = rospy.Service(
            "/workbench_nav/release_base_hold", ReleaseBaseHold, self._on_release_hold)
        self._reset_service = rospy.Service(
            "/workbench_nav/reset_base_fault", ResetBaseFault, self._on_reset_fault)
        self._readiness_sub = rospy.Subscriber(
            "/workbench_nav/travel_readiness", TravelReadiness,
            self._on_travel_readiness, queue_size=1)
        rospy.on_shutdown(self._on_shutdown)
        self._publish_state(
            BaseAuthorityState.NONE,
            "",
            "none",
            rospy.Time.now(),
        )
        self._output_thread = threading.Thread(
            target=self._run_output_loop,
            name="base-command-arbiter-output",
            daemon=True,
        )
        self._output_thread.start()
        self._effective_report = start_ros_report("base_arbiter", base_configuration(config))

    def _on_obstacle_capture(self,message):
        # Decode outside the output lock. Repeated timer messages retain the
        # source callback's stamps and receipt, never this callback's receipt.
        capture=None
        try:
            health=SafetyHealth(message.state)
            cloud=message.obstacle_points
            if (health not in (SafetyHealth.CLEAR,SafetyHealth.BLOCKED) or
                    message.header.frame_id!='base_link' or cloud.header.frame_id!='base_link' or
                    message.capture_pose.header.frame_id!='odom' or
                    message.capture_pose.header.stamp!=message.header.stamp or
                    cloud.header.stamp!=message.header.stamp):
                raise ValueError('invalid capture frames/stamps/health')
            decoded=decode_xyz_points(data=cloud.data,width=cloud.width,height=cloud.height,
                fields=cloud.fields,point_step=cloud.point_step,row_step=cloud.row_step,
                is_bigendian=cloud.is_bigendian,compact=True)
            if decoded.state!=SafetyHealth.CLEAR:
                raise ValueError(decoded.reason)
            pose=message.capture_pose.pose
            capture=ObstacleCapture(message.cloud_sequence,message.header.stamp.to_sec(),
                message.transform_stamp.to_sec(),message.transform_static,message.receipt_steady_s,
                message.source_frame,CapturePose(tuple(getattr(pose.position,a) for a in 'xyz'),
                    tuple(getattr(pose.orientation,a) for a in 'xyzw')),decoded.points,
                message.policy_digest,message.filter_digest)
        except Exception:
            health=SafetyHealth.INVALID
        with self._lock:
            self._sweep_guard.update(capture,health)

    def _check_navigation_sweep(self,command,motion,source_now,receipt_now):
        pose=self._pose_estimator.current_base_pose
        if pose is None or self._obstacle_sub.get_num_connections()!=1:
            return 'current_pose_or_obstacle_owner_unavailable'
        source,receipt,xyz,q=pose
        return self._sweep_guard.check(command,motion,CapturePose(xyz,q),source,receipt,source_now,receipt_now)

    def _on_navigation(self, message):
        self._update_command(BaseAuthority.NAVIGATION, message)

    def _on_docking(self, message):
        self._update_command(BaseAuthority.DOCKING, message)

    def _update_command(self, source, message):
        try:
            twist = message.twist
            if any(value != 0.0 for value in (twist.linear.z, twist.angular.x, twist.angular.y)):
                raise ValueError("unsupported Twist component")
            command = Velocity2D(
                twist.linear.x,
                twist.linear.y,
                twist.angular.z,
            )
        except ValueError:
            command = object()
        with self._lock:
            self._arbiter.update_authorized_command(
                source,
                command,
                session_id=message.session_id,
                grant_id=message.grant_id,
                source_goal_id=message.source_goal_id,
                sequence=message.sequence,
                header_stamp_s=message.header.stamp.to_sec(),
                receipt_now_s=time.monotonic(),
                header_now_s=rospy.Time.now().to_sec(),
            )

    def _on_lidar(self, message):
        with self._lock:
            self._arbiter.update_lidar(
                health=message.state,
                blocked_directions=message.blocked_directions,
                receipt_now_s=time.monotonic(),
                header_stamp_s=message.header.stamp.to_sec(),
                header_now_s=rospy.Time.now().to_sec(),
                forward_clearance_m=message.forward_clearance_m,
                reverse_clearance_m=message.reverse_clearance_m,
                left_clearance_m=message.left_clearance_m,
                right_clearance_m=message.right_clearance_m,
                rotation_clearance_m=message.rotation_clearance_m,
                required_stop_distance_m=message.required_stop_distance_m,
                reason=message.reason,
            )

    def _on_odometry(self, message):
        # Acquisition receipt precedes TF work. Only check-now clocks may move
        # forward afterward; a delayed lookup must not freshen measured progress.
        sample_receipt = time.monotonic()
        stamp = message.header.stamp.to_sec()
        with self._lock:
            if not math.isfinite(stamp) or stamp <= 0:
                self._reject_odometry(stamp, sample_receipt, 'source_stamp_invalid')
                return  # ROS stamp zero means latest TF, never measured-time TF.
            if self._odometry_source_highwater is not None and stamp <= self._odometry_source_highwater:
                self._reject_odometry(stamp, sample_receipt, 'source_stamp_not_increasing')
                return
            self._odometry_source_highwater = stamp
            if (self._odometry_receipt_highwater is not None and
                    sample_receipt <= self._odometry_receipt_highwater):
                self._reject_odometry(stamp, sample_receipt, 'receipt_not_increasing')
                return
            self._odometry_receipt_highwater = sample_receipt
            policy = self._arbiter.config.pose_feedback
            if message.header.frame_id != policy.parent_frame or message.child_frame_id != policy.child_frame:
                self._reject_odometry(stamp, sample_receipt, 'frame_mismatch')
                return
            pose = message.pose.pose
            position = tuple(getattr(pose.position, axis) for axis in 'xyz')
            quaternion = tuple(getattr(pose.orientation, axis) for axis in 'xyzw')
            if (not all(math.isfinite(value) for value in position + quaternion) or
                    abs(math.hypot(*quaternion) - 1.) > policy.quaternion_norm_tolerance):
                self._reject_odometry(stamp, sample_receipt, 'invalid_pose_or_extrinsic')
                return
            # Bound memory without silently discarding a physical observation.
            # The existing source and receipt leases bound waiting, not a new
            # independently renewable TF timeout.
            if len(self._pending_odometry) >= 16:
                self._reject_odometry(stamp, sample_receipt, 'tf_queue_overflow')
                return
            self._pending_odometry.append((message.header.stamp, stamp, sample_receipt, position, quaternion))
            self._drain_odometry()

    def _drain_odometry(self):
        # Caller owns the arbiter lock. TF calls are strictly nonblocking; the
        # regular output loop continues to enforce every existing stop lease.
        if self._draining_odometry:
            return
        self._draining_odometry = True
        try:
            while self._pending_odometry:
                entry = self._pending_odometry[0]
                ros_stamp, stamp, receipt, position, quaternion = entry
                config = self._arbiter.config
                if self._odometry_wait_expired(stamp, receipt):
                    self._reject_odometry(stamp, receipt, 'tf_wait_expired')
                    return
                # /odom and /clock are independently transported. Keep an
                # already-validated future sample queued on its original
                # receipt until its source time arrives; never replace the
                # previous fresh pose with future evidence for a sweep.
                if stamp > rospy.Time.now().to_sec():
                    return
                policy = config.pose_feedback
                try:
                    transform = self._tf_buffer.lookup_transform(
                        policy.child_frame, policy.command_frame, ros_stamp, rospy.Duration(0))
                except tf2_ros.ExtrapolationException:
                    return  # Retry this original stamp on a later output tick.
                except Exception:
                    self._reject_odometry(stamp, receipt, 'tf_unavailable')
                    return
                if not self._pending_odometry or self._pending_odometry[0] is not entry:
                    continue  # A reentrant rejection already invalidated it.
                if self._odometry_wait_expired(stamp, receipt):
                    self._reject_odometry(stamp, receipt, 'tf_wait_expired')
                    return
                self._pending_odometry.popleft()
                result = self._pose_estimator.update(
                    position, quaternion,
                    tuple(getattr(transform.transform.translation, axis) for axis in 'xyz'),
                    tuple(getattr(transform.transform.rotation, axis) for axis in 'xyzw'),
                    source_stamp_s=stamp, receipt_s=receipt,
                    parent_frame=policy.parent_frame, child_frame=policy.child_frame,
                    command_frame=policy.command_frame)
                motion = Velocity2D(result.vx_mps, result.vy_mps, result.wz_radps) if result.valid else None
                self._arbiter.update_measured_motion(motion, invalid_reason=result.reason,
                    sample_receipt_s=receipt, header_stamp_s=stamp,
                    receipt_now_s=time.monotonic(), header_now_s=rospy.Time.now().to_sec())
                if not result.valid and result.reason != 'first_sample':
                    self._pending_odometry.clear()
                    self._publish_tick()
                    return
        finally:
            self._draining_odometry = False

    def _odometry_wait_expired(self, stamp, receipt):
        config = self._arbiter.config
        now, source_now = time.monotonic(), rospy.Time.now().to_sec()
        return (not math.isfinite(now) or not math.isfinite(source_now) or source_now <= 0 or
                not 0 <= now - receipt <= config.odometry_receipt_s or
                not -config.maximum_header_future_skew_s <= source_now - stamp <= config.odometry_header_s)

    def _reject_odometry(self, stamp, sample_receipt, reason):
        with self._lock:
            self._pending_odometry.clear()
            self._pose_estimator.invalidate(reason, source_stamp_s=stamp,
                                            receipt_s=sample_receipt)
            self._arbiter.update_measured_motion(None, invalid_reason=reason,
                sample_receipt_s=sample_receipt, header_stamp_s=stamp,
                receipt_now_s=time.monotonic(), header_now_s=rospy.Time.now().to_sec())
            # Do not let later good callbacks hide an invalid observation before
            # the next scheduled output/lease check. A fault remains latched.
            self._publish_tick()

    def _on_authority_request(self, request):
        ros_now = rospy.Time.now()
        with self._lock:
            decision = self._arbiter.request_grant(
                request.requested_state,
                request.expected_session_id,
                request.source_goal_id,
                receipt_now_s=time.monotonic(),
                header_now_s=ros_now.to_sec(),
            )
            state_reason = decision.reason
            if decision.state == BaseAuthority.FAULT:
                state_reason = self._arbiter.fault_reason or decision.reason
            self._publish_state(
                int(decision.state),
                self._selected_source(decision.state),
                state_reason,
                ros_now,
            )
        return SetBaseAuthorityResponse(
            accepted=decision.accepted,
            state=int(decision.state),
            reason=decision.reason,
            session_id=decision.session_id,
            grant_id=decision.grant_id,
            source_goal_id=decision.source_goal_id,
        )

    def _run_output_loop(self):
        next_tick_s = time.monotonic()
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                self._publish_tick()
            except Exception as error:
                self._handle_output_exception(error)
            next_tick_s += self._output_period_s
            now_s = time.monotonic()
            if next_tick_s < now_s - self._output_period_s:
                next_tick_s = now_s
            self._stop_event.wait(max(0.0, next_tick_s - now_s))

    def _on_manipulation_hold(self, _request):
        with self._lock:
            decision = self._arbiter.hold_for_manipulation(
                receipt_now_s=time.monotonic(),
                header_now_s=rospy.Time.now().to_sec())
            # Publish immediately; the regular output loop keeps enforcing it
            # after this service client or its entire mission process exits.
            self._publish_tick()
            return TriggerResponse(success=decision.accepted, message=decision.reason)

    def _on_acquire_hold(self, request):
        with self._lock:
            result = self._arbiter.acquire_hold(
                request.run_id, request.operation_id, request.reason,
                receipt_now_s=time.monotonic(), header_now_s=rospy.Time.now().to_sec())
            self._publish_tick()
            return AcquireBaseHoldResponse(
                accepted=result.accepted, stopped_confirmed=result.stopped_confirmed,
                session_id=result.session_id, hold_epoch=result.hold_epoch, reason=result.reason)

    def _on_travel_readiness(self, message):
        evidence = ReadinessEvidence(
            header_stamp_s=message.header.stamp.to_sec(), run_id=message.run_id,
            departure_id=message.departure_id, session_id=message.session_id,
            hold_epoch=message.hold_epoch, sequence=message.sequence,
            state_revision=message.state_revision, config_id=message.config_id,
            travel_profile_id=message.travel_profile_id, arm_ready=message.arm_ready,
            lift_ready=message.lift_ready, producers_stopped=message.producers_stopped,
            clearance_verified=message.clearance_verified, payload_state=message.payload_state,
            payload_ref=message.payload_ref, container_profile_id=message.container_profile_id,
            ready=message.ready, reason=message.reason)
        with self._lock:
            self._arbiter.update_travel_readiness(evidence, receipt_now_s=time.monotonic(),
                                                header_now_s=rospy.Time.now().to_sec())
            self._publish_tick()

    def _on_prepare_departure(self, request):
        with self._lock:
            result = self._arbiter.prepare_departure(
                request.run_id, request.departure_id, request.expected_session_id,
                request.expected_hold_epoch, request.readiness_sequence, request.state_revision,
                receipt_now_s=time.monotonic(), header_now_s=rospy.Time.now().to_sec())
            self._publish_tick()
            return PrepareBaseDepartureResponse(
                accepted=result.accepted, reason=result.reason, permit_id=result.permit_id,
                session_id=result.session_id, hold_epoch=result.hold_epoch, valid_for_s=result.valid_for_s)

    def _on_release_hold(self, request):
        with self._lock:
            result = self._arbiter.release_hold(
                request.run_id, request.departure_id, request.expected_session_id,
                request.expected_hold_epoch, request.permit_id,
                receipt_now_s=time.monotonic(), header_now_s=rospy.Time.now().to_sec())
            self._publish_tick()
            return ReleaseBaseHoldResponse(accepted=result.accepted, reason=result.reason,
                                          session_id=result.session_id, hold_epoch=result.hold_epoch)

    def _on_reset_fault(self, request):
        with self._lock:
            result = self._arbiter.reset_fault(
                request.expected_session_id, request.expected_hold_epoch, request.reason,
                receipt_now_s=time.monotonic(), header_now_s=rospy.Time.now().to_sec())
            self._publish_tick()
            return ResetBaseFaultResponse(accepted=result.accepted, state=int(result.state), reason=result.reason)

    def _publish_tick(self):
        with self._lock:
            self._drain_odometry()
            ros_now = rospy.Time.now()
            output = self._arbiter.output(
                receipt_now_s=time.monotonic(),
                header_now_s=ros_now.to_sec(),
            )
            self._command_pub.publish(_twist(output.command))
            self._publish_state(
                int(output.state),
                output.selected_source,
                output.reason,
                ros_now,
            )

    def _handle_output_exception(self, error):
        try:
            rospy.logerr_throttle(
                1.0,
                "base command arbiter output failure: %s",
                error,
            )
        except Exception:
            pass
        try:
            with self._lock:
                output = self._arbiter.emergency_stop(
                    "arbiter_wrapper_exception",
                    receipt_now_s=time.monotonic(),
                )
                self._command_pub.publish(_twist(output.command))
                self._publish_state(
                    int(output.state),
                    output.selected_source,
                    self._arbiter.fault_reason or output.reason,
                    rospy.Time.now(),
                )
        except Exception:
            pass

    def _publish_state(self, state, selected_source, reason, stamp):
        message = BaseAuthorityState()
        message.header.stamp = stamp
        message.state = state
        message.selected_source = selected_source
        message.reason = reason
        message.manipulation_hold_active = self._arbiter.manipulation_hold_active
        message.session_id = self._arbiter.session_id
        message.hold_epoch = self._arbiter.hold_epoch
        message.grant_id = self._arbiter.command_grant_id
        message.source_goal_id = self._arbiter.source_goal_id
        config = self._arbiter.config
        motion = self._arbiter.motion_feedback(receipt_now_s=time.monotonic(), header_now_s=stamp.to_sec())
        message.motion_valid = motion.valid
        message.motion_revision = motion.revision
        message.motion_stamp = rospy.Time.from_sec(motion.header_stamp_s)
        message.motion_frame_id = config.pose_feedback.command_frame
        message.motion = _twist(motion.command)
        message.motion_receipt_age_s = motion.receipt_age_s
        message.motion_receipt_limit_s = config.odometry_receipt_s
        message.motion_header_limit_s = config.odometry_header_s
        message.motion_future_tolerance_s = config.maximum_header_future_skew_s
        message.linear_progress_lead_mps = config.max_linear_speed_lead_mps
        message.angular_progress_lead_radps = config.max_angular_speed_lead_radps
        self._state_pub.publish(message)

    @staticmethod
    def _selected_source(state):
        if state == BaseAuthority.NAVIGATION:
            return "navigation"
        if state == BaseAuthority.DOCKING:
            return "docking"
        return ""

    def _on_shutdown(self):
        self._stop_event.set()
        with self._lock:
            output = self._arbiter.shutdown(
                receipt_now_s=time.monotonic(),
            )
            try:
                self._command_pub.publish(_twist(output.command))
            except Exception:
                pass


def main():
    rospy.init_node("base_command_arbiter")
    BaseCommandArbiterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
