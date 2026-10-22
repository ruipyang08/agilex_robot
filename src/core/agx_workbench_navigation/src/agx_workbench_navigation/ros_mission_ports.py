"""ROS1 runtime adapters for the workbench mission action wrapper."""

from __future__ import annotations

from dataclasses import replace
import json
import math
import os
import threading
import time
import uuid
from hashlib import sha256
from pathlib import Path

import actionlib
from actionlib_msgs.msg import GoalStatus
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import rospkg
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Empty
import tf2_geometry_msgs
import tf2_ros

from agx_workbench_navigation.contracts import (
    BaseAuthority,
    MotionDirection,
    OdomPose2D,
    SafetyHealth,
)
from agx_workbench_navigation.docking import DockingConfig, DockingController
from agx_workbench_navigation.departure import (
    DepartureStep,
    ReverseDepartureController,
    select_registered_departure,
)
from agx_workbench_navigation.mission import (
    AuthorityEvidence,
    AuthorityReply,
    EvidenceState,
    handoff_rejection_is_retryable,
    public_docking_reason,
)
from agx_workbench_navigation.msg import AuthorizedBaseCommand, BaseAuthorityState, LidarSafetyState, CoarseNavigationState
from agx_workbench_navigation.localization_health import LocalizationHealth
from agx_workbench_navigation.motion_progress import MotionProgressCache
from agx_workbench_navigation.parking_geometry import build_handoff
from agx_workbench_navigation.registry import (
    EXPECTED_CAMERA_FRAME,
    RegistryError,
    load_camera_identity,
    load_lidar_identity,
    load_parking_identity,
    load_registry,
)
from agx_workbench_navigation.srv import (
    SetBaseAuthority, SetBaseAuthorityRequest,
    BindBaseCommandSource, BindBaseCommandSourceRequest,
)
from agx_workbench_navigation.tag_filter import (
    TagObservation,
    TagObservationFilter,
)
from agx_workbench_navigation.wall_runtime import (
    WallCallTimeout,
    call_with_wall_timeout,
)


MOVE_BASE_ACTION = "/move_base"
AUTHORITY_SERVICE = "/workbench_nav/set_authority"
BIND_NAVIGATION_SERVICE = "/workbench_nav/bind_navigation_source"
AUTHORITY_STATE_TOPIC = "/workbench_nav/authority_state"
LIDAR_TOPIC = "/workbench_nav/lidar_safety_state"
MARKER_TOPIC = "/cam2/ar_pose_marker"


def age_is_fresh(receipt_s, stamp_s, receipt_limit_s=0.25, stamp_limit_s=0.25):
    if receipt_s is None or stamp_s is None or stamp_s <= 0.0:
        return False
    receipt_age = time.monotonic() - receipt_s
    stamp_age = rospy.Time.now().to_sec() - stamp_s
    return (
        0.0 <= receipt_age <= receipt_limit_s
        and 0.0 <= stamp_age <= stamp_limit_s
    )


def zero_twist():
    return Twist()


def twist_message(command):
    message = Twist()
    message.linear.x = command.linear_x
    message.linear.y = command.linear_y
    message.angular.z = command.angular_z
    return message


def yaw_from_quaternion(rotation):
    sin_yaw = 2.0 * (
        rotation.w * rotation.z + rotation.x * rotation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        rotation.y * rotation.y + rotation.z * rotation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class RegistryPort:
    def __init__(self):
        package = rospkg.RosPack().get_path("agx_workbench_navigation")
        self._registry_path = rospy.get_param("~registry_path")
        self._map_yaml = rospy.get_param("~map_yaml")
        self._mode = rospy.get_param("~operation_mode", "hardware")
        self._commissioning = rospy.get_param("~commissioning", False)
        if self._mode == "hardware":
            camera_path = rospy.get_param(
                "~camera_config",
                os.path.join(package, "config/cam2.yaml"),
            )
            parking_path = rospy.get_param(
                "~parking_config",
                os.path.join(package, "config/parking.yaml"),
            )
            lidar_path = rospy.get_param(
                "~lidar_config",
                os.path.join(package, "config/lidar_safety.yaml"),
            )
            self._camera = load_camera_identity(camera_path)
            self._parking = load_parking_identity(parking_path)
            self._lidar = load_lidar_identity(lidar_path)
        else:
            self._camera = None
            self._parking = None
            self._lidar = None
        self._registry = None

    @property
    def mode(self):
        return self._mode

    @property
    def commissioning(self):
        return self._commissioning

    def _load(self):
        return load_registry(
            self._registry_path,
            map_yaml=self._map_yaml,
            camera=self._camera,
            parking=self._parking,
            lidar=self._lidar,
            mode=self._mode,
            commissioning=self._commissioning,
        )

    def lookup(self, workbench_id):
        self._registry = self._load()
        return self._registry.workbench(workbench_id)

    def still_valid(self, entry):
        try:
            current = self._load()
            return current.workbench(entry.workbench_id) == entry
        except RegistryError:
            return False

    def entries(self):
        if self._registry is None:
            raise RegistryError("registered destinations have not been loaded")
        return self._registry.workbenches

    def route_binding(self, target):
        # Bind one validated reload to the exact registry bytes. The registry
        # loader separately verifies its referenced map and calibration hashes.
        path = Path(self._registry_path)
        before = path.read_bytes()
        registry = self._load()
        if path.read_bytes() != before:
            raise RegistryError('registry changed while resolving route')
        identity = registry.map_identity
        map_digest = 'sha256:'+sha256((identity.yaml_sha256+'|'+identity.image_sha256).encode('ascii')).hexdigest()
        return (registry.workbench(target), registry.workbenches,
                'sha256:'+sha256(before).hexdigest(), map_digest)


class MoveBasePort:
    _ACTIVE = (GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING)
    _TERMINAL = (GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                 GoalStatus.REJECTED, GoalStatus.RECALLED)

    def __init__(self, authority):
        self._authority = authority
        self._goal_started = False
        self._last_goal_stamp = None
        self._lock = threading.RLock()
        self._intent_lock = threading.RLock()
        self._goal_generation = 0
        self._route_context = None
        self._route_failure=None
        self._execution_binding=None
        self._prepared_goal = None
        self._route_sub=rospy.Subscriber(rospy.get_param('/move_base/RangerLocalPlanner/state_topic',
            '/workbench_nav/coarse_navigation_state'),CoarseNavigationState,
            self._route_state,queue_size=1)
        self._bind = rospy.ServiceProxy(BIND_NAVIGATION_SERVICE, BindBaseCommandSource)
        self._client = actionlib.SimpleActionClient(
            MOVE_BASE_ACTION,
            MoveBaseAction,
        )

    def start(self, staging_pose):
        with self._lock:
            return self._start(staging_pose)

    def prepare(self, staging_pose):
        """Bind/refresh the native producer's zero without sending a move goal."""
        with self._lock:
            return self._start(staging_pose, dispatch=False)

    def configure_route(self, invocation_id, route_id, task_deadline_s):
        with self._lock,self._intent_lock:
            self._route_context = (invocation_id, route_id, task_deadline_s)
            self._route_failure=None;self._execution_binding=None
            self._prepared_goal = None

    def _route_state(self,message):
        with self._intent_lock:
            if (message.state!=CoarseNavigationState.BLOCKED or self._route_context is None or
                    (message.invocation_id,message.route_id)!=self._route_context[:2] or
                    (message.session_id,message.grant_id,message.source_goal_id)!=self._execution_binding):
                return
            if self._route_failure is None:
                self._route_failure=message.reason[:256]
                self._goal_generation+=1
                self._client.cancel_goal()

    def _start(self, staging_pose, dispatch=True):
        with self._intent_lock:
            generation = self._goal_generation
        # SimpleActionClient owns the tracked goal. LOST is absence of proof,
        # not terminal evidence with which a later operation may be rebound.
        if self._goal_started and self._client.get_state() not in self._TERMINAL:
            return False
        binding = self._authority.command_binding(BaseAuthority.NAVIGATION)
        if binding is None or staging_pose["frame"] != "map":
            return False
        try:
            server_ready = call_with_wall_timeout(
                lambda: self._client.wait_for_server(rospy.Duration(5.0)),
                5.0,
            )
        except WallCallTimeout:
            return False
        if not server_ready:
            return False
        prepared = self._prepared_goal
        if prepared is not None:
            if prepared[:2] != (binding, generation):
                return False
            goal = prepared[2]
        else:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = staging_pose["frame"]
            goal.target_pose.header.stamp = rospy.Time.now()
            if goal.target_pose.header.stamp.to_nsec() <= 0 or (
                    self._last_goal_stamp is not None and
                    goal.target_pose.header.stamp <= self._last_goal_stamp):
                return False
        position = staging_pose["position_m"]
        orientation = staging_pose["orientation_xyzw"]
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        try:
            route = self._route_context or ('', '', 0.)
            if route[1] and (time.monotonic() >= route[2] or not route[0]):
                return False
            response = call_with_wall_timeout(lambda: self._bind(BindBaseCommandSourceRequest(
                session_id=binding[0], grant_id=binding[1], source_goal_id=binding[2],
                expected_goal=goal.target_pose, invocation_id=route[0],route_id=route[1],
                task_deadline_s=route[2])), 2.0)
        except (WallCallTimeout, rospy.ServiceException):
            return False
        if (not response.accepted or self._authority.command_binding(BaseAuthority.NAVIGATION) != binding or
                (route[1] and time.monotonic() >= route[2])):
            return False
        with self._intent_lock:
            if generation != self._goal_generation:
                return False
            self._last_goal_stamp = goal.target_pose.header.stamp
            self._execution_binding=binding
            self._prepared_goal = (binding, generation, goal)
            if dispatch:
                self._client.send_goal(goal)
                self._goal_started = True
                self._prepared_goal = None
        return True

    def failure_reason(self):
        with self._intent_lock:
            return self._route_failure or ''

    def status(self):
        with self._intent_lock:
            if self._route_failure is not None:return "route_blocked"
        state = self._client.get_state()
        if state in self._ACTIVE:
            return "active"
        if state == GoalStatus.SUCCEEDED:
            return "succeeded"
        if state == GoalStatus.REJECTED:
            return "rejected"
        return "aborted"

    def cancel(self):
        with self._intent_lock:
            self._goal_generation += 1
            self._prepared_goal = None
            self._client.cancel_goal()


class _FreshTopicPort:
    def __init__(self):
        self._lock = threading.RLock()
        self._receipt_s = None
        self._stamp_s = None

    def _record(self, stamp):
        with self._lock:
            self._receipt_s = time.monotonic()
            self._stamp_s = stamp.to_sec()

    def ready(self):
        with self._lock:
            return age_is_fresh(self._receipt_s, self._stamp_s)


class LocalizationPort(_FreshTopicPort):
    def __init__(self, tf_buffer):
        super().__init__()
        self._tf_buffer = tf_buffer
        self._health = LocalizationHealth(
            receipt_age_s=float(rospy.get_param("~localization_receipt_age_s", .75)),
            source_age_s=float(rospy.get_param("~localization_source_age_s", .75)))
        self._refresh_thread = None
        self._last_refresh_s = -math.inf
        refresh_service = rospy.get_param("~localization_nomotion_service", "/request_nomotion_update")
        self._nomotion = rospy.ServiceProxy(refresh_service, Empty) if refresh_service else None
        self._subscriber = rospy.Subscriber(
            rospy.get_param("~localization_pose_topic", "/amcl_pose"),
            PoseWithCovarianceStamped, self._callback, queue_size=1)
        self._odom_subscriber = rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)

    @staticmethod
    def _values(pose):
        p, q = pose.position, pose.orientation
        return (p.x,p.y,p.z), (q.x,q.y,q.z,q.w)

    def _callback(self, message):
        position, orientation = self._values(message.pose.pose)
        with self._lock:
            self._health.observe_pose(frame=message.header.frame_id, position=position,
                orientation=orientation, covariance=message.pose.covariance,
                stamp_s=message.header.stamp.to_sec(), receipt_s=time.monotonic(),
                source_now_s=rospy.Time.now().to_sec())

    def _odom_callback(self, message):
        position, orientation = self._values(message.pose.pose)
        linear, angular = message.twist.twist.linear, message.twist.twist.angular
        with self._lock:
            self._health.observe_odometry(frame=message.header.frame_id,
                child_frame=message.child_frame_id, position=position, orientation=orientation,
                covariance=message.pose.covariance,
                velocity=(linear.x,linear.y,linear.z,angular.x,angular.y,angular.z),
                stamp_s=message.header.stamp.to_sec(), receipt_s=time.monotonic(),
                source_now_s=rospy.Time.now().to_sec())

    def _refresh_stationary_amcl(self, now):
        if (self._nomotion is None or not self._health.pose_needs_refresh(now) or
                now-self._last_refresh_s < .25 or
                (self._refresh_thread is not None and self._refresh_thread.is_alive())):
            return
        self._last_refresh_s = now
        def request():
            try:
                self._nomotion()
            except Exception:
                pass  # Only a later real pose callback can refresh health.
        # At most one outstanding request, even if ROS transport hangs. The
        # mission never waits for it and stale evidence still fails closed.
        self._refresh_thread = threading.Thread(target=request, daemon=True)
        self._refresh_thread.start()

    def ready(self):
        with self._lock:
            self._refresh_stationary_amcl(time.monotonic())
        try:
            # Poll the existing TF cache. A ROS-time wait can hang while /clock
            # is paused and must not stall the mission's steady-clock cleanup.
            transform = self._tf_buffer.lookup_transform("map", "rslidar", rospy.Time(0), rospy.Duration(0.))
            if not self._tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(0.)):
                return False
            t, q = transform.transform.translation, transform.transform.rotation
            with self._lock:
                return self._health.ready(receipt_now_s=time.monotonic(),
                    source_now_s=rospy.Time.now().to_sec(), transform_frame=transform.header.frame_id,
                    transform_child=transform.child_frame_id, transform_position=(t.x,t.y,t.z),
                    transform_orientation=(q.x,q.y,q.z,q.w))
        except Exception:
            return False

    def pose_map(self):
        if not self.ready():
            raise RuntimeError("fresh map-frame base pose unavailable")
        transform = self._tf_buffer.lookup_transform(
            "map", "base_link", rospy.Time(0), rospy.Duration(0.))
        t, q = transform.transform.translation, transform.transform.rotation
        return OdomPose2D(t.x, t.y, yaw_from_quaternion(q))


class CameraPort(_FreshTopicPort):
    def __init__(self):
        super().__init__()
        self._image_receipt_s = None
        self._image_stamp_s = None
        self._info_receipt_s = None
        self._info_stamp_s = None
        self._image_sub = rospy.Subscriber(
            "/cam2/color/image_raw",
            Image,
            self._on_image,
            queue_size=1,
        )
        self._info_sub = rospy.Subscriber(
            "/cam2/color/camera_info",
            CameraInfo,
            self._on_info,
            queue_size=1,
        )

    def _on_image(self, message):
        with self._lock:
            self._image_receipt_s = time.monotonic()
            self._image_stamp_s = message.header.stamp.to_sec()

    def _on_info(self, message):
        with self._lock:
            self._info_receipt_s = time.monotonic()
            self._info_stamp_s = message.header.stamp.to_sec()

    def ready(self):
        with self._lock:
            return age_is_fresh(
                self._image_receipt_s,
                self._image_stamp_s,
            ) and age_is_fresh(
                self._info_receipt_s,
                self._info_stamp_s,
            )


class LidarPort:
    def __init__(self):
        self._lock = threading.RLock()
        self._message = None
        self._receipt_s = None
        self._receipt_limit_s = rospy.get_param(
            "~lidar_state_receipt_timeout_s", 0.25
        )
        self._subscriber = rospy.Subscriber(
            LIDAR_TOPIC,
            LidarSafetyState,
            self._callback,
            queue_size=1,
        )

    def _callback(self, message):
        with self._lock:
            self._message = message
            self._receipt_s = time.monotonic()

    def health(self):
        with self._lock:
            message = self._message
            receipt = self._receipt_s
        if message is None:
            return SafetyHealth.STALE
        if not age_is_fresh(
            receipt,
            message.header.stamp.to_sec(),
            receipt_limit_s=self._receipt_limit_s,
        ):
            return SafetyHealth.STALE
        try:
            return SafetyHealth(message.state)
        except ValueError:
            return SafetyHealth.INVALID

    def forward_clear(self):
        with self._lock:
            message = self._message
        health = self.health()
        return (
            health in (SafetyHealth.CLEAR, SafetyHealth.BLOCKED)
            and message is not None
            and not (
                self.blocked_directions() & MotionDirection.FORWARD
            )
            and message.forward_clearance_m > message.required_stop_distance_m
        )

    def reverse_clear(self):
        with self._lock:
            message = self._message
        health = self.health()
        return (
            health in (SafetyHealth.CLEAR, SafetyHealth.BLOCKED)
            and message is not None
            and not (self.blocked_directions() & MotionDirection.REVERSE)
            and message.reverse_clearance_m > message.required_stop_distance_m
        )

    def blocked_directions(self):
        with self._lock:
            message = self._message
        health = self.health()
        if (
            health not in (SafetyHealth.CLEAR, SafetyHealth.BLOCKED)
            or message is None
        ):
            return MotionDirection.ALL
        try:
            blocked = MotionDirection(message.blocked_directions)
        except (TypeError, ValueError):
            return MotionDirection.ALL
        if int(blocked) & ~int(MotionDirection.ALL):
            return MotionDirection.ALL
        return blocked


class AuthorityPort:
    def __init__(self):
        self._lock = threading.RLock()
        self._on_grant = None
        self._motion_progress = MotionProgressCache()
        self._state = BaseAuthority.NONE
        self._receipt_s = None
        self._reason = "state_unavailable"
        self._session_id = ""
        self._operation_source = None
        self._operation_id = ""
        self._binding = None
        self._release_timeout_s = rospy.get_param(
            "~authority_release_timeout_s",
            0.50,
        )
        self._proxy = rospy.ServiceProxy(AUTHORITY_SERVICE, SetBaseAuthority)
        self._subscriber = rospy.Subscriber(
            AUTHORITY_STATE_TOPIC,
            BaseAuthorityState,
            self._callback,
            queue_size=1,
        )

    def _callback(self, message):
        with self._lock:
            try:
                self._state = BaseAuthority(message.state)
            except ValueError:
                self._state = BaseAuthority.FAULT
            self._reason = str(message.reason)
            self._session_id = str(message.session_id)
            self._receipt_s = time.monotonic()
            self._motion_progress.observe(message, receipt_now_s=self._receipt_s)

    def set_grant_observer(self, callback):
        with self._lock:
            self._on_grant = callback

    def request(self, state):
        with self._lock:
            if state == BaseAuthority.NONE:
                self._operation_source = None
                self._operation_id = ""
                self._binding = None
            elif self._operation_source != state:
                self._operation_source = state
                self._operation_id = uuid.uuid4().hex
                self._binding = None
            session, operation = self._session_id, self._operation_id
            on_grant = self._on_grant
        response = self._proxy(
            SetBaseAuthorityRequest(requested_state=int(state),
                expected_session_id=session, source_goal_id=operation)
        )
        try:
            response_state = BaseAuthority(response.state)
        except ValueError:
            response_state = BaseAuthority.FAULT
        with self._lock:
            # A late service response cannot replace a newer local operation.
            if (self._operation_source, self._operation_id) != (
                    None if state == BaseAuthority.NONE else state, operation):
                return AuthorityReply(False, self._state, "superseded_authority_request")
            if response.accepted and state != BaseAuthority.NONE:
                if (response.session_id != session or response.source_goal_id != operation or
                        not response.grant_id):
                    return AuthorityReply(False, self._state, "invalid_command_grant")
                self._binding = (response.session_id, response.grant_id, operation)
            self._state = response_state
            self._reason = str(response.reason)
            self._receipt_s = time.monotonic()
        if response.accepted and state != BaseAuthority.NONE and on_grant is not None:
            try:
                on_grant(dict(source=int(state), session_id=response.session_id,
                    grant_id=response.grant_id, source_goal_id=operation,
                    source_time_s=rospy.Time.now().to_sec()))
            except Exception:
                pass  # Read-only diagnostics never change an authority decision.
        return AuthorityReply(
            response.accepted,
            response_state,
            response.reason,
        )

    def command_binding(self, source):
        with self._lock:
            return self._binding if self._operation_source == source else None

    def motion_progress(self, binding, *, now_s):
        with self._lock:
            if (binding is None or binding != self._binding
                    or self._operation_source != BaseAuthority.DOCKING):
                return None
            return self._motion_progress.snapshot(binding=binding,
                source=BaseAuthority.DOCKING, now_s=now_s,
                receipt_now_s=time.monotonic())

    def state(self):
        with self._lock:
            return self._state

    def status(self):
        with self._lock:
            state = self._state
            receipt_s = self._receipt_s
            reason = self._reason
        fresh = (
            receipt_s is not None
            and 0.0 <= time.monotonic() - receipt_s <= 0.25
        )
        return AuthorityEvidence(state, fresh, reason)

    def release(self):
        return call_with_wall_timeout(
            lambda: self.request(BaseAuthority.NONE),
            self._release_timeout_s,
        )


class OdometryPort:
    def __init__(self, docking_config, receipt_limit_s):
        if isinstance(receipt_limit_s, bool):
            raise ValueError("odometry receipt limit must be positive")
        try:
            self._receipt_limit_s = float(receipt_limit_s)
        except (TypeError, ValueError):
            raise ValueError("odometry receipt limit must be positive")
        if (
            not math.isfinite(self._receipt_limit_s)
            or self._receipt_limit_s <= 0.0
        ):
            raise ValueError("odometry receipt limit must be positive")
        self._lock = threading.RLock()
        self._config = docking_config
        self._pose = None
        self._linear_speed = math.inf
        self._angular_speed = math.inf
        self._receipt_s = None
        self._stamp_s = None
        self._settled_since_s = None
        self._subscriber = rospy.Subscriber(
            "/odom",
            Odometry,
            self._callback,
            queue_size=1,
        )

    def _callback(self, message):
        pose = message.pose.pose
        velocity = message.twist.twist
        linear_speed = math.hypot(velocity.linear.x, velocity.linear.y)
        angular_speed = abs(velocity.angular.z)
        receipt = time.monotonic()
        stopped = (
            linear_speed <= self._config.stop_linear_speed_mps
            and angular_speed <= self._config.stop_angular_speed_radps
        )
        with self._lock:
            self._pose = OdomPose2D(
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
            self._linear_speed = linear_speed
            self._angular_speed = angular_speed
            self._receipt_s = receipt
            self._stamp_s = message.header.stamp.to_sec()
            if stopped:
                if self._settled_since_s is None:
                    self._settled_since_s = receipt
            else:
                self._settled_since_s = None

    def snapshot(self):
        with self._lock:
            return (
                self._pose,
                self._linear_speed,
                self._angular_speed,
                self._receipt_s,
                self._stamp_s,
            )

    def fresh(self):
        with self._lock:
            return age_is_fresh(
                self._receipt_s,
                self._stamp_s,
                receipt_limit_s=self._receipt_limit_s,
                stamp_limit_s=self._config.handoff_evidence_max_age_s,
            )

    def settled(self):
        with self._lock:
            return (
                self.fresh()
                and self._settled_since_s is not None
                and time.monotonic() - self._settled_since_s
                >= self._config.stop_dwell_s
            )

    def final_within_tolerance(self, estimate):
        return (
            self.settled()
            and estimate.error.position_m <= self._config.position_tolerance_m
            and abs(estimate.error.yaw_rad) <= self._config.max_yaw_drift_rad
        )


class TagPort:
    def __init__(self, tf_buffer):
        self._lock = threading.RLock()
        self._tf_buffer = tf_buffer
        self._filter = None
        self._requested_marker_id = None
        self._evidence_max_age_s = 0.25
        self._receipt_max_age_s = 0.25
        self._state = EvidenceState.WAITING
        self._detail = "waiting_for_expected_tag"
        self._receipt_s = None
        self._subscriber = rospy.Subscriber(
            MARKER_TOPIC,
            AlvarMarkers,
            self._callback,
            queue_size=1,
        )

    def configure(self, marker_id):
        required_samples = rospy.get_param("~tag_required_samples", 8)
        min_span_s = rospy.get_param("~tag_min_span_s", 0.35)
        history_max_age_s = rospy.get_param("~tag_history_max_age_s", 0.60)
        evidence_max_age_s = rospy.get_param("~tag_evidence_max_age_s", 0.25)
        if (
            isinstance(evidence_max_age_s, bool)
            or not math.isfinite(float(evidence_max_age_s))
            or float(evidence_max_age_s) <= 0.0
        ):
            raise ValueError("tag_evidence_max_age_s must be positive")
        evidence_max_age_s = float(evidence_max_age_s)
        receipt_max_age_s = rospy.get_param(
            "~tag_receipt_max_age_s", evidence_max_age_s
        )
        if isinstance(receipt_max_age_s, bool) or not isinstance(
            receipt_max_age_s, (int, float)
        ):
            raise ValueError("tag_receipt_max_age_s must be a finite positive number")
        try:
            receipt_max_age_s = float(receipt_max_age_s)
        except (OverflowError, ValueError, TypeError) as error:
            raise ValueError(
                "tag_receipt_max_age_s must be a finite positive number"
            ) from error
        if not math.isfinite(receipt_max_age_s) or receipt_max_age_s <= 0.0:
            raise ValueError("tag_receipt_max_age_s must be a finite positive number")
        observation_filter = TagObservationFilter(
            marker_id,
            required_samples=required_samples,
            min_span_s=min_span_s,
            max_age_s=history_max_age_s,
            max_range_m=2.0,
            expected_floor_z_m=rospy.get_param(
                "~expected_floor_z_m",
                -0.32,
            ),
            max_floor_height_error_m=0.03,
            max_floor_normal_error_rad=0.10,
            max_translation_jump_m=0.08,
            max_angular_jump_rad=0.15,
            expected_frame="base_link",
        )
        with self._lock:
            self._requested_marker_id = marker_id
            self._filter = observation_filter
            self._evidence_max_age_s = float(evidence_max_age_s)
            self._receipt_max_age_s = receipt_max_age_s
            self._state = EvidenceState.WAITING
            self._detail = "waiting_for_expected_tag"
            self._receipt_s = None

    def _callback(self, message):
        try:
            self._process_message(message)
        except Exception:
            with self._lock:
                self._receipt_s = time.monotonic()
                self._state = EvidenceState.INVALID
                self._detail = "invalid_tag_pose"

    def _process_message(self, message):
        with self._lock:
            observation_filter = self._filter
            requested_marker_id = self._requested_marker_id
        if observation_filter is None:
            return
        markers = tuple(message.markers)
        marker_ids = tuple(marker.id for marker in markers)
        expected = [
            marker for marker in markers if marker.id == requested_marker_id
        ]
        if not expected:
            with self._lock:
                if self._filter is not observation_filter:
                    return
                self._receipt_s = time.monotonic()
                self._state = EvidenceState.WAITING
                self._detail = (
                    "wrong_marker_id" if marker_ids else "tag_not_visible"
                )
            return
        marker = expected[0]
        pose = PoseStamped()
        pose.header = marker.header
        pose.pose = marker.pose.pose
        if pose.header.frame_id != EXPECTED_CAMERA_FRAME:
            with self._lock:
                if self._filter is not observation_filter:
                    return
                self._receipt_s = time.monotonic()
                self._state = EvidenceState.INVALID
                self._detail = "unexpected_tag_frame"
            return
        try:
            # TF must be evaluated at the observation stamp, never at "latest".
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                pose.header.frame_id,
                pose.header.stamp,
                rospy.Duration(0.10),
            )
            base_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            with self._lock:
                if self._filter is not observation_filter:
                    return
                self._receipt_s = time.monotonic()
                self._state = EvidenceState.TF_UNAVAILABLE
                self._detail = "tf_unavailable"
            return
        position = base_pose.pose.position
        rotation = base_pose.pose.orientation
        observation = TagObservation(
            marker_id=marker.id,
            detected_marker_ids=marker_ids,
            stamp_s=pose.header.stamp.to_sec(),
            frame_id="base_link",
            translation_m=(position.x, position.y, position.z),
            quaternion_xyzw=(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
            # The same source-stamped TF that located the marker also locates
            # the camera. Detector range is measured from this optical origin,
            # not from the rover's base_link origin.
            camera_position_m=(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ),
        )
        with self._lock:
            if self._filter is not observation_filter:
                return
            decision = observation_filter.update(
                observation,
                rospy.Time.now().to_sec(),
            )
            self._receipt_s = time.monotonic()
            if decision.acquired:
                self._state = EvidenceState.ACQUIRED
            elif decision.accepted:
                self._state = EvidenceState.WAITING
            else:
                self._state = EvidenceState.INVALID
            self._detail = decision.reason

    @staticmethod
    def _freshness_detail(decision, now_ros_s, receipt_age, evidence_age,
                          limit_s, required_samples, min_span_s, receipt_limit_s):
        # One fixed snapshot of values already used by this evidence poll.
        def scalar(value):
            return "none" if value is None else format(value, ".17g")

        pose = decision.filtered_pose
        window = decision.window
        span = window[-1].stamp_s - window[0].stamp_s if window else None
        detail = (
            "tag_not_fresh: acquired={} pose={} source_now_s={} pose_stamp_s={} "
            "receipt_age_s={} evidence_age_s={} limit_s={} window_samples={} "
            "window_span_s={} required_samples={} min_span_s={} receipt_limit_s={}"
        ).format(
            int(decision.acquired), int(pose is not None), scalar(now_ros_s),
            scalar(pose.stamp_s if pose is not None else None),
            scalar(receipt_age), scalar(evidence_age), scalar(limit_s),
            len(window), scalar(span), required_samples, scalar(min_span_s),
            scalar(receipt_limit_s))
        return detail if len(detail) <= 576 else "tag_not_fresh"

    def evidence(self):
        try:
            now_ros_s = rospy.Time.now().to_sec()
        except Exception:
            with self._lock:
                self._state = EvidenceState.INVALID
                self._detail = "invalid_tag_pose"
                return self._state, self._detail, None
        with self._lock:
            if self._filter is None:
                return self._state, self._detail, None
            previous_state = self._state
            try:
                decision = self._filter.refresh(now_ros_s)
            except Exception:
                self._state = EvidenceState.INVALID
                self._detail = "invalid_tag_pose"
                return self._state, self._detail, None
            pose = decision.filtered_pose
            receipt_age = (
                math.inf
                if self._receipt_s is None
                else time.monotonic() - self._receipt_s
            )
            evidence_age = (
                math.inf if pose is None else now_ros_s - pose.stamp_s
            )
            fresh = (
                pose is not None
                and 0.0 <= receipt_age <= self._receipt_max_age_s
                and 0.0 <= evidence_age <= self._evidence_max_age_s
            )
            if previous_state in (
                EvidenceState.INVALID,
                EvidenceState.TF_UNAVAILABLE,
            ):
                return previous_state, self._detail, pose
            if decision.acquired and fresh:
                self._state = EvidenceState.ACQUIRED
                return self._state, self._detail, pose
            if previous_state == EvidenceState.ACQUIRED:
                self._state = EvidenceState.INVALID
                self._detail = "tag_not_fresh"
                try:
                    self._detail = self._freshness_detail(
                        decision, now_ros_s, receipt_age, evidence_age,
                        self._evidence_max_age_s, self._filter.required_samples,
                        self._filter.min_span_s, self._receipt_max_age_s)
                except Exception:
                    pass  # Optional detail must not suppress the invalid result.
                return self._state, self._detail, None
            self._state = EvidenceState.WAITING
            if self._detail == "accepted":
                self._detail = "waiting_for_consistent_tag_window"
            return self._state, self._detail, pose

    def sample_window(self):
        with self._lock:
            if self._filter is None:
                return ()
            return tuple(self._filter.window)


class ReverseDeparturePort:
    """Use the existing docking grant and command topic for measured undocking."""

    def __init__(self, config, publisher, registry, localization,
                 odometry, lidar, authority):
        self._config = config
        self._publisher = publisher
        self._registry = registry
        self._localization = localization
        self._odometry = odometry
        self._lidar = lidar
        self._authority = authority
        self._lock = threading.RLock()
        self._binding = None
        self._sequence = 0
        self._controller = None
        self._route = self._route_owner = self._route_goal = None
        self._route_cancelled = lambda: True

    def configure_route(self, route, owner, goal, cancelled):
        self._route, self._route_owner, self._route_goal = route, owner, goal
        self._route_cancelled = cancelled
        self._controller = self._binding = None

    def plan(self, target):
        plan = select_registered_departure(
            self._localization.pose_map(), self._registry.entries(),
            target.workbench_id, self._config.max_rollback_travel_m)
        if self._route is None:
            raise RuntimeError("departure requires the admitted action route")
        if plan is None:
            if self._route.departure_source_id:
                raise RuntimeError("admitted dock start no longer matches measured pose")
            return None
        if plan.source_id != self._route.departure_source_id:
            raise RuntimeError("departure source differs from the admitted route")
        if not self._route_owner.validate_departure(self._route_goal, self._route_cancelled):
            raise RuntimeError("registered departure route no longer ready")
        return plan

    def start(self, plan):
        with self._lock:
            if not self._odometry.fresh() or not self._odometry.settled():
                raise RuntimeError("reverse departure requires a stopped odometry origin")
            pose, _, _, _, _ = self._odometry.snapshot()
            binding = self._authority.command_binding(BaseAuthority.DOCKING)
            if pose is None or binding is None:
                raise RuntimeError("reverse departure has no measured pose or docking grant")
            controller = ReverseDepartureController(plan, self._config)
            controller.start(pose, rospy.Time.now().to_sec())
            self._binding = binding
            self._sequence = 0
            self._controller = controller
            self._publish_zero()

    def _publish(self, linear_x, stamp):
        if self._binding is None:
            return
        self._sequence += 1
        command = AuthorizedBaseCommand()
        command.header.stamp = stamp
        command.header.frame_id = "base_link"
        command.session_id, command.grant_id, command.source_goal_id = self._binding
        command.sequence = self._sequence
        command.twist.linear.x = linear_x
        self._publisher.publish(command)

    def _publish_zero(self):
        self._publish(0.0, rospy.Time.now())

    def step(self):
        with self._lock:
            if self._controller is None:
                raise RuntimeError("reverse departure has not started")
            stamp = rospy.Time.now()
            pose, linear, angular, _, _ = self._odometry.snapshot()
            output = self._controller.step(
                pose, linear, angular, self._odometry.fresh(),
                self._lidar.reverse_clear(), stamp.to_sec())
            self._publish(output.command_mps, stamp)
            if output.complete:
                verified = self._route_owner.complete_departure(self._route_goal, self._route_cancelled)
                if self._route_cancelled():
                    return DepartureStep(progress_m=output.progress_m, failure_reason="cancelled")
                if not verified:
                    return DepartureStep(progress_m=output.progress_m, failure_reason="route_invalidated")
            return output

    def stop(self):
        with self._lock:
            self._publish_zero()


class DockingPort:
    def __init__(
        self,
        config,
        command_publisher,
        tags,
        odometry,
        lidar,
        operation_mode,
        commissioning,
        authority,
    ):
        self._authority = authority
        self._command_lock = threading.RLock()
        self._command_binding = None
        self._command_sequence = 0
        self._config = config
        self._publisher = command_publisher
        self._tags = tags
        self._odometry = odometry
        self._lidar = lidar
        self._operation_mode = operation_mode
        self._commissioning = commissioning
        self._controller = None
        self._entry = None
        self._handoff_rejection_reason = None
        self._attempt_index = 1

    def start(self, entry):
        with self._command_lock:
            self._command_binding = None
            self._command_sequence = 0
            self._motion_progress_diagnostic = None
            self._motion_progress_diagnostic_captured = False
            self._start(entry)

    def _start(self, entry):
        self._entry = entry
        entry_maximum = float(entry.parking_geometry["max_blind_travel_m"])
        config_values = dict(vars(self._config))
        config_values["max_blind_travel_m"] = entry_maximum
        self._controller = DockingController(
            DockingConfig(**config_values),
            requested_marker_id=entry.marker_id,
            expected_frame="base_link",
        )
        self._handoff_rejection_reason = None
        self._attempt_index = 1

    def _publish_command(self, twist, stamp, binding):
        with self._command_lock:
            if binding is None or binding != self._command_binding:
                return
            self._command_sequence += 1
            message = AuthorizedBaseCommand()
            message.header.stamp = stamp
            message.header.frame_id = "base_link"
            message.session_id, message.grant_id, message.source_goal_id = binding
            message.sequence = self._command_sequence
            message.twist = twist
            self._publisher.publish(message)
            return message.sequence  # Local publish returned; not a transport acknowledgement.

    def publish_zero(self):
        with self._command_lock:
            # Bind only once at operation start; authority topics never relabel
            # an existing controller or computation with a replacement grant.
            if self._command_binding is None:
                self._command_binding = self._authority.command_binding(BaseAuthority.DOCKING)
            self._publish_command(zero_twist(), rospy.Time.now(), self._command_binding)

    def stop(self):
        self.publish_zero()

    def step(self):
        with self._command_lock:
            return self._step()

    def _step(self):
        binding = self._command_binding
        command_stamp = rospy.Time.now()
        if binding is None:
            raise RuntimeError("docking operation has no command grant")
        if self._controller is None:
            raise RuntimeError("docking controller is not configured")
        # Motion-rate limits must advance with the same clock as the robot
        # state.  Under /use_sim_time this is Isaac physics time; on hardware
        # ROS time advances with real time.  Transport receipt guards remain
        # on the monotonic wall clock in their sensor ports so paused
        # simulation cannot make stale evidence look newly received.
        controller_now_s = command_stamp.to_sec()
        motion_progress = self._authority.motion_progress(binding, now_s=controller_now_s)
        monotonic_now_s = time.monotonic()
        odom, linear, angular, _, _ = self._odometry.snapshot()
        if odom is None:
            odom = OdomPose2D(0.0, 0.0, 0.0)
        tag_state, tag_detail, tag = self._tags.evidence()
        lidar_health = self._lidar.health()
        output = self._controller.step(
            tag=tag,
            odom=odom,
            lidar_clear=lidar_health
            in (SafetyHealth.CLEAR, SafetyHealth.BLOCKED),
            lidar_blocked_directions=self._lidar.blocked_directions(),
            now_s=controller_now_s,
            monotonic_now_s=monotonic_now_s,
            motion_progress=motion_progress,
            tag_fresh=tag_state == EvidenceState.ACQUIRED,
            odom_fresh=self._odometry.fresh(),
            lidar_fresh=lidar_health
            in (SafetyHealth.CLEAR, SafetyHealth.BLOCKED),
            odom_linear_speed_mps=linear,
            odom_angular_speed_radps=angular,
        )
        public_reason = public_docking_reason(
            output.reason,
            self._handoff_rejection_reason,
            tag_detail=tag_detail,
        )
        if public_reason != output.reason:
            output = replace(output, reason=public_reason)
        if output.attempt_index != self._attempt_index:
            self._attempt_index = output.attempt_index
            self._tags.configure(self._entry.marker_id)
        failure = None
        if output.terminal and output.reason == "motion_progress_infeasible":
            try:
                failure = self._controller.motion_progress_failure
            except Exception:
                pass
        published_sequence = None
        try:
            if output.command_generated:
                published_sequence = self._publish_command(
                    twist_message(output.command), command_stamp, binding)
        finally:
            if failure is not None:
                try:
                    self._remember_motion_progress_failure(
                        failure, binding, command_stamp, published_sequence)
                except Exception:
                    pass  # Preserve both safe output and any original publish exception.
        if self._controller.handoff_ready:
            self._accept_handoff(controller_now_s)
        return output

    def _remember_motion_progress_failure(self, failure, binding, stamp, sequence):
        if not self._motion_progress_diagnostic_captured:
            self._motion_progress_diagnostic_captured = True
            self._motion_progress_diagnostic = (binding, stamp.to_nsec(), sequence, failure)

    def log_motion_progress_failure(self):
        """Drain once after terminal cleanup; never called by the safety poll."""
        try:
            with self._command_lock:
                record = getattr(self, "_motion_progress_diagnostic", None)
                self._motion_progress_diagnostic = None
            if record is None:
                return
            binding, stamp_ns, sequence, failure = record
            progress = failure.progress

            def unsigned(value):
                if value is None:
                    return None
                if isinstance(value, int) and not isinstance(value, bool) and value >= 0 and value.bit_length() <= 64:
                    return value
                return "out_of_range"

            def axes(command):
                return (command.linear_x, command.linear_y, command.angular_z)

            payload = json.dumps(dict(
                session_id=binding[0][:64], grant_id=binding[1][:64], source_goal_id=binding[2][:64],
                identity_truncated=any(len(value) > 64 for value in binding),
                source_stamp_ns=unsigned(stamp_ns), zero_publish_sequence=unsigned(sequence),
                phase=failure.phase.value, previous=axes(failure.previous), candidate=axes(failure.candidate),
                maximum_change=axes(failure.maximum_change), source_now_s=failure.source_now_s,
                source_dt_s=failure.source_dt_s, evaluation_monotonic_s=failure.receipt_now_s,
                motion_revision=unsigned(progress.revision), motion_stamp_s=progress.stamp_s,
                motion_stamp_ns=unsigned(progress.stamp_ns), motion_receipt_s=progress.receipt_s,
                motion_upstream_age_s=progress.upstream_age_s, velocity=axes(progress.velocity),
                linear_lead_mps=progress.linear_lead_mps, angular_lead_radps=progress.angular_lead_radps,
                receipt_limit_s=progress.receipt_limit_s, header_limit_s=progress.header_limit_s,
                future_tolerance_s=progress.future_tolerance_s), allow_nan=False, separators=(",", ":"))
            if len(payload) <= 4096:
                rospy.logwarn("docking_motion_progress_infeasible %s", payload)
        except Exception:
            pass  # Diagnostics never alter the completed action or its cleanup.

    def _accept_handoff(self, controller_now_s):
        parking = self._entry.parking_geometry
        front_reference = parking["front_reference"]
        decision = build_handoff(
            samples=self._tags.sample_window(),
            requested_marker_id=self._entry.marker_id,
            expected_frame="base_link",
            front_reference_x_m=front_reference["x_m"],
            longitudinal_docking_offset_m=self._entry.docking_offset.get("longitudinal_m"),
            marker_size_m=self._entry.marker_size_m,
            parking_target=self._entry.parking_target,
            front_reference_state=front_reference["state"],
            operation_mode=self._operation_mode,
            parking_safety_distance_m=parking[
                "parking_safety_distance_m"
            ],
            max_blind_travel_m=parking["max_blind_travel_m"],
            absolute_max_blind_travel_m=parking[
                "absolute_max_blind_travel_m"
            ],
            now_s=controller_now_s,
            rover_settled=self._odometry.settled(),
            odom_fresh=self._odometry.fresh(),
            lidar_fresh=self._lidar.health()
            in (SafetyHealth.CLEAR, SafetyHealth.BLOCKED),
            lidar_forward_clear=self._lidar.forward_clear(),
            commissioning=self._commissioning,
            max_lateral_error_m=self._config.lateral_tolerance_m,
            max_yaw_error_rad=self._config.yaw_tolerance_rad,
        )
        self._handoff_rejection_reason = (
            None if decision.accepted else decision.reason
        )
        # A transiently rejected bundle authorizes no state transition and no
        # motion.  Remain in the controller-owned zero-command settling phase
        # and rebuild the bounded evidence window on the next control tick.
        if decision.accepted is not True:
            if not handoff_rejection_is_retryable(decision.reason):
                # The controller validates the rejected decision without
                # consuming its authorization, then latches a terminal zero.
                # The adapter restores the stable public rejection reason on
                # the next tick through public_docking_reason().
                self._controller.accept_handoff(
                    decision=decision,
                    controller_now_s=controller_now_s,
                    evidence_now_s=controller_now_s,
                )
            return
        self._controller.accept_handoff(
            decision=decision,
            controller_now_s=controller_now_s,
            evidence_now_s=controller_now_s,
        )
