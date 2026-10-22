#!/usr/bin/env python3
"""Isolated real move_base action regression; synthetic inputs, not robot proof.

Only the test process owns this graph's clock/TF/measurements. There is no driver
or command arbiter. The cancellation protocol explicitly separates action
PREEMPTED from an owned producer zero requested by same-binding replay; it is not
the mission's normal authority-revocation or physical stop confirmation path.
"""
import copy
import json
import hashlib
import math
import os
from pathlib import Path
import re
import threading
import time
import unittest
from urllib.parse import urlparse
import xml.etree.ElementTree as ET

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
import rospkg
import rospy
from rosgraph_msgs.msg import Clock
import rostest
from std_msgs.msg import String
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from agx_workbench_navigation.msg import (
    AuthorizedBaseCommand, BaseAuthorityState, CoarseNavigationState, LidarSafetyState, LidarObstacleSnapshot)
from agx_workbench_navigation.srv import (BindBaseCommandSource, BindBaseCommandSourceRequest,
    PlanNavigationRoute, PlanNavigationRouteRequest, ManageNavigationRoute, ManageNavigationRouteRequest)


NS = '/navigation_progress_action_test'
TICK_NS = 50_000_000                 # 20 Hz source clock; TF ingestion adds a wall-only fixture delay
WALL_TICK_S = .01


def quaternion(yaw):
    return (math.sin(yaw / 2), math.cos(yaw / 2))


def components(twist):
    return [twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z]


class NavigationProgressMoveBase(unittest.TestCase):
    def setUp(self):
        self.assertTrue(rospy.get_param('~synthetic_fixture', False))
        self.assertTrue(rospy.get_param('/use_sim_time', False))
        self.assertEqual(rospy.get_namespace().rstrip('/'), NS)
        self.assertIn(urlparse(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')).hostname,
                      ('localhost', '127.0.0.1', '::1'))
        package = Path(rospkg.RosPack().get_path('move_base')) / 'package.xml'
        self.assertEqual(ET.parse(str(package)).getroot().findtext('version'), '1.17.3')
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.source_ns = 100_000_000_000
        self.revision = 0
        self.operation = ''
        self.grant = ''
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.requested_rate = 0.0
        self.commands = []
        self.states = []
        self.plans = []
        self.diagnostics = []
        self.reports = []
        self.feedback_rows = []
        self.events = []
        self.pump_error = None
        self.clock = rospy.Publisher('/clock', Clock, queue_size=1)
        self.odom = rospy.Publisher(NS + '/odom', Odometry, queue_size=1)
        self.authority = rospy.Publisher(NS + '/authority_state', BaseAuthorityState, queue_size=1)
        self.lidar = rospy.Publisher(NS + '/lidar_safety_state', LidarSafetyState, queue_size=1)
        nav_package = Path(rospkg.RosPack().get_path('agx_workbench_navigation'))
        def digest(path): return 'sha256:' + hashlib.sha256(path.read_bytes()).hexdigest()
        self.policy_digest = digest(nav_package / 'test/fixtures/motion_policy.yaml')
        self.registry_digest = digest(nav_package / 'test/fixtures/registry.yaml')
        self.filter_digest = digest(nav_package / 'config/lidar_safety.yaml')
        self.pose_pub = rospy.Publisher(NS + '/amcl_pose', PoseWithCovarianceStamped, queue_size=1)
        self.cloud_pub = rospy.Publisher(NS + '/lidar_obstacle_snapshot', LidarObstacleSnapshot, queue_size=1)
        self.map_pub = rospy.Publisher(NS + '/map', OccupancyGrid, queue_size=1, latch=True)
        self.tf = tf2_ros.TransformBroadcaster()
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        self.subscribers = [
            rospy.Subscriber(NS + '/authorized_command', AuthorizedBaseCommand, self.command, queue_size=1000),
            rospy.Subscriber(NS + '/coarse_state', CoarseNavigationState, self.state, queue_size=1000),
            rospy.Subscriber(NS + '/move_base/GlobalPlanner/plan', RosPath, self.plan, queue_size=100),
            rospy.Subscriber(NS + '/diagnostic_cmd_vel', Twist, self.diagnostic, queue_size=1000),
            rospy.Subscriber(NS + '/effective/local_planner', String, self.report, queue_size=10),
        ]
        self.publish_static_world()
        self.thread = threading.Thread(target=self.pump, daemon=True)
        self.thread.start()
        self.addCleanup(self.cleanup)
        # A ROS-time timeout created at zero would expire on the first 100 s tick.
        self.wait_wall(lambda: rospy.Time.now().to_nsec() > 0, 5, 'initial source clock')
        self.client = actionlib.SimpleActionClient(NS + '/move_base', MoveBaseAction)
        self.assertTrue(self.client.wait_for_server(rospy.Duration(50)), 'move_base action unavailable')
        rospy.wait_for_service(NS + '/bind_source', timeout=10)
        self.bind = rospy.ServiceProxy(NS + '/bind_source', BindBaseCommandSource)
        self.wait_wall(lambda: bool(self.reports), 5, 'loaded planner report')
        report = self.reports[-1]['configuration']['navigation_progress']
        self.assertEqual(report['policy'], 'operation_pose_displacement')
        self.assertEqual(report['clock'], 'ros_source')
        self.assertEqual((report['timeout_s'], report['translation_m'], report['rotation_rad']), (45, .2, .1))
        self.assertEqual(report['stock_oscillation_timeout_requested_s'], 0)

    def cleanup(self):
        if hasattr(self, 'client'):
            self.client.cancel_goal()
        self.stop.set()
        self.thread.join(timeout=2)
        evidence = dict(synthetic_measurements=True, physical_stop_proof=False,
                        source_tick_s=TICK_NS / 1e9, wall_tick_s=WALL_TICK_S,
                        events=self.events, commands=self.commands, states=self.states,
                        global_plans=self.plans, diagnostic_cmd_vel=self.diagnostics,
                        action_feedback=self.feedback_rows, effective_reports=self.reports, pump_error=self.pump_error)
        path = Path(rospy.get_param('~evidence_file', '/tmp/navigation_progress_move_base_evidence.json'))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(evidence, indent=2, allow_nan=False) + '\n')

    def command(self, message):
        with self.lock:
            if message.twist.angular.z != 0:
                self.yaw_rate = self.requested_rate
            self.commands.append(dict(stamp=message.header.stamp.to_sec(), session=message.session_id,
                grant=message.grant_id, operation=message.source_goal_id,
                sequence=message.sequence, twist=components(message.twist)))

    def state(self, message):
        with self.lock:
            self.states.append(dict(stamp=message.header.stamp.to_sec(), state=message.state,
                                    reason=message.reason, operation=self.operation))

    def plan(self, message):
        if not message.poses:
            return
        with self.lock:
            endpoint = message.poses[-1]
            self.plans.append(dict(stamp=message.header.stamp.to_sec(),
                goal_stamp_ns=endpoint.header.stamp.to_nsec(), goal_frame=endpoint.header.frame_id,
                goal=[endpoint.pose.position.x, endpoint.pose.position.y,
                      endpoint.pose.orientation.z, endpoint.pose.orientation.w]))

    def diagnostic(self, message):
        with self.lock:
            self.diagnostics.append(dict(stamp=self.source_ns / 1e9, twist=components(message)))

    def report(self, message):
        with self.lock:
            self.reports.append(json.loads(message.data))

    def feedback(self, message):
        pose = message.base_position
        q = pose.pose.orientation
        with self.lock:
            self.feedback_rows.append(dict(operation=self.operation, stamp=pose.header.stamp.to_sec(),
                frame=pose.header.frame_id, x=pose.pose.position.x, y=pose.pose.position.y,
                yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))

    def publish_static_world(self):
        transforms = []
        for parent, child, x in [('map', 'odom', 0), ('rslidar', 'base_link', -.4)]:
            value = TransformStamped()
            value.header.frame_id = parent
            value.child_frame_id = child
            value.header.stamp = rospy.Time(100)
            value.transform.translation.x = x
            value.transform.rotation.w = 1
            transforms.append(value)
        self.static_tf.sendTransform(transforms)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = rospy.Time(100)
        grid.info.resolution = .1
        grid.info.width = grid.info.height = 100
        grid.info.origin.position.x = grid.info.origin.position.y = -5
        grid.info.origin.orientation.w = 1
        grid.data = [0] * 10000
        self.map_pub.publish(grid)

    def pump(self):
        try:
            while not self.stop.is_set() and not rospy.is_shutdown():
                with self.lock:
                    self.source_ns += TICK_NS
                    self.revision += 1
                    self.yaw += self.yaw_rate * TICK_NS / 1e9
                    stamp = rospy.Time(self.source_ns // 1_000_000_000,
                                       self.source_ns % 1_000_000_000)
                    yaw, rate, operation, grant = self.yaw, self.yaw_rate, self.operation, self.grant
                    revision = self.revision
                # Old evidence remains within its unchanged .25/.35 second
                # leases while this tick's exact-stamp TF and messages arrive.
                self.clock.publish(Clock(clock=rospy.Time((self.source_ns + TICK_NS) // 1_000_000_000,
                                                               (self.source_ns + TICK_NS) % 1_000_000_000)))
                transform = TransformStamped()
                transform.header.frame_id = 'odom'
                transform.child_frame_id = 'rslidar'
                transform.header.stamp = stamp
                transform.transform.translation.x = .4 * math.cos(yaw)
                transform.transform.translation.y = .4 * math.sin(yaw)
                transform.transform.rotation.z, transform.transform.rotation.w = quaternion(yaw)
                self.tf.sendTransform(transform)
                # Let each isolated listener ingest original-stamp TF before
                # publishing the measurement that requires that exact stamp.
                self.stop.wait(.012)
                odom = Odometry()
                odom.header = copy.deepcopy(transform.header)
                odom.child_frame_id = 'rslidar'
                odom.pose.pose.position.x = transform.transform.translation.x
                odom.pose.pose.position.y = transform.transform.translation.y
                odom.pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
                odom.twist.twist.linear.y = .4 * rate
                odom.twist.twist.angular.z = rate
                self.odom.publish(odom)
                pose = PoseWithCovarianceStamped()
                pose.header = Header(stamp=stamp, frame_id='map')
                pose.pose.pose.position = copy.deepcopy(odom.pose.pose.position)
                pose.pose.pose.orientation = copy.deepcopy(odom.pose.pose.orientation)
                self.pose_pub.publish(pose)
                cloud = LidarObstacleSnapshot()
                cloud.header = Header(stamp=stamp, frame_id='base_link')
                cloud.transform_stamp = stamp
                cloud.source_frame = 'rslidar'
                cloud.cloud_sequence = revision
                cloud.receipt_steady_s = time.monotonic()
                cloud.policy_digest, cloud.filter_digest = self.policy_digest, self.filter_digest
                cloud.capture_pose.header = Header(stamp=stamp, frame_id='odom')
                cloud.capture_pose.pose.orientation = copy.deepcopy(odom.pose.pose.orientation)
                cloud.state = cloud.CLEAR
                cloud.obstacle_points = PointCloud2(header=copy.deepcopy(cloud.header), height=1, width=0,
                    fields=[PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                            for i,k in enumerate(('x','y','z'))], point_step=12, row_step=0, data=b'', is_dense=True)
                self.cloud_pub.publish(cloud)
                state = BaseAuthorityState()
                state.header.stamp = stamp
                state.session_id = 'synthetic-session'
                state.grant_id, state.source_goal_id = grant, operation
                state.state = BaseAuthorityState.NAVIGATION if operation else BaseAuthorityState.NONE
                state.selected_source = 'navigation' if operation else ''
                state.motion_valid = True
                state.motion_revision = revision
                state.motion_stamp = stamp
                state.motion_frame_id = 'base_link'
                state.motion.angular.z = rate
                state.motion_receipt_age_s = 0
                state.motion_receipt_limit_s = .25
                state.motion_header_limit_s = .35
                state.motion_future_tolerance_s = .05
                state.linear_progress_lead_mps, state.angular_progress_lead_radps = .002, .004
                self.authority.publish(state)
                lidar = LidarSafetyState()
                lidar.header.stamp = stamp
                lidar.state = LidarSafetyState.CLEAR
                lidar.reason = 'isolated synthetic free-space fixture'
                self.lidar.publish(lidar)
                self.stop.wait(WALL_TICK_S)
        except Exception as error:
            self.pump_error = repr(error)
            self.stop.set()

    def now(self):
        with self.lock:
            return rospy.Time.now().to_sec()

    def wait_wall(self, predicate, timeout, detail):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assertIsNone(self.pump_error)
            if predicate():
                return
            time.sleep(.01)
        self.fail('wall timeout waiting for ' + detail)

    def start_goal(self, operation, goal_yaw, rate):
        with self.lock:
            self.operation, self.grant = operation, 'grant-' + operation
            self.yaw_rate = 0
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = quaternion(goal_yaw)
            request = BindBaseCommandSourceRequest(session_id='synthetic-session', grant_id=self.grant,
                source_goal_id=operation, expected_goal=copy.deepcopy(goal.target_pose))
        prepare = rospy.ServiceProxy(NS + '/move_base/RangerGlobalPlanner/plan_navigation_route', PlanNavigationRoute)
        manage = rospy.ServiceProxy(NS + '/move_base/RangerGlobalPlanner/manage_navigation_route', ManageNavigationRoute)
        prepare.wait_for_service(timeout=5)
        route_request = PlanNavigationRouteRequest(invocation_id=operation,
            model_digest=self.policy_digest, registry_digest=self.registry_digest)
        route_request.goal = copy.deepcopy(goal.target_pose)
        with self.lock:
            start = copy.deepcopy(goal.target_pose)
            start.pose.orientation.z, start.pose.orientation.w = quaternion(self.yaw)
        route_request.measured_start = route_request.coarse_start = start
        deadline = time.monotonic() + 5
        while True:
            candidate = prepare(route_request)
            if candidate.status == candidate.READY: break
            self.assertLess(time.monotonic(), deadline, candidate.detail)
            time.sleep(.02)
        request.invocation_id, request.route_id = operation, candidate.route.route_id
        request.task_deadline_s = time.monotonic() + 60
        pinned = manage(ManageNavigationRouteRequest(invocation_id=operation,
            route_id=request.route_id, task_deadline_s=request.task_deadline_s))
        self.assertTrue(pinned.accepted, pinned.reason)
        before = self.now()
        response = self.bind(request)
        self.assertTrue(response.accepted, response.reason)
        after = self.now()
        self.events.append(dict(event='bind', operation=operation, before=before, after=after,
                                goal_stamp_ns=goal.target_pose.header.stamp.to_nsec()))
        self.wait_wall(lambda: self.now() >= after + .15, 2, 'fresh post-binding measurement')
        with self.lock:
            self.requested_rate = rate
        self.client.send_goal(goal, feedback_cb=self.feedback)
        self.wait_wall(lambda: self.client.get_state() == GoalStatus.ACTIVE, 5, operation + ' ACTIVE')
        return request, before, after

    def operation_commands(self, operation):
        with self.lock:
            return [row for row in self.commands if row['operation'] == operation]

    def faults(self, operation):
        with self.lock:
            return [row for row in self.states if row['operation'] == operation and
                    row['reason'].startswith('navigation_progress_no_progress')]

    def test_real_action_turn_then_stall_across_replans(self):
        turn, turn_start, _ = self.start_goal('turn', -3, -.02)
        self.wait_wall(lambda: self.now() >= turn_start + 60, 40, '60 source seconds of point turning')
        self.assertEqual(self.client.get_state(), GoalStatus.ACTIVE)
        self.assertFalse(self.faults('turn'))
        turn_commands = self.operation_commands('turn')
        self.assertTrue(any(row['stamp'] >= turn_start + 46 and row['twist'][5] < 0
                            for row in turn_commands), 'no owned turning proposal after the old XY timeout')
        self.assertTrue(all(row['twist'][0] == 0 and row['twist'][1] == 0 for row in turn_commands))
        turn_feedback = [row for row in self.feedback_rows if row['operation'] == 'turn']
        self.assertGreater(len(turn_feedback), 100)
        self.assertTrue(all(row['frame'] == 'map' and math.hypot(row['x'], row['y']) < .2
                            for row in turn_feedback), 'real action feedback is not a point turn')
        self.assertLess(turn_feedback[-1]['yaw'] - turn_feedback[0]['yaw'], -1)
        self.events.append(dict(event='turn_alive_after_60_source_s', stamp=self.now(),
                                synthetic_base_xy=[0, 0], synthetic_yaw=self.yaw))

        cancel_requested_at = self.now()
        self.client.cancel_goal()
        self.wait_wall(lambda: self.client.get_state() == GoalStatus.PREEMPTED, 5, 'confirmed action PREEMPTED')
        cancel_at = self.now()
        # Action terminal, owned producer zero and synthetic stationary evidence
        # are separate assertions. Replay is not a new goal or timeout renewal.
        old_sequence = max(row['sequence'] for row in self.operation_commands('turn'))
        self.assertTrue(self.bind(turn).accepted)
        self.wait_wall(lambda: any(row['sequence'] > old_sequence and not any(row['twist'])
                                  for row in self.operation_commands('turn')), 2, 'owned same-binding zero')
        with self.lock:
            self.yaw_rate = 0
        self.wait_wall(lambda: self.now() >= cancel_at + .5, 2, 'synthetic stationary interval')
        self.assertTrue(any(not any(row['twist']) and row['stamp'] >= cancel_requested_at
                            for row in self.diagnostics), 'stock cancellation diagnostic zero absent')
        self.events.append(dict(event='cancelled_then_owned_zero_then_synthetic_stop', stamp=self.now(),
                                physical_stop_proof=False))

        # Tiny measured rotation starts the active follower, but remains well
        # below the progress threshold; it cannot renew the 45-second budget.
        manager = rospy.ServiceProxy(NS + '/move_base/RangerGlobalPlanner/manage_navigation_route', ManageNavigationRoute)
        released = manager(ManageNavigationRouteRequest(invocation_id=turn.invocation_id,
            route_id=turn.route_id, release=True))
        self.assertTrue(released.accepted, released.reason)
        stall, stall_start, stall_bound_after = self.start_goal('stall', 1.5, .0002)
        for elapsed in (15, 30, 44):
            self.wait_wall(lambda: self.now() >= stall_start + elapsed, 12, 'stall replay checkpoint')
            self.assertEqual(self.client.get_state(), GoalStatus.ACTIVE)
            self.assertFalse(self.faults('stall'))
            self.assertTrue(self.bind(stall).accepted)
        self.wait_wall(lambda: bool(self.faults('stall')), 5, 'finite guard failure')
        first = self.faults('stall')[0]
        fault_source = float(re.search(r' source=([^ ]+)', first['reason']).group(1))
        anchor_source = float(re.search(r' last_progress=([^ ]+)', first['reason']).group(1))
        self.assertGreaterEqual(anchor_source, stall_start - .05)
        self.assertLessEqual(anchor_source, stall_bound_after + .05)
        self.assertGreaterEqual(fault_source - anchor_source, 45)
        self.assertLessEqual(fault_source - anchor_source, 45.25)
        self.assertFalse(self.bind(stall).accepted)  # blocked route cannot be rebound
        self.wait_wall(lambda: self.client.get_state() == GoalStatus.ABORTED, 12, 'move_base controller-patience abort')
        aborted_at = self.now()
        self.assertLessEqual(aborted_at - fault_source, 15.5)
        self.wait_wall(lambda: self.now() >= aborted_at + .5, 2, 'post-abort outputs')
        commands = self.operation_commands('stall')
        self.assertTrue(any(any(row['twist']) and row['stamp'] < fault_source for row in commands),
                        'stalled action never requested motion')
        after = [row for row in commands if row['stamp'] >= fault_source]
        self.assertGreaterEqual(len(after), 1)  # owned zero before route rejection stops control callbacks
        self.assertTrue(all(not any(row['twist']) for row in after), 'nonzero proposal after terminal guard fault')
        self.assertTrue(all(row['session'] == 'synthetic-session' and row['grant'] == 'grant-stall'
                            for row in commands))
        self.assertTrue(all(b['sequence'] > a['sequence'] for a, b in zip(commands, commands[1:])))
        plans = [row for row in self.plans if row['goal_stamp_ns'] == stall.expected_goal.header.stamp.to_nsec()]
        self.assertGreaterEqual(len([row for row in plans if row['stamp'] < fault_source]), 10,
                                'actual global planner did not repeatedly replan during the stall')
        self.assertTrue(all(row['goal_frame'] == 'map' for row in plans))
        self.events.append(dict(event='stall_aborted', first_fault=first, fault_source=fault_source,
                                anchor_source=anchor_source, aborted_at=aborted_at,
                                matching_global_plan_count=len(plans), zero_after_fault_count=len(after)))


if __name__ == '__main__':
    rospy.init_node('navigation_progress_move_base_test')
    rostest.rosrun('agx_workbench_navigation', 'navigation_progress_move_base', NavigationProgressMoveBase)
