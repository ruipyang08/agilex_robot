#!/usr/bin/env python3
"""Real mission service, installed profile loading and transport; no robot graph."""
import hashlib
from pathlib import Path
import threading
import tempfile
import time
from types import SimpleNamespace
import unittest

import rospkg
import rospy
import rostest
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from agx_workbench_navigation.contracts import OdomPose2D
from agx_workbench_navigation.msg import (NavigateToWorkbenchGoal,
    PlannedMotionSegment, PlannedMotionSample)
from agx_workbench_navigation.ros_route_preflight import RoutePreflightPort
from agx_workbench_navigation.route_preflight import RouteStatus
from agx_workbench_navigation.srv import (PlanNavigationRoute, PlanNavigationRouteResponse,
    PrepareNavigationRoute, PrepareNavigationRouteRequest)


class RouteMissionPreflight(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(rospkg.RosPack().get_path('agx_workbench_navigation')).parents[3]
        cls.profile = root/'config/isaacsim/travel_profiles/original_default_empty.yaml'
        # Preserve the simulation model/profile identity while disabling motion
        # in this isolated fixture. Never modify the installed active policy.
        cls.policy_dir = tempfile.TemporaryDirectory(prefix='route-mission-policy-')
        policy = Path(cls.policy_dir.name)/'policy.yaml'
        source = (root/'config/isaacsim/navigation_motion_policy.yaml').read_text()
        assert 'execution_enabled: true' in source
        policy.write_text(source.replace('execution_enabled: true', 'execution_enabled: false').replace('qualification: experimental_simulation', 'qualification: unqualified'))
        rospy.set_param('~route_profile_files', [str(cls.profile)])
        rospy.set_param('~motion_policy_file', str(policy))
        cls.output_count = 0
        cls.outputs = [rospy.Subscriber('/cmd_vel', Twist, cls.output),
            rospy.Subscriber('/workbench_nav/command_operation', String, cls.output)]
        cls.native = rospy.Service('/move_base/RangerGlobalPlanner/plan_navigation_route',
                                   PlanNavigationRoute, cls.plan)
        cls.entry = SimpleNamespace(workbench_id='wb-100', map_pose={'position_m': (2, 0, 0)},
            staging_pose={'position_m': (1, 0, 0), 'orientation_xyzw': (0, 0, 0, 1)})
        cls.registry = SimpleNamespace(mode='simulation', route_binding=cls.context)
        cls.localization = SimpleNamespace(pose_map=lambda: OdomPose2D(0, 0, 0))
        cls.port = RoutePreflightPort(cls.registry, cls.localization, 2.)
        cls.client = rospy.ServiceProxy('/workbench_nav/prepare_navigation_route', PrepareNavigationRoute)
        cls.reset()

    @classmethod
    def tearDownClass(cls):
        cls.native.shutdown()
        cls.policy_dir.cleanup()

    @classmethod
    def reset(cls):
        cls.map_digest, cls.registry_digest = 'map-a', 'registry-a'
        cls.plan_count = 0
        cls.entered, cls.release = threading.Event(), threading.Event()
        cls.release.set()

    def setUp(self):
        self.reset()

    @classmethod
    def output(cls, _message):
        cls.output_count += 1

    @classmethod
    def context(cls, target):
        if target != 'wb-100':
            raise ValueError('unknown destination')
        return cls.entry, (cls.entry,), cls.registry_digest, cls.map_digest

    @classmethod
    def plan(cls, request):
        cls.plan_count += 1; cls.entered.set()
        if not cls.release.wait(3):
            raise rospy.ServiceException('test planner wait expired')
        reply = PlanNavigationRouteResponse(status=PlanNavigationRouteResponse.READY,
                                           detail='isolated read-only global planner')
        route = reply.route
        route.header.frame_id = 'map'; route.header.stamp = rospy.Time.now()
        route.invocation_id = request.invocation_id
        route.route_id = request.existing_route_id or 'route-'+request.invocation_id
        route.model_digest = request.model_digest
        route.registry_digest = request.registry_digest; route.map_digest = cls.map_digest
        route.departure_source_id = request.departure_source_id
        for source, dest in ((request.measured_start, route.measured_start),
                             (request.coarse_start, route.coarse_start), (request.goal, route.goal)):
            dest.x, dest.y = source.pose.position.x, source.pose.position.y
            dest.theta = 0
        segment = PlannedMotionSegment()
        if request.departure_source_id:
            segment.kind = PlannedMotionSegment.DOCK_DEPARTURE
        segment.samples = [PlannedMotionSample(), PlannedMotionSample()]
        route.segments = [segment]
        return reply

    def request(self, invocation):
        return PrepareNavigationRouteRequest(invocation_id=invocation, target='wb-100',
            travel_profile_id='ranger_xarm6_original_default_empty_sim_v1',
            travel_profile_digest='sha256:'+hashlib.sha256(self.profile.read_bytes()).hexdigest())

    def test_registered_departure_reaches_read_only_native_boundary(self):
        source = SimpleNamespace(workbench_id='source', map_pose={
            'position_m': (.36, 0, 0), 'orientation_xyzw': (0, 0, 0, 1)},
            staging_pose={'position_m': (-1, 0, 0)})
        original = self.port._registry
        self.port._registry = SimpleNamespace(mode='simulation', route_binding=lambda target:
            (self.entry, (self.entry, source), self.registry_digest, self.map_digest))
        try:
            reply = self.client(self.request('departure-preview'))
            self.assertEqual(reply.status, reply.READY, reply.detail)
            self.assertEqual(reply.route.departure_source_id, 'source')
            self.assertEqual(reply.route.coarse_start.x, -1)
            self.assertEqual(reply.route.segments[0].kind, PlannedMotionSegment.DOCK_DEPARTURE)
            self.assertEqual(self.output_count, 0)
        finally:
            self.port._registry = original

    def test_read_only_round_trip_and_pre_release_revalidation(self):
        request = self.request('round-trip')
        reply = self.client(request)
        self.assertEqual(reply.status, reply.READY, reply.detail)
        self.assertEqual(reply.route.invocation_id, request.invocation_id)
        request.existing_route_id = reply.route.route_id
        revalidated = self.client(request)
        self.assertEqual(revalidated.status, revalidated.READY, revalidated.detail)
        self.assertEqual(revalidated.route.route_id, reply.route.route_id)
        self.assertEqual(self.output_count, 0)

    def test_disabled_execution_and_standalone_action_never_plan_or_grant(self):
        request = self.request('execution-disabled'); request.for_execution = True
        reply = self.client(request)
        self.assertEqual(reply.status, reply.EVIDENCE_UNAVAILABLE)
        self.assertIn('disabled', reply.detail)
        standalone = self.port.admit_goal(NavigateToWorkbenchGoal(workbench_id='wb-100'), lambda: False)
        self.assertEqual(standalone.status, RouteStatus.EVIDENCE_UNAVAILABLE)
        self.assertEqual(self.plan_count, 0)
        self.assertEqual(self.output_count, 0)

    def test_foreign_token_and_changed_registration_fail_before_global_rpc(self):
        request = self.request('owned')
        reply = self.client(request)
        request.existing_route_id = reply.route.route_id
        request.invocation_id = 'foreign'
        self.assertEqual(self.client(request).status, reply.ROUTE_INVALIDATED)
        request.invocation_id = 'owned'
        type(self).registry_digest = 'registry-b'
        self.assertEqual(self.client(request).status, reply.ROUTE_INVALIDATED)
        self.assertEqual(self.plan_count, 1)

    def test_concurrent_request_receives_busy_without_starting_another_rpc(self):
        self.release.clear(); replies = []
        worker = threading.Thread(target=lambda: replies.append(self.client(self.request('busy-one'))))
        worker.start()
        try:
            self.assertTrue(self.entered.wait(2))
            other = rospy.ServiceProxy('/workbench_nav/prepare_navigation_route', PrepareNavigationRoute)
            began = time.monotonic(); reply = other(self.request('busy-two'))
            self.assertEqual(reply.status, reply.EVIDENCE_UNAVAILABLE)
            self.assertLess(time.monotonic()-began, .5)
            self.assertEqual(self.plan_count, 1)
        finally:
            self.release.set(); worker.join(3)
        self.assertEqual(replies[0].status, replies[0].READY)
        self.assertEqual(self.output_count, 0)


if __name__ == '__main__':
    rospy.init_node('route_mission_preflight_ros')
    rostest.rosrun('agx_workbench_navigation', 'route_mission_preflight', RouteMissionPreflight)
