"""Read-only ROS boundary for the mission-owned installed route policy."""
from dataclasses import dataclass
import math
import threading
import time
import uuid

import rospy
from geometry_msgs.msg import PoseStamped

from .contracts import PlanarPose
from .departure import select_registered_departure
from .route_preflight import (InstalledRouteProfiles, RouteContext, RoutePreflight,
    RouteReply, RouteRequest, RouteStatus)
from .srv import (PlanNavigationRoute, PlanNavigationRouteRequest,
    ManageNavigationRoute, ManageNavigationRouteRequest,
    PrepareNavigationRoute, PrepareNavigationRouteResponse)
from .wall_runtime import call_with_wall_timeout


def _pose(pose):
    return PlanarPose(pose.x, pose.y, pose.theta)


def _stamped(pose):
    message = PoseStamped()
    message.header.frame_id = 'map'
    # Request time is not sensor freshness. The global owner supplies evidence.
    message.header.stamp = rospy.Time.now()
    message.pose.position.x, message.pose.position.y = pose.x_m, pose.y_m
    message.pose.orientation.z = math.sin(pose.yaw_rad/2.)
    message.pose.orientation.w = math.cos(pose.yaw_rad/2.)
    return message


@dataclass(frozen=True)
class _RouteView:
    route_id: str
    invocation_id: str
    model_digest: str
    registry_digest: str
    map_digest: str
    measured_start: PlanarPose
    coarse_start: PlanarPose
    goal: PlanarPose
    segments: tuple
    message: object
    departure_source_id: str = ''


class RoutePreflightPort:
    def __init__(self, registry, localization, max_reverse_distance_m):
        self._registry, self._localization = registry, localization
        self._max_reverse = max_reverse_distance_m
        self._rpc = threading.Lock()
        self._subject = self._installed = None
        self._unavailable = 'installed route profiles are not configured'
        self._planner = rospy.ServiceProxy('/move_base/RangerGlobalPlanner/plan_navigation_route', PlanNavigationRoute)
        self._manager = rospy.ServiceProxy('/move_base/RangerGlobalPlanner/manage_navigation_route', ManageNavigationRoute)
        try:
            self._installed = InstalledRouteProfiles(rospy.get_param('~route_profile_files', []),
                rospy.get_param('~motion_policy_file', ''), registry.mode)
            self._subject = RoutePreflight(self._installed.profiles, self._snapshot, self._plan, time.monotonic)
        except Exception as error:
            self._unavailable = 'installed route policy unavailable: {}'.format(error)
            rospy.logwarn('%s', self._unavailable)
        self._service = rospy.Service('/workbench_nav/prepare_navigation_route',
                                      PrepareNavigationRoute, self._prepare)

    def _snapshot(self, target):
        self._installed.assert_unchanged()
        entry, entries, registry_digest, map_digest = self._registry.route_binding(target)
        measured = self._localization.pose_map()
        departure = select_registered_departure(measured, entries, target, self._max_reverse)
        p, q = entry.staging_pose['position_m'], entry.staging_pose['orientation_xyzw']
        goal = PlanarPose(p[0], p[1], math.atan2(2*q[2]*q[3], 1-2*q[2]*q[2]))
        start = PlanarPose(measured.x_m, measured.y_m, measured.yaw_rad)
        coarse = None if departure is None else PlanarPose(
            start.x_m-departure.distance_m*math.cos(start.yaw_rad),
            start.y_m-departure.distance_m*math.sin(start.yaw_rad), start.yaw_rad)
        return RouteContext(start, goal, registry_digest, map_digest, departure is not None,
                            departure.source_id if departure else '', coarse)

    def _plan(self, request):
        message = PlanNavigationRouteRequest(invocation_id=request.invocation_id,
            existing_route_id=request.existing_route_id, model_digest=request.model_digest,
            registry_digest=request.registry_digest, measured_start=_stamped(request.measured_start),
            coarse_start=_stamped(request.coarse_start), goal=_stamped(request.goal),
            departure_source_id=request.departure_source_id)
        if not self._rpc.acquire(False):
            return RouteReply(RouteStatus.EVIDENCE_UNAVAILABLE, 'previous global route RPC is still outstanding')
        def call():
            try:
                return self._planner(message)
            finally:
                self._rpc.release()
        response = call_with_wall_timeout(call, 6.5)
        status = RouteStatus(response.status)
        if status != RouteStatus.READY:
            return RouteReply(status, response.detail)
        route = response.route
        return RouteReply(status, response.detail, _RouteView(route.route_id, route.invocation_id,
            route.model_digest, route.registry_digest, route.map_digest, _pose(route.measured_start),
            _pose(route.coarse_start), _pose(route.goal), tuple(route.segments), route, route.departure_source_id))

    @staticmethod
    def _response(reply):
        response = PrepareNavigationRouteResponse(status=int(reply.status), detail=reply.detail)
        if reply.status == RouteStatus.READY:
            response.route = reply.route.message
        return response

    def _prepare(self, request):
        if self._subject is None:
            return self._response(RouteReply(RouteStatus.EVIDENCE_UNAVAILABLE, self._unavailable))
        selected = RouteRequest(request.invocation_id, request.target, request.travel_profile_id,
            request.travel_profile_digest, request.existing_route_id, request.for_execution)
        return self._response(self._subject.prepare(selected, cancelled=rospy.is_shutdown))

    def admit_goal(self, goal, cancelled):
        """Also gate standalone callers before any motion-producing grant."""
        if self._subject is None:
            return RouteReply(RouteStatus.EVIDENCE_UNAVAILABLE, self._unavailable)
        if bool(goal.invocation_id) != bool(goal.route_id):
            return RouteReply(RouteStatus.ROUTE_INVALIDATED, 'incomplete action route identity')
        if goal.route_id:
            request = self._subject.execution_request(goal.invocation_id, goal.route_id, goal.workbench_id)
            if request is None:
                return RouteReply(RouteStatus.ROUTE_INVALIDATED, 'action has no owned candidate for this task/destination')
        else:
            # Installed default only; an action cannot supply a path/polygon.
            profile_id = rospy.get_param('~default_route_profile_id', next(iter(self._installed.profiles)))
            profile = self._installed.profiles.get(profile_id)
            if profile is None:
                return RouteReply(RouteStatus.EVIDENCE_UNAVAILABLE, 'default route profile is not installed')
            request = RouteRequest(uuid.uuid4().hex, goal.workbench_id, profile.profile_id,
                                   profile.profile_digest, for_execution=True)
        reply = self._subject.prepare(request, cancelled=cancelled)
        if reply.status == RouteStatus.READY:
            goal.invocation_id, goal.route_id = reply.route.invocation_id, reply.route.route_id
        return reply

    def activate(self, goal, cancelled):
        deadline = goal.task_deadline_s
        if (not math.isfinite(deadline) or deadline <= time.monotonic() or
                cancelled() or not goal.invocation_id or not goal.route_id):
            return False
        try:
            reply = call_with_wall_timeout(lambda: self._manager(ManageNavigationRouteRequest(
                invocation_id=goal.invocation_id, route_id=goal.route_id,
                task_deadline_s=deadline)), min(2., deadline-time.monotonic()))
            return (reply.accepted and not cancelled() and time.monotonic() < deadline and
                    self._subject.pin(goal.invocation_id, goal.route_id, deadline))
        except Exception:
            return False

    def validate_departure(self, goal, cancelled):
        request = self._subject.execution_request(goal.invocation_id, goal.route_id, goal.workbench_id)
        if request is None or cancelled() or time.monotonic() >= goal.task_deadline_s:
            return False
        return self._subject.prepare(request, cancelled).status == RouteStatus.READY

    def complete_departure(self, goal, cancelled):
        """Request verification; the native owner supplies all completion evidence."""
        if (self._subject.execution_request(goal.invocation_id, goal.route_id, goal.workbench_id) is None or
                cancelled() or time.monotonic() >= goal.task_deadline_s):
            return False
        try:
            reply = call_with_wall_timeout(lambda: self._manager(ManageNavigationRouteRequest(
                invocation_id=goal.invocation_id, route_id=goal.route_id, complete_departure=True)),
                min(2., goal.task_deadline_s-time.monotonic()))
            if not reply.accepted:
                rospy.logwarn('registered departure handover rejected: %s', reply.reason)
            return reply.accepted and not cancelled() and time.monotonic() < goal.task_deadline_s
        except Exception as error:
            rospy.logwarn('registered departure handover unavailable: %s', error)
            return False

    def release(self, goal):
        if not goal.route_id or not goal.invocation_id:
            return
        if self._subject is not None:
            self._subject.release(goal.invocation_id, goal.route_id)
        try:
            call_with_wall_timeout(lambda: self._manager(ManageNavigationRouteRequest(
                invocation_id=goal.invocation_id, route_id=goal.route_id, release=True)), 2.)
        except Exception:
            pass
