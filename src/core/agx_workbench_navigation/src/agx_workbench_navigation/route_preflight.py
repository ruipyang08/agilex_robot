"""Mission-owned, read-only route admission (ROS1/Python 3.8 compatible).

Only the installed profile table, registry/localization owner and global planner
are inputs. There is deliberately no command, authority, arm or lift port.
Native planning owns collision, source freshness and the bounded route store.
This layer binds its result to the selected task/profile/registered destination.
"""
from dataclasses import dataclass, replace
from enum import IntEnum
import math
import threading
from hashlib import sha256
from pathlib import Path

import yaml

from .contracts import ActionResultCode, PlanarPose
from .motion_policy import parse_motion_policy
from .registry import _UniqueKeyLoader


class RouteStatus(IntEnum):
    READY = 0
    NO_ROUTE = 1
    SEARCH_LIMIT = 2
    EVIDENCE_UNAVAILABLE = 3
    START_MISMATCH = 4
    ROUTE_INVALIDATED = 5
    INVALID_REQUEST = 6


def route_failure_code(status):
    return {
        RouteStatus.NO_ROUTE: ActionResultCode.ROUTE_UNAVAILABLE,
        RouteStatus.SEARCH_LIMIT: ActionResultCode.ROUTE_SEARCH_LIMIT,
        RouteStatus.ROUTE_INVALIDATED: ActionResultCode.ROUTE_INVALIDATED,
        RouteStatus.START_MISMATCH: ActionResultCode.ROUTE_START_MISMATCH,
    }.get(status, ActionResultCode.ROUTE_EVIDENCE_UNAVAILABLE)


def _id(value):
    return isinstance(value, str) and bool(value.strip()) and len(value) <= 128


def _same_pose(first, second):
    # PoseStamped's quaternion round trip is not bit-exact at arbitrary yaw.
    # This is serialization tolerance (nanometres/radians), not tracking slack.
    return (isinstance(first, PlanarPose) and isinstance(second, PlanarPose) and
            math.hypot(first.x_m-second.x_m, first.y_m-second.y_m) <= 1e-9 and
            abs(math.atan2(math.sin(first.yaw_rad-second.yaw_rad),
                           math.cos(first.yaw_rad-second.yaw_rad))) <= 1e-9)


@dataclass(frozen=True)
class InstalledRouteProfile:
    profile_id: str
    profile_digest: str
    model_digest: str
    execution_enabled: bool = False

    def __post_init__(self):
        if not all(_id(value) for value in (self.profile_id, self.profile_digest, self.model_digest)):
            raise ValueError('installed route profile identities required')
        if type(self.execution_enabled) is not bool:
            raise ValueError('execution_enabled must be an installed boolean')


@dataclass(frozen=True)
class RouteContext:
    measured_start: PlanarPose
    staging_goal: PlanarPose
    registry_digest: str
    map_digest: str
    departure_required: bool = False
    departure_source_id: str = ''
    coarse_start: PlanarPose = None

    def __post_init__(self):
        if not isinstance(self.measured_start, PlanarPose) or not isinstance(self.staging_goal, PlanarPose):
            raise ValueError('owner measured/registered planar poses required')
        if not _id(self.registry_digest) or not _id(self.map_digest):
            raise ValueError('installed map and registry identities required')
        if type(self.departure_required) is not bool:
            raise ValueError('departure_required must be an owner boolean')


@dataclass(frozen=True)
class RouteRequest:
    invocation_id: str
    target: str
    travel_profile_id: str
    travel_profile_digest: str
    existing_route_id: str = ''
    for_execution: bool = False


@dataclass(frozen=True)
class PlannerRequest:
    invocation_id: str
    existing_route_id: str
    model_digest: str
    registry_digest: str
    measured_start: PlanarPose
    coarse_start: PlanarPose
    goal: PlanarPose
    departure_source_id: str = ''


@dataclass(frozen=True)
class RouteReply:
    status: RouteStatus
    detail: str
    route: object = None

    def __post_init__(self):
        if type(self.status) is not RouteStatus:
            raise ValueError('typed route status required')
        if self.status != RouteStatus.READY and self.route is not None:
            raise ValueError('failed planning cannot carry a route')


@dataclass(frozen=True)
class _Pending:
    request: RouteRequest
    context: RouteContext
    route: object
    expires_s: float
    pinned: bool = False


class RoutePreflight:
    def __init__(self, profiles, snapshot, plan, steady_clock):
        self._profiles = dict(profiles)
        if not self._profiles or any(not isinstance(p, InstalledRouteProfile) or key != p.profile_id
                                     for key, p in self._profiles.items()):
            raise ValueError('nonempty installed profile table required')
        self._snapshot, self._plan, self._clock = snapshot, plan, steady_clock
        self._worker = threading.Lock()
        self._pending = {}
        self._last_now = -1.

    def _now(self):
        now = self._clock()
        if (isinstance(now, bool) or not isinstance(now, (int, float)) or
                not math.isfinite(now) or now < 0 or now < self._last_now):
            self._pending.clear()
            raise ValueError('steady clock regressed or unavailable')
        self._last_now = now
        self._pending = {key: value for key, value in self._pending.items() if now < value.expires_s}
        return now

    @staticmethod
    def _same_installation(first, current):
        return (first.registry_digest == current.registry_digest and first.map_digest == current.map_digest and
                _same_pose(first.staging_goal, current.staging_goal) and first.departure_required == current.departure_required and
                first.departure_source_id == current.departure_source_id)

    def prepare(self, request, cancelled=lambda: False):
        if not self._worker.acquire(False):
            return RouteReply(RouteStatus.EVIDENCE_UNAVAILABLE, 'route worker is busy')
        try:
            return self._prepare(request, cancelled)
        except Exception as error:
            # Owner/transport failures cannot supply an executable route.
            if isinstance(request, RouteRequest):
                pending = self._pending.get(request.existing_route_id)
                if pending and pending.request.invocation_id == request.invocation_id:
                    self._pending.pop(request.existing_route_id, None)
            return RouteReply(RouteStatus.EVIDENCE_UNAVAILABLE, 'route evidence unavailable: {}'.format(error))
        finally:
            self._worker.release()

    def _prepare(self, request, cancelled):
        def fail(status, detail):
            if isinstance(request, RouteRequest) and request.existing_route_id:
                pending = self._pending.get(request.existing_route_id)
                # A foreign invocation cannot erase another owner's candidate.
                if pending and pending.request.invocation_id == request.invocation_id:
                    self._pending.pop(request.existing_route_id, None)
            return RouteReply(status, detail)

        self._now()
        if cancelled():
            return fail(RouteStatus.ROUTE_INVALIDATED, 'preflight cancelled')
        if (not isinstance(request, RouteRequest) or
                not all(_id(v) for v in (request.invocation_id, request.target,
                                        request.travel_profile_id, request.travel_profile_digest)) or
                not isinstance(request.existing_route_id, str) or len(request.existing_route_id) > 128 or
                type(request.for_execution) is not bool):
            return fail(RouteStatus.INVALID_REQUEST, 'invalid task/profile identity')
        profile = self._profiles.get(request.travel_profile_id)
        if profile is None or profile.profile_digest != request.travel_profile_digest:
            return fail(RouteStatus.INVALID_REQUEST, 'travel profile is not the installed admitted profile')
        if request.for_execution and not profile.execution_enabled:
            return fail(RouteStatus.EVIDENCE_UNAVAILABLE, 'route execution is disabled; read-only preview remains available')
        context = self._snapshot(request.target)
        if not isinstance(context, RouteContext):
            return fail(RouteStatus.EVIDENCE_UNAVAILABLE, 'invalid owner snapshot')
        if context.departure_required and (not _id(context.departure_source_id) or
                not isinstance(context.coarse_start, PlanarPose)):
            return fail(RouteStatus.EVIDENCE_UNAVAILABLE, 'registered departure phase admission is not connected')
        if not context.departure_required and (context.departure_source_id or context.coarse_start is not None):
            return fail(RouteStatus.EVIDENCE_UNAVAILABLE, 'unexpected departure identity for an open-space start')
        pending = None
        if request.existing_route_id:
            pending = self._pending.get(request.existing_route_id)
            if (pending is None or replace(request, existing_route_id='', for_execution=False) != pending.request or
                    not self._same_installation(pending.context, context)):
                return fail(RouteStatus.ROUTE_INVALIDATED, 'candidate expired or task/profile/installation changed')
        elif len(self._pending) >= 8:
            return fail(RouteStatus.EVIDENCE_UNAVAILABLE, 'pending route store is full')
        start = pending.route.measured_start if pending else context.measured_start
        coarse = pending.route.coarse_start if pending else (context.coarse_start or start)
        planned = self._plan(PlannerRequest(request.invocation_id, request.existing_route_id,
            profile.model_digest, context.registry_digest, start, coarse, context.staging_goal,
            context.departure_source_id))
        self._now()
        if cancelled():
            return fail(RouteStatus.ROUTE_INVALIDATED, 'preflight cancelled before retaining result')
        if not isinstance(planned, RouteReply):
            return fail(RouteStatus.EVIDENCE_UNAVAILABLE, 'invalid global planner response')
        if planned.status != RouteStatus.READY:
            return fail(planned.status, planned.detail)
        route = planned.route
        if (route is None or not _id(route.route_id) or route.invocation_id != request.invocation_id or
                route.model_digest != profile.model_digest or route.registry_digest != context.registry_digest or
                route.map_digest != context.map_digest or not _same_pose(route.goal, context.staging_goal) or
                not _same_pose(route.measured_start, start) or not _same_pose(route.coarse_start, coarse) or not route.segments or
                len(route.segments) > 20000 or any(not s.samples for s in route.segments) or
                sum(len(s.samples) for s in route.segments) > 20000 or
                (pending and route.route_id != request.existing_route_id)):
            return fail(RouteStatus.ROUTE_INVALIDATED, 'global route does not match the admitted request')
        if (context.departure_required and (getattr(route, 'departure_source_id', '') != context.departure_source_id or
                getattr(route.segments[0], 'kind', None) != 2)):
            return fail(RouteStatus.ROUTE_INVALIDATED, 'global departure source does not match the registered request')
        current = self._snapshot(request.target)
        now = self._now()
        if not self._same_installation(context, current):
            return fail(RouteStatus.ROUTE_INVALIDATED, 'installation changed during planning')
        if cancelled():
            return fail(RouteStatus.ROUTE_INVALIDATED, 'preflight cancelled before retaining result')
        if pending and request.existing_route_id not in self._pending:
            return fail(RouteStatus.ROUTE_INVALIDATED, 'candidate lease expired during validation')
        if not pending:
            if route.route_id in self._pending:
                return fail(RouteStatus.ROUTE_INVALIDATED, 'global route token was reused')
            self._pending[route.route_id] = _Pending(replace(request, for_execution=False), context, route, now+60.)
        return planned

    def pin(self, invocation_id, route_id, deadline):
        """Retain the candidate only after native pinning; never renew a deadline."""
        if not self._worker.acquire(False):
            return False
        try:
            now = self._now()
            pending = self._pending.get(route_id)
            if (pending is None or pending.request.invocation_id != invocation_id or
                    isinstance(deadline, bool) or not isinstance(deadline, (int, float)) or
                    not math.isfinite(deadline) or not now < deadline <= now+86400):
                return False
            if pending.pinned:
                return deadline == pending.expires_s
            self._pending[route_id] = replace(pending, expires_s=deadline, pinned=True)
            return True
        finally:
            self._worker.release()

    def release(self, invocation_id, route_id):
        with self._worker:
            pending = self._pending.get(route_id)
            if pending and pending.request.invocation_id == invocation_id:
                self._pending.pop(route_id)

    def execution_request(self, invocation_id, route_id, target):
        """Resolve action metadata from the owned candidate, never caller paths."""
        if not self._worker.acquire(False):
            return None
        try:
            self._now()
            pending = self._pending.get(route_id)
            if (pending is None or pending.request.invocation_id != invocation_id or
                    pending.request.target != target):
                return None
            return replace(pending.request, existing_route_id=route_id, for_execution=True)
        finally:
            self._worker.release()


class InstalledRouteProfiles:
    """Freeze trusted launch files; a service request cannot add or replace one.

    Source-byte digests match the ROS1 task's TravelProfile.config_id. The
    runtime model and offline preview use the exact installed policy bytes.
    The parser's semantic digest is not a runtime route identity, and an offline
    result never grants a live route token or motion authority.
    """
    def __init__(self, profile_files, policy_file, mode):
        if (mode not in ('simulation', 'hardware') or not isinstance(profile_files, (tuple, list)) or
                not profile_files or len(profile_files) > 8):
            raise ValueError('one to eight installed profiles and explicit mode required')
        self._sources = {}
        def load(path):
            path = Path(path).resolve()
            raw = self._bytes(path)
            digest = 'sha256:'+sha256(raw).hexdigest()
            self._sources[path] = digest
            return yaml.load(raw.decode('utf-8'), Loader=_UniqueKeyLoader), digest
        raw_policy, self.model_digest = load(policy_file)
        self.policy = parse_motion_policy(raw_policy)
        if self.policy.execution_enabled:
            self.policy.require_execution(mode)
        self.profiles = {}
        limits = dict(self.policy.motion_limits)
        for path in profile_files:
            raw, digest = load(path)
            if not isinstance(raw, dict) or type(raw.get('schema_version')) is not int or raw['schema_version'] != 1:
                raise ValueError('unsupported installed travel profile')
            qualification = raw.get(mode)
            if not isinstance(qualification, dict) or qualification.get('enabled') is not True or not qualification.get('evidence_id'):
                raise ValueError('installed travel profile is not qualified for '+mode)
            profile = InstalledRouteProfile(raw.get('profile_id'), digest, self.model_digest,
                                            self.policy.execution_enabled)
            if profile.profile_id in self.profiles:
                raise ValueError('duplicate installed travel profile ID')
            base = raw['parameters']['base']
            footprint = tuple((p['x_m'], p['y_m']) for p in base['footprint'])
            expected = self.policy.footprint_xy_m
            # A polygon's starting vertex and winding do not change its area.
            # Preserve cyclic edges; set equality would also accept a bow tie.
            equivalents = [points[i:]+points[:i] for points in (expected, tuple(reversed(expected)))
                           for i in range(len(points))]
            if footprint not in equivalents:
                raise ValueError('installed travel footprint does not match the selected route model')
            for policy_limit, profile_limit in (
                ('max_speed_mps', 'linear_velocity_mps'), ('max_yaw_rate_radps', 'angular_velocity_rad_s'),
                ('acceleration_mps2', 'linear_acceleration_mps2'),
                ('angular_acceleration_radps2', 'angular_acceleration_rad_s2')):
                value = base[profile_limit]
                if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                        not math.isfinite(value) or limits[policy_limit] > value):
                    raise ValueError('route motion exceeds the installed travel envelope')
            self.profiles[profile.profile_id] = profile
        self.assert_unchanged()

    def assert_unchanged(self):
        for path, digest in self._sources.items():
            if 'sha256:'+sha256(self._bytes(path)).hexdigest() != digest:
                raise ValueError('installed route configuration changed: '+str(path))

    @staticmethod
    def _bytes(path):
        with path.open('rb') as stream:
            raw = stream.read(1024*1024+1)
        if not 0 < len(raw) <= 1024*1024:
            raise ValueError('installed configuration exceeds size bound')
        return raw
