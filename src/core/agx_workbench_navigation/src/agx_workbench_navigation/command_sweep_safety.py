"""Independent finite command/braking check against original filtered returns.

No planning or motion authority lives here. The ROS owner supplies measured base
pose/motion and original clocks; reading a capture cannot freshen it. This flat
travel check is separate from docking/departure's existing phase guards.
"""
import math
import time
import numpy as np

from .contracts import SafetyHealth, Velocity2D
from .lidar_snapshot import CapturePose, ObstacleCapture, snapshot_health
from .lidar_safety import CompactXYZ


def _rotation(pose):
    x, y, z, w = pose.orientation_xyzw
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def stopping_sweeps(measured, candidate, limits, latency):
    """Keep measured, requested and conservative transition sweeps separate."""
    if not isinstance(measured, Velocity2D) or not isinstance(candidate, Velocity2D):
        raise ValueError('measured and candidate planar motion required')
    vx, vy, wz = measured.linear_x, measured.linear_y, measured.angular_z
    cx, cw = candidate.linear_x, candidate.angular_z
    if (not math.isfinite(latency) or not 0 <= latency <= 2 or candidate.linear_y != 0 or
            not 0 <= cx <= limits['max_speed_mps']+1e-6 or
            abs(cw) > limits['max_yaw_rate_radps']+1e-6):
        raise ValueError('unsupported travel command or response budget')
    decel, yaw_decel = limits['deceleration_mps2'], limits['angular_acceleration_radps2']
    paths = []
    for x, y, w in ((vx, vy, wz), (cx, 0., cw),
                    (max(abs(vx), cx), vy, wz if abs(wz)>abs(cw) else cw)):
        linear, angular = math.hypot(x,y), abs(w)
        if linear > 2*limits['max_speed_mps'] or angular > 2*limits['max_yaw_rate_radps']:
            raise ValueError('measured motion exceeds sweep bound')
        duration = latency+max(linear/decel, angular/yaw_decel)
        dt = min(.02, .004/max(.001,linear+angular))
        if duration/dt > 5000:
            raise ValueError('sweep sample bound')
        px=py=yaw=t=0.; poses=[(0.,0.,0.)]
        while t < duration:
            step=min(dt,duration-t); braking=max(0.,t+step/2-latency)
            scale=max(0.,1-decel*braking/linear) if linear else 0.
            omega=math.copysign(max(0.,angular-yaw_decel*braking),w)
            heading=yaw+omega*step/2
            px+=(x*math.cos(heading)-y*math.sin(heading))*scale*step
            py+=(x*math.sin(heading)+y*math.cos(heading))*scale*step
            yaw+=omega*step; t+=step; poses.append((px,py,yaw))
        paths.append(np.asarray(poses))
    return paths


class CommandSweepGuard:
    def __init__(self, policy, policy_digest, filter_digest, *, source_max_age_s=.35,
                 receipt_max_age_s=.25, max_evaluation_s=.04, clock=time.monotonic):
        self.policy, self.policy_digest, self.filter_digest = policy, policy_digest, filter_digest
        self.source_max_age_s, self.receipt_max_age_s = source_max_age_s, receipt_max_age_s
        self.max_evaluation_s, self.clock = max_evaluation_s, clock
        self._capture=self._points=None; self._last=None
        self._health=SafetyHealth.INVALID
        self._vertices=np.asarray(policy.footprint_xy_m)
        area=sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(self._vertices,np.roll(self._vertices,-1,axis=0)))
        if area < 0:
            self._vertices=self._vertices[::-1]

    def invalidate(self):
        self._capture=self._points=None; self._health=SafetyHealth.INVALID

    def update(self, capture, health):
        self.invalidate()
        if (not isinstance(capture, ObstacleCapture) or type(health) is not SafetyHealth or
                health not in (SafetyHealth.CLEAR,SafetyHealth.BLOCKED) or
                capture.policy_digest != self.policy_digest or capture.filter_digest != self.filter_digest):
            return
        if self._last is not None:
            if (capture.cloud_stamp_s < self._last.cloud_stamp_s or
                    capture.receipt_steady_s < self._last.receipt_steady_s or
                    (capture.cloud_stamp_s == self._last.cloud_stamp_s and capture != self._last)):
                return
        # Keep XYZ until both original capture and current full transforms are
        # applied. A yaw-only relocation would misplace pitched sensor returns.
        points=(capture.points.xyz_m if isinstance(capture.points, CompactXYZ) else
                np.asarray([(p.x,p.y,p.z) for p in capture.points],dtype=float).reshape((-1,3)))
        self._points=points@_rotation(capture.capture_pose).T+np.asarray(capture.capture_pose.position_xyz_m)
        self._capture=self._last=capture; self._health=health

    def check(self, candidate, measured, current_pose, pose_source_stamp_s, pose_receipt_s,
              source_now_s, receipt_now_s):
        begin=self.clock()
        health=snapshot_health(self._capture,self._health,source_now_s,receipt_now_s,
                               self.source_max_age_s,self.receipt_max_age_s)
        if health not in (SafetyHealth.CLEAR,SafetyHealth.BLOCKED):
            return 'obstacle_capture_stale' if health==SafetyHealth.STALE else 'obstacle_capture_invalid'
        if (not isinstance(current_pose,CapturePose) or
                not all(math.isfinite(v) for v in (source_now_s,receipt_now_s,pose_source_stamp_s,pose_receipt_s)) or
                not 0 <= source_now_s-pose_source_stamp_s <= self.source_max_age_s or
                not 0 <= receipt_now_s-pose_receipt_s <= self.receipt_max_age_s):
            return 'current_pose_stale'
        try:
            paths=stopping_sweeps(measured,candidate,dict(self.policy.motion_limits),
                                  source_now_s-pose_source_stamp_s+.1)
        except (ValueError,OverflowError):
            return 'invalid_command_stopping_sweep'
        points=((self._points-np.asarray(current_pose.position_xyz_m))@_rotation(current_pose))[:,:2]
        # Body radius <=1 m; integration/interpolation adds at most 5 mm.
        padding=self.policy.hard_padding_m+.005
        for poses in paths:
            c,s=np.cos(poses[:,2]),np.sin(poses[:,2])
            vertices=self._vertices
            vx=poses[:,0,None]+c[:,None]*vertices[:,0]-s[:,None]*vertices[:,1]
            vy=poses[:,1,None]+s[:,None]*vertices[:,0]+c[:,None]*vertices[:,1]
            near=points[(points[:,0]>=vx.min()-padding)&(points[:,0]<=vx.max()+padding)&
                        (points[:,1]>=vy.min()-padding)&(points[:,1]<=vy.max()+padding)]
            for offset in range(0,len(poses),32):
                if len(near):
                    # Back-transform to the body at each sampled pose. Test the
                    # filled polygon and rounded edge padding, retaining single returns.
                    dx=near[:,0][None,:]-poses[offset:offset+32,0,None]
                    dy=near[:,1][None,:]-poses[offset:offset+32,1,None]
                    cc=c[offset:offset+32,None]; ss=s[offset:offset+32,None]
                    x,y=cc*dx+ss*dy,-ss*dx+cc*dy
                    inside=np.ones(x.shape,dtype=bool); close=np.zeros(x.shape,dtype=bool)
                    for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
                        ex,ey=b-a; rx,ry=x-a[0],y-a[1]
                        inside &= ex*ry-ey*rx >= -1e-12
                        u=np.clip((rx*ex+ry*ey)/(ex*ex+ey*ey),0.,1.)
                        close |= (rx-u*ex)**2+(ry-u*ey)**2 <= padding*padding
                    if np.any(inside|close):
                        return 'command_stopping_sweep_blocked'
                if self.clock()-begin > self.max_evaluation_s:
                    return 'command_sweep_computation_overrun'
        if self.clock()-begin > self.max_evaluation_s:
            return 'command_sweep_computation_overrun'
        return None
