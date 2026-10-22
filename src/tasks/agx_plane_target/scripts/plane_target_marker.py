#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Put points on the taught M6 locations, plus grasp / home / stop services.

The previous joint-slider console and the draggable red dot are retired.
This node erases that dot (interactive marker name "target") on startup so
an already-open RViz drops it, and does not insert it again. Cancel a motion
with rosservice call /plane_target/stop. The arm stays powered and holds its
last commanded point, so the physical e-stop remains the primary safety
device.

PUT (2026-10-14, decisions.md D22): six taught put points (measurements.md
M6, expressed in the static frame link_base_lift0) are drawn as six red
spheres with number labels on plane_target/put_points, and plane_target/put
places a held object on one of them in ONE atomic action: move to the point's
approach point 150 mm above it (free plan), straight-line descent onto the
point (Cartesian), open the gripper, straight-line lift back, then plan back
to the SRDF initial pose. The terminal interface is `put <1..6>` (scripts/put),
which follows plane_target/put_status.

Marker frame is deck_top (static; the dot does NOT ride the lift height).
Planning numbers (deck_top frame): rectangle x [0.106, 0.565],
y [-0.25, 0.25], plane z = 0.100.

Parameters:
  ~vel_scaling    (default 0.2)  MoveIt velocity scaling factor
  ~planning_time  (default 10.0) MoveIt allowed planning time [s]
  ~plane_z        (default 0.100) dot height above deck_top [m]
  ~keep_in_zone   (default true)  keep the end effector inside the yellow
                      safety box for the WHOLE motion (MoveIt path
                      constraint, not just the endpoint)
  ~zone_margin    (default 0.0)   shrink the box by this margin [m] on all
                      six sides. 0 = exactly the yellow box. NOTE the dot is
                      only 2 mm above the box floor, so a margin > 0.002
                      makes most dots unreachable.
  ~zone_links     (default "tcp_link") comma-separated links to keep inside;
                      add link6/link5 for a stricter sweep limit
  ~home_named_state (default "initial") SRDF named state for plane_target/home
  ~rpy_deg        (default "180,0,0") desired TCP orientation (deg). 180,0,0
                  = tcp +Z (the AG95 approach axis) pointing straight down.
                  Re-read on EVERY GO, so it can be tuned without a restart:
                  rosparam set /plane_target_marker/rpy_deg "180,0,90"
  ~enable_arm     (default true)  true = plan AND execute on the real arm;
                       false = dry-run (plan only, no motion). Read once at
                       startup: change it via launch arg / _enable_arm:=...
                       and restart the node.

Put-only parameters, read fresh on EVERY plane_target/put call:
  ~put_cartesian_step (default 0.01) interpolation step [m] of the two
                       straight-line put legs (MoveIt compute_cartesian_path)

Put service: plane_target/put takes the point number in the request
(agx_plane_target/PutPoint, int32 point 1..6). A param + std_srvs/Trigger
would leave a window where another motion can slip in between "set the
number" and "call the service", so the number travels with the call.

Grasp-only parameters, all read fresh on EVERY plane_target/grasp call so the
console can retune them live:
  ~grasp_position     (default 0)   closing target 0..1000. 0 = close as far
                      as the force allows, i.e. the fingers stop on the object
                      whatever its width; raise it to stop the fingers earlier
                      (fragile or already-gripped items).
  ~grasp_force        (default 50)   grip force 20..100 (AG95 range)
  ~grasp_speed        (default 50)   finger speed
  ~grasp_preopen      (default true) open the gripper first, so a closed
                      gripper cannot knock the target while moving
  ~grasp_settle_timeout (default 3.0)  give up waiting after this many seconds
  ~grasp_settle_tol   (default 2)    counts (of 1000) counted as "not moving"
  ~grasp_settle_samples (default 5)  consecutive still readings that mean 到位

Services:
  plane_target/stop   std_srvs/Trigger, cancel the current motion
  plane_target/go     std_srvs/Trigger, same as a short-click on the dot
  plane_target/home   std_srvs/Trigger, plan+execute back to the SRDF named
                      state ~home_named_state (default "initial") with the
                      same keep-in-zone constraint
  plane_target/reset  std_srvs/Trigger, move the dot back to the rectangle centre
  plane_target/grasp  std_srvs/Trigger, full pick as ONE atomic action:
                      (optionally open) -> MoveIt onto the dot -> close the
                      gripper to ~grasp_position -> report the reading the
                      gripper actually stopped at. ~grasp_position is read on
                      every call, so the console sets it with rosparam right
                      before calling and no restart is needed.
  plane_target/put    agx_plane_target/PutPoint, full PUT as ONE atomic action
                      for the point number in the request (1..6, the taught M6
                      table): close the gripper to 200, plan+execute to the
                      approach point 150 mm above the point, straight-line
                      descent onto it, open the gripper to 1000, straight-line
                      lift back up, then plan+execute back to the SRDF named
                      state ~home_named_state. Called by the terminal CLI
                      scripts/put (`put <1..6>`).
  plane_target/pick   agx_plane_target/PickPoint, full PICK as ONE atomic
                      action on the same six points: open the gripper to 1000 and
                      require that reading before the arm moves, then
                      plan+execute to the point's safety height (the same
                      150 mm approach), straight-line descent, close the
                      gripper to 200, straight-line lift, then back to the
                      SRDF named state. Called by scripts/pick (`pick <1..6>`).

Topics:
  plane_target/target        latched PoseStamped, current dot pose in deck_top
  plane_target/nudge         geometry_msgs/Vector3, x/y = relative step [m];
                             lets the console panel (joint_slider_gui) walk the
                             dot with its up/down/left/right buttons
  plane_target/grasp_status  latched std_msgs/String, "<state>|<text>" with
                             state in busy/ok/fail. Progress feedback for the
                             console 夹取 row; a plain string on purpose so the
                             start layer needs no new message dependency and a
                             future state machine can just subscribe.
  plane_target/put_status    latched std_msgs/String, same "<state>|<text>"
                             format, but for plane_target/put only. A SEPARATE
                             topic on purpose: reusing grasp_status would make
                             the console 夹取 row report put progress as if it
                             were a grasp. scripts/put follows this topic.
  plane_target/pick_status   latched std_msgs/String, same format, for
                             plane_target/pick only. scripts/pick follows it.
                             Not grasp_status and not put_status.
  plane_target/put_points    latched visualization_msgs/MarkerArray: the six
                             taught put points as red spheres (ns "put_points",
                             id = point number, labels id 100+n) in
                             link_base_lift0. Display only; nothing listens.

Mutual exclusion: the group controller is shared with the joint slider GUI
and admittance teach; do not operate them while a motion is running. A grasp
holds that reservation for its WHOLE duration (move + close), a put holds
it for its whole five steps (approach, descent, open, lift, home), and a
pick holds it for its whole five steps (approach, descent, close, lift,
home), so a GO/home cannot slip in between two halves.
"""

import math
import sys
import threading
import time

import rospy
import tf
from agx_plane_target.srv import (
    PickPoint,
    PickPointResponse,
    PutPoint,
    PutPointResponse,
)
from dh_gripper_msgs.msg import GripperCtrl, GripperState
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler, quaternion_matrix
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    InteractiveMarkerUpdate,
    Marker,
    MarkerArray,
)

MARKER_FRAME = "deck_top"
TARGET_NS = "plane_target"
STOP_SERVICE = "plane_target/stop"
GO_SERVICE = "plane_target/go"
HOME_SERVICE = "plane_target/home"
RESET_SERVICE = "plane_target/reset"
GRASP_SERVICE = "plane_target/grasp"
PUT_SERVICE = "plane_target/put"
PICK_SERVICE = "plane_target/pick"
TARGET_TOPIC = "plane_target/target"
NUDGE_TOPIC = "plane_target/nudge"
GRASP_STATUS_TOPIC = "plane_target/grasp_status"
PUT_STATUS_TOPIC = "plane_target/put_status"
PICK_STATUS_TOPIC = "plane_target/pick_status"
PUT_MARKER_TOPIC = "plane_target/put_points"
GRIPPER_CTRL_TOPIC = "/gripper/ctrl"
GRIPPER_STATE_TOPIC = "/gripper/states"   # PLURAL, see doc/interface.md

# DH AG95 position convention, same as the console slider: 1000 = fully open,
# 0 = fully closed. Force and speed are the values the console has always
# sent; they stay parameters so they can be tuned without touching code.
GRASP_OPEN = 1000
# Close target shared by put (before the arm leaves) and pick (after the
# straight descent). 200 is partly closed (not crushed shut); put's release
# after arrival is GRASP_OPEN.
PUT_GRIP_CLOSE = 200
# A pick may leave the initial pose only after the fingers have actually
# opened. 1000 is fully open; a reading still below this after an open
# command means the gripper is closed or still holding something, so the
# arm stays put.
PICK_OPEN_MIN = 980
# A close that settles here or higher never left the open range. 941 was
# reported as "closed to 200" and the arm then tried to lift. Do not lift.
PICK_STILL_OPEN = 900
DEFAULT_GRASP_FORCE = 50
DEFAULT_GRASP_SPEED = 50
# "position stopped changing" tolerance, in position counts out of 1000
GRASP_SETTLE_TOL = 2

# Safety rectangle, same numbers as safety_plane_viz.py PLANE.
PLANE_X = (0.106, 0.565)
PLANE_Y = (-0.25, 0.25)
PLANE_Z = 0.100

DOT_INIT = (0.3355, 0.0)   # rectangle centre
DOT_SIZE = 0.045
# 鼠标抓取用的加大手柄：红球只有 45mm，正常视距下几乎点不中
GRAB_SIZE = 0.100
HINT_TEXT_SIZE = 0.028
HINT_TEXT = "拖动移动 · 短按执行 · 控制台方向键也行"

# 黄色暂定安全区的数字**不再在本文件写死**：单一真源是
# src/start/agx_xarm_bringup/scripts/safety_plane_viz.py 的 PLANE，它启动时把盒子
# 写进参数 /safety_plane/box，这里每次运动都重读（见 _zone_box）。
# 分层方向 tasks -> start 是 AGENTS 允许的，反向禁止，所以数字走 param server 而不是
# 让 start import tasks。
ZONE_PARAM = "/safety_plane/box"
# 兜底常量：只在 safety_plane_viz 没起（或参数被删）时用，必须与它保持一致。
# 底面 0.028 / 顶面 1.298（deck_top 系）。红点在 z=0.100，比底面高 72 mm。
FALLBACK_ZONE_SIZE = (0.459, 0.50, 1.270)
FALLBACK_ZONE_CENTER = (0.3355, 0.0, 0.663)
# 只约束末端（tcp_link）。臂基座本身在盒子外面（link_base 在 x=0，盒子从 x=0.106 起），
# 所以不可能把整条臂都塞进盒子；link6/link5 可以按需要加进 ~zone_links。
DEFAULT_ZONE_LINKS = "tcp_link"

# ---------------------------------------------------------------------------
# 放入点（put points）：2026-10-14 示教，记录在 tasks/doc/measurements.md 的
# M6。坐标记在**静止系 `link_base_lift0`**（lift=0 的底座快照），因此不随升降
# 移动；每次执行放入前用当时的 TF 变到当前的 `link_base` 再交给 MoveIt（规划
# 参考系是 link_base）。点号 1~6 与 M6 表格从上到下一致。
# 下面的数字是代码里唯一一份；M6 是实测记录。重新示教时先改 M6，再同步这里。
# ---------------------------------------------------------------------------
PUT_FRAME = "link_base_lift0"
PUT_MARKER_NS = "put_points"
# 上方接近点 = 放入点沿 +Z 抬高 0.150 m（安全高度；下降和抬高共用这一段）。
PUT_APPROACH_RISE = 0.150
# 记录姿态（sxyz、度），六个点相同。工具轴 tcp +Z 指向 link_base_lift0 的 -Z。
# ⚠ quaternion_from_euler 吃弧度（WORKLOG 坑 18）：直接喂度不报错、只是安静地
# 算出另一个合法姿态，所以这里的 math.radians 不能省。
PUT_RPY_DEG = (180.00, 0.00, 1.32)
PUT_QUAT = quaternion_from_euler(*[math.radians(v) for v in PUT_RPY_DEG])
PUT_BALL_SIZE = 0.035        # 红球直径 [m]
PUT_LABEL_SIZE = 0.026       # 点号文字高度 [m]
PUT_BALL_RGBA = (1.0, 0.08, 0.08, 0.85)
PUT_CARTESIAN_STEP = 0.01    # compute_cartesian_path 的 eef_step [m]
PUT_POINTS = {
    2: ("中心", 0.329030, 0.007922, -0.078002),
    3: ("绿轴 +75 mm", 0.330758, -0.067058, -0.078002),
    1: ("绿轴 -75 mm", 0.327302, 0.082902, -0.078002),
    5: ("中心，红轴 -75 mm", 0.254050, 0.006194, -0.078003),
    6: ("绿轴 +75 mm，红轴 -75 mm", 0.255778, -0.068786, -0.078003),
    4: ("绿轴 -75 mm，红轴 -75 mm", 0.252322, 0.081174, -0.078003),
}

CLICK_TIME_S = 0.5
CLICK_MOVE_M = 0.005

# Markers dropped from this file stay in a running RViz forever (RViz never
# times out the interactive markers of a stopped node). Right after startup
# we repeat an erase for them during this window, so already-open RViz
# windows clean themselves up without a restart. "target" is the previous
# draggable red dot; "stop" is the retired stop cube.
RETIRED_MARKERS = ("stop", "target")
RETIRE_ERASE_GRACE_S = 60.0

COLOR_IDLE = (1.0, 0.10, 0.10)   # red
COLOR_MOVING = (1.0, 0.75, 0.0)  # yellow
COLOR_OK = (0.15, 0.85, 0.25)    # green flash on success
COLOR_FAIL = (1.0, 0.45, 0.0)    # orange flash on failure
FLASH_S = 2.0


def _axes_of(q):
    """Axes of a frame with orientation q=(x,y,z,w), expressed in the parent
    frame: returns (X, Y, Z) unit vectors. Z is the tool/approach axis."""
    m = quaternion_matrix(q)
    return tuple((m[0][i], m[1][i], m[2][i]) for i in (0, 1, 2))


def _tilt_deg(axis):
    """Angle between a tool axis and straight down (0, 0, -1) in the
    (z-up) base frame. 0 deg = tool pointing down."""
    dot = max(-1.0, min(1.0, -axis[2]))
    return math.degrees(math.acos(dot))


def _inside_bounds(bounds, pos):
    """Is `pos` inside the ((xmin,xmax),(ymin,ymax),(zmin,zmax)) box?"""
    return all(bounds[i][0] <= pos[i] <= bounds[i][1] for i in range(3))


def _grow_bounds(bounds, pos):
    """Grow the axis-aligned box minimally so it contains `pos`."""
    for i in range(3):
        bounds[i][0] = min(bounds[i][0], pos[i])
        bounds[i][1] = max(bounds[i][1], pos[i])


class PlaneTargetMarker(object):
    def __init__(self):
        self.server = InteractiveMarkerServer(TARGET_NS)
        self.tf_listener = tf.TransformListener()

        self.dot_xy = DOT_INIT
        self.lock = threading.RLock()
        self.busy = False
        self.motion_thread = None
        self._group = None
        # Set by _stop() (console 急停 / plane_target/stop). The AG95 has no
        # cancel, so this is what lets an abort also freeze a closing grasp.
        self._stop_flag = threading.Event()

        # gripper feedback used for the "did it stop moving yet" check
        self._grip_pos = 0
        self._grip_prev = None
        self._grip_still = 0
        self._grip_stamp = None
        self._grip_tol = GRASP_SETTLE_TOL

        self._down = None  # (rospy wall time, xy) of the last MOUSE_DOWN

        self.vel_scaling = rospy.get_param("~vel_scaling", 0.2)
        self.planning_time = rospy.get_param("~planning_time", 10.0)
        self.plane_z = rospy.get_param("~plane_z", PLANE_Z)
        self.enable_arm = rospy.get_param("~enable_arm", True)
        # workspace keep-in constraint (re-read on every motion, see _zone)
        self.zone = self._read_zone_params()
        # goal orientation; re-read on every GO (see _goal_quat) so it can be
        # tuned live with `rosparam set /plane_target_marker/rpy_deg ...`
        self.target_quat, self.rpy_deg = self._goal_quat()

        self.target_pub = rospy.Publisher(
            TARGET_TOPIC, PoseStamped, queue_size=1, latch=True)
        rospy.Service(STOP_SERVICE, Trigger, self.on_stop_service)
        rospy.Service(GO_SERVICE, Trigger, self.on_go_service)
        rospy.Service(HOME_SERVICE, Trigger, self.on_home_service)
        rospy.Service(RESET_SERVICE, Trigger, self.on_reset_service)
        rospy.Service(GRASP_SERVICE, Trigger, self.on_grasp_service)
        rospy.Service(PUT_SERVICE, PutPoint, self.on_put_service)
        rospy.Service(PICK_SERVICE, PickPoint, self.on_pick_service)
        # console panel arrow buttons: relative steps on the plane
        self.nudge_sub = rospy.Subscriber(
            NUDGE_TOPIC, Vector3, self.on_nudge, queue_size=4)
        self.grasp_status_pub = rospy.Publisher(
            GRASP_STATUS_TOPIC, String, queue_size=5, latch=True)
        # put progress goes on its OWN topic: the console 夹取 row subscribes
        # to grasp_status only, so a put can never make it show 夹取 progress.
        self.put_status_pub = rospy.Publisher(
            PUT_STATUS_TOPIC, String, queue_size=5, latch=True)
        self.pick_status_pub = rospy.Publisher(
            PICK_STATUS_TOPIC, String, queue_size=5, latch=True)
        self.put_markers_pub = rospy.Publisher(
            PUT_MARKER_TOPIC, MarkerArray, queue_size=1, latch=True)
        self.grip_ctrl_pub = rospy.Publisher(
            GRIPPER_CTRL_TOPIC, GripperCtrl, queue_size=1)
        self.grip_sub = rospy.Subscriber(
            GRIPPER_STATE_TOPIC, GripperState, self._gripper_cb, queue_size=1)

        self._insert_all()
        self._publish_put_markers()
        if self.enable_arm:
            rospy.logwarn("[%s] enable_arm=true: GO will MOVE the real arm. "
                          "Clear the area and keep the physical e-stop ready.",
                          TARGET_NS)
        else:
            rospy.logwarn("[%s] enable_arm=false: dry-run, GO plans only and "
                          "never moves the arm", TARGET_NS)
        rospy.loginfo(
            "[%s] 可拖动红点和关节控制台已去掉；取消动作用 "
            "rosservice call /plane_target/stop", TARGET_NS)
        grasp = self._grasp_params()
        rospy.logwarn(
            "[%s] %s closes the real gripper: closing target %d (0 = close as "
            "far as force %d allows), pre-open=%s%s", TARGET_NS, GRASP_SERVICE,
            grasp["position"], grasp["force"],
            "on" if grasp["preopen"] else "off",
            "" if self.enable_arm else ", but enable_arm=false so 干跑：不会闭合")
        axis = _axes_of(self.target_quat)[2]
        rospy.loginfo(
            "[%s] goal orientation rpy=(%.1f, %.1f, %.1f) deg -> tcp +Z "
            "(tool axis) = (%+.2f, %+.2f, %+.2f) in link_base, %.1f deg off "
            "vertical-down%s", TARGET_NS, self.rpy_deg[0], self.rpy_deg[1],
            self.rpy_deg[2], axis[0], axis[1], axis[2], _tilt_deg(axis),
            "  <== NOT pointing down, check ~rpy_deg" if _tilt_deg(axis) > 5.0
            else "")
        enabled, margin, links, size, center, source = self.zone
        rospy.loginfo(
            "[%s] workspace keep-in: %s (%s), box from %s, bottom z=%.3f "
            "top z=%.3f%s", TARGET_NS, "ON" if enabled else "OFF",
            "/".join(links), source, center[2] - size[2] / 2.0,
            center[2] + size[2] / 2.0,
            "" if margin == 0.0 else ", shrunk %.0f mm" % (margin * 1000.0))
        rospy.loginfo(
            "[%s] %s: 放入点 1~6（M6 示教值，坐标 %s）；%d 个编号红球发到 /%s；"
            "终端用法 put <1~6>（scripts/put，自己订阅 %s 显示进度）",
            TARGET_NS, PUT_SERVICE, PUT_FRAME, len(PUT_POINTS), PUT_MARKER_TOPIC,
            PUT_STATUS_TOPIC)
        rospy.logwarn(
            "[%s] %s %s", TARGET_NS, PUT_SERVICE,
            "会真实运动：先闭合到 %d -> 上方点 -> 直线下降 150 mm -> "
            "松开 -> 直线抬高 -> 回 initial" % PUT_GRIP_CLOSE
            if self.enable_arm else
            "干跑：只规划到上方点这一步，绝不会闭合/下降/松开/抬高/回家")
        rospy.loginfo(
            "[%s] %s: 抓取点与放入相同的 1~6；终端用法 pick <1~6>"
            "（scripts/pick，自己订阅 %s 显示进度）",
            TARGET_NS, PICK_SERVICE, PICK_STATUS_TOPIC)
        rospy.logwarn(
            "[%s] %s %s", TARGET_NS, PICK_SERVICE,
            "会真实运动：先确认张开到 %d -> 上方安全高度 -> 直线下降 -> "
            "闭合到 %d -> 直线抬高 -> 回 initial"
            % (GRASP_OPEN, PUT_GRIP_CLOSE)
            if self.enable_arm else
            "干跑：只规划到上方点这一步，绝不会下降/闭合/抬高/回家")

    # ------------------------------------------------------------------ #
    # goal orientation
    # ------------------------------------------------------------------ #
    def _goal_quat(self):
        """Read ~rpy_deg (roll,pitch,yaw in DEGREES, tcp_link orientation
        expressed in link_base) -> (quaternion, rpy_deg).

        180,0,0 = tcp +Z (the AG95 approach axis) pointing straight down.
        Read on every GO, so it can be tuned live:
        rosparam set /plane_target_marker/rpy_deg "180,0,90"

        ⚠ quaternion_from_euler() takes RADIANS. Feeding it degrees silently
        yields a valid but wrong orientation (180 deg was being applied as
        180 rad = 233.3 deg, i.e. 53 deg off vertical-down), so the deg->rad
        conversion is mandatory. The self-check below guards against a
        regression here.
        """
        raw = rospy.get_param("~rpy_deg", "180,0,0")
        try:
            rpy = [float(v) for v in str(raw).split(",")]
            if len(rpy) != 3:
                raise ValueError("need 3 values, got %r" % (raw,))
            return quaternion_from_euler(*[math.radians(v) for v in rpy]), rpy
        except Exception as exc:
            rospy.logwarn("[%s] bad ~rpy_deg=%r (%s), falling back to 180,0,0",
                          TARGET_NS, raw, exc)
            return quaternion_from_euler(math.pi, 0.0, 0.0), [180.0, 0.0, 0.0]

    # ------------------------------------------------------------------ #
    # workspace keep-in constraint (yellow safety box)
    # ------------------------------------------------------------------ #
    def _zone_box(self):
        """(size, center, source) of the yellow safety box.

        The numbers live in exactly one place -- safety_plane_viz.py writes them
        to /safety_plane/box at startup -- and are re-read on every motion, so
        editing PLANE there changes the enforced box after a restart with no
        second copy to keep in sync. Falls back to the local constants (with a
        throttled warning) when the viz node is not running."""
        try:
            d = rospy.get_param(ZONE_PARAM)
            size = [float(v) for v in d["size"]]
            center = [float(v) for v in d["center"]]
            if len(size) == 3 and len(center) == 3:
                return size, center, ZONE_PARAM
            raise ValueError("need 3 size + 3 center values, got %r" % (d,))
        except Exception as exc:
            rospy.logwarn_throttle(
                60.0,
                "[%s] 读不到 %s (%s)，退回内置兜底盒子（若 safety_plane_viz "
                "没起就是这个情况；黄盒数字只以它为准）", TARGET_NS, ZONE_PARAM,
                exc)
            return (list(FALLBACK_ZONE_SIZE), list(FALLBACK_ZONE_CENTER),
                    "built-in fallback")

    def _read_zone_params(self):
        """(enabled, margin_m, [links], size, center, source) read live, so it
        can be tuned with rosparam without restarting the node."""
        enabled = bool(rospy.get_param("~keep_in_zone", True))
        margin = float(rospy.get_param("~zone_margin", 0.0))
        links = [s.strip() for s in
                 str(rospy.get_param("~zone_links", DEFAULT_ZONE_LINKS)).split(",")
                 if s.strip()]
        if not links:
            links = ["tcp_link"]
        size, center, source = self._zone_box()
        return enabled, max(0.0, margin), links, size, center, source

    def _zone_constraints(self, links, bounds):
        """Constraints keeping each link's origin inside the yellow box
        (deck_top frame). A BOX primitive in constraint_region means "this
        point must stay inside this box"; MoveIt re-checks path constraints at
        every sampled state, so it limits the whole sweep, not the endpoint."""
        from moveit_msgs.msg import Constraints, PositionConstraint
        from shape_msgs.msg import SolidPrimitive

        size = [max(0.01, bounds[i][1] - bounds[i][0]) for i in range(3)]
        center = [(bounds[i][0] + bounds[i][1]) / 2.0 for i in range(3)]
        cons = Constraints()
        cons.name = "plane_target_zone"
        for link in links:
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = size
            box = Pose()
            box.position.x = center[0]
            box.position.y = center[1]
            box.position.z = center[2]
            box.orientation.w = 1.0
            c = PositionConstraint()
            c.header.frame_id = MARKER_FRAME
            c.link_name = link
            c.constraint_region.primitives.append(prim)
            c.constraint_region.primitive_poses.append(box)
            c.weight = 1.0
            cons.position_constraints.append(c)
        return cons

    def _zone_bounds(self, size, center, margin, links, goal_point=None,
                     floor=None):
        """Axis-aligned bounds (deck_top) actually used, as
        ((xmin, xmax), (ymin, ymax), (zmin, zmax)).

        A MoveIt path constraint also applies to the START state, so if the arm
        is parked outside the yellow box every plan would fail with a
        confusing timeout. In that case the box is grown just enough to contain
        the current end-effector positions (logged as a warning: the guarantee
        is weaker until the arm is back inside).

        `goal_point` and `floor` are extras used by the PLANNED put legs only
        (see _apply_zone / decisions.md D22):
        - goal_point: one (x, y, z) in deck_top (the put approach point). The
          taught put points may sit outside the yellow box (M6), and a goal the
          box does not contain can never be planned, so the box is grown the
          same way as for a start state outside (with its own warning).
        - floor: raise the box bottom to at least this z (deck_top) so the put
          legs never descend back to the put height (M6 plan requirement).

        Returns ((bounds), [(link, pos) outside], [goal grown around],
        floor_raised)."""
        bounds = [[center[i] - size[i] / 2.0 + margin,
                   center[i] + size[i] / 2.0 - margin] for i in range(3)]
        outside = []
        for link in links:
            try:
                pos, _rot = self._lookup_in(MARKER_FRAME, link)
            except Exception:
                continue
            if not _inside_bounds(bounds, pos):
                outside.append((link, pos))
        for _link, pos in outside:
            _grow_bounds(bounds, pos)
        grown = []
        if goal_point is not None and not _inside_bounds(bounds, goal_point):
            grown.append(tuple(goal_point))
            _grow_bounds(bounds, goal_point)
        raised = False
        if floor is not None and floor > bounds[2][0]:
            bounds[2][0] = floor
            raised = True
        return bounds, outside, grown, raised

    def _lookup_in(self, frame, link):
        """(position, orientation) of `link` expressed in `frame`."""
        res = self.tf_listener.lookupTransform(frame, link, rospy.Time(0))
        if len(res) == 2:
            return tuple(res[0]), tuple(res[1])
        flat = tuple(res)
        return flat[0:3], flat[3:7]

    def _apply_zone(self, group, goal_point=None, floor=None):
        """Install (or drop) the keep-in constraint on the MoveGroup. Called on
        every motion so a param change takes effect on the next click.

        `goal_point` / `floor` are the put-leg extras described in
        _zone_bounds. They are used by the two PLANNED put legs
        (initial <-> approach point); the two straight-line vertical legs
        deliberately run without any path constraint (_execute_cartesian),
        because a constrained straight descent onto a taught point that may sit
        outside the yellow box would fail to plan (M6 / D22)."""
        self.zone = self._read_zone_params()
        enabled, margin, links, size, center, source = self.zone
        if not enabled and floor is None:
            group.clear_path_constraints()
            rospy.logwarn(
                "[%s] keep_in_zone=false: 轨迹不受黄色安全区约束（自己看着办）",
                TARGET_NS)
            return
        if enabled:
            bounds, outside, grown, raised = self._zone_bounds(
                size, center, margin, links, goal_point, floor)
            group.set_path_constraints(self._zone_constraints(links, bounds))
            for link, pos in outside:
                rospy.logwarn(
                    "[%s] %s 当前 (%.3f, %.3f, %.3f) 在黄区外，安全区已临时扩大"
                    "把它包进来以免规划直接失败；把手臂收回黄区内可恢复严格约束",
                    TARGET_NS, link, pos[0], pos[1], pos[2])
            for pos in grown:
                rospy.logwarn(
                    "[%s] 目标 (%.3f, %.3f, %.3f) 在黄区外（放入点可能本来就"
                    "在黄区外，见 M6），安全区已临时扩大包住目标",
                    TARGET_NS, pos[0], pos[1], pos[2])
            rospy.loginfo(
                "[%s] 安全区约束生效：%s 全程必须留在 %s 系 x[%.3f,%.3f] "
                "y[%.3f,%.3f] z[%.3f,%.3f] 内（盒子来自 %s）%s%s", TARGET_NS,
                "/".join(links), MARKER_FRAME, bounds[0][0], bounds[0][1],
                bounds[1][0], bounds[1][1], bounds[2][0], bounds[2][1], source,
                "，黄区内缩 %.0f mm" % (margin * 1000.0) if margin > 0 else "",
                "，底面抬到放入点 z=%.3f（不回到放入高度以下）" % bounds[2][0]
                if raised else "")
        else:
            # keep_in_zone=false, but this leg still carries the put floor:
            # a bare half-space box (unbounded in xy) keeps the "never back
            # down to the put height" guarantee without re-enabling the zone.
            group.set_path_constraints(
                self._zone_constraints(links, [[-1.0, 1.0], [-1.0, 1.0],
                                               [floor, 2.0]]))
            rospy.logwarn(
                "[%s] keep_in_zone=false，但放入腿仍加下限：%s z >= %.3f"
                "（不回到放入高度以下；xy 不受约束）", TARGET_NS, MARKER_FRAME,
                floor)

    # ------------------------------------------------------------------ #
    # marker construction
    # ------------------------------------------------------------------ #
    def _insert_all(self):
        # Do not insert the draggable red dot. Publish erases for it and the
        # old stop cube so an already-open RViz drops them. server.erase()
        # only works for markers this process inserted, so the update is
        # published directly. A client that has not connected yet misses one
        # shot, so repeat it for a while; RViz ignores unknown names.
        # server.publish() fills in server_id/seq_num.
        self._retire_until = rospy.Time.now() + rospy.Duration(
            RETIRE_ERASE_GRACE_S)
        self.retire_timer = rospy.Timer(rospy.Duration(1.0),
                                       self._publish_retire_erase)

    def _publish_retire_erase(self, _event=None):
        try:
            if rospy.Time.now() > self._retire_until:
                self.retire_timer.shutdown()
                return
            retired = InteractiveMarkerUpdate()
            retired.erases.extend(RETIRED_MARKERS)
            self.server.publish(retired)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "[%s] retire erase failed: %s",
                                  TARGET_NS, exc)

    def _plane_pose(self, x, y, offset_z=0.0):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = self.plane_z + offset_z
        pose.orientation.w = 1.0
        return pose

    # ------------------------------------------------------------------ #
    # put points: markers (six red balls) + pose helpers
    # ------------------------------------------------------------------ #
    def _publish_put_markers(self, action=Marker.ADD):
        """Latched MarkerArray with the six taught put points (M6): one red
        sphere per point (id = point number) plus a small number label
        (id = 100 + point number) so `put <n>` maps to a visible ball.

        Plain markers, NOT interactive ones: the points are taught values, and
        the interactive server is shared with the red dot (adding drag handles
        here would fight the dot's picking)."""
        arr = MarkerArray()
        # Stamp 0: these balls are fixed in link_base_lift0. A real stamp
        # freezes at publish time, and a RViz opened later drops the latched
        # message once that time falls out of the TF buffer, so the balls
        # vanish even though the topic still has them.
        stamp = rospy.Time()
        for number in sorted(PUT_POINTS):
            name, x, y, z = PUT_POINTS[number]
            ball = Marker()
            ball.header.frame_id = PUT_FRAME
            ball.header.stamp = stamp
            ball.ns = PUT_MARKER_NS
            ball.id = number
            ball.type = Marker.SPHERE
            ball.action = action
            ball.pose.position.x = x
            ball.pose.position.y = y
            ball.pose.position.z = z
            ball.pose.orientation.w = 1.0
            ball.scale.x = ball.scale.y = ball.scale.z = PUT_BALL_SIZE
            ball.color.r, ball.color.g, ball.color.b, ball.color.a = \
                PUT_BALL_RGBA
            ball.lifetime = rospy.Duration(0)
            arr.markers.append(ball)

            label = Marker()
            label.header.frame_id = PUT_FRAME
            label.header.stamp = stamp
            label.ns = PUT_MARKER_NS
            label.id = 100 + number
            label.type = Marker.TEXT_VIEW_FACING
            label.action = action
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + PUT_BALL_SIZE / 2.0 + PUT_LABEL_SIZE * 0.7
            label.pose.orientation.w = 1.0
            label.scale.z = PUT_LABEL_SIZE
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.9
            label.text = str(number)
            label.lifetime = rospy.Duration(0)
            arr.markers.append(label)
        self.put_markers_pub.publish(arr)

    def _put_pose(self, x, y, z):
        """Pose in PUT_FRAME (link_base_lift0) at the recorded M6 orientation."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = PUT_QUAT[0]
        pose.orientation.y = PUT_QUAT[1]
        pose.orientation.z = PUT_QUAT[2]
        pose.orientation.w = PUT_QUAT[3]
        return pose

    def _pose_in(self, frame, pose, source_frame):
        """`pose` (expressed in source_frame) transformed into `frame` through
        the live TF. Raises if the transform does not appear in 3 s."""
        ps = PoseStamped()
        ps.header.frame_id = source_frame
        ps.pose = pose
        self.tf_listener.waitForTransform(
            frame, source_frame, rospy.Time(0), rospy.Duration(3.0))
        return self.tf_listener.transformPose(frame, ps).pose

    def _dot_marker(self, rgb):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = MARKER_FRAME
        int_marker.name = "target"
        int_marker.description = ("drag to pick / short-click to go")
        int_marker.pose = self._plane_pose(*self.dot_xy)

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = DOT_SIZE
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = rgb + (1.0,)

        control = InteractiveMarkerControl()
        control.name = "drag_plane"
        # identity orientation -> plane normal = deck_top +z (drag on xy)
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # without this RViz only draws the control while the mouse hovers it,
        # i.e. the dot would look like it does not exist
        control.always_visible = True
        control.markers.append(sphere)
        int_marker.controls.append(control)

        # 抓取手柄：一个明显更大的半透明球，专供鼠标抓。正常视距下 45mm 的红球
        # 只有几个像素，很难点中。两个 control 都是 MOVE_PLANE（**不要**再叠一个
        # BUTTON：RViz 拾取顺序不确定，拖动和点击会互相抢，见 WORKLOG 坑）。
        grab = Marker()
        grab.type = Marker.SPHERE
        grab.scale.x = grab.scale.y = grab.scale.z = GRAB_SIZE
        grab.color.r, grab.color.g, grab.color.b, grab.color.a = rgb + (0.22,)
        grab.pose.position.z = 0.0
        handle = InteractiveMarkerControl()
        handle.name = "grab_handle"
        handle.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        handle.always_visible = True
        handle.markers.append(grab)
        int_marker.controls.append(handle)

        # 操作提示文字（纯显示、不可拾取：interaction_mode = NONE）
        hint = Marker()
        hint.type = Marker.TEXT_VIEW_FACING
        hint.scale.z = HINT_TEXT_SIZE
        hint.color.r, hint.color.g, hint.color.b, hint.color.a = 1.0, 1.0, 1.0, 0.85
        hint.pose.position.z = GRAB_SIZE / 2.0 + 0.03
        hint.text = HINT_TEXT
        label = InteractiveMarkerControl()
        label.name = "hint"
        label.interaction_mode = InteractiveMarkerControl.NONE
        label.markers.append(hint)
        int_marker.controls.append(label)

        return int_marker

    def _set_dot_color(self, rgb):
        """Colour hook for the retired red dot. Motion code still calls this;
        it must not draw the dot again."""
        return

    # ------------------------------------------------------------------ #
    # feedback: dragging + click detection
    # ------------------------------------------------------------------ #
    def on_feedback(self, feedback):
        name = feedback.marker_name
        if name == "target":
            event = feedback.event_type
            pose = feedback.pose.position
            if event == InteractiveMarkerFeedback.MOUSE_DOWN:
                self._down = (rospy.get_rostime(), (pose.x, pose.y))
            elif event == InteractiveMarkerFeedback.POSE_UPDATE:
                x = min(max(pose.x, PLANE_X[0]), PLANE_X[1])
                y = min(max(pose.y, PLANE_Y[0]), PLANE_Y[1])
                if (x, y) != (pose.x, pose.y):
                    # snap back to the rectangle border when out of bounds
                    self.server.setPose("target", self._plane_pose(x, y),
                                        feedback.header)
                    self.server.applyChanges()
                self.dot_xy = (x, y)
                self._publish_target()
            elif event == InteractiveMarkerFeedback.MOUSE_UP:
                self._maybe_click(feedback)
            return

    def _maybe_click(self, feedback):
        down = self._down
        self._down = None
        if down is None:
            return
        down_time, (dx, dy) = down
        dt = rospy.get_rostime() - down_time
        moved = abs(feedback.pose.position.x - dx) + \
            abs(feedback.pose.position.y - dy)
        if dt < CLICK_TIME_S and moved < CLICK_MOVE_M:
            self.on_go()

    # ------------------------------------------------------------------ #
    # trigger / stop
    # ------------------------------------------------------------------ #
    def on_go(self):
        """Start a motion; returns (accepted, message) for the service reply."""
        with self.lock:
            if self.busy:
                rospy.logwarn("[%s] motion in progress, ignored", TARGET_NS)
                return False, "运动中，已忽略本次 GO"
            self.busy = True
            x, y = self.dot_xy
        self._stop_flag.clear()
        self.motion_thread = threading.Thread(target=self._run, args=(x, y))
        self.motion_thread.start()
        if self.enable_arm:
            return True, "已触发规划并执行（注意安全）"
        return True, "已触发规划（干跑模式，不会真动机械臂）"

    def on_go_service(self, req):
        accepted, message = self.on_go()
        return TriggerResponse(success=accepted, message=message)

    def on_home_service(self, req):
        """Return to a named SRDF state through MoveIt, so the path is planned
        inside the yellow box. The console's 回到初始位置 button used to send a
        bare joint trajectory straight to the controller (no planning, no
        collision check), which is what made the arm swing out of the zone."""
        with self.lock:
            if self.busy:
                return TriggerResponse(success=False, message="运动中，请先停止")
            self.busy = True
        self._stop_flag.clear()
        state = rospy.get_param("~home_named_state", "initial")
        self.motion_thread = threading.Thread(target=self._run_home)
        self.motion_thread.start()
        if self.enable_arm:
            return TriggerResponse(
                success=True,
                message="已触发回到 %s（受安全区约束）" % state)
        return TriggerResponse(
            success=True, message="已触发规划回 %s（干跑，不动机械臂）" % state)

    def on_stop_service(self, req):
        with self.lock:
            busy = self.busy
        if not busy:
            return TriggerResponse(success=True, message="空闲，无需停止")
        self._stop()
        return TriggerResponse(success=True, message="已发送停止指令")

    def on_reset_service(self, req):
        self._move_dot(*DOT_INIT)
        rospy.loginfo("[%s] dot reset to centre (%.4f, %.4f)",
                      TARGET_NS, DOT_INIT[0], DOT_INIT[1])
        return TriggerResponse(success=True, message="红点已回到中心")

    # ------------------------------------------------------------------ #
    # 夹取: (open) -> MoveIt onto the dot -> close, as ONE atomic action
    # ------------------------------------------------------------------ #
    def on_grasp_service(self, req):
        """Full pick. Atomic for the caller: self.busy stays reserved for the
        whole sequence, so a GO / home / another grasp cannot start in between
        the move and the close (they share one xarm6 controller)."""
        with self.lock:
            if self.busy:
                return TriggerResponse(
                    success=False, message="运动/夹取/放入/抓取中，已忽略本次夹取")
            self.busy = True
            x, y = self.dot_xy
        self._stop_flag.clear()
        params = self._grasp_params()
        self.motion_thread = threading.Thread(target=self._run_grasp,
                                              args=(x, y, params))
        self.motion_thread.start()
        if not self.enable_arm:
            return TriggerResponse(
                success=True,
                message="已触发夹取（干跑：只规划移到红点，夹爪不会闭合；"
                        "闭合目标 %d 不会下发）" % params["position"])
        return TriggerResponse(
            success=True,
            message="已触发夹取：红点 (%.0f, %.0f) mm，闭合目标 %d"
                    % (x * 1000.0, y * 1000.0, params["position"]))

    def _grasp_params(self):
        """Grasp numbers, read fresh on every grasp. Same live-tuning trick as
        ~rpy_deg, so the console writes the spin box into the param server right
        before calling the service and nothing needs a restart:
            rosparam set /plane_target_marker/grasp_position 300
        """
        pos = int(rospy.get_param("~grasp_position", 0))
        force = int(rospy.get_param("~grasp_force", DEFAULT_GRASP_FORCE))
        speed = int(rospy.get_param("~grasp_speed", DEFAULT_GRASP_SPEED))
        return {
            # 0 = close as far as the force allows, so the fingers stop on the
            # object whatever its width; larger values stop them earlier.
            "position": max(0, min(1000, pos)),
            # AG95 force register is 20..100 (see dh_ag95_can.cpp SetTargetForce)
            "force": max(20, min(100, force)),
            "speed": max(1, min(200, speed)),
            "preopen": bool(rospy.get_param("~grasp_preopen", True)),
            "timeout": max(0.5, float(
                rospy.get_param("~grasp_settle_timeout", 3.0))),
            "tol": max(0, int(
                rospy.get_param("~grasp_settle_tol", GRASP_SETTLE_TOL))),
            "samples": max(1, int(
                rospy.get_param("~grasp_settle_samples", 5))),
        }

    def _status(self, state, text):
        """Progress feedback for the console 夹取 row (and any future state
        machine). Format "<state>|<text>", state in busy/ok/fail."""
        if state == "fail":
            rospy.logwarn("[%s] grasp %s: %s", TARGET_NS, state, text)
        else:
            rospy.loginfo("[%s] grasp %s: %s", TARGET_NS, state, text)
        self.grasp_status_pub.publish(String(data="%s|%s" % (state, text)))

    def _gripper_cb(self, msg):
        """/gripper/states (PLURAL) -> settle detection input.

        Deliberately does NOT read grip_state: its 0/1/2 semantics were never
        measured (doc/measurements.md M4), and guessing them here would turn
        "到位" into a guess. "position stopped changing" is a fact we can
        actually observe, so that is what 到位 means in this node."""
        pos = int(round(msg.position))
        with self.lock:
            if (self._grip_prev is not None and
                    abs(pos - self._grip_prev) <= self._grip_tol):
                self._grip_still += 1
            else:
                self._grip_still = 0
            self._grip_prev = pos
            self._grip_pos = pos
            self._grip_stamp = time.time()

    def _gripper_ready(self):
        """(ok, message): can we actually talk to the DH driver?"""
        if self.grip_ctrl_pub.get_num_connections() == 0:
            return False, ("/gripper/ctrl 没有订阅者：夹爪驱动没起"
                           "（setup_arm.launch 用 open_gripper:=false 关过？）")
        with self.lock:
            stamp, pos = self._grip_stamp, self._grip_pos
        if stamp is None or time.time() - stamp > 1.0:
            return False, ("收不到 %s（注意是复数），夹爪驱动没起"
                           % GRIPPER_STATE_TOPIC)
        return True, "夹爪已连接（当前 %d）" % pos

    def _send_gripper(self, position, params):
        cmd = GripperCtrl()
        cmd.initialize = False
        cmd.position = float(position)
        cmd.force = float(params["force"])
        cmd.speed = float(params["speed"])
        self.grip_ctrl_pub.publish(cmd)
        return cmd

    def _gripper_move(self, position, params, what):
        """Command one gripper position and block until it stops moving.

        The AG95 has no cancel/stop input, so "已停止" is implemented as
        "re-command the position it has already reached" (a hold). That is also
        what lets the console 急停 abort a closing grasp instead of only the
        arm motion.

        Returns (ok, final_position, message)."""
        with self.lock:
            self._grip_prev = None      # restart the stillness counter
            self._grip_still = 0
            self._grip_tol = params["tol"]
        self._send_gripper(position, params)
        rospy.loginfo("[%s] %s: 目标 %d (force %d, speed %d)", TARGET_NS, what,
                      position, params["force"], params["speed"])
        start = time.time()
        deadline = start + params["timeout"]
        while time.time() < deadline:
            if self._stop_flag.is_set():
                with self.lock:
                    held = self._grip_pos
                self._send_gripper(int(held), params)   # hold where it is
                rospy.logwarn("[%s] %s 被急停中断，保持在 %d", TARGET_NS, what,
                              held)
                return False, held, "%s 被急停中断（保持在 %d）" % (what, held)
            with self.lock:
                pos, still, stamp = (self._grip_pos, self._grip_still,
                                     self._grip_stamp)
            if stamp is None or time.time() - stamp > 0.5:
                return False, pos, ("夹爪状态流中断（%s），读数不可信"
                                    % GRIPPER_STATE_TOPIC)
            # 0.25 s 起步窗口：指令刚发出时手指还没动，那段"静止"是假的，
            # 不加这个窗口会立刻误判到位。
            if time.time() - start > 0.25 and still >= params["samples"]:
                return True, pos, "%s 完成，停在 %d（目标 %d）" % (
                    what, pos, position)
            time.sleep(0.02)
        with self.lock:
            pos = self._grip_pos
        rospy.logwarn("[%s] %s 超时 %.1f s 仍在 %d（目标 %d）", TARGET_NS, what,
                      params["timeout"], pos, position)
        return False, pos, ("%s 超时 %.1f s，夹爪还停在 %d（目标 %d）；"
                            "确认没卡住或夹到东西" % (what, params["timeout"],
                                                  pos, position))

    def _run_grasp(self, x, y, params):
        try:
            # 先播一次 busy，让控制台立刻把按钮置灰；干跑路径也要有，
            # 否则按钮会一直亮着直到最后一条状态才变。
            self._status("busy", "夹取准备中：红点 (%.0f, %.0f) mm，闭合目标 %d"
                         % (x * 1000.0, y * 1000.0, params["position"]))
            if not self.enable_arm:
                # 干跑: planning is safe, closing the real gripper is not.
                self._status("busy", "干跑：只规划「移到红点」，不会闭合夹爪")
                self._execute_motion(
                    "夹取(干跑): 只会规划这一步",
                    self._dot_goal(x, y), flash=False)
                self._flash(COLOR_IDLE)
                self._status("ok", "干跑：只规划了「移到红点」，夹爪未闭合"
                           "（闭合目标 %d 未下发）" % params["position"])
                return

            ok, why = self._gripper_ready()
            if not ok:
                self._status("fail", why)
                self._flash(COLOR_FAIL)
                return

            if params["preopen"]:
                self._status("busy", "夹取 1/3：先张开夹爪")
                ok, _pos, msg = self._gripper_move(GRASP_OPEN, params, "张开")
                if not ok:
                    self._status("fail", msg)
                    self._flash(COLOR_FAIL)
                    return
                # 把实际读数也报出来：万一张开就没到 1000（机械卡滞、
                # 里面还夹着东西），后面闭合时就会在一个错误起点上合拢。
                self._status("busy", "夹取 1/3：%s" % msg)

            self._status("busy", "夹取 2/3：移动到红点 (%.0f, %.0f) mm"
                      % (x * 1000.0, y * 1000.0))
            ok, detail = self._execute_motion(
                "夹取: dot (%.3f, %.3f) in %s, z=%.3f"
                % (x, y, MARKER_FRAME, self.plane_z),
                self._dot_goal(x, y), flash=False)
            if not ok:
                self._status("fail", detail)
                self._flash(COLOR_FAIL)
                return

            self._status("busy", "夹取 3/3：闭合到 %d" % params["position"])
            ok, _pos, msg = self._gripper_move(params["position"], params,
                                               "闭合")
            if not ok:
                self._status("fail", msg)
                self._flash(COLOR_FAIL)
                return

            self._status("ok", "夹取完成：%s" % msg)
            self._flash(COLOR_OK)
        except Exception as exc:
            rospy.logerr("[%s] grasp failed: %s", TARGET_NS, exc)
            self._status("fail", "夹取异常：%s" % exc)
            self._flash(COLOR_FAIL)
        finally:
            with self.lock:
                self.busy = False

    # ------------------------------------------------------------------ #
    # 放入 (put): one atomic action per taught point (measurements.md M6)
    # ------------------------------------------------------------------ #
    def on_put_service(self, req):
        """Place the held object on put point 1..6 as ONE atomic action.

        Starts from the current pose. Before the arm moves toward the ball the
        gripper closes to PUT_GRIP_CLOSE (200). After the straight descent it
        opens, lifts clear, then returns to the SRDF initial pose. self.busy
        stays reserved for the whole sequence, so no GO / home / grasp can
        slip in (they share one xarm6 controller)."""
        point = int(req.point)
        if point not in PUT_POINTS:
            return PutPointResponse(
                success=False,
                message="点号必须是 1~6（收到 %d；点号与 measurements.md M6 "
                        "的表格顺序一致）" % point)
        with self.lock:
            if self.busy:
                return PutPointResponse(
                    success=False, message="运动/夹取/放入/抓取中，已忽略本次放入")
            self.busy = True
        self._stop_flag.clear()
        self.motion_thread = threading.Thread(target=self._run_put,
                                              args=(point,))
        self.motion_thread.start()
        name = PUT_POINTS[point][0]
        if not self.enable_arm:
            return PutPointResponse(
                success=True,
                message="已触发放入点 %d（%s）干跑：只规划到上方点，不会闭合/"
                        "下降/松开/抬高/回家" % (point, name))
        return PutPointResponse(
            success=True,
            message="已触发放入点 %d（%s）：先闭合到 %d -> 上方点 -> 直线下降 "
                    "-> 松开 -> 直线抬高 -> 回 initial；进度看 %s"
                    % (point, name, PUT_GRIP_CLOSE, PUT_STATUS_TOPIC))

    def _put_status(self, state, text):
        """Progress feedback for scripts/put (and any future state machine).
        Same "<state>|<text>" format as grasp_status but on its OWN topic, so
        the console 夹取 row never reports put progress as if it were a
        grasp."""
        if state == "fail":
            rospy.logwarn("[%s] put %s: %s", TARGET_NS, state, text)
        else:
            rospy.loginfo("[%s] put %s: %s", TARGET_NS, state, text)
        self.put_status_pub.publish(String(data="%s|%s" % (state, text)))

    def _run_put(self, point):
        """The five steps of one put (module docstring / decisions.md D22)."""
        name, x, y, z = PUT_POINTS[point]
        state = rospy.get_param("~home_named_state", "initial")
        approach_lift0 = self._put_pose(x, y, z + PUT_APPROACH_RISE)
        put_lift0 = self._put_pose(x, y, z)
        try:
            self._put_status("busy", "放入点 %d（%s）：准备（把 %s 的示教点"
                             "变到当前 link_base；本次调用内固定）"
                             % (point, name, PUT_FRAME))
            # one TF snapshot per call: the taught points live in the static
            # lift0 frame, the planner needs the CURRENT (possibly lifted)
            # link_base. Moving the lift afterwards does not move this goal.
            approach = self._pose_in("link_base", approach_lift0, PUT_FRAME)
            put = self._pose_in("link_base", put_lift0, PUT_FRAME)
            put_deck = self._pose_in(MARKER_FRAME, put_lift0, PUT_FRAME)
            floor = put_deck.position.z
            rospy.loginfo(
                "[%s] put %d (%s): approach (%.3f, %.3f, %.3f) / point "
                "(%.3f, %.3f, %.3f) in link_base; floor z=%.3f in %s",
                TARGET_NS, point, name, approach.position.x,
                approach.position.y, approach.position.z, put.position.x,
                put.position.y, put.position.z, floor, MARKER_FRAME)

            # Close before the arm leaves for the ball. Travelling open is
            # what left the gripper open for the whole put.
            if self.enable_arm:
                self._put_status(
                    "busy", "放入：到达红球前闭合夹爪到 %d" % PUT_GRIP_CLOSE)
                ok, why = self._gripper_ready()
                if not ok:
                    self._put_status(
                        "fail", "放入中止（闭合到 %d）：%s；手臂未动"
                        % (PUT_GRIP_CLOSE, why))
                    return
                params = self._grasp_params()
                ok, _pos, close_msg = self._gripper_move(
                    PUT_GRIP_CLOSE, params, "闭合")
                if not ok:
                    self._put_status(
                        "fail", "放入中止（闭合到 %d）：%s；手臂未动"
                        % (PUT_GRIP_CLOSE, close_msg))
                    return
                if self._stop_flag.is_set():
                    self._put_status(
                        "fail", "放入被急停中断：夹爪停在闭合过程中，手臂未动")
                    return

            # 1/5: plan+execute to the approach point 150 mm above the point.
            self._put_status("busy", "放入 1/5：移动到上方接近点（+%.0f mm）"
                             % (PUT_APPROACH_RISE * 1000.0))
            ok, detail = self._execute_motion(
                "put %d: approach (%.3f, %.3f, %.3f) in link_base"
                % (point, approach.position.x, approach.position.y,
                   approach.position.z),
                self._pose_goal(approach, "放入 1/5 上方点"),
                flash=False,
                zone_goal=(put_deck.position.x, put_deck.position.y, floor),
                zone_floor=floor)
            if not ok:
                held = ("；夹爪已闭合到 %d" % PUT_GRIP_CLOSE
                        if self.enable_arm else "")
                self._put_status(
                    "fail", "放入中止（1/5 到上方点）：%s%s" % (detail, held))
                return
            if self._stop_flag.is_set():
                self._put_status(
                    "fail", "放入被急停中断：夹爪已闭合到 %d，停在上方点，"
                    "未下降、未松开" % PUT_GRIP_CLOSE)
                return
            if not self.enable_arm:
                # Dry run: without moving, the straight descent cannot be
                # planned from a hypothetical approach pose, so it is reported
                # as NOT checked instead of being faked. The gripper is not
                # commanded either.
                self._put_status("busy", "干跑：跳过闭合/下降/松开/抬高/回家"
                                 "（不会动机械臂、不碰夹爪）")
                self._put_status("ok", "干跑完成：只规划了 1/5「移动到上方点」；"
                                 "闭合到 %d、直线下降 150 mm、松开到 %d、抬高、"
                                 "回 %s 都未执行"
                                 % (PUT_GRIP_CLOSE, GRASP_OPEN, state))
                return

            # 2/5: straight-line descent. No zone constraint on purpose: the
            # taught point may sit outside the yellow box and a constrained
            # straight line would not plan (M6). Collision checking stays on.
            self._put_status("busy", "放入 2/5：直线下降到放入点（%.0f mm，"
                             "无安全区约束）" % (PUT_APPROACH_RISE * 1000.0))
            ok, detail = self._execute_cartesian(
                "put %d: descend (%.3f, %.3f, %.3f) in link_base"
                % (point, put.position.x, put.position.y, put.position.z),
                put)
            if not ok:
                self._put_status("fail", "放入中止（2/5 下降）：%s；手臂停在"
                                 "当前高度，物体还夹着" % detail)
                return
            if self._stop_flag.is_set():
                self._put_status("fail",
                                 "放入被急停中断：停在放入点，物体还夹着（未松开）")
                return

            # 3/5: release. Same open target (1000), force/speed params and
            # "fingers stopped moving" check as the grasp close.
            self._put_status("busy", "放入 3/5：松开夹爪（张开到 %d）"
                             % GRASP_OPEN)
            ok, why = self._gripper_ready()
            if not ok:
                self._put_status("fail", "放入中止（3/5 松开）：%s；手臂停在"
                                 "放入点，物体可能还夹着，人工确认后再处理"
                                 % why)
                return
            params = self._grasp_params()
            ok, _pos, msg = self._gripper_move(GRASP_OPEN, params, "松开")
            if not ok:
                self._put_status("fail", "放入中止（3/5 松开）：%s；手臂停在"
                                 "放入点，物体可能还夹着" % msg)
                return
            if self._stop_flag.is_set():
                self._put_status("fail", "放入被急停中断：已松开，停在放入点"
                                 "（未抬高、未回家）")
                return

            # 4/5: straight-line lift back to the approach point. Same line as
            # step 2, reversed; the only two steps allowed below the approach
            # point are these two.
            self._put_status("busy", "放入 4/5：直线抬高 %.0f mm"
                             % (PUT_APPROACH_RISE * 1000.0))
            ok, detail = self._execute_cartesian(
                "put %d: lift (%.3f, %.3f, %.3f) in link_base"
                % (point, approach.position.x, approach.position.y,
                   approach.position.z),
                approach)
            if not ok:
                self._put_status("fail", "放入中止（4/5 抬高）：%s；物体已松开"
                                 % detail)
                return
            if self._stop_flag.is_set():
                self._put_status("fail", "放入被急停中断：已松开并停在上方点"
                                 "（未回家）")
                return

            # 5/5: planned return to the SRDF initial pose (same as /home,
            # still with the put floor).
            self._put_status("busy", "放入 5/5：回到 %s" % state)
            ok, detail = self._execute_motion(
                "put %d: home (%s)" % (point, state), self._home_goal(),
                flash=False, zone_floor=floor)
            if not ok:
                self._put_status("fail", "放入中止（5/5 回 %s）：%s；物体已"
                                 "放下，手臂停在上方点" % (state, detail))
                return

            self._put_status("ok", "放入完成：点 %d（%s）已放下并回到 %s；%s"
                             % (point, name, state, msg))
        except Exception as exc:
            rospy.logerr("[%s] put failed: %s", TARGET_NS, exc)
            self._put_status("fail", "放入异常：%s" % exc)
        finally:
            # _execute_motion paints the red dot yellow while it works; the
            # dot belongs to the red-dot feature, so leave it back at idle.
            try:
                self._flash(COLOR_IDLE)
            except Exception:
                pass
            with self.lock:
                self.busy = False

    # ------------------------------------------------------------------ #
    # 抓取 (pick): same six points as put, opposite gripper timing
    # ------------------------------------------------------------------ #
    def on_pick_service(self, req):
        """Pick from point 1..6 as ONE atomic action.

        The arm is expected to start at the SRDF initial pose. Before it
        moves, the gripper must open to GRASP_OPEN and the reading must be
        at least PICK_OPEN_MIN (empty, not still closed from the last pick).
        Then it plans to the safety height above the point, descends in a
        straight line, closes to PUT_GRIP_CLOSE (200), lifts straight back,
        then returns to initial. self.busy stays reserved for the whole
        sequence."""
        point = int(req.point)
        if point not in PUT_POINTS:
            return PickPointResponse(
                success=False,
                message="点号必须是 1~6（收到 %d；点号与 measurements.md M6 "
                        "的表格顺序一致，和放入是同一批点）" % point)
        with self.lock:
            if self.busy:
                return PickPointResponse(
                    success=False,
                    message="运动/夹取/放入/抓取中，已忽略本次抓取")
            self.busy = True
        self._stop_flag.clear()
        self.motion_thread = threading.Thread(target=self._run_pick,
                                              args=(point,))
        self.motion_thread.start()
        name = PUT_POINTS[point][0]
        if not self.enable_arm:
            return PickPointResponse(
                success=True,
                message="已触发抓取点 %d（%s）干跑：只规划到上方安全高度，"
                        "不会张开/下降/闭合/抬高/回家" % (point, name))
        return PickPointResponse(
            success=True,
            message="已触发抓取点 %d（%s）：先确认夹爪张开到 %d -> 上方安全高度 "
                    "(+%.0f mm) -> 直线下降 -> 闭合到 %d -> 直线抬高 -> "
                    "回 initial；进度看 %s"
                    % (point, name, GRASP_OPEN, PUT_APPROACH_RISE * 1000.0,
                       PUT_GRIP_CLOSE, PICK_STATUS_TOPIC))

    def _pick_status(self, state, text):
        """Progress for scripts/pick. Own topic, same "<state>|<text>" shape."""
        if state == "fail":
            rospy.logwarn("[%s] pick %s: %s", TARGET_NS, state, text)
        else:
            rospy.loginfo("[%s] pick %s: %s", TARGET_NS, state, text)
        self.pick_status_pub.publish(String(data="%s|%s" % (state, text)))

    def _run_pick(self, point):
        """Five steps: approach, descend, close to 200, lift, home."""
        name, x, y, z = PUT_POINTS[point]
        state = rospy.get_param("~home_named_state", "initial")
        approach_lift0 = self._put_pose(x, y, z + PUT_APPROACH_RISE)
        pick_lift0 = self._put_pose(x, y, z)
        try:
            self._pick_status(
                "busy", "抓取点 %d（%s）：准备（把 %s 的示教点变到当前 "
                "link_base；本次调用内固定）" % (point, name, PUT_FRAME))
            approach = self._pose_in("link_base", approach_lift0, PUT_FRAME)
            pick = self._pose_in("link_base", pick_lift0, PUT_FRAME)
            pick_deck = self._pose_in(MARKER_FRAME, pick_lift0, PUT_FRAME)
            floor = pick_deck.position.z
            rospy.loginfo(
                "[%s] pick %d (%s): approach (%.3f, %.3f, %.3f) / point "
                "(%.3f, %.3f, %.3f) in link_base; floor z=%.3f in %s",
                TARGET_NS, point, name, approach.position.x,
                approach.position.y, approach.position.z, pick.position.x,
                pick.position.y, pick.position.z, floor, MARKER_FRAME)

            # The previous pick comes home still closed. Do not leave initial
            # until the fingers are actually open and empty.
            if self.enable_arm:
                self._pick_status(
                    "busy", "抓取：出发前确认夹爪张开到 %d（没有夹着东西）"
                    % GRASP_OPEN)
                ok, why = self._gripper_ready()
                if not ok:
                    self._pick_status(
                        "fail", "抓取中止（张开确认）：%s；手臂未动" % why)
                    return
                params = self._grasp_params()
                ok, open_pos, open_msg = self._gripper_move(
                    GRASP_OPEN, params, "张开")
                if not ok:
                    self._pick_status(
                        "fail", "抓取中止（张开确认）：%s；手臂未动" % open_msg)
                    return
                if self._stop_flag.is_set():
                    self._pick_status(
                        "fail", "抓取被急停中断：夹爪停在张开过程中，手臂未动")
                    return
                if open_pos < PICK_OPEN_MIN:
                    self._pick_status(
                        "fail", "抓取中止（张开确认）：夹爪停在 %d，没有张开到 "
                        "%d，可能还夹着东西；手臂未动"
                        % (open_pos, PICK_OPEN_MIN))
                    return
                self._pick_status(
                    "busy", "抓取：夹爪已张开（%s）" % open_msg)

            # 1/5: from the current pose (the arm should already be at
            # initial, gripper open) to the safety height above the point.
            self._pick_status(
                "busy", "抓取 1/5：从初始位置到上方安全高度（+%.0f mm）"
                % (PUT_APPROACH_RISE * 1000.0))
            ok, detail = self._execute_motion(
                "pick %d: approach (%.3f, %.3f, %.3f) in link_base"
                % (point, approach.position.x, approach.position.y,
                   approach.position.z),
                self._pose_goal(approach, "抓取 1/5 上方点"),
                flash=False,
                zone_goal=(pick_deck.position.x, pick_deck.position.y, floor),
                zone_floor=floor)
            if not ok:
                self._pick_status(
                    "fail", "抓取中止（1/5 到上方点）：%s；夹爪未动" % detail)
                return
            if self._stop_flag.is_set():
                self._pick_status(
                    "fail", "抓取被急停中断：停在上方点，未下降、未闭合")
                return
            if not self.enable_arm:
                self._pick_status(
                    "busy", "干跑：跳过张开/下降/闭合/抬高/回家"
                    "（不会动机械臂、不碰夹爪）")
                self._pick_status(
                    "ok", "干跑完成：只规划了 1/5「移动到上方安全高度」；"
                    "张开到 %d、直线下降 %.0f mm、闭合到 %d、抬高、回 %s 都未执行"
                    % (GRASP_OPEN, PUT_APPROACH_RISE * 1000.0,
                       PUT_GRIP_CLOSE, state))
                return

            # 2/5: straight descent. Same exemption as put: the taught point
            # may sit outside the yellow box.
            self._pick_status(
                "busy", "抓取 2/5：直线下降到目标点（%.0f mm，无安全区约束）"
                % (PUT_APPROACH_RISE * 1000.0))
            ok, detail = self._execute_cartesian(
                "pick %d: descend (%.3f, %.3f, %.3f) in link_base"
                % (point, pick.position.x, pick.position.y, pick.position.z),
                pick)
            if not ok:
                self._pick_status(
                    "fail", "抓取中止（2/5 下降）：%s；手臂停在当前高度，"
                    "夹爪未闭合" % detail)
                return
            if self._stop_flag.is_set():
                self._pick_status(
                    "fail", "抓取被急停中断：停在目标点，夹爪未闭合")
                return

            # 3/5: close. Force and speed are the live ~grasp_* parameters.
            self._pick_status(
                "busy", "抓取 3/5：闭合夹爪到 %d" % PUT_GRIP_CLOSE)
            ok, why = self._gripper_ready()
            if not ok:
                self._pick_status(
                    "fail", "抓取中止（3/5 闭合到 %d）：%s；手臂停在目标点，"
                    "夹爪未闭合" % (PUT_GRIP_CLOSE, why))
                return
            params = self._grasp_params()
            ok, close_pos, close_msg = self._gripper_move(
                PUT_GRIP_CLOSE, params, "闭合")
            if not ok:
                self._pick_status(
                    "fail", "抓取中止（3/5 闭合到 %d）：%s；手臂停在目标点"
                    % (PUT_GRIP_CLOSE, close_msg))
                return
            if close_pos >= PICK_STILL_OPEN:
                self._pick_status(
                    "fail", "抓取中止（3/5 闭合到 %d）：夹爪仍停在 %d，没有离开"
                    "张开位置，不抬升；手臂停在目标点"
                    % (PUT_GRIP_CLOSE, close_pos))
                return
            if self._stop_flag.is_set():
                self._pick_status(
                    "fail", "抓取被急停中断：夹爪停在闭合过程中，停在目标点"
                    "（未抬高、未回家）")
                return

            # 4/5: straight lift back to the safety height.
            self._pick_status(
                "busy", "抓取 4/5：直线抬高 %.0f mm"
                % (PUT_APPROACH_RISE * 1000.0))
            ok, detail = self._execute_cartesian(
                "pick %d: lift (%.3f, %.3f, %.3f) in link_base"
                % (point, approach.position.x, approach.position.y,
                   approach.position.z),
                approach)
            if not ok:
                self._pick_status(
                    "fail", "抓取中止（4/5 抬高）：%s；夹爪已闭合到 %d，"
                    "手臂停在当前高度" % (detail, PUT_GRIP_CLOSE))
                return
            if self._stop_flag.is_set():
                self._pick_status(
                    "fail", "抓取被急停中断：已闭合到 %d 并停在上方点（未回家）"
                    % PUT_GRIP_CLOSE)
                return

            # 5/5: planned return to initial, still with the pick floor.
            self._pick_status("busy", "抓取 5/5：回到 %s" % state)
            ok, detail = self._execute_motion(
                "pick %d: home (%s)" % (point, state), self._home_goal(),
                flash=False, zone_floor=floor)
            if not ok:
                self._pick_status(
                    "fail", "抓取中止（5/5 回 %s）：%s；夹爪已闭合到 %d，"
                    "手臂停在上方点" % (state, detail, PUT_GRIP_CLOSE))
                return

            self._pick_status(
                "ok", "抓取完成：点 %d（%s）已闭合到 %d 并回到 %s；%s"
                % (point, name, PUT_GRIP_CLOSE, state, close_msg))
        except Exception as exc:
            rospy.logerr("[%s] pick failed: %s", TARGET_NS, exc)
            self._pick_status("fail", "抓取异常：%s" % exc)
        finally:
            try:
                self._flash(COLOR_IDLE)
            except Exception:
                pass
            with self.lock:
                self.busy = False

    def on_nudge(self, msg):
        """Relative step from the console panel; clamped to the rectangle."""
        self._move_dot(self.dot_xy[0] + msg.x, self.dot_xy[1] + msg.y)

    def _move_dot(self, x, y):
        x = min(max(x, PLANE_X[0]), PLANE_X[1])
        y = min(max(y, PLANE_Y[0]), PLANE_Y[1])
        if (x, y) == self.dot_xy:
            return
        self.dot_xy = (x, y)
        self.server.setPose("target", self._plane_pose(x, y))
        self.server.applyChanges()
        self._publish_target()

    def _stop(self):
        # Also aborts the gripper phase of a grasp: the AG95 has no cancel, so
        # the only way to halt a closing command is to re-command the position
        # it already reached and stop waiting for it.
        self._stop_flag.set()
        try:
            if self._group is not None:
                self._group.stop()
        except Exception as exc:  # stop must never take down the node
            rospy.logerr("[%s] stop failed: %s", TARGET_NS, exc)

    def _lazy_group(self):
        if self._group is None:
            group = MoveGroupCommander("xarm6")
            group.set_pose_reference_frame("link_base")
            group.set_end_effector_link("tcp_link")
            group.set_planning_time(self.planning_time)
            group.set_max_velocity_scaling_factor(self.vel_scaling)
            self._group = group
        return self._group

    def _dot_goal(self, x, y):
        """apply_goal(group) aiming tcp_link at the dot with the live ~rpy_deg
        orientation. Extracted so a grasp reuses the exact same guarded goal
        as a plain GO instead of duplicating it."""

        def apply_goal(group):
            # re-read every GO: the goal orientation can be tuned live
            self.target_quat, self.rpy_deg = self._goal_quat()
            tool_axis = _axes_of(self.target_quat)[2]
            rospy.loginfo(
                "[%s] goal rpy=(%.1f, %.1f, %.1f) deg -> tcp +Z (tool axis) "
                "= (%+.2f, %+.2f, %+.2f) in link_base, %.1f deg off "
                "vertical-down", TARGET_NS, self.rpy_deg[0], self.rpy_deg[1],
                self.rpy_deg[2], tool_axis[0], tool_axis[1], tool_axis[2],
                _tilt_deg(tool_axis))
            # bind the plane -> arm-base transform NOW: if the lift is moved
            # afterwards we still go where the dot WAS when clicked
            target = self._plane_in_link_base(x, y)
            group.set_pose_target(target)
            self._goal_point = target.position
        return apply_goal

    def _run(self, x, y):
        self._goal_point = None
        self._motion(
            "GO: dot (%.3f, %.3f) in %s, z=%.3f" % (
                x, y, MARKER_FRAME, self.plane_z), self._dot_goal(x, y))

    def _pose_goal(self, target, what):
        """apply_goal for a free-planned move to `target` (Pose in link_base)
        with the recorded M6 put orientation. Same shape as _dot_goal: the
        commanded point is remembered for the _report_tcp read-back."""

        def apply_goal(group):
            axis = _axes_of((target.orientation.x, target.orientation.y,
                             target.orientation.z, target.orientation.w))[2]
            rospy.loginfo(
                "[%s] %s goal (%.3f, %.3f, %.3f) in link_base, tcp +Z = "
                "(%+.2f, %+.2f, %+.2f), %.1f deg off vertical-down",
                TARGET_NS, what, target.position.x, target.position.y,
                target.position.z, axis[0], axis[1], axis[2], _tilt_deg(axis))
            group.set_pose_target(target)
            self._goal_point = target.position
        return apply_goal

    def _home_goal(self):
        """apply_goal for the SRDF named state used by plane_target/home and
        by the last step of a put (one definition, so they cannot drift)."""
        state = rospy.get_param("~home_named_state", "initial")

        def apply_goal(group):
            group.set_named_target(state)
        return apply_goal

    def _run_home(self):
        state = rospy.get_param("~home_named_state", "initial")
        self._goal_point = None
        self._motion("HOME: named state '%s'（受安全区约束）" % state,
                     self._home_goal())

    def _motion(self, desc, apply_goal):
        """_execute_motion plus the busy release, for the single-shot
        GO / home actions."""
        try:
            self._execute_motion(desc, apply_goal)
        finally:
            with self.lock:
                self.busy = False

    def _execute_motion(self, desc, apply_goal, flash=True, zone_goal=None,
                        zone_floor=None):
        """Plan + execute ONE guarded motion, returning (ok, detail).

        apply_goal(group) installs the goal; the keep-in zone constraint is
        installed first so every motion this node produces (dot, home, or the
        move half of a grasp) is planned inside the yellow box.

        zone_goal / zone_floor are the put-leg extras handed to _apply_zone
        (grow the box around the goal; raise its bottom to the put height).

        Deliberately does NOT touch self.busy: the caller owns that reservation,
        so a composite action (grasp = move + close) can keep the xarm6
        controller to itself for its whole duration instead of briefly handing
        it back between the two halves.

        flash=False leaves the dot colour to the caller (a grasp keeps it yellow
        across both halves and only reports the final outcome)."""
        dry = not self.enable_arm
        try:
            rospy.loginfo("[%s] %s%s", TARGET_NS, desc,
                          " (dry-run)" if dry else "")
            group = self._lazy_group()
            group.clear_pose_targets()
            self._apply_zone(group, zone_goal, zone_floor)
            apply_goal(group)

            self._set_dot_color(COLOR_MOVING)
            ok, plan, _t, err = group.plan()
            if not ok:
                detail = ("规划失败（code %d）：目标可能超出可达范围、被安全区"
                          "约束挡住，或当前升降高度够不着；可拖动红点/调升降"
                          "高度，或放宽 ~zone_margin 再试" % err.val)
                rospy.logwarn("[%s] %s", TARGET_NS, detail)
                if flash:
                    self._flash(COLOR_FAIL)
                return False, detail
            if dry:
                waypoints = len(plan.joint_trajectory.points)
                rospy.loginfo(
                    "[%s] dry-run: plan OK (%d waypoints), not executed",
                    TARGET_NS, waypoints)
                if flash:
                    self._flash(COLOR_IDLE)
                return True, "干跑：规划通过（%d 个路点），未执行" % waypoints
            reached = group.execute(plan, wait=True)
            if not reached:
                rospy.logwarn("[%s] execution canceled/stopped", TARGET_NS)
                if flash:
                    self._flash(COLOR_IDLE)
                return False, "执行被取消/停止"
            rospy.loginfo("[%s] motion done: %s", TARGET_NS, desc)
            self._report_tcp(self._goal_point)
            if flash:
                self._flash(COLOR_OK)
            return True, "已到达红点上方"
        except Exception as exc:
            rospy.logerr("[%s] motion failed: %s", TARGET_NS, exc)
            if flash:
                self._flash(COLOR_FAIL)
            # MoveIt 抛的异常有中文的也有英文的（缺 robot_description 时是英文
            # "Unable to construct robot model"）。控制台一行放不下英文堆栈，
            # 所以归一下类，末尾保留原文细节供查日志。
            return False, "规划/运动失败（%s）：%s" % (
                "MoveIt 初始化或规划出错，检查 move_group / robot_description "
                "/ 黄色安全区参数" if "robot model" in str(exc)
                or "Unable to construct" in str(exc)
                else "见 /plane_target_marker 日志", exc)

    def _execute_cartesian(self, desc, target):
        """Plan + execute a STRAIGHT-LINE (Cartesian) move of tcp_link onto
        `target` (Pose in link_base), returning (ok, detail).

        Used for the two vertical put legs only. Deliberately installs NO
        keep-in zone constraint: the taught put points may sit outside the
        yellow box and a constrained straight line onto them would not plan
        (M6 / D22). Collision checking stays ON (avoid_collisions default).

        The CartesianPath service always time-parameterizes at full speed
        (IterativeParabolicTimeParameterization, factor 1.0) and ignores the
        group's velocity scaling, so the plan is retimed here with
        ~vel_scaling before it is executed."""
        dry = not self.enable_arm
        try:
            step = max(0.001, float(
                rospy.get_param("~put_cartesian_step", PUT_CARTESIAN_STEP)))
            rospy.loginfo(
                "[%s] %s%s（直线运动；不套黄区约束，碰撞检查保留；"
                "eef_step=%.3f）", TARGET_NS, desc, " (dry-run)" if dry else "",
                step)
            group = self._lazy_group()
            group.clear_pose_targets()
            group.clear_path_constraints()
            plan, fraction = group.compute_cartesian_path([target], step)
            if fraction < 0.999:
                detail = ("直线只规划出 %.0f%%（中途被碰撞/可达性挡住），"
                          "拒绝执行：放入点或路径被挡，或该点超出当前升降"
                          "高度的可达范围" % (fraction * 100.0))
                rospy.logwarn("[%s] %s", TARGET_NS, detail)
                return False, detail
            if dry:
                return True, ("干跑：直线规划通过（%.0f%%，未执行）"
                              % (fraction * 100.0))
            plan = self._retime(group, plan)
            reached = group.execute(plan, wait=True)
            if not reached:
                rospy.logwarn("[%s] %s execution canceled/stopped", TARGET_NS,
                              desc)
                return False, "执行被取消/停止"
            rospy.loginfo("[%s] motion done: %s", TARGET_NS, desc)
            return True, "已到达"
        except Exception as exc:
            rospy.logerr("[%s] cartesian motion failed: %s", TARGET_NS, exc)
            return False, "直线规划/运动失败：%s" % exc

    def _retime(self, group, plan):
        """Re-time a Cartesian plan with ~vel_scaling.

        move_group's CartesianPath service always times the path at 100%
        (capability calls computeTimeStamps(rt, 1.0)), so without this the
        descent would slam down at full speed. Retiming happens client-side in
        moveit_commander with the same algorithm and our scaling factor; on
        any failure the original (fast) plan is used with a warning."""
        try:
            ref = group.get_current_state()
            retimed = group.retime_trajectory(
                ref, plan, self.vel_scaling, self.vel_scaling,
                "iterative_time_parameterization")
            if not retimed.joint_trajectory.points:
                raise ValueError("retimed trajectory is empty")
            return retimed
        except Exception as exc:
            rospy.logwarn(
                "[%s] 直线轨迹重定时失败（%s）：按规划器原始速度执行（会比 "
                "~vel_scaling=%.2f 快，注意）", TARGET_NS, exc, self.vel_scaling)
            return plan

    def _lookup_tcp(self):
        """(position, orientation) of tcp_link in link_base from TF."""
        return self._lookup_in("link_base", "tcp_link")

    def _report_tcp(self, goal):
        """Read back where tcp_link actually ended up and print its position
        error plus its three axes in link_base. 0 deg on the tilt = the tool
        axis really points down as commanded; a large value means the motion
        did not converge (stopped early / clamped) or the model and the
        physical arm disagree (joint zero offsets, gripper mounting).
        goal may be None (named-state moves) -> position error is skipped."""
        try:
            time.sleep(0.5)
            pos, rot = self._lookup_tcp()
            axes = _axes_of(rot)
            where = "(%.3f, %.3f, %.3f)" % pos
            if goal is not None:
                err = ((pos[0] - goal.x) ** 2 + (pos[1] - goal.y) ** 2 +
                       (pos[2] - goal.z) ** 2) ** 0.5
                where += ", %.1f mm from the commanded point" % (err * 1000.0)
            rospy.loginfo(
                "[%s] tcp_link now %s | X=(%+.2f, %+.2f, %+.2f) Y=(%+.2f, "
                "%+.2f, %+.2f) Z=(%+.2f, %+.2f, %+.2f) | tool axis %.1f deg "
                "off vertical-down", TARGET_NS, where,
                axes[0][0], axes[0][1], axes[0][2], axes[1][0], axes[1][1],
                axes[1][2], axes[2][0], axes[2][1], axes[2][2], _tilt_deg(axes[2]))
        except Exception as exc:
            rospy.logwarn("[%s] could not read back tcp_link: %s",
                          TARGET_NS, exc)

    def _plane_in_link_base(self, x, y):
        ps = PoseStamped()
        ps.header.frame_id = MARKER_FRAME
        ps.pose = self._plane_pose(x, y)
        ps.pose.orientation.x = self.target_quat[0]
        ps.pose.orientation.y = self.target_quat[1]
        ps.pose.orientation.z = self.target_quat[2]
        ps.pose.orientation.w = self.target_quat[3]
        # raise on failure with a message the operator can act on
        self.tf_listener.waitForTransform(
            "link_base", MARKER_FRAME, rospy.Time(0), rospy.Duration(3.0))
        return self.tf_listener.transformPose("link_base", ps).pose

    def _flash(self, rgb):
        if rgb is COLOR_IDLE:
            self._set_dot_color(COLOR_IDLE)
            return
        self._set_dot_color(rgb)
        t = threading.Timer(FLASH_S, self._set_dot_color, args=(COLOR_IDLE,))
        t.daemon = True
        t.start()

    def _publish_target(self):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = MARKER_FRAME
        ps.pose = self._plane_pose(*self.dot_xy)
        self.target_pub.publish(ps)

    def spin(self):
        rospy.spin()

    def clear(self):
        # cancel a pending motion and remove the markers on shutdown
        try:
            with self.lock:
                busy = self.busy
            if busy:
                self._stop()
        except Exception:
            pass
        try:
            # the put balls are plain markers: RViz would keep them forever,
            # so erase them explicitly (latched message, one publish is enough)
            self._publish_put_markers(Marker.DELETE)
        except Exception:
            pass
        retired = InteractiveMarkerUpdate()
        retired.erases.extend(RETIRED_MARKERS)
        self.server.publish(retired)
        self.server.applyChanges()


def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("plane_target_marker")
    node = PlaneTargetMarker()
    rospy.on_shutdown(node.clear)
    node.spin()
    roscpp_shutdown()


if __name__ == "__main__":
    main()
