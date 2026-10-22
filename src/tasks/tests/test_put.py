#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plane_target_marker.py 放入（put）逻辑测试

覆盖 /plane_target/put 这条链路上**纯逻辑、可离线**的部分：
  1. 六个放入点与 M6 记录一致；上方接近点 = 放入点沿 +Z 抬高 150 mm
  2. 姿态：记录 RPY (180, 0, 1.32) 度 -> 工具轴竖直向下（坑 18 回归 + 对照）
  3. 五步顺序：上方点 -> 直线下降 -> 松开(1000) -> 直线抬高 -> 回 initial
  4. 坐标变换：每次执行用 _pose_in 把 link_base_lift0 变到当时的
     link_base/deck_top（不许把 lift0 数字直接当 link_base 用）
  5. 约束豁免：竖直两段走 _execute_cartesian（不套黄区约束、清掉约束、
     但保留碰撞检查）；规划两段带 zone_floor（不回到放入高度以下）和
     zone_goal（目标在黄区外时扩盒）
  6. _zone_bounds / _apply_zone：目标扩盒、底面抬高、keep_in_zone=false
     时仍保留放入下限
  7. 干跑（enable_arm=false）：只规划 1/5「移动到上方点」，绝不发夹爪指令
  8. busy / 非法点号拒绝；下降/松开/抬高/回家任一失败都如实报 fail 并释放 busy
  9. 急停：标志置位后不在步骤之间继续往下走
 10. 六红球 Marker：一批 12 个（6 球 + 6 编号），坐标系 link_base_lift0
 11. put_status 话题格式 "<state>|<text>"

⚠ 本测试不碰串口、不碰真机，MoveIt / rospy / tf / 交互 marker 全部打桩，
节点用 object.__new__ 绕过 __init__，因此**不需要 roscore**，也不会和正在运行的
setup_arm.launch 抢节点名 / 抢 xarm6 控制器。
消息类优先用真实包（和 test_grasp 一样）；包不存在时退回内置替身，这样在
没建工作区的开发机上也能先跑一遍纯逻辑。

跑法：
    source /opt/ros/noetic/setup.bash && source ~/agilex_ws/devel/setup.bash
    python3 src/tasks/tests/test_put.py
"""

import importlib.util
import math
import os
import sys
import threading
import time
import types

SCRIPT_PY = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "agx_plane_target", "scripts", "plane_target_marker.py"))

_P = {}      # rospy.get_param 的参数表
_LOG = []


# ---- 打桩：让测试完全不依赖 roscore / MoveIt / 真机 ------------------------
class _StubPublisher(object):
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)

    def get_num_connections(self):
        return 1


rospy_stub = types.ModuleType("rospy")
rospy_stub.get_param = lambda name, default=None: _P.get(name, default)
rospy_stub.Publisher = lambda *a, **k: _StubPublisher()
rospy_stub.Subscriber = lambda *a, **k: None
rospy_stub.Service = lambda *a, **k: None
rospy_stub.Timer = lambda *a, **k: None
rospy_stub.Duration = lambda secs=0.0, nsecs=0: None
rospy_stub.Time = lambda *a, **k: None
# 普通 lambda，不用 staticmethod 包装：Python 3.8（真机/容器）里直接
# staticmethod 对象不可调用（3.10 起才可以），会 TypeError。
rospy_stub.Time.now = lambda: None
for _fn in ("loginfo", "logwarn", "logerr", "logdebug",
            "logwarn_throttle", "logerr_throttle", "loginfo_throttle"):
    setattr(rospy_stub, _fn, lambda *a, **k: _LOG.append(a))
sys.modules["rospy"] = rospy_stub


# tf 打桩：真 tf 在 import 期会拉 tf2_ros/actionlib（要 roscore）。
# tf.transformations 是纯数学，这里等价实现被用到的两个函数，
# 顺便让测试守得住 WORKLOG 坑 18（度当弧度喂欧拉角）。
def _quaternion_from_euler(roll, pitch, yaw):
    """⚠ 吃弧度。传度进来不报错、只是安静地算出另一个合法姿态——正是坑 18。"""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _quaternion_matrix(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0.0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0.0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


tf_transformations_stub = types.ModuleType("tf.transformations")
tf_transformations_stub.quaternion_from_euler = _quaternion_from_euler
tf_transformations_stub.quaternion_matrix = _quaternion_matrix
tf_stub = types.ModuleType("tf")
tf_stub.TransformListener = lambda *a, **k: None
tf_stub.transformations = tf_transformations_stub
sys.modules["tf"] = tf_stub
sys.modules["tf.transformations"] = tf_transformations_stub

im_stub = types.ModuleType("interactive_markers")
im_server_stub = types.ModuleType("interactive_markers.interactive_marker_server")
im_server_stub.InteractiveMarkerServer = lambda *a, **k: None
im_stub.interactive_marker_server = im_server_stub
sys.modules["interactive_markers"] = im_stub
sys.modules["interactive_markers.interactive_marker_server"] = im_server_stub

mg_stub = types.ModuleType("moveit_commander")
mg_stub.MoveGroupCommander = lambda *a, **k: None
mg_stub.roscpp_initialize = lambda *a, **k: None
mg_stub.roscpp_shutdown = lambda *a, **k: None
sys.modules["moveit_commander"] = mg_stub


# ---- 消息类：真包优先，缺了就退回同字段替身（开发机也能跑） ----------------
def _use_real_or_stub(name):
    try:
        __import__(name)
        return True
    except ImportError:
        return False


if not _use_real_or_stub("geometry_msgs.msg"):
    def _make_msg_module(name, classes):
        mod = types.ModuleType(name)
        for cls_name, cls in classes.items():
            setattr(mod, cls_name, cls)
        parts = name.split(".")
        sys.modules[name] = mod
        if parts[0] not in sys.modules:
            pkg = types.ModuleType(parts[0])
            sys.modules[parts[0]] = pkg
        setattr(sys.modules[parts[0]], parts[-1], mod)
        return mod

    class _Vec3(object):
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

    class _Quat(object):
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x, self.y, self.z, self.w = x, y, z, w

    class _Header(object):
        def __init__(self):
            self.frame_id = ""
            self.stamp = None

    class _Pose(object):
        def __init__(self):
            self.position = _Vec3()
            self.orientation = _Quat()

    class _PoseStamped(object):
        def __init__(self):
            self.header = _Header()
            self.pose = _Pose()

    class _Vector3(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class _Color(object):
        def __init__(self):
            self.r = self.g = self.b = self.a = 0.0

    class _Marker(object):
        CUBE, SPHERE, TEXT_VIEW_FACING = 1, 2, 9
        ADD, DELETE, DELETEALL = 0, 2, 3

        def __init__(self):
            self.header = _Header()
            self.ns = ""
            self.id = 0
            self.type = 0
            self.action = 0
            self.pose = _Pose()
            self.scale = _Vec3()
            self.color = _Color()
            self.lifetime = None
            self.text = ""

    class _MarkerArray(object):
        def __init__(self):
            self.markers = []

    class _String(object):
        def __init__(self, data=""):
            self.data = data

    class _Trigger(object):
        pass

    class _TriggerResponse(object):
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message

    class _InteractiveMarker(object):
        def __init__(self):
            self.header = _Header()
            self.name = ""
            self.description = ""
            self.pose = _Pose()
            self.controls = []

    class _Control(object):
        MOVE_PLANE, NONE, BUTTON = 3, 0, 1

        def __init__(self):
            self.name = ""
            self.interaction_mode = 0
            self.always_visible = False
            self.markers = []

    class _Feedback(object):
        MOUSE_DOWN, MOUSE_UP, POSE_UPDATE = 4, 5, 6

    class _Update(object):
        def __init__(self):
            self.erases = []

    _make_msg_module("geometry_msgs.msg", {
        "Pose": _Pose, "PoseStamped": _PoseStamped, "Vector3": _Vector3})
    _make_msg_module("std_msgs.msg", {"String": _String})
    _make_msg_module("std_srvs.srv", {
        "Trigger": _Trigger, "TriggerResponse": _TriggerResponse})
    _make_msg_module("visualization_msgs.msg", {
        "Marker": _Marker, "MarkerArray": _MarkerArray,
        "InteractiveMarker": _InteractiveMarker,
        "InteractiveMarkerControl": _Control,
        "InteractiveMarkerFeedback": _Feedback,
        "InteractiveMarkerUpdate": _Update})

if not _use_real_or_stub("dh_gripper_msgs.msg"):
    class _GripperCtrl(object):
        def __init__(self):
            self.initialize = False
            self.position = 0.0
            self.force = 0.0
            self.speed = 0.0

    class _GripperState(object):
        def __init__(self, position=0.0, grip_state=0, is_initialized=True,
                     target_position=0.0):
            self.position = position
            self.grip_state = grip_state
            self.is_initialized = is_initialized
            self.target_position = target_position

    _make_msg_module("dh_gripper_msgs.msg", {
        "GripperCtrl": _GripperCtrl, "GripperState": _GripperState})

if not _use_real_or_stub("agx_plane_target.srv"):
    class _PutPoint(object):
        """替身：真定义见 srv/PutPoint.srv（工作区建好后就自动用真的）。"""

    class _PutPointRequest(object):
        def __init__(self, point=0):
            self.point = point

    class _PutPointResponse(object):
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message

    class _PickPoint(object):
        """替身：真定义见 srv/PickPoint.srv。"""

    class _PickPointResponse(object):
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message

    _make_msg_module("agx_plane_target.srv", {
        "PutPoint": _PutPoint, "PutPointRequest": _PutPointRequest,
        "PutPointResponse": _PutPointResponse,
        "PickPoint": _PickPoint, "PickPointResponse": _PickPointResponse})

if not _use_real_or_stub("moveit_msgs.msg"):
    class _Constraints(object):
        def __init__(self):
            self.name = ""
            self.position_constraints = []

    class _PositionConstraint(object):
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id="")
            self.link_name = ""
            self.constraint_region = types.SimpleNamespace(
                primitives=[], primitive_poses=[])
            self.weight = 0.0

    class _SolidPrimitive(object):
        BOX = 1

        def __init__(self):
            self.type = 0
            self.dimensions = []

    _make_msg_module("moveit_msgs.msg", {
        "Constraints": _Constraints, "PositionConstraint": _PositionConstraint})
    _make_msg_module("shape_msgs.msg", {"SolidPrimitive": _SolidPrimitive})

import agx_plane_target.srv as _plane_srv  # noqa: E402
if not hasattr(_plane_srv, "PickPoint"):
    # 工作区还没重新生成 PickPoint.srv 时，真模块里只有 PutPoint。
    class _PickPointFallback(object):
        pass

    class _PickPointResponseFallback(object):
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message

    _plane_srv.PickPoint = _PickPointFallback
    _plane_srv.PickPointResponse = _PickPointResponseFallback

from dh_gripper_msgs.msg import GripperState  # noqa: E402

# ---- 载入被测脚本 ----------------------------------------------------------
_spec = importlib.util.spec_from_file_location("plane_target_marker", SCRIPT_PY)
_ptm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_ptm)
PlaneTargetMarker = _ptm.PlaneTargetMarker

_failures = []


def check(name, condition, detail=""):
    if condition:
        print("  ok   %s" % name)
    else:
        print("  FAIL %s %s" % (name, detail))
        _failures.append(name)


class _FakeThread(object):
    """替换 threading.Thread：让 on_put_service 只"接受"不真跑，
    busy 的断言不会被后台线程的 finally 抢走。"""

    def __init__(self, target=None, args=()):
        self.target = target
        self.args = args

    def start(self):
        pass


def without_thread(fn, *a, **k):
    saved = _ptm.threading.Thread
    _ptm.threading.Thread = _FakeThread
    try:
        return fn(*a, **k)
    finally:
        _ptm.threading.Thread = saved


def make_node(enable_arm=True, busy=False):
    """A PlaneTargetMarker with only the attributes the put code touches.
    __init__ is bypassed on purpose: it would open markers and a TF listener."""
    node = object.__new__(PlaneTargetMarker)
    node.lock = threading.RLock()
    node.busy = busy
    node.enable_arm = enable_arm
    node.motion_thread = None
    node._stop_flag = threading.Event()
    node._grip_pos = 0
    node._grip_prev = None
    node._grip_still = 0
    node._grip_stamp = None
    node._grip_tol = _ptm.GRASP_SETTLE_TOL
    node.grip_ctrl_pub = _StubPublisher()
    node.grasp_status_pub = _StubPublisher()
    node.put_status_pub = _StubPublisher()
    node.put_markers_pub = _StubPublisher()
    node._group = None
    node._goal_point = None
    node.server = None
    node._set_dot_color = lambda rgb: None
    node._flash = lambda rgb: None
    node._report_tcp = lambda goal: None
    node.plane_z = _ptm.PLANE_Z
    node.target_quat = None
    node.rpy_deg = None
    return node


def statuses(node):
    return [getattr(m, "data", m) for m in node.put_status_pub.published]


LIFT_Z = 0.300       # 模拟升降在 300mm：link_base 比 link_base_lift0 高 0.3
DECK_DZ = 0.208      # deck_top 在 link_base_lift0 下方 208mm（D18）


def install_pose_in(node, log=None):
    """_pose_in 替身：喂进 lift0 系的点，按"当前升降"换算到 link_base，按
    D18 的 208mm 换算到 deck_top；把每次调用记进 log。"""

    def fake(frame, pose, source_frame):
        if log is not None:
            log.append((frame, source_frame,
                        (pose.position.x, pose.position.y, pose.position.z),
                        (pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w)))
        out = _ptm.Pose()
        out.orientation = pose.orientation
        out.position.x = pose.position.x
        out.position.y = pose.position.y
        if frame == "link_base" and source_frame == _ptm.PUT_FRAME:
            out.position.z = pose.position.z - LIFT_Z
        elif frame == _ptm.MARKER_FRAME and source_frame == _ptm.PUT_FRAME:
            out.position.z = pose.position.z + DECK_DZ
        else:
            raise AssertionError("unexpected transform %s <- %s"
                                 % (frame, source_frame))
        return out

    node._pose_in = fake


def install_motion_stubs(node, log, move_fail_at=None, cart_fail_at=None):
    """_execute_motion / _execute_cartesian 替身：把 apply_goal 作用到一个
    假 group 上记录目标（位姿或命名位姿），只记录调用和参数。"""
    counts = {"move": 0, "cart": 0}

    class _Capture(object):
        def __init__(self):
            self.pose = None
            self.named = None

        def set_pose_target(self, p):
            self.pose = p

        def set_named_target(self, name):
            self.named = name

    def fake_move(desc, apply_goal, flash=True, zone_goal=None,
                  zone_floor=None):
        counts["move"] += 1
        cap = _Capture()
        apply_goal(cap)
        target = None
        if cap.pose is not None:
            target = (cap.pose.position.x, cap.pose.position.y,
                      cap.pose.position.z)
        log.append(("move", desc, target, zone_goal, zone_floor, cap.named))
        if counts["move"] == move_fail_at:
            return False, "规划失败（测试注入）"
        return True, "已到达"

    def fake_cart(desc, target):
        counts["cart"] += 1
        log.append(("cart", desc,
                    (target.position.x, target.position.y, target.position.z)))
        if counts["cart"] == cart_fail_at:
            return False, "直线规划失败（测试注入）"
        return True, "已到达"

    node._execute_motion = fake_move
    node._execute_cartesian = fake_cart
    return log


def install_gripper_stubs(node, log, ready=True, fail=False, fail_at=None):
    node._gripper_ready = lambda: (ready, "夹爪已连接（测试）")
    calls = {"n": 0}

    def fake_move(position, params, what):
        calls["n"] += 1
        log.append(("gripper", position, what))
        if fail or calls["n"] == fail_at:
            return False, position, "%s 超时（测试注入）" % what
        return True, position, "%s 完成，停在 %d（目标 %d）" % (
            what, position, position)

    node._gripper_move = fake_move


def feed(node, positions, period=0.02):
    """后台线程推 /gripper/states 读数。"""
    def run():
        for pos in positions:
            node._gripper_cb(GripperState(position=float(pos)))
            time.sleep(period)
        last = positions[-1] if positions else 0
        while not node._stop_flag.is_set() and getattr(node, "_keep_feeding",
                                                       False):
            node._gripper_cb(GripperState(position=float(last)))
            time.sleep(period)
    t = threading.Thread(target=run)
    t.daemon = True
    t.start()
    return t


# ===========================================================================
print("1. 六个放入点与 M6 记录一致；上方点 = 放入点 +150 mm")
# ===========================================================================
_M6 = {
    1: ("中心", 0.329030, 0.007922, -0.078002),
    2: ("绿轴 +75 mm", 0.330758, -0.067058, -0.078002),
    3: ("绿轴 -75 mm", 0.327302, 0.082902, -0.078002),
    4: ("中心，红轴 -75 mm", 0.254050, 0.006194, -0.078003),
    5: ("绿轴 +75 mm，红轴 -75 mm", 0.255778, -0.068786, -0.078003),
    6: ("绿轴 -75 mm，红轴 -75 mm", 0.252322, 0.081174, -0.078003),
}
check("正好 6 个点，点号 1..6", sorted(_ptm.PUT_POINTS) == [1, 2, 3, 4, 5, 6],
      sorted(_ptm.PUT_POINTS))
for number, (name, x, y, z) in sorted(_M6.items()):
    got = _ptm.PUT_POINTS[number]
    check("点 %d（%s）数值与 M6 一致" % (number, name),
          got[0] == name and [round(v, 6) for v in got[1:]] == [x, y, z], got)
check("坐标帧是 link_base_lift0（不随升降动）",
      _ptm.PUT_FRAME == "link_base_lift0", _ptm.PUT_FRAME)
check("上方接近高度 150 mm", abs(_ptm.PUT_APPROACH_RISE - 0.150) < 1e-12,
      _ptm.PUT_APPROACH_RISE)
node = make_node()
for number, (name, x, y, z) in sorted(_M6.items()):
    pose = node._put_pose(x, y, z + _ptm.PUT_APPROACH_RISE)
    check("点 %d 上方点 z = 放入点 +150 mm（%.6f）" % (number, pose.position.z),
          abs(pose.position.z - (z + 0.150)) < 1e-12, pose.position.z)

# ===========================================================================
print("2. 记录姿态：RPY (180, 0, 1.32) 度 -> 工具轴竖直向下（坑 18 回归）")
# ===========================================================================
q = _ptm.PUT_QUAT
tilt = _ptm._tilt_deg(_ptm._axes_of(q)[2])
check("工具轴离竖直向下 < 0.01 deg（%.4f）" % tilt, abs(tilt) < 0.01, tilt)
# 记录值（M6）：xyzw = (0.999934, 0.011519, 0, 0)
check("四元数与 M6 记录一致（%.6f, %.6f, %.6f, %.6f）" % q,
      all(abs(a - b) < 1e-5 for a, b in
          zip(q, (0.999934, 0.011519, 0.0, 0.0))), q)
# 拧向：绕工具轴（竖直）转 1.32 度，X 轴应在水平面内偏 1.32 度
ax = _ptm._axes_of(q)
yaw = math.degrees(math.atan2(ax[0][1], ax[0][0]))
check("红轴（X）偏 1.32 度（%.3f）" % yaw, abs(abs(yaw) - 1.32) < 0.01, yaw)
# 对照组：把 180 度当弧度直接喂进去（坑 18 的 bug）会明显歪掉
wrong = _ptm._tilt_deg(_ptm._axes_of(
    _quaternion_from_euler(*_ptm.PUT_RPY_DEG))[2])
check("对照组：度当弧度喂会歪 %.1f deg（证明断言有区分力）" % wrong,
      wrong > 20.0, wrong)

# ===========================================================================
print("3. 顺序：闭合 200 -> 上方点 -> 直线下降 -> 松开 1000 -> 直线抬高 -> 回 initial")
# ===========================================================================
_P.clear()
node = make_node()
log = []
flash_calls = []
node._flash = lambda rgb: flash_calls.append(rgb)
install_pose_in(node)
install_motion_stubs(node, log)
install_gripper_stubs(node, log)
node._run_put(3)
check("六步都跑了：%s" % [c[0] for c in log],
      [c[0] for c in log] ==
      ["gripper", "move", "cart", "gripper", "cart", "move"],
      log)
name, x, y, z = _M6[3]
approach_lb = (x, y, z + 0.150 - LIFT_Z)
check("出发前闭合到 200", log[0][1] == _ptm.PUT_GRIP_CLOSE, log[0])
target = log[1][2]
check("第 1 步到上方点（link_base 系，已减升降高度）",
      all(abs(a - b) < 1e-9 for a, b in zip(target, approach_lb)),
      (target, approach_lb))
check("第 2 步直线下降到放入点",
      abs(log[2][2][0] - x) < 1e-9 and abs(log[2][2][2] - (z - LIFT_Z)) < 1e-9,
      log[2])
check("到位后松开到 1000", log[3][1] == 1000, log[3])
check("第 4 步直线抬高回上方点",
      abs(log[4][2][2] - (z + 0.150 - LIFT_Z)) < 1e-9, log[4])
check("第 5 步回 initial（命名位姿）", log[5][5] == "initial", log[5])
texts = statuses(node)
check("先播 busy 再播 ok", texts and texts[0].startswith("busy|") and
      texts[-1].startswith("ok|"), texts)
check("ok 文本写明已放下并回 initial",
      "放入完成" in texts[-1] and "initial" in texts[-1], texts[-1])
check("结束时释放 busy", node.busy is False)
check("结束时红点颜色复位（_flash(COLOR_IDLE)，红点状态不被放入改掉）",
      flash_calls and flash_calls[-1] is _ptm.COLOR_IDLE, flash_calls)
check("松开的力度/速度沿用 ~grasp_* 参数（force=50 默认）",
      node._grasp_params()["force"] == 50)

# 姿态确实用的是 M6 记录姿态
pose_log = []
node2 = make_node()
install_pose_in(node2, pose_log)
install_motion_stubs(node2, [])
install_gripper_stubs(node2, [])
node2._run_put(1)
q_in = pose_log[0][3]
check("送进 _pose_in 的朝向 = M6 四元数",
      all(abs(a - b) < 1e-9 for a, b in zip(q_in, _ptm.PUT_QUAT)), q_in)

# ===========================================================================
print("4. 竖直两段不套黄区约束；规划两段带放入下限与黄区外目标扩盒")
# ===========================================================================
_P.clear()
node = make_node()
log = []
install_pose_in(node)
install_motion_stubs(node, log)
install_gripper_stubs(node, log)
node._run_put(6)
floor = _M6[6][3] + DECK_DZ
moves = [c for c in log if c[0] == "move"]
check("第 1 段带 zone_floor = 放入点高度（deck_top）",
      moves[0][4] is not None and abs(moves[0][4] - floor) < 1e-9, moves[0])
check("第 1 段带 zone_goal = 放入点（xy 相同，z=底面）",
      abs(moves[0][3][0] - _M6[6][1]) < 1e-9 and
      abs(moves[0][3][2] - floor) < 1e-9, moves[0])
check("回 initial 也带同一个下限",
      moves[1][3] is None and abs(moves[1][4] - floor) < 1e-9, moves[1])
check("竖直两段走 _execute_cartesian（实现里清约束，见第 5 组）",
      sum(1 for c in log if c[0] == "cart") == 2, log)

# _zone_bounds：目标在区外扩盒、底面抬高
node = make_node()
node._lookup_in = lambda frame, link: ((0.3, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))
zone_size = list(_ptm.FALLBACK_ZONE_SIZE)
zone_center = list(_ptm.FALLBACK_ZONE_CENTER)
bounds, outside, grown, raised = node._zone_bounds(
    zone_size, zone_center, 0.0, ["tcp_link"], (0.9, 0.0, 0.5), 0.13)
check("区外目标被扩盒包进来", grown == [(0.9, 0.0, 0.5)] and
      bounds[0][1] >= 0.9 and outside == [], (grown, bounds))
check("底面抬到放入点高度", raised is True and abs(bounds[2][0] - 0.13) < 1e-12,
      bounds[2])
bounds2, _o2, grown2, raised2 = node._zone_bounds(
    zone_size, zone_center, 0.0, ["tcp_link"], (0.3, 0.0, 0.23), 0.13)
check("目标在区内不扩盒", grown2 == [] and
      all(abs(a - b) < 1e-9 for a, b in zip(bounds2[0], [0.106, 0.565])),
      (grown2, bounds2))
check("区内时只抬底面", raised2 is True and abs(bounds2[2][0] - 0.13) < 1e-12,
      bounds2[2])
bounds3, _o3, _g3, raised3 = node._zone_bounds(
    zone_size, zone_center, 0.0, ["tcp_link"], None, 0.0)
check("floor 比盒底还低时不抬（保持 0.028）",
      raised3 is False and abs(bounds3[2][0] - 0.028) < 1e-9, bounds3[2])
check("黄区自身数字没被动过（单一真源仍是 safety_plane_viz）",
      abs(_ptm.FALLBACK_ZONE_CENTER[2] - _ptm.FALLBACK_ZONE_SIZE[2] / 2.0
          - 0.028) < 1e-9)

# ===========================================================================
print("5. _execute_cartesian：清掉黄区约束、保留碰撞检查、力度重定时、fraction 检查")
# ===========================================================================
_P.clear()


class _FakeGroup(object):
    def __init__(self, fraction=1.0):
        self.fraction = fraction
        self.cleared = 0
        self.executed = []
        self.retimed = []
        self.calls = []
        self.scaling_seen = None

    def clear_pose_targets(self):
        self.calls.append("clear_pose_targets")

    def clear_path_constraints(self):
        self.cleared += 1

    def compute_cartesian_path(self, waypoints, step):
        self.calls.append(("cartesian", step, len(waypoints)))
        plan = types.SimpleNamespace(joint_trajectory=types.SimpleNamespace(
            points=["p1", "p2"]))
        return plan, self.fraction

    def get_current_state(self):
        return "state"

    def retime_trajectory(self, ref, plan, vel, acc, algo):
        self.retimed.append((ref, vel, acc, algo))
        return types.SimpleNamespace(joint_trajectory=types.SimpleNamespace(
            points=["t1", "t2"]))

    def execute(self, plan, wait=True):
        self.executed.append(plan)
        return True


node = make_node()
node.vel_scaling = 0.2
node._lazy_group = lambda: group
group = _FakeGroup()
ok, detail = node._execute_cartesian("desc", _ptm.Pose())
check("直线段成功", ok is True, detail)
check("明确清了路径约束（竖直段不套黄区）", group.cleared == 1, group.cleared)
check("清完没有再装约束（set_path_constraints 根本没这个替身）", True)
check("按 ~vel_scaling 重定时", group.retimed and
      group.retimed[0][1] == 0.2 and group.retimed[0][2] == 0.2,
      group.retimed)
check("执行的是重定时后的轨迹", group.executed and
      group.executed[0].joint_trajectory.points == ["t1", "t2"],
      group.executed)

group = _FakeGroup(fraction=0.5)
node = make_node()
node.vel_scaling = 0.2
node._lazy_group = lambda: group
ok, detail = node._execute_cartesian("desc", _ptm.Pose())
check("fraction 0.5 -> 失败且不执行", ok is False and not group.executed, detail)
check("失败原因里带百分比", "50" in detail, detail)


class _NoExecGroup(_FakeGroup):
    def execute(self, plan, wait=True):
        return False


group = _NoExecGroup()
node = make_node()
node.vel_scaling = 0.2
node._lazy_group = lambda: group
ok, detail = node._execute_cartesian("desc", _ptm.Pose())
check("执行被取消/停止 -> 失败", ok is False and "取消" in detail, detail)

# ===========================================================================
print("6. 干跑（enable_arm=false）：只规划 1/5，绝不下发夹爪指令")
# ===========================================================================
_P.clear()
node = make_node(enable_arm=False)
log = []
install_pose_in(node)
install_motion_stubs(node, log)
install_gripper_stubs(node, log)
node._run_put(2)
check("只规划了一步（移动到上方点）",
      [c[0] for c in log] == ["move"], log)
check("没有下发任何夹爪指令（真夹爪一个字都没收到）",
      len(node.grip_ctrl_pub.published) == 0)
check("没有走直线下降/抬高", not any(c[0] == "cart" for c in log), log)
texts = statuses(node)
check("干跑状态里写明跳过与未执行",
      any("跳过" in t for t in texts) and any("都未执行" in t for t in texts),
      texts)
check("干跑结束释放 busy", node.busy is False)
resp = without_thread(node.on_put_service, type("Req", (), {"point": 2})())
check("服务回复也是干跑口径", resp.success is True and "干跑" in resp.message,
      resp.message)

# ===========================================================================
print("7. busy / 非法点号：拒绝且不动")
# ===========================================================================
node = make_node(busy=True)
resp = node.on_put_service(type("Req", (), {"point": 3})())
check("busy 时 success=False", resp.success is False, resp.message)
check("busy 时 busy 保持 True", node.busy is True)
check("busy 时没起线程", node.motion_thread is None)

node = make_node()
for bad in (0, 7, -1):
    resp = node.on_put_service(type("Req", (), {"point": bad})())
    check("非法点号 %d 被拒" % bad, resp.success is False, resp.message)
check("非法点号没有占用 busy", node.busy is False)
check("非法点号的提示里指向 M6", "M6" in resp.message, resp.message)

node = make_node()
resp = without_thread(node.on_put_service, type("Req", (), {"point": 4})())
check("合法点号接受并占住 busy（整段动作独占控制器）",
      resp.success is True and node.busy is True, resp.message)
check("回复里带点号与名字", "4" in resp.message and "红轴" in resp.message,
      resp.message)

# ===========================================================================
print("8. 失败路径：任一环节失败都如实报 fail、释放 busy、不硬走下一步")
# ===========================================================================
# (标签, 运动替身注入, 夹爪就绪, 张开失败, 期望停在哪些调用)
cases = [
    ("闭合到 200", dict(), True, True, None, ["gripper"]),
    ("1/5 到上方点", dict(move_fail_at=1), True, False, None,
     ["gripper", "move"]),
    ("2/5 下降", dict(cart_fail_at=1), True, False, None,
     ["gripper", "move", "cart"]),
    ("3/5 松开（张开超时）", dict(), True, False, 2,
     ["gripper", "move", "cart", "gripper"]),
    ("4/5 抬高", dict(cart_fail_at=2), True, False, None,
     ["gripper", "move", "cart", "gripper", "cart"]),
    ("5/5 回 initial", dict(move_fail_at=2), True, False, None,
     ["gripper", "move", "cart", "gripper", "cart", "move"]),
]
for tag, motion_kw, ready, grip_fail, fail_at, expect in cases:
    _P.clear()
    node = make_node()
    log = []
    install_pose_in(node)
    install_motion_stubs(node, log, **motion_kw)
    install_gripper_stubs(node, log, ready=ready, fail=grip_fail,
                          fail_at=fail_at)
    node._run_put(1)
    texts = statuses(node)
    check("%s 失败 -> fail 状态" % tag, texts[-1].startswith("fail|"), texts[-1])
    check("%s 失败 -> 停在该步（%s）" % (tag, [c[0] for c in log]),
          [c[0] for c in log] == expect, log)
    check("%s 失败 -> 释放 busy" % tag, node.busy is False)

# 夹爪没驱动：松开那步明确报错（还没下发任何指令）
_P.clear()
node = make_node()
log = []
install_pose_in(node)
install_motion_stubs(node, log)
install_gripper_stubs(node, log, ready=False)
node._run_put(1)
texts = statuses(node)
check("夹爪没起 -> fail 且提示夹爪", texts[-1].startswith("fail|") and
      "夹爪" in texts[-1], texts[-1])
check("夹爪没起 -> 没有下发任何夹爪指令",
      len(node.grip_ctrl_pub.published) == 0)

# ===========================================================================
print("9. 急停：标志置位后不在步骤之间继续")
# ===========================================================================
_P.clear()
node = make_node()
log = []


def stop_after_first(desc, apply_goal, flash=True, zone_goal=None,
                     zone_floor=None):
    log.append("move")
    node._stop_flag.set()          # 模拟第 1 步执行期间按下急停
    return True, "已到达"


node._execute_motion = stop_after_first
node._execute_cartesian = lambda desc, target: (
    log.append("cart") or (True, "已到达"))
node._gripper_ready = lambda: (True, "ok")
node._gripper_move = lambda *a: (log.append("gripper") or (True, 1000, "ok"))
install_pose_in(node)
node._run_put(1)
texts = statuses(node)
check("急停后不再下降（log=%s）" % log, log == ["gripper", "move"], log)
check("急停报 fail 且说明停在哪", texts[-1].startswith("fail|") and
      "急停" in texts[-1], texts[-1])
check("急停后释放 busy", node.busy is False)

# ===========================================================================
print("10. 六红球 Marker：一批 12 个（6 球 + 6 编号），坐标系 link_base_lift0")
# ===========================================================================
node = make_node()
node.put_markers_pub = _StubPublisher()
node._publish_put_markers()
arr = node.put_markers_pub.published[0]
markers = arr.markers
check("一批 12 个 marker", len(markers) == 12, len(markers))
balls = [m for m in markers if m.type == _ptm.Marker.SPHERE]
labels = [m for m in markers if m.type == _ptm.Marker.TEXT_VIEW_FACING]
check("6 个球 + 6 个编号", len(balls) == 6 and len(labels) == 6,
      (len(balls), len(labels)))
check("球 id = 点号 1..6", sorted(m.id for m in balls) == [1, 2, 3, 4, 5, 6],
      [m.id for m in balls])
check("编号 id = 100+点号", sorted(m.id for m in labels) ==
      [101, 102, 103, 104, 105, 106], [m.id for m in labels])
check("全部在 link_base_lift0 系",
      all(m.header.frame_id == "link_base_lift0" for m in markers))
check("同一个 ns（RViz 配置只列 put_points）",
      all(m.ns == "put_points" for m in markers))
check("球是红的", all(m.color.r > 0.8 and m.color.g < 0.3 for m in balls),
      [(m.color.r, m.color.g) for m in balls])
for m in balls:
    name, x, y, z = _M6[m.id]
    ok = (abs(m.pose.position.x - x) < 1e-9 and
          abs(m.pose.position.y - y) < 1e-9 and
          abs(m.pose.position.z - z) < 1e-9)
    check("点 %d 球在 M6 坐标上" % m.id, ok,
          (m.pose.position.x, m.pose.position.y, m.pose.position.z))
check("编号文本 = 点号", [m.text for m in labels] ==
      ["1", "2", "3", "4", "5", "6"], [m.text for m in labels])
node._publish_put_markers(_ptm.Marker.DELETE)
check("关闭时发 DELETE（不然 RViz 永远留着）",
      all(m.action == _ptm.Marker.DELETE
          for m in node.put_markers_pub.published[-1].markers))

# ===========================================================================
print("11. put_status 话题格式")
# ===========================================================================
node = make_node()
node._put_status("busy", "放入 2/5：直线下降到放入点")
node._put_status("fail", "放入中止（2/5 下降）：直线只规划出 50%")
data = statuses(node)
check("busy 带分隔符和文本", data[0] == "busy|放入 2/5：直线下降到放入点",
      data)
check("文本里的 | 不会被误切（只切第一个）", data[1].count("|") == 1, data)

print("")
if _failures:
    print("==> %d 项失败：%s" % (len(_failures), ", ".join(_failures)))
    sys.exit(1)
print("==> test_put 全部通过")
