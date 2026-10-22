#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plane_target_marker.py 抓取（pick）逻辑测试

与放入平行，顺序相反的是夹爪时机：
  从初始位置 -> 上方安全高度 -> 直线下降 -> 闭合到 200 -> 直线抬高 -> 回 initial
下降之前不下发任何夹爪指令。

不碰串口、不碰真机、不需要 roscore。
"""

import importlib.util
import os
import sys
import threading
import types

SCRIPT_PY = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "agx_plane_target", "scripts", "plane_target_marker.py"))

_P = {}


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
rospy_stub.Time.now = lambda: None
for _fn in ("loginfo", "logwarn", "logerr", "logdebug",
            "logwarn_throttle", "logerr_throttle", "loginfo_throttle"):
    setattr(rospy_stub, _fn, lambda *a, **k: None)
sys.modules["rospy"] = rospy_stub

tf_stub = types.ModuleType("tf")
tf_stub.TransformListener = lambda *a, **k: None
tf_trans = types.ModuleType("tf.transformations")


def _quaternion_from_euler(roll, pitch, yaw):
    import math
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _quaternion_matrix(q):
    import math
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0.0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0.0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


tf_trans.quaternion_from_euler = _quaternion_from_euler
tf_trans.quaternion_matrix = _quaternion_matrix
tf_stub.transformations = tf_trans
sys.modules["tf"] = tf_stub
sys.modules["tf.transformations"] = tf_trans

im_stub = types.ModuleType("interactive_markers")
im_server = types.ModuleType("interactive_markers.interactive_marker_server")
im_server.InteractiveMarkerServer = lambda *a, **k: None
im_stub.interactive_marker_server = im_server
sys.modules["interactive_markers"] = im_stub
sys.modules["interactive_markers.interactive_marker_server"] = im_server

mg_stub = types.ModuleType("moveit_commander")
mg_stub.MoveGroupCommander = lambda *a, **k: None
mg_stub.roscpp_initialize = lambda *a, **k: None
mg_stub.roscpp_shutdown = lambda *a, **k: None
sys.modules["moveit_commander"] = mg_stub

try:
    import agx_plane_target.srv as _plane_srv
except ImportError:
    _plane_srv = types.ModuleType("agx_plane_target.srv")
    _pkg = types.ModuleType("agx_plane_target")
    _pkg.srv = _plane_srv
    sys.modules["agx_plane_target"] = _pkg
    sys.modules["agx_plane_target.srv"] = _plane_srv

if not hasattr(_plane_srv, "PutPoint"):
    class _PutPoint(object):
        pass

    class _PutPointResponse(object):
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message

    _plane_srv.PutPoint = _PutPoint
    _plane_srv.PutPointResponse = _PutPointResponse

if not hasattr(_plane_srv, "PickPoint"):
    class _PickPoint(object):
        pass

    class _PickPointResponse(object):
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message

    _plane_srv.PickPoint = _PickPoint
    _plane_srv.PickPointResponse = _PickPointResponse

_spec = importlib.util.spec_from_file_location("plane_target_marker_pick", SCRIPT_PY)
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
    node.pick_status_pub = _StubPublisher()
    node.put_markers_pub = _StubPublisher()
    node._group = None
    node.server = None
    node._set_dot_color = lambda rgb: None
    node._flash = lambda rgb: None
    node._report_tcp = lambda goal: None
    return node


def statuses(node):
    return [getattr(m, "data", m) for m in node.pick_status_pub.published]


LIFT_Z = 0.300
DECK_DZ = 0.208


def install_pose_in(node):
    def fake(frame, pose, source_frame):
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


def install_gripper_stubs(node, log, ready=True, fail=False, fail_at=None,
                          settle_at=None):
    """settle_at maps 1-based call index -> reported stop position.
    A successful open must report >= PICK_OPEN_MIN; a successful close
    must report below that, or the pick refuses to lift."""
    node._gripper_ready = lambda: (ready, "夹爪已连接（测试）" if ready
                                   else "夹爪没起（测试）")
    calls = {"n": 0}

    def fake_move(position, params, what):
        calls["n"] += 1
        stopped = position
        if settle_at and calls["n"] in settle_at:
            stopped = settle_at[calls["n"]]
        log.append(("gripper", position, what, stopped))
        if fail or calls["n"] == fail_at:
            return False, stopped, "%s 超时（测试注入）" % what
        return True, stopped, "%s 完成，停在 %d（目标 %d）" % (
            what, stopped, position)

    node._gripper_move = fake_move


def arm(node, **motion_kw):
    log = []
    install_pose_in(node)
    install_motion_stubs(node, log, **motion_kw)
    install_gripper_stubs(node, log)
    return log


print("1. 顺序：先张开确认 -> 上方安全高度 -> 下降 -> 闭合 200 -> 抬高 -> 回 initial")
_P.clear()
node = make_node()
log = arm(node)
node._run_pick(3)
name, x, y, z = _ptm.PUT_POINTS[3]
approach_z = z + _ptm.PUT_APPROACH_RISE - LIFT_Z
point_z = z - LIFT_Z
check("出发前先张开，再运动，到位后再闭合",
      [c[0] for c in log] ==
      ["gripper", "move", "cart", "gripper", "cart", "move"], log)
check("第一条夹爪指令是张开到 1000",
      log[0][1] == _ptm.GRASP_OPEN and log[0][3] >= _ptm.PICK_OPEN_MIN, log[0])
check("到位后闭合目标是 200", log[3][1] == _ptm.PUT_GRIP_CLOSE, log[3])
check("第 1 步到上方安全高度",
      abs(log[1][2][0] - x) < 1e-9 and abs(log[1][2][2] - approach_z) < 1e-9,
      log[1])
check("第 1 步带放入下限", log[1][4] is not None, log[1])
check("第 2 步直线下降到目标点",
      abs(log[2][2][0] - x) < 1e-9 and abs(log[2][2][2] - point_z) < 1e-9,
      log[2])
check("第 4 步直线抬回安全高度",
      abs(log[4][2][2] - approach_z) < 1e-9, log[4])
check("第 5 步回 initial", log[5][5] == "initial", log[5])
texts = statuses(node)
check("先 busy 再 ok", texts and texts[0].startswith("busy|") and
      texts[-1].startswith("ok|"), texts)
check("ok 写明闭合到 200 并回 initial",
      "抓取完成" in texts[-1] and "200" in texts[-1] and "initial" in texts[-1],
      texts[-1])
check("结束释放 busy", node.busy is False)
check("放入状态话题没被抓取占用", node.put_status_pub.published == [])

print("2. 干跑：只规划上方点，不碰夹爪")
node = make_node(enable_arm=False)
log = arm(node)
node._run_pick(1)
check("只规划了一步", [c[0] for c in log] == ["move"], log)
check("没有夹爪指令", not any(c[0] == "gripper" for c in log), log)
texts = statuses(node)
check("干跑说明未执行闭合", any("都未执行" in t for t in texts), texts)
check("干跑释放 busy", node.busy is False)

print("3. busy / 非法点号")
node = make_node(busy=True)
resp = node.on_pick_service(type("Req", (), {"point": 3})())
check("busy 拒绝", resp.success is False and node.busy is True, resp.message)
check("busy 没起线程", node.motion_thread is None)
node = make_node()
for bad in (0, 7, -1):
    resp = node.on_pick_service(type("Req", (), {"point": bad})())
    check("非法点号 %d 被拒" % bad, resp.success is False, resp.message)
check("非法点号不占 busy", node.busy is False)
resp = without_thread(node.on_pick_service, type("Req", (), {"point": 2})())
check("合法点号占住 busy", resp.success is True and node.busy is True,
      resp.message)
check("回复里有张开确认和闭合 200",
      "2" in resp.message and "张开" in resp.message and "200" in resp.message,
      resp.message)

print("4. 失败停在该步")
cases = [
    ("张开确认失败", {}, dict(fail_at=1), ["gripper"]),
    ("张开后仍闭合（停在 200）", {}, dict(settle_at={1: 200}), ["gripper"]),
    ("1/5 到上方点", dict(move_fail_at=1), {}, ["gripper", "move"]),
    ("2/5 下降", dict(cart_fail_at=1), {}, ["gripper", "move", "cart"]),
    ("3/5 闭合", {}, dict(fail_at=2),
     ["gripper", "move", "cart", "gripper"]),
    ("闭合后仍停在张开（941）", {}, dict(settle_at={2: 941}),
     ["gripper", "move", "cart", "gripper"]),
    ("夹爪没起", {}, dict(ready=False), []),
    ("4/5 抬高", dict(cart_fail_at=2), {},
     ["gripper", "move", "cart", "gripper", "cart"]),
    ("5/5 回 initial", dict(move_fail_at=2), {},
     ["gripper", "move", "cart", "gripper", "cart", "move"]),
]
for tag, motion_kw, grip_kw, expect in cases:
    node = make_node()
    log = []
    install_pose_in(node)
    install_motion_stubs(node, log, **motion_kw)
    install_gripper_stubs(node, log, **grip_kw)
    node._run_pick(1)
    texts = statuses(node)
    check("%s -> fail" % tag, texts[-1].startswith("fail|"), texts[-1])
    check("%s -> 停在 %s" % (tag, expect),
          [c[0] for c in log] == expect, log)
    check("%s -> 释放 busy" % tag, node.busy is False)

print("5. 急停：到上方点后不再下降")
node = make_node()
log = []


def stop_after_approach(desc, apply_goal, flash=True, zone_goal=None,
                        zone_floor=None):
    log.append("move")
    node._stop_flag.set()
    return True, "已到达"


node._execute_motion = stop_after_approach
node._execute_cartesian = lambda desc, target: (log.append("cart") or (True, "ok"))
node._gripper_ready = lambda: (True, "ok")


def fake_grip(position, params, what):
    log.append(("gripper", position))
    return True, position, "ok"


node._gripper_move = fake_grip
install_pose_in(node)
node._run_pick(1)
texts = statuses(node)
check("急停后不再下降",
      log == [("gripper", _ptm.GRASP_OPEN), "move"], log)
check("出发前已经张开", log[0][1] == _ptm.GRASP_OPEN, log)
check("急停说明未闭合", texts[-1].startswith("fail|") and "未闭合" in texts[-1],
      texts[-1])
check("急停释放 busy", node.busy is False)

print("")
if _failures:
    print("==> %d 项失败：%s" % (len(_failures), ", ".join(_failures)))
    sys.exit(1)
print("==> test_pick 全部通过")
