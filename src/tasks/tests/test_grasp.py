#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plane_target_marker.py 夹取（move + close）逻辑测试

覆盖 /plane_target/grasp 这条新链路上**纯逻辑、可离线**的部分：
  1. ~grasp_* 参数钳位（position 0~1000 / force 20~100 / speed）
  2. 到位判定：位置不再变化才算到位（不猜 grip_state，见 measurements.md M4）
  3. 起步窗口：指令刚发出、手指还没动时不能误判到位
  4. 急停能中断闭合，并下发"保持"指令（AG95 没有 cancel）
  5. 超时 / 状态流中断都要报**实际读数**，不能谎报成功
  6. busy 时拒绝夹取
  7. 干跑（enable_arm=false）**只规划、绝不闭合真夹爪**
  8. 正常流程的指令顺序：先张开 1000，再移到红点，最后闭合到目标值

⚠ 本测试**不碰串口、不碰真机**。rospy / tf / interactive_markers / moveit_commander
全部打桩，节点用 object.__new__ 绕过 __init__，因此**不需要 roscore**，
也不会和正在运行的 setup_arm.launch 抢节点名 / 抢 xarm6 控制器。
MoveIt 那一步同样被替换成桩，所以**不会规划也不会动机械臂**。

跑法：
    source /opt/ros/noetic/setup.bash && source ~/agilex_ws/devel/setup.bash
    python3 src/tasks/tests/test_grasp.py
"""

import importlib.util
import math
import os
import sys
import threading
import time
import types

SCRIPT_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "agx_plane_target", "scripts", "plane_target_marker.py")
SCRIPT_PY = os.path.normpath(SCRIPT_PY)

# ---- 打桩：让测试完全不依赖 roscore / MoveIt / 真机 ------------------------
_PARAMS = {}
_LOG = []


class _StubPublisher(object):
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)

    def get_num_connections(self):
        return 1


rospy_stub = types.ModuleType("rospy")
rospy_stub.get_param = lambda name, default=None: _PARAMS.get(name, default)
rospy_stub.Publisher = lambda *a, **k: _StubPublisher()
rospy_stub.Subscriber = lambda *a, **k: None
rospy_stub.Service = lambda *a, **k: None
rospy_stub.Timer = lambda *a, **k: None
# tf2_ros 在 import 期就会摸 rospy.Duration，所以这两个必须先摆好（见下方 tf 打桩）
rospy_stub.Duration = lambda secs=0.0, nsecs=0: None
rospy_stub.Time = lambda *a, **k: None
for _fn in ("loginfo", "logwarn", "logerr", "logdebug",
            "logwarn_throttle", "logerr_throttle", "loginfo_throttle"):
    setattr(rospy_stub, _fn, lambda *a, **k: _LOG.append(a))
sys.modules["rospy"] = rospy_stub

# tf 换成打桩：真 tf 在 import 期就会拉 tf2_ros/actionlib，那条链要 roscore。
# tf.transformations 本身是纯数学，这里等价实现两个函数（只实现被用到的），
# 顺便让测试能守住 WORKLOG 坑 18（度当弧度喂给欧拉角函数）那个回归。
def _quaternion_from_euler(roll, pitch, yaw):
    """⚠ 吃弧度。传度进来不报错、只是安静地算出另一个合法姿态——这正是坑 18。"""
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

# plane_target_marker 现在还会 import agx_plane_target.srv（放入服务 PutPoint，
# 见 srv/PutPoint.srv）。工作区没（重）构建时名字不存在，退回同字段替身，
# 让这个纯逻辑测试照样能跑；构建过之后用的就是真消息。
try:
    from agx_plane_target.srv import PutPoint, PutPointResponse  # noqa: F401
except ImportError:
    _put_srv_mod = types.ModuleType("agx_plane_target.srv")

    class _PutPointStub(object):
        pass

    class _PutPointResponseStub(object):
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message

    _put_srv_mod.PutPoint = _PutPointStub
    _put_srv_mod.PutPointResponse = _PutPointResponseStub
    _pkg_stub = types.ModuleType("agx_plane_target")
    _pkg_stub.srv = _put_srv_mod
    sys.modules["agx_plane_target"] = _pkg_stub
    sys.modules["agx_plane_target.srv"] = _put_srv_mod

import agx_plane_target.srv as _plane_srv  # noqa: E402
if not hasattr(_plane_srv, "PickPoint"):
    class _PickPointStub(object):
        pass

    class _PickPointResponseStub(object):
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message

    _plane_srv.PickPoint = _PickPointStub
    _plane_srv.PickPointResponse = _PickPointResponseStub

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


def make_node(enable_arm=True, busy=False):
    """A PlaneTargetMarker with only the attributes the grasp code touches.
    __init__ is bypassed on purpose: it would open markers and a TF listener."""
    node = object.__new__(PlaneTargetMarker)
    node.lock = threading.RLock()
    node.busy = busy
    node.dot_xy = (0.3355, 0.0)
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
    node._group = None
    node._goal_point = None
    # marker / TF 都打桩：夹取流程只会碰这两个来闪颜色和读位姿
    node.server = None
    node._set_dot_color = lambda rgb: None
    node._flash = lambda rgb: None
    node._report_tcp = lambda goal: None
    node.plane_z = _ptm.PLANE_Z
    node.target_quat = None
    node.rpy_deg = None
    return node


class _FakeThread(object):
    """替换 threading.Thread：让 on_grasp_service 只"接受"不真跑，
    这样 busy 的断言不会被后台线程的 finally 抢走。"""

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


def feed(node, positions, period=0.02):
    """Background thread pushing /gripper/states readings."""
    def run():
        for pos in positions:
            node._gripper_cb(GripperState(position=float(pos)))
            time.sleep(period)
        # keep the stream alive (fresh stamp) at the last value
        last = positions[-1] if positions else 0
        while not node._stop_flag.is_set() and getattr(node, "_keep_feeding", False):
            node._gripper_cb(GripperState(position=float(last)))
            time.sleep(period)
    t = threading.Thread(target=run)
    t.daemon = True
    t.start()
    return t


def sent_positions(node):
    return [int(round(c.position)) for c in node.grip_ctrl_pub.published]


def sent_forces(node):
    return [int(round(c.force)) for c in node.grip_ctrl_pub.published]


# ===========================================================================
print("1. ~grasp_* 参数钳位")
# ===========================================================================
_PARAMS.clear()
node = make_node()
p = node._grasp_params()
check("默认 position=0", p["position"] == 0, p)
check("默认 force=50", p["force"] == 50, p)
check("默认 speed=50", p["speed"] == 50, p)
check("默认 preopen=True", p["preopen"] is True, p)

_PARAMS.clear()
_PARAMS["~grasp_position"] = 5000
_PARAMS["~grasp_force"] = 999
_PARAMS["~grasp_speed"] = 0
p = node._grasp_params()
check("position 钳到 1000", p["position"] == 1000, p)
check("force 钳到 100（AG95 上限）", p["force"] == 100, p)
check("speed 钳到 1", p["speed"] == 1, p)

_PARAMS.clear()
_PARAMS["~grasp_position"] = -5
_PARAMS["~grasp_force"] = 5
p = node._grasp_params()
check("position 钳到 0", p["position"] == 0, p)
check("force 钳到 20（AG95 下限，写 5 会被驱动静默改写）",
      p["force"] == 20, p)

# ===========================================================================
print("2. 到位判定：位置不再变化才算到位")
# ===========================================================================
_PARAMS.clear()
node = make_node()
node._keep_feeding = True
# 1000 -> 300 匀速过去然后停住
feed(node, list(range(1000, 299, -25)) + [300] * 40)
ok, pos, msg = node._gripper_move(300, node._grasp_params(), "闭合")
check("闭合到位返回 ok", ok is True, msg)
check("读到实际停住位置 300", pos == 300, "pos=%s" % pos)
check("回报里带目标值", "300" in msg, msg)
check("只下发了一条指令", len(node.grip_ctrl_pub.published) == 1,
      sent_positions(node))
check("指令位置=目标 300", sent_positions(node) == [300], sent_positions(node))
node._keep_feeding = False

# ===========================================================================
print("3. 起步窗口：手指还没动时不能误判到位")
# ===========================================================================
_PARAMS.clear()
node = make_node()
node._keep_feeding = True
node._grip_pos = 300
node._gripper_cb(GripperState(position=300.0))   # 已经在 300，指令也是 300
# 指令发出后 0.3 s 才开始动，模拟"驱动响应慢"
def late_motion():
    time.sleep(0.3)
    for pos in range(300, 199, -20):
        node._gripper_cb(GripperState(position=float(pos)))
        time.sleep(0.02)
    for _ in range(40):
        node._gripper_cb(GripperState(position=200.0))
        time.sleep(0.02)
threading.Thread(target=late_motion, daemon=True).start()
t0 = time.time()
ok, pos, msg = node._gripper_move(200, node._grasp_params(), "闭合")
dt = time.time() - t0
check("等到了真正停下的 200", ok is True and pos == 200, "pos=%s %s" % (pos, msg))
check("没有在 0.3 s 起步窗口内误判（耗时 %.2fs）" % dt, dt >= 0.3, "%.3f" % dt)
node._keep_feeding = False

# ===========================================================================
print("4. 急停中断闭合 -> 下发保持指令")
# ===========================================================================
_PARAMS.clear()
node = make_node()
node._keep_feeding = True
feed(node, [1000, 800, 600, 400] * 30)      # 一直在动，不会自己停
time.sleep(0.1)
threading.Timer(0.15, node._stop_flag.set).start()   # 模拟点「急停」
ok, pos, msg = node._gripper_move(0, node._grasp_params(), "闭合")
check("急停后返回失败", ok is False, msg)
check("回报说是被急停中断", "急停" in msg, msg)
check("下发了保持指令（AG95 无 cancel，只能保持）",
      len(node.grip_ctrl_pub.published) == 2 and
      sent_positions(node)[1] == int(round(pos)),
      sent_positions(node))
node._keep_feeding = False

# ===========================================================================
print("5. 超时 / 状态流中断 -> 报实际读数，不谎报成功")
# ===========================================================================
_PARAMS.clear()
_PARAMS["~grasp_settle_timeout"] = 0.6
_PARAMS["~grasp_settle_tol"] = 0      # 任何抖动都算在动
node = make_node()
node._keep_feeding = True
feed(node, [1000, 999, 1000, 999] * 60)   # 永远在抖，永远不到位
ok, pos, msg = node._gripper_move(0, node._grasp_params(), "闭合")
check("超时返回失败", ok is False, msg)
check("回报里带超时和实际位置", "超时" in msg and str(pos) in msg, msg)
check("实际位置是抖动中的读数", pos in (999, 1000), pos)
node._keep_feeding = False

_PARAMS.clear()
node = make_node()          # 从没收到过 /gripper/states
ok, pos, msg = node._gripper_move(0, node._grasp_params(), "闭合")
check("状态流中断返回失败", ok is False, msg)
check("明说是状态流问题", "状态流" in msg, msg)

# ===========================================================================
print("6. busy 时拒绝夹取")
# ===========================================================================
_PARAMS.clear()
node = make_node(busy=True)
resp = node.on_grasp_service(None)
check("busy 时 success=False", resp.success is False, resp.message)
check("busy 时不发指令", len(node.grip_ctrl_pub.published) == 0)

node = make_node(busy=False)
# 不真的起线程，直接验证 busy 被占用 + 回复内容
resp = without_thread(node.on_grasp_service, None)
check("空闲时 success=True", resp.success is True, resp.message)
check("接受后 busy=True（整段流程独占 xarm6 控制器）", node.busy is True)
check("回复里带闭合目标", "闭合目标 0" in resp.message, resp.message)

# 干跑模式的回复也必须回显闭合目标，否则控制台无法确认参数写进去了
node = make_node(enable_arm=False)
resp = without_thread(node.on_grasp_service, None)
check("干跑回复带闭合目标", "闭合目标 0" in resp.message, resp.message)
check("干跑回复说明不会下发", "不会下发" in resp.message, resp.message)

# ===========================================================================
print("7. 干跑（enable_arm=false）绝不闭合真夹爪")
# ===========================================================================
_PARAMS.clear()
node = make_node(enable_arm=False)
node._keep_feeding = True
planned = []


def fake_execute(desc, apply_goal, flash=True):
    planned.append(desc)
    return True, "干跑：规划通过"


node._execute_motion = fake_execute
node._run_grasp(0.3355, 0.0, node._grasp_params())
check("干跑只规划了一次", len(planned) == 1, planned)
check("干跑没有下发任何夹爪指令",
      len(node.grip_ctrl_pub.published) == 0, sent_positions(node))
_status_data = [getattr(m, "data", m) for m in node.grasp_status_pub.published]
check("干跑状态回报里写明未闭合",
      any("未闭合" in d for d in _status_data), _status_data)
check("干跑也先播 busy（控制台据此把按钮置灰）",
      _status_data and _status_data[0].startswith("busy|"), _status_data)
check("干跑的 busy/ok 都带闭合目标（能确认参数写进去了）",
      any("闭合目标 0" in d for d in _status_data), _status_data)
check("干跑结束释放 busy", node.busy is False)
node._keep_feeding = False

# ===========================================================================
print("8. 正常流程指令顺序：张开 1000 -> 移到红点 -> 闭合目标")
# ===========================================================================
_PARAMS.clear()
node = make_node()
node._keep_feeding = True
order = []


def fake_execute2(desc, apply_goal, flash=True):
    order.append("move")
    return True, "已到达红点上方"


node._execute_motion = fake_execute2
node._set_dot_color = lambda rgb: None
node._flash = lambda rgb: None
node._report_tcp = lambda goal: None

# 第一次夹取：张开 -> 移动 -> 闭合到 0（夹爪一路走到 0 才算到位）
feed(node, [1000] * 5 + list(range(1000, -1, -100)) * 6)
node._run_grasp(0.3355, 0.0, node._grasp_params())
check("移动被调用了一次", order == ["move"], order)
sent = sent_positions(node)
check("第一条指令是张开 1000", sent and sent[0] == 1000, sent)
check("最后一条指令是闭合目标 0", sent and sent[-1] == 0, sent)
check("力度 50 保持不变（和夹爪滑条一直发的一致）",
      set(sent_forces(node)) == {50}, sent_forces(node))
# 张开那一步也要报出实际读数：万一张开没到 1000（卡滞/里面还夹着东西），
# 后面闭合就是在错误起点上合拢，操作者必须看得到。
texts = [getattr(m, "data", m) for m in node.grasp_status_pub.published]
check("张开步骤回报了实际停住位置",
      any("张开" in t and "停在" in t for t in texts), texts)
check("干跑/正常流程的每一步都有状态（不会静默）",
      sum(1 for t in texts if t.startswith("busy|")) >= 3, texts)
node._keep_feeding = False

# 第二次：preopen 关掉，就不该再发张开
_PARAMS.clear()
_PARAMS["~grasp_preopen"] = False
node = make_node()
node._keep_feeding = True
order = []
node._execute_motion = fake_execute2
node._set_dot_color = lambda rgb: None
node._flash = lambda rgb: None
feed(node, [1000] * 5 + list(range(1000, 299, -100)) * 8)
node._run_grasp(0.3355, 0.0, node._grasp_params())
sent = sent_positions(node)
check("preopen=false 时第一条就是闭合目标 0", sent == [0], sent)
check("移动仍被调用一次", order == ["move"], order)
node._keep_feeding = False

# ===========================================================================
print("9. _gripper_ready：驱动没起时必须明确报错而不是干等超时")
# ===========================================================================
_PARAMS.clear()
node = make_node()


class _NoConnPublisher(_StubPublisher):
    def get_num_connections(self):
        return 0


node.grip_ctrl_pub = _NoConnPublisher()
ok, why = node._gripper_ready()
check("没有订阅者 -> 不就绪", ok is False, why)
check("提示里点名 /gripper/ctrl", "/gripper/ctrl" in why, why)

node = make_node()
ok, why = node._gripper_ready()
check("没有状态回读 -> 不就绪", ok is False, why)
check("提示里提醒是复数话题", "复数" in why, why)

node = make_node()
node._gripper_cb(GripperState(position=777.0))
ok, why = node._gripper_ready()
check("有回读 -> 就绪", ok is True, why)
check("就绪提示带当前读数 777", "777" in why, why)

# ===========================================================================
print("10. _status 的话题格式")
# ===========================================================================
node = make_node()
node._status("busy", "夹取 2/3：移动到红点")
node._status("ok", "夹取完成：闭合 完成，停在 300（目标 300）")
data = [getattr(m, "data", m) for m in node.grasp_status_pub.published]
check("busy 状态带分隔符和文本",
      data[0] == "busy|夹取 2/3：移动到红点", data)
check("文本里的 | 不会被误切（只切第一个）",
      data[1].count("|") == 1, data)

# ===========================================================================
print("11. 夹取复用的 _dot_goal 仍把 ~rpy_deg 的【度】当度用（坑 18 回归）")
# ===========================================================================


class _FakeGroup(object):
    def set_pose_target(self, target):
        self.target = target


_PARAMS.clear()
_PARAMS["~rpy_deg"] = "180,0,0"
node = make_node()
node._plane_in_link_base = lambda x, y: _ptm.Pose()
group = _FakeGroup()
node._dot_goal(0.3355, 0.0)(group)
tilt = _ptm._tilt_deg(_ptm._axes_of(node.target_quat)[2])
check("rpy 读回来是 180 度（不是 180 弧度）",
      [round(v, 3) for v in node.rpy_deg] == [180.0, 0.0, 0.0], node.rpy_deg)
check("工具轴竖直向下（%.2f deg）" % tilt, abs(tilt) < 0.01, tilt)
# 对照组：把 180 直接当弧度喂进去（坑 18 那个 bug）会歪 53.2 deg，
# 所以上面那条断言不是恒真的，确实在守 degrees->radians 这一步
wrong = _ptm._tilt_deg(_ptm._axes_of(_quaternion_from_euler(180, 0, 0))[2])
check("对照组：误当弧度会歪 53.2 deg（证明断言有区分力）",
      abs(wrong - 53.2) < 0.2, wrong)

_PARAMS.clear()
_PARAMS["~rpy_deg"] = "180,0,90"
node = make_node()
node._plane_in_link_base = lambda x, y: _ptm.Pose()
node._dot_goal(0.3355, 0.0)(_FakeGroup())
ax = _ptm._axes_of(node.target_quat)
# yaw 是绕工具轴（竖直）旋转，所以工具轴倾角**应该**保持 0 deg，
# 变的是夹爪绕竖直方向的"拧向"：X 从 (1,0,0) 转到 (0,1,0)
check("~rpy_deg 改 90 度确实生效（工具轴仍竖直）",
      abs(_ptm._tilt_deg(ax[2])) < 0.01, _ptm._tilt_deg(ax[2]))
check("拧向确实转了 90 deg（X 轴 (1,0,0) -> (0,1,0)）",
      abs(ax[0][0]) < 1e-6 and abs(ax[0][1] - 1.0) < 1e-6,
      tuple(round(v, 3) for v in ax[0]))

_PARAMS.clear()
_PARAMS["~rpy_deg"] = "180,0"          # 少一个值
node = make_node()
node._plane_in_link_base = lambda x, y: _ptm.Pose()
node._dot_goal(0.3355, 0.0)(_FakeGroup())
check("~rpy_deg 写坏时退回 180,0,0（不是静默算错）",
      [round(v, 3) for v in node.rpy_deg] == [180.0, 0.0, 0.0], node.rpy_deg)
check("退回后工具轴仍然竖直向下",
      abs(_ptm._tilt_deg(_ptm._axes_of(node.target_quat)[2])) < 0.01)

print("")
if _failures:
    print("==> %d 项失败：%s" % (len(_failures), ", ".join(_failures)))
    sys.exit(1)
print("==> test_grasp 全部通过")
