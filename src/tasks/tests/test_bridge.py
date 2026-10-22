#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lift_joint_bridge.py 高度持久化 / 回放逻辑测试（decisions.md D20）

覆盖：
  1. 冷启动无历史文件 -> 不猜高度
  2. 收到可信高度 -> 记住 + 原子落盘
  3. initState=False（断电后位置基准丢失）-> 不采纳不可信读数，保留上次高度
  4. 重新冷启动 -> 从磁盘恢复并回放给 MoveIt（mm -> m）

⚠ 本测试**不碰串口、不碰真机**。rospy 的 Publisher / get_param / 日志全部被替换成桩，
因此**不需要 roscore**，也不会和正在运行的机器人栈抢节点名。
⚠ 落盘路径被强制指向临时目录，测试跑完不会污染 ~/.ros/lift_last_height.json。

跑法：
    source /opt/ros/noetic/setup.bash && source ~/agilex_ws/devel/setup.bash
    python3 src/tasks/tests/test_bridge.py
"""

import importlib.util
import json
import os
import sys
import tempfile

BRIDGE_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "start", "agx_xarm_bringup", "scripts", "lift_joint_bridge.py")
BRIDGE_PY = os.path.normpath(BRIDGE_PY)

import rospy  # noqa: E402
from bt_task_msgs.msg import LiftMotorMsg  # noqa: E402

# ---- 打桩：让测试完全不依赖 roscore ----------------------------------------
STATE_FILE = os.path.join(tempfile.mkdtemp(prefix="agx_lift_bridge_test_"),
                          "lift_last_height.json")
_published = []


class _StubPublisher(object):
    def publish(self, msg):
        _published.append(msg)


rospy.Publisher = lambda *a, **k: _StubPublisher()
rospy.Subscriber = lambda *a, **k: None
#### ~joint_name 覆盖项由用例自行写入（见第 5 组）。
_PARAMS = {}
rospy.get_param = lambda name, default=None: (
    STATE_FILE if name.endswith("state_file") else _PARAMS.get(name, default))
for _fn in ("loginfo", "logwarn", "logerr", "logdebug"):
    setattr(rospy, _fn, (lambda *a, **k: None))


class _StubStamp(object):
    """最小可用的 header.stamp 替身。"""

    def __init__(self):
        self.secs = 0
        self.nsecs = 0


class _StubTime(object):
    """rospy.Time.now() 在未 init_node 时会抛 ROSInitException，这里打桩。"""

    @staticmethod
    def now():
        return _StubStamp()


rospy.Time = _StubTime
# ---------------------------------------------------------------------------

spec = importlib.util.spec_from_file_location("ljb_under_test", BRIDGE_PY)
ljb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ljb)

fails = []


def check(name, got, want):
    ok = got == want
    print(("  PASS  " if ok else "  FAIL  ") + "%s: got=%r want=%r" % (name, got, want))
    if not ok:
        fails.append(name)


def msg(init_state, height):
    m = LiftMotorMsg()
    m.initState = init_state
    m.backHeight = height
    return m


# ---------------------------------------------------------------------------
print("\n=== 0. 前置检查：落盘路径必须在临时目录内 ===")
check("state_file 指向临时目录", os.path.realpath(STATE_FILE).startswith(
    os.path.realpath(tempfile.gettempdir())), True)

print("\n=== 1. 冷启动：没有历史文件，不猜高度 ===")
b = ljb.LiftJointBridge()
check("last_height_mm 为 None", b.last_height_mm, None)

print("\n=== 2. 收到可信高度 -> 记住 + 落盘 ===")
b.state_cb(msg(True, 250))
check("记住高度", b.last_height_mm, 250)
check("已落盘", json.load(open(STATE_FILE))["height_mm"], 250)

print("\n=== 3. initState=False（断电后基准丢失）-> 不采纳，保持上次高度 ===")
_published.clear()
b.state_cb(msg(False, 0))
check("不采纳不可信读数", b.last_height_mm, 250)
check("落盘值未被 0 覆盖", json.load(open(STATE_FILE))["height_mm"], 250)
check("仍发布上次高度给 MoveIt", [round(p, 3) for p in _published[-1].position], [0.25])

print("\n=== 4. 重新冷启动 -> 从磁盘恢复并回放 ===")
b2 = ljb.LiftJointBridge()
check("从磁盘恢复", b2.last_height_mm, 250)
check("用的是同一份文件", b2.state_file, b.state_file)
_published.clear()
b2.state_cb(msg(False, 0))
check("回放用旧高度 (mm->m)", [round(p, 3) for p in _published[-1].position], [0.25])
check("joint 名默认就是 canonical 模型名（2026-10-13 合并：lifting_joint，匹配 "
      "ranger_xarm6_lifting_2431）", _published[-1].name, ["lifting_joint"])

print("\n=== 5. ~joint_name 参数可覆盖（真机 temp 变体模型叫 lift_joint 时用）===")
_PARAMS["~joint_name"] = "lift_joint"
b3 = ljb.LiftJointBridge()
_published.clear()
b3.state_cb(msg(False, 0))
check("参数覆盖生效", _published[-1].name, ["lift_joint"])
_PARAMS.pop("~joint_name", None)

print("\n" + ("ALL PASS" if not fails else "FAILED: %s" % fails))
sys.exit(1 if fails else 0)
