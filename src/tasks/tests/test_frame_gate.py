#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""升降坐标系列定门禁逻辑测试（decisions.md D20）

被测对象：lifting_ctrl_service_node.py 的 CheckFrameAtStartup / FrameGateAllows /
SaveFrameFile / initState 发布合成。

⚠ 本测试**不碰串口、不碰真机**：C_ROS_Server 用 object.__new__ 绕过 __init__
（不打开串口、不注册服务和话题），rospy 日志被打桩，电机状态对象是手工构造的。
因此**不需要 roscore**，也不会和正在运行的机器人栈抢节点名。
⚠ 落盘路径被强制指向临时目录，测试跑完不会污染 ~/.ros/lift_frame_m1.json。

跑法：
    source /opt/ros/noetic/setup.bash && source ~/agilex_ws/devel/setup.bash
    python3 src/tasks/tests/test_frame_gate.py
"""

import json
import os
import sys
import tempfile

LIFTING_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "start", "lifting_ws", "src", "lifting_ctrl", "scripts"))
sys.path.insert(0, LIFTING_SCRIPTS)

import rospy  # noqa: E402

for _fn in ("loginfo", "logwarn", "logerr", "logdebug"):
    setattr(rospy, _fn, (lambda *a, **k: None))
rospy.get_param = lambda name, default=None: default

from lifting_ctrl_service_node import C_ROS_Server, C_LiftingMotorCtrl  # noqa: E402
from bt_task_msgs.srv import LiftMotorSrvRequest  # noqa: E402

fails = []


def check(name, got, want):
    ok = got == want
    print(("  PASS  " if ok else "  FAIL  ") + "%s: got=%r want=%r" % (name, got, want))
    if not ok:
        fails.append(name)


def make(height, mode=0, saved=None, tol=20, first_read_ok=True):
    """构造一个绕开 __init__ 的服务节点（不碰串口）。"""
    s = object.__new__(C_ROS_Server)
    s.motor_id = 1
    s.mode = mode
    s.frame_check = True
    s.frame_tolerance = tol
    s.frame_file = os.path.join(
        tempfile.mkdtemp(prefix="agx_lift_frame_test_"), "lift_frame_m1.json")
    s.frame_valid = False
    s.last_saved_height = None
    s.last_save_time = 0.0
    # init_state 是上游语义（"限位初始化流程已跑完"），门禁不能改它
    s.init_state = (mode != 1)
    s.back_height = height
    if first_read_ok:
        s.motor_states = C_LiftingMotorCtrl.motor_msg()
        s.motor_states.back_lift_height = height
    else:
        s.motor_states = False
    if saved is not None:
        with open(s.frame_file, "w", encoding="utf-8") as f:
            json.dump({"height_mm": saved}, f)
    return s


def published_init_state(s):
    """复现 PublishMotorMsgs 里的合成逻辑。"""
    return s.init_state and s.frame_valid


# ---------------------------------------------------------------------------
print("\n=== 1. CheckFrameAtStartup：上电判定坐标系是否可信 ===")
s = make(300, saved=300); s.CheckFrameAtStartup()
check("高度一致 -> 可信", s.frame_valid, True)

s = make(0, saved=300); s.CheckFrameAtStartup()
check("断电后读数0/记录300 -> 不可信", s.frame_valid, False)
check("断电后 init_state 仍为 True（不误伤急停判据）", s.init_state, True)

s = make(15, saved=0, tol=20); s.CheckFrameAtStartup()
check("差 15mm <= 容差 20 -> 可信", s.frame_valid, True)
s = make(25, saved=0, tol=20); s.CheckFrameAtStartup()
check("差 25mm > 容差 20 -> 不可信", s.frame_valid, False)

s = make(300, saved=None); s.CheckFrameAtStartup()
check("无历史记录 -> 暂信（下次运行起才有基准）", s.frame_valid, True)

s = make(0, saved=300, first_read_ok=False); s.CheckFrameAtStartup()
check("首次读取失败 -> 不可信", s.frame_valid, False)
check("首次读取失败 -> init_state 仍为 True", s.init_state, True)

s = make(0, saved=300, mode=1); s.CheckFrameAtStartup()
check("initMode=1 -> 等初始化跑完再建零", s.frame_valid, False)

s = make(0, saved=300); s.frame_check = False; s.CheckFrameAtStartup()
check("frame_check=false -> 跳过检查", s.frame_valid, True)

print("\n=== 2. FrameGateAllows：未标定时的指令门禁 ===")
s = make(0, saved=300); s.CheckFrameAtStartup()
R = lambda m, v: LiftMotorSrvRequest(val=v, mode=m)
check("mode=0  位置    -> 拒", s.FrameGateAllows(R(0, 300)), False)
check("mode=-4 上行    -> 拒", s.FrameGateAllows(R(-4, 50)), False)
check("mode=-7 上行    -> 拒", s.FrameGateAllows(R(-7, 500)), False)
check("★mode=-4 下行   -> 拒（会一直跑到撞限位）", s.FrameGateAllows(R(-4, -50)), False)
check("★mode=-7 下行   -> 拒（会一直跑到撞限位）", s.FrameGateAllows(R(-7, -500)), False)
check("mode=1  初始化  -> 放", s.FrameGateAllows(R(1, 0)), True)
check("mode=-2 急停    -> 放", s.FrameGateAllows(R(-2, 0)), True)
check("mode=-3 清故障  -> 放", s.FrameGateAllows(R(-3, 0)), True)
check("mode=-6 位置清零 -> 放", s.FrameGateAllows(R(-6, 0)), True)

print("\n=== 3. FrameGateAllows：坐标系有效时 ===")
s = make(300, saved=300); s.CheckFrameAtStartup()
check("mode=0  位置 -> 放", s.FrameGateAllows(R(0, 300)), True)
check("mode=-4 下行 -> 放", s.FrameGateAllows(R(-4, -50)), True)
check("mode=-4 上行 -> 放", s.FrameGateAllows(R(-4, 50)), True)

print("\n=== 4. 发布给下游的 initState 合成 ===")
s = make(0, saved=300); s.CheckFrameAtStartup()
check("断电后 -> 下游看到 initState=False", published_init_state(s), False)
s = make(300, saved=300); s.CheckFrameAtStartup()
check("一致   -> 下游看到 initState=True", published_init_state(s), True)
s = make(0, saved=300, mode=1); s.CheckFrameAtStartup()
check("初始化中 -> 下游看到 initState=False", published_init_state(s), False)

print("\n=== 5. SaveFrameFile：落盘 / 节流 / 原子性 ===")
s = make(300, saved=300); s.CheckFrameAtStartup()
s.SaveFrameFile(300, force=True)
check("落盘 300", json.load(open(s.frame_file))["height_mm"], 300)
s.frame_valid = False
s.SaveFrameFile(999, force=True)
check("force=True 时即使未标定也写（初始化路径要用）",
      json.load(open(s.frame_file))["height_mm"], 999)
s2 = make(0, saved=300); s2.CheckFrameAtStartup()
s2.last_save_time = 1e18          # 假装刚写过，触发节流
s2.SaveFrameFile(0)
check("未标定且非 force -> 不覆盖已有基准",
      json.load(open(s2.frame_file))["height_mm"], 300)
check("没有留下 .tmp 残留", os.path.exists(s2.frame_file + ".tmp"), False)

print("\n=== 6. 端到端：上电丢失 -> init 重建 -> 再上电 ===")
s = make(0, saved=300); s.CheckFrameAtStartup()
check("上电（基准丢失）-> 不可信", s.frame_valid, False)
s.frame_valid = True              # 模拟 init 跑完：压下限位 + 清零
s.init_state = True
s.SaveFrameFile(0, force=True)    # 立刻落盘新基准
s2 = make(0, saved=0); s2.CheckFrameAtStartup()
check("init 后再上电 -> 可信", s2.frame_valid, True)

print("\n" + ("ALL PASS" if not fails else "FAILED: %s" % fails))
sys.exit(1 if fails else 0)
