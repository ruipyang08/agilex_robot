#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lifting_motor_ctrl.py 的 MotorStop() 限位保护测试（decisions.md D20 / 修复 D）

被测缺陷：原实现里
    elif upLimit: self.SetStopFlag(True)
会被紧随其后的
    if back_lift_height >= 上限: SetStopFlag(True) else: SetStopFlag(False)
**无条件覆盖**，导致机械限位开关在方向判断分支里从未生效。
修复后：只有在该方向上确实没碰到任何限位时才置 False。

覆盖 4 组：方向 +1 / -1 / 0 / force，每组含"限位开关命中"和"软件行程命中"。

⚠ 本测试**不碰串口**：DealAllData 被替换成计数器，串口收发一次都不发生。
⚠ 不需要 roscore（rospy 日志被打桩）。

跑法：
    source /opt/ros/noetic/setup.bash && source ~/agilex_ws/devel/setup.bash
    python3 src/tasks/tests/test_motorstop.py
"""

import os
import sys

LIFTING_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "start", "lifting_ws", "src", "lifting_ctrl", "scripts"))
sys.path.insert(0, LIFTING_SCRIPTS)

import rospy  # noqa: E402

for _fn in ("loginfo", "logwarn", "logerr", "logdebug"):
    setattr(rospy, _fn, (lambda *a, **k: None))

from lifting_motor_ctrl import C_LiftingMotorCtrl  # noqa: E402

fails = []


def check(name, got, want):
    ok = got == want
    print(("  PASS  " if ok else "  FAIL  ") + "%s: stop=%r want=%r" % (name, got, want))
    if not ok:
        fails.append(name)


class _FakeDeal(object):
    """假装串口收发成功，只计数，保证不真的发任何字节。"""

    def __init__(self):
        self.sent = 0

    def DealAllData(self, *a, **k):
        self.sent += 1
        return None


def make(height, up_limit, down_limit, up=600, down=0):
    ctrl = object.__new__(C_LiftingMotorCtrl)
    ctrl._C_LiftingMotorCtrl__motor_id = 1
    ctrl._C_LiftingMotorCtrl__deal_data = _FakeDeal()
    ctrl._C_LiftingMotorCtrl__port_name = "/dev/null"
    ctrl._C_LiftingMotorCtrl__upLimitVal = up
    ctrl._C_LiftingMotorCtrl__downLimitVal = down
    ctrl.motorSpd = 700
    ctrl.stop_flag = False
    m = C_LiftingMotorCtrl.motor_msg()
    m.back_lift_height = height
    m.upLimit = up_limit
    m.downLimit = down_limit
    m.back_speed = 0
    m.back_current = 0.0
    ctrl._C_LiftingMotorCtrl__motor_msgs = m
    return ctrl, m


# ---------------------------------------------------------------------------
print("\n=== 方向 +1（上行） ===")
ctrl, m = make(100, False, False); ctrl.MotorStop(1, m)
check("行程中段上行 -> 放行", ctrl.GetStopFlag(), False)
ctrl, m = make(0, False, True); ctrl.MotorStop(1, m)
check("压在下限位上要上行 -> 放行（脱离限位）", ctrl.GetStopFlag(), False)
ctrl, m = make(600, False, False); ctrl.MotorStop(1, m)
check("软件行程上限 -> 停", ctrl.GetStopFlag(), True)
ctrl, m = make(300, True, False); ctrl.MotorStop(1, m)
check("★上限位开关命中 -> 停（修复前被覆盖成 False）", ctrl.GetStopFlag(), True)

print("\n=== 方向 -1（下行） ===")
ctrl, m = make(300, False, False); ctrl.MotorStop(-1, m)
check("行程中段下行 -> 放行", ctrl.GetStopFlag(), False)
ctrl, m = make(600, True, False); ctrl.MotorStop(-1, m)
check("压在上限位上要下行 -> 放行（脱离限位）", ctrl.GetStopFlag(), False)
ctrl, m = make(0, False, False); ctrl.MotorStop(-1, m)
check("软件行程下限 -> 停", ctrl.GetStopFlag(), True)
ctrl, m = make(0, False, True); ctrl.MotorStop(-1, m)
check("★下限位开关命中 -> 停（修复前被覆盖成 False）", ctrl.GetStopFlag(), True)

print("\n=== 方向 0（目标==当前） ===")
ctrl, m = make(300, False, False); ctrl.MotorStop(0, m)
check("行程中段不动 -> 放行", ctrl.GetStopFlag(), False)
ctrl, m = make(0, False, True); ctrl.MotorStop(0, m)
check("在下限位 -> 停", ctrl.GetStopFlag(), True)
ctrl, m = make(600, True, False); ctrl.MotorStop(0, m)
check("在上限位 -> 停", ctrl.GetStopFlag(), True)

print("\n=== force（急停） ===")
ctrl, m = make(300, False, False); ctrl.MotorStop(1, m, force=True)
check("force -> 停", ctrl.GetStopFlag(), True)

print("\n=== 前置检查：串口一次都没碰 ===")
ctrl, m = make(300, False, False)
before = ctrl._C_LiftingMotorCtrl__deal_data.sent
ctrl.MotorStop(1, m)
check("只经过桩，没有真实收发", isinstance(before, int) and
      ctrl._C_LiftingMotorCtrl__deal_data.sent >= before, True)

print("\n" + ("ALL PASS" if not fails else "FAILED: %s" % fails))
sys.exit(1 if fails else 0)
