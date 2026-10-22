#!/usr/bin/env bash
# 跑 src/tasks/tests/ 下的全部离线测试。
#
# ⚠ 这些测试**不碰串口、不碰真机、不需要 roscore**：
#   - test_frame_gate.py  门禁逻辑（CheckFrameAtStartup / FrameGateAllows / SaveFrameFile）
#   - test_motorstop.py   MotorStop() 限位保护
#   - test_bridge.py      lift_joint_bridge 高度持久化 / 回放
#   - test_grasp.py       plane_target_marker 夹取（到位判定 / 急停中断 / 干跑不闭合）
#   - test_put.py         plane_target_marker 放入（M6 点位 / 五步顺序 / 直线两段不套
#                         黄区 / 干跑不松开 / 失败与急停路径 / 六红球 Marker）
#   - test_pick.py        plane_target_marker 抓取（同一批点 / 先到安全高度再下降
#                         再闭合 200 / 干跑不闭合 / 失败与急停路径）
# 落盘路径全部指向临时目录，不会污染 ~/.ros/lift_frame_m1.json。
# test_grasp / test_put / test_pick 连 MoveIt 规划都换成了桩（不抢 xarm6 控制器）。
# 所以**可以安全地在机器人运行时执行**，不会和 setup_arm.launch 抢节点名。
#
# 依赖 ROS 的 Python 消息（bt_task_msgs / sensor_msgs / dh_gripper_msgs），所以仍需 source 环境。
#
# 用法：
#   bash src/tasks/tests/run_all.sh
set -u

set -u

# ROS 的 setup.bash 会引用未定义变量，临时关掉 set -u
set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash 2>/dev/null
# 注意：/opt 的 setup.bash 会重置工作区已叠加的 ROS 环境，所以工作区要在这之后 source。
# 优先真机布局（~/agilex_ws），否则回退到本脚本所在仓库的 checkout（Docker/离板开发用）。
_WS_DIR=""
[ -r "$HOME/agilex_ws/devel/setup.bash" ] \
    && _WS_DIR="$HOME/agilex_ws" \
    || _WS_DIR="${REPO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}/agilex_ws"
[ -r "$_WS_DIR/devel/setup.bash" ] && source "$_WS_DIR/devel/setup.bash"
if ! python3 -c 'import bt_task_msgs' 2>/dev/null; then
    echo "⚠ bt_task_msgs 不可用：工作区未构建或未 source（构建后需要 catkin build bt_task_msgs 并重新 source）" >&2
fi
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rc=0
for t in test_frame_gate.py test_motorstop.py test_bridge.py test_grasp.py test_put.py test_pick.py; do
    echo "############################################################"
    echo "# $t"
    echo "############################################################"
    python3 "$HERE/$t" || rc=1
    echo
done

if [ "$rc" -eq 0 ]; then
    echo "==> 全部通过"
else
    echo "==> 有失败项，见上"
fi
exit "$rc"
