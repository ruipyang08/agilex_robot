#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Zero-force joint admittance for xArm drag teaching.

xArm teaching mode is a joint-space admittance: an external torque produces
joint velocity, and the firmware compensates gravity and friction so the arm
can be led by hand. Teach sensitivity (1 stiff, 5 light) is the compliance.

This node does not servo from raw joint torque. That signal still contains
gravity, so a torque loop without the manufacturer's model would make the arm
fall. Cartesian impedance also needs the official six-axis sensor, which this
driver does not publish.

After teaching, the arm is returned to servo mode and the trajectory
controller is seeded with the pose the arm was dragged to, so MoveIt and the
RViz sliders do not pull it back to the pre-teach command.
"""

import threading

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dh_gripper_msgs.msg import GripperCtrl, GripperState
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import ClearErr, SetAxis, SetInt16

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MODE_SERVO = 1
MODE_TEACH_JOINT = 2
MODE_TEACH_CART = 3
STATE_START = 0
STATE_STOP = 4
ESTOP = {
    1: "急停按钮被按下",
    2: "控制柜急停IO被触发",
    3: "三态开关急停被按下",
}


class AdmittanceTeach(object):
    def __init__(self):
        self.ns = rospy.get_param("~robot_ns", "xarm").strip("/")
        self.sensitivity = int(rospy.get_param("~sensitivity", 5))
        self.cartesian = bool(rospy.get_param("~cartesian", False))
        # AG95 accepts 20-100. 20 is the lightest hold, so the fingers can be
        # pushed by hand. The follow loop then adopts that opening.
        self._gripper_teach_force = max(20, min(100, int(rospy.get_param("~gripper_teach_force", 20))))
        self._gripper_hold_force = max(20, min(100, int(rospy.get_param("~gripper_hold_force", 50))))
        self.command_topic = rospy.get_param(
            "~command_topic", "/xarm/xarm6_traj_controller/command")
        self._lock = threading.Lock()
        self._active = False
        self._arm = None
        self._gripper_pos = None
        self._gripper_target = None
        self._gripper_cmd = None
        self._gripper_grip = 1
        self._gripper_push = 0
        self._gripper_yielded = False
        self._gripper_follow = False
        self._gripper_timer = None

        rospy.wait_for_service(self._service("set_mode"))
        rospy.wait_for_service(self._service("set_state"))
        rospy.wait_for_service(self._service("set_teach_sensitivity"))
        rospy.wait_for_service(self._service("clear_err"))
        rospy.wait_for_service(self._service("motion_ctrl"))
        self._set_mode = rospy.ServiceProxy(self._service("set_mode"), SetInt16)
        self._set_state = rospy.ServiceProxy(self._service("set_state"), SetInt16)
        self._set_sensitivity = rospy.ServiceProxy(
            self._service("set_teach_sensitivity"), SetInt16)
        self._clear_err = rospy.ServiceProxy(self._service("clear_err"), ClearErr)
        self._motion_ctrl = rospy.ServiceProxy(self._service("motion_ctrl"), SetAxis)
        self._command_pub = rospy.Publisher(
            self.command_topic, JointTrajectory, queue_size=1)
        # Latched. The hardware interface stops setServoJ while this is true,
        # including when the status report still says mode 0.
        self._drag_pub = rospy.Publisher(
            self._service("drag_teach"), Bool, queue_size=1, latch=True)
        self._state_sub = rospy.Subscriber(
            self._service("xarm_states"), RobotMsg, self._on_arm, queue_size=1)
        self._gripper_pub = rospy.Publisher("/gripper/ctrl", GripperCtrl, queue_size=1)
        self._gripper_sub = rospy.Subscriber(
            "/gripper/states", GripperState, self._on_gripper, queue_size=1)
        self._set_drag(False)

        rospy.Service("~start", Trigger, self.start)
        rospy.Service("~stop", Trigger, self.stop)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "拖动示教就绪：%s 导纳，灵敏度 %d。服务 ~start / ~stop",
            "笛卡尔" if self.cartesian else "关节",
            self._clamped_sensitivity())

    def _service(self, name):
        return "/%s/%s" % (self.ns, name)

    def _clamped_sensitivity(self):
        return max(1, min(5, self.sensitivity))

    def _call(self, proxy, value, what):
        response = proxy(data=int(value))
        if response.ret < 0:
            raise RuntimeError("%s 失败 (ret=%s, %s)" % (what, response.ret, response.message))
        rospy.loginfo("%s: %s", what, response.message)

    def _on_arm(self, msg):
        self._arm = msg

    def _on_gripper(self, msg):
        pos = max(0, min(1000, int(round(msg.position))))
        prev = self._gripper_pos
        if prev is not None and abs(pos - prev) >= 2:
            self._gripper_push = 1 if pos > prev else -1
        self._gripper_pos = pos
        self._gripper_target = max(0, min(1000, int(round(msg.target_position))))
        self._gripper_grip = int(msg.grip_state)

    def _command_gripper(self, position, force):
        cmd = GripperCtrl()
        cmd.initialize = False
        cmd.position = float(max(0, min(1000, int(position))))
        cmd.force = float(force)
        cmd.speed = 40.0
        self._gripper_pub.publish(cmd)
        self._gripper_cmd = int(cmd.position)

    def _begin_gripper_follow(self):
        self._gripper_follow = True
        self._gripper_yielded = False
        self._gripper_cmd = None
        if self._gripper_timer is None:
            self._gripper_timer = rospy.Timer(rospy.Duration(0.15), self._gripper_tick)
        # Hold the gripper's own target and only drop the force. Commanding the
        # feedback here yanks the fingers if that reading is stale.
        hold = self._gripper_target if self._gripper_target is not None else self._gripper_pos
        if hold is None:
            rospy.logwarn("还没有 /gripper/states。夹爪驱动起来之后才会放开手指")
            return
        self._command_gripper(hold, self._gripper_teach_force)
        rospy.loginfo("夹爪进入示教：力度 %d。用力掰到电流保护后会让开；示教中也可以拖夹爪条", self._gripper_teach_force)

    def _gripper_tick(self, _event):
        if not self._gripper_follow or self._gripper_pos is None:
            return
        # 0: still moving to a command. 2: fingers stopped because something
        # pushed harder than the force limit. 1/3: sitting at the target.
        if self._gripper_grip == 0:
            self._gripper_yielded = False
            return
        if self._gripper_grip == 2:
            if self._gripper_yielded:
                return
            direction = self._gripper_push
            if direction == 0 and self._gripper_cmd is not None and abs(self._gripper_pos - self._gripper_cmd) >= 2:
                direction = 1 if self._gripper_pos > self._gripper_cmd else -1
            if direction == 0:
                return
            give = max(0, min(1000, self._gripper_pos + direction * 40))
            self._command_gripper(give, self._gripper_teach_force)
            self._gripper_yielded = True
            self._gripper_push = 0
            return
        self._gripper_yielded = False
        if self._gripper_cmd is not None and abs(self._gripper_pos - self._gripper_cmd) >= 8:
            self._command_gripper(self._gripper_pos, self._gripper_teach_force)

    def _end_gripper_follow(self):
        was_following = self._gripper_follow
        self._gripper_follow = False
        if self._gripper_timer is not None:
            self._gripper_timer.shutdown()
            self._gripper_timer = None
        if not was_following or self._gripper_pos is None:
            return
        self._command_gripper(self._gripper_pos, self._gripper_hold_force)
        rospy.loginfo("夹爪停在示教位置 %d，力度 %d", self._gripper_pos, self._gripper_hold_force)

    def _set_drag(self, enabled):
        self._drag_pub.publish(Bool(data=bool(enabled)))

    def _read_state(self):
        if self._arm is None:
            self._arm = rospy.wait_for_message(
                self._service("xarm_states"), RobotMsg, timeout=1.0)
        return self._arm

    def _enable_motion(self):
        response = self._motion_ctrl(id=8, data=1)
        if response.ret < 0:
            raise RuntimeError("使能关节失败 (ret=%s, %s)" % (response.ret, response.message))
        rospy.loginfo("使能关节: %s", response.message)

    def _require_no_estop(self, state):
        reason = ESTOP.get(int(state.err))
        if reason is None:
            return
        raise RuntimeError(
            "机械臂被急停锁住（err=%s，%s）。松开急停后才能运动和示教" % (state.err, reason))

    def _clear_fault(self):
        response = self._clear_err()
        if response.ret < 0:
            raise RuntimeError("清除报错失败 (ret=%s, %s)" % (response.ret, response.message))
        rospy.loginfo("清除报错: %s", response.message or ("ret=%s" % response.ret))

    def _current_positions(self):
        state = rospy.wait_for_message("/%s/joint_states" % self.ns, JointState, timeout=1.0)
        index = {name: i for i, name in enumerate(state.name)}
        missing = [name for name in JOINTS if name not in index]
        if missing:
            raise RuntimeError("关节状态缺少 %s" % ", ".join(missing))
        return [state.position[index[name]] for name in JOINTS]

    def _hold_pose(self, positions):
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = rospy.Duration(0.2)
        trajectory = JointTrajectory()
        # Leave stamp at zero. A wall-clock stamp is rejected with
        # "Time is out of dual 32-bit range" and the arm is not retargeted.
        trajectory.joint_names = list(JOINTS)
        trajectory.points = [point]
        self._command_pub.publish(trajectory)

    def _teach_is_ready(self, state, mode):
        return state.err == 0 and state.mode == mode and state.state in (0, 1, 2)

    def _apply_mode(self, mode, what):
        # set_mode itself stops the arm. Start must be the last command.
        self._call(self._set_mode, mode, what)
        self._call(self._set_state, STATE_START, "启动运动")

    def _enter_teach(self, mode):
        # Stop joint streaming before the mode change. A setServoJ in flight
        # is rejected in teach mode and the controller then forces STOP.
        self._set_drag(True)
        rospy.sleep(0.3)
        self._clear_fault()
        rospy.sleep(0.3)
        self._require_no_estop(self._read_state())
        self._enable_motion()
        self._call(
            self._set_sensitivity,
            self._clamped_sensitivity(),
            "设置示教灵敏度")
        self._call(self._set_state, STATE_STOP, "先停止再切示教")
        rospy.sleep(0.2)
        self._apply_mode(mode, "进入拖动示教")
        rospy.sleep(0.4)
        last = self._read_state()
        if self._teach_is_ready(last, mode):
            return last
        self._require_no_estop(last)
        rospy.logwarn(
            "第一次没有进入示教 mode=%s state=%s err=%s，再试一次",
            last.mode, last.state, last.err)
        self._call(self._set_state, STATE_STOP, "再次停止")
        rospy.sleep(0.2)
        self._apply_mode(mode, "再次进入拖动示教")
        rospy.sleep(0.4)
        last = self._read_state()
        if not self._teach_is_ready(last, mode):
            raise RuntimeError(
                "机械臂没有进入可拖动状态：mode=%s state=%s err=%s" % (
                    last.mode, last.state, last.err))
        return last

    def start(self, _req):
        with self._lock:
            if self._active:
                return TriggerResponse(success=True, message="已经在拖动示教中")
            mode = MODE_TEACH_CART if self.cartesian else MODE_TEACH_JOINT
            try:
                state = self._enter_teach(mode)
            except (rospy.ServiceException, RuntimeError, rospy.ROSException) as exc:
                try:
                    self._apply_mode(MODE_SERVO, "恢复伺服模式")
                except Exception:
                    pass
                self._end_gripper_follow()
                self._set_drag(False)
                return TriggerResponse(success=False, message=str(exc))
            self._active = True
            self._begin_gripper_follow()
            message = "拖动示教已开始（mode=%s state=%s）。用手带动机械臂；夹爪要用力掰，或直接拖夹爪条" % (
                state.mode, state.state)
            rospy.loginfo(message)
            return TriggerResponse(success=True, message=message)

    def stop(self, _req):
        with self._lock:
            # Always leave teach, even if this node does not think it is active.
            # A previous attempt can leave the arm in mode 2.
            try:
                self._end_gripper_follow()
                # Stay in drag until servo mode is set, then drop the flag.
                # Dropping it restarts the trajectory controller on the live
                # pose. Publishing the old goal here jumps the arm and trips
                # self-collision (err 22), after which the sliders do nothing.
                self._apply_mode(MODE_SERVO, "回到伺服模式")
                rospy.sleep(0.3)
                self._set_drag(False)
            except (rospy.ServiceException, RuntimeError, rospy.ROSException) as exc:
                self._set_drag(False)
                return TriggerResponse(
                    success=False,
                    message="退出拖动示教失败，机械臂可能仍在示教模式: %s" % exc)
            self._set_drag(False)
            self._active = False
            message = "拖动示教已结束，机械臂回到伺服模式，滑条可以继续控制"
            rospy.loginfo(message)
            return TriggerResponse(success=True, message=message)

    def _shutdown(self):
        if not self._active:
            return
        try:
            self.stop(None)
        except Exception as exc:
            rospy.logwarn("退出时恢复伺服模式失败: %s", exc)


def main():
    rospy.init_node("admittance_teach")
    AdmittanceTeach()
    rospy.spin()


if __name__ == "__main__":
    main()
