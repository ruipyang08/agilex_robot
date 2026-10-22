#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish lift_joint joint states so MoveIt tracks the lifting column height.

Read-only bridge: subscribes to /lifter_1/LiftMotorStatePub
(bt_task_msgs/LiftMotorMsg, 50 Hz) and republishes backHeight (mm) as the
prismatic URDF joint lift_joint (m). The lift itself is still driven by the
console slider / lift CLI; this node never sends commands to it.

If the lift node stops or reports an uninitialized state, publishing stops and
joint_state_publisher keeps its last known height instead of a wrong one.

Why the last known height is persisted (2026-09-29):
    The 830ABS drive loses its position reference across a power cycle, so after
    a reboot in mid-travel the driver reports backHeight ~0 while the column is
    physically somewhere in the middle of its stroke. lifting_ctrl then refuses
    position commands and reports initState=false (see CheckFrameAtStartup in
    lifting_ctrl_service_node.py). If this bridge simply stopped publishing,
    MoveIt would fall back to the URDF default of 0 for a fresh
    joint_state_publisher - still wrong. So the last trusted height is written
    to disk and re-published until the driver becomes trustworthy again. MoveIt
    is then slightly stale rather than confidently wrong.

Joint model facts are field-calibrated (tasks/doc/measurements.md M1):
  - lift command up => arm up (axis 0 0 1)
  - backHeight unit is mm, 1:1 with commanded travel
  - lift 0 = mechanical lowest, travel 0..600 mm
"""

import json
import os
import time

import rospy
from bt_task_msgs.msg import LiftMotorMsg
from sensor_msgs.msg import JointState

STATE_TOPIC = "/lifter_1/LiftMotorStatePub"
# The prismatic joint name must match robot_description. The canonical
# combined xacro (ranger_xarm6_lifting_2431) names it "lifting_joint"
# (limit 0..0.7 m, merges the robot-side calibrated model from temp —
# see tasks/doc/measurements.md M1). Set ~joint_name to override if a
# variant model uses a different name.
JOINT_NAME_PARAM = "~joint_name"
JOINT_NAME_DEFAULT = "lifting_joint"
MM_TO_M = 0.001
SAVE_INTERVAL_S = 2.0


def _default_state_file():
    ros_home = os.environ.get("ROS_HOME", os.path.expanduser("~/.ros"))
    return os.path.join(ros_home, "lift_last_height.json")


class LiftJointBridge(object):
    def __init__(self):
        self.joint_pub = rospy.Publisher("lift_slider/joint_states", JointState, queue_size=1)
        self.state_sub = rospy.Subscriber(
            STATE_TOPIC, LiftMotorMsg, self.state_cb, queue_size=1)
        self.joint_name = rospy.get_param(JOINT_NAME_PARAM, JOINT_NAME_DEFAULT)
        self.state_file = rospy.get_param("~state_file", _default_state_file())
        self.last_height_mm = None
        self.last_saved_mm = None
        self.last_saved_at = 0.0
        self.trusted = False

        restored = self._load()
        if restored is not None:
            self.last_height_mm = restored
            rospy.logwarn("Lift joint bridge: no trusted height yet, replaying last known "
                          "%.0f mm from %s. Run 'lift init' to re-establish the 830ABS "
                          "position reference.", restored, self.state_file)
            self._publish(restored)
        rospy.loginfo("Lift joint bridge: %s -> lift_slider/joint_states (%s)",
                      STATE_TOPIC, self.joint_name)

    def _load(self):
        try:
            with open(self.state_file, "r", encoding="utf-8") as f:
                return int(json.load(f)["height_mm"])
        except Exception:
            return None

    def _save(self, height_mm):
        now = time.time()
        if self.last_saved_mm == height_mm:
            return
        if (now - self.last_saved_at) < SAVE_INTERVAL_S:
            return
        try:
            os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
            tmp = self.state_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"height_mm": int(height_mm), "saved_at": int(now)}, f)
            os.replace(tmp, self.state_file)
            self.last_saved_mm = height_mm
            self.last_saved_at = now
        except Exception as e:
            rospy.logwarn("Lift joint bridge: could not persist height to %s: %s",
                          self.state_file, e)

    def _publish(self, height_mm):
        # The driver stamps each report (lifting_ctrl_service_node.py); fall
        # back to now() only if a build ever sends an unstamped message.
        stamp = rospy.Time.now()
        js = JointState()
        js.header.stamp = stamp
        js.name = [self.joint_name]
        js.position = [height_mm * MM_TO_M]
        js.velocity = [0.0]
        self.joint_pub.publish(js)

    def state_cb(self, msg):
        if not msg.initState:
            # Not initialized / position reference lost: the reported height is
            # not trustworthy. Hold the last known height instead, so MoveIt is
            # stale rather than confidently wrong.
            if self.last_height_mm is not None:
                self._publish(self.last_height_mm)
            return
        self.trusted = True
        self.last_height_mm = msg.backHeight
        self._save(msg.backHeight)
        self._publish(msg.backHeight)


def main():
    rospy.init_node("lift_joint_bridge")
    LiftJointBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
