#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Expose finger_joint as a FollowJointTrajectory action for MoveIt.

The real DH gripper is commanded on /gripper/ctrl. MoveIt plans the gripper
group in finger_joint radians: 0 is open, about 0.65 rad is closed.
"""

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from dh_gripper_msgs.msg import GripperCtrl

JOINT_NAME = "finger_joint"
CLOSED_RAD = 0.65


def radians_to_gripper_position(rad):
    position = 1000.0 * (1.0 - float(rad) / CLOSED_RAD)
    return max(0.0, min(1000.0, position))


class GripperTrajectoryBridge(object):
    def __init__(self):
        self.pub = rospy.Publisher("/gripper/ctrl", GripperCtrl, queue_size=1, latch=True)
        self.server = actionlib.SimpleActionServer(
            "gripper_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("Gripper controller ready: gripper_controller/follow_joint_trajectory")

    def execute(self, goal):
        result = FollowJointTrajectoryResult()
        points = goal.trajectory.points
        names = list(goal.trajectory.joint_names)
        if JOINT_NAME not in names or not points:
            result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            self.server.set_aborted(result, "trajectory has no finger_joint")
            return
        if self.pub.get_num_connections() == 0:
            rospy.logerr("No subscriber on /gripper/ctrl. Start: roslaunch agx_xarm_bringup open_gripper.launch")
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self.server.set_aborted(result, "gripper driver is not running")
            return

        index = names.index(JOINT_NAME)
        elapsed = 0.0
        for point in points:
            if self.server.is_preempt_requested():
                result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                self.server.set_preempted(result)
                return
            target_time = point.time_from_start.to_sec()
            delay = target_time - elapsed
            if delay > 0.0:
                rospy.sleep(delay)
            elapsed = target_time
            cmd = GripperCtrl()
            cmd.initialize = False
            cmd.position = radians_to_gripper_position(point.positions[index])
            cmd.force = 50.0
            cmd.speed = 50.0
            self.pub.publish(cmd)
            rospy.loginfo("Gripper command position %.0f (finger_joint %.3f rad)", cmd.position, point.positions[index])

        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.server.set_succeeded(result)


def main():
    rospy.init_node("gripper_traj_bridge")
    GripperTrajectoryBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
