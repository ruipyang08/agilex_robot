#!/usr/bin/env python3
"""机载蓝盒绿盖试管的取—放回一条龙（2026-11-04 现场验证的动作序列）。

参数来自 config/green_tube_grasp.yaml；默认 dry-run，只有显式 --execute 才会动机械臂。

用法:
  rosrun agx_place_tasks green_tube_cycle.py                      # 只打印将要执行的动作
  rosrun agx_place_tasks green_tube_cycle.py --execute             # 取起再放回原孔
  rosrun agx_place_tasks green_tube_cycle.py --execute --pick-only # 只取
"""
import argparse
import math
import os
import sys
import time

import rospy
import yaml
from geometry_msgs.msg import Pose

import moveit_commander


def quaternion_from_rpy_deg(rpy):
    roll, pitch, yaw = [math.radians(v) / 2.0 for v in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def twist_about_tool_z(quaternion, degrees):
    half = math.radians(degrees) / 2.0
    return multiply(quaternion, [0.0, 0.0, math.sin(half), math.cos(half)])


def load(path):
    with open(path) as handle:
        return yaml.safe_load(handle)["green_tube"]


def pose(x, y, z, quaternion):
    message = Pose()
    message.position.x, message.position.y, message.position.z = x, y, z
    message.orientation.x = quaternion[0]
    message.orientation.y = quaternion[1]
    message.orientation.z = quaternion[2]
    message.orientation.w = quaternion[3]
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "green_tube_grasp.yaml"))
    parser.add_argument("--execute", action="store_true", help="真正驱动机械臂")
    parser.add_argument("--pick-only", action="store_true", help="只取不放下")
    parser.add_argument("--position", default=None,
                        help="positions 表里的名字，如 green_a / green_b / blue_edge")
    arguments = parser.parse_args()

    spec = load(arguments.config)
    positions = spec.get("positions") or {}
    name = arguments.position or spec.get("default")
    approach_z = spec["approach_z_m"]
    if name and name in positions:
        entry = positions[name]
        if isinstance(entry, dict):
            x, y = entry["xy"]
            approach_z = entry.get("approach_z_m", approach_z)
        else:
            x, y = entry
    else:
        x, y = spec["center_xy_m"]
    grasp_z = spec["grasp_z_m"]
    gripper = spec["gripper"]
    motion = spec["motion"]
    quaternion_without_offset = quaternion_from_rpy_deg(spec["tool_rpy_deg"])
    quaternion = twist_about_tool_z(quaternion_without_offset, spec["tool_yaw_offset_deg"])

    print("position       %s" % (name or "(center_xy_m)"))
    print("frame          %s" % spec["frame"])
    print("grasp point    (%.4f, %.4f, %.4f)" % (x, y, grasp_z))
    print("approach point (%.4f, %.4f, %.4f)" % (x, y, approach_z))
    print("gripper        open %.0f -> close %.0f (force %.0f)"
          % (gripper["open_before_descent"], gripper["close_on_cap"], gripper["force"]))
    if not arguments.execute:
        print("\ndry-run: 加 --execute 才会动机械臂")
        return 0

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("green_tube_cycle")
    group = moveit_commander.MoveGroupCommander("xarm6")
    group.set_pose_reference_frame(spec["frame"])
    group.set_end_effector_link("tcp_link")
    group.set_planning_time(10.0)
    group.set_max_velocity_scaling_factor(motion["velocity_scaling"])
    group.set_max_acceleration_scaling_factor(motion["acceleration_scaling"])

    from dh_gripper_msgs.msg import GripperCtrl, GripperState

    latest = {}

    def on_state(message):
        latest["position"] = message.position

    rospy.Subscriber("/gripper/states", GripperState, on_state)
    command = rospy.Publisher("/gripper/ctrl", GripperCtrl, queue_size=1, latch=True)
    rospy.sleep(1.0)

    def drive(position):
        message = GripperCtrl()
        message.initialize = False
        message.position = float(position)
        message.force = float(gripper["force"])
        message.speed = float(gripper["speed"])
        command.publish(message)

    def settled(timeout=15.0, quiet=0.6):
        start = time.time()
        previous = None
        changed = time.time()
        while time.time() - start < timeout:
            value = latest.get("position")
            if value is not None and (previous is None or abs(value - previous) > 2.0):
                previous = value
                changed = time.time()
            elif value is not None and time.time() - changed > quiet:
                return value
            rospy.sleep(0.05)
        return latest.get("position")

    def straight(z, label):
        path, fraction = group.compute_cartesian_path(
            [pose(x, y, z, quaternion)], motion["eef_step_m"], True)
        print("  %-8s fraction=%.3f" % (label, fraction))
        if fraction < 0.999:
            raise RuntimeError(label + " 不可达，拒绝执行")
        group.execute(path, wait=True)
        group.stop()
        rospy.sleep(0.4)

    drive(gripper["open_before_descent"])
    print("gripper open: %.0f" % (settled() or -1))

    # 0 度姿态平移到目标上方（观察/平移都不带 45 度扭转）
    plain = pose(x, y, approach_z, quaternion_without_offset)
    print("approach (0 deg):", group.go(plain, wait=True))
    group.clear_pose_targets()
    # 只在下降前原地旋转到 45 度
    twisted = pose(x, y, approach_z, quaternion)
    print("twist (+45 deg):", group.go(twisted, wait=True))
    group.clear_pose_targets()

    straight(grasp_z, "descend")
    drive(gripper["close_on_cap"])
    closed = settled()
    print("closed at: %.0f (305 附近=夹住, 300=空)" % (closed or -1))
    straight(approach_z, "retreat")

    if arguments.pick_only:
        print("pick-only: 管留在夹爪里")
        return 0

    straight(grasp_z, "descend2")
    drive(gripper["open_before_descent"] if False else 1000.0)
    print("released at: %.0f" % (settled(20.0) or -1))
    straight(approach_z, "retreat2")
    print("untwist (0 deg):", group.go(pose(x, y, approach_z, quaternion_without_offset), wait=True))
    group.clear_pose_targets()
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
