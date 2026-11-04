#!/usr/bin/env python3
"""Dry-run reachability check for the rack mounted on the robot itself.

Plans to every configured slot and reports planning success, joint-limit margin,
wrist singularity margin and path length. It NEVER commands the arm: there is no
call to go(), execute() or any trajectory action client in this file.

This answers the first open question of the 2026-10-29 handoff note (section 7):
can the arm reach its own rack at all? Run it before writing any placement
motion, because a negative answer changes the mechanical design.

  # hardware model, arm already running (setup_arm.launch publishes link_base_lift0)
  rosrun agx_place_tasks check_rack_reach.py \
      --rack-file $(rospack find agx_place_tasks)/config/onboard_rack_profiles.yaml \
      --rack-id onboard_rack_v1

  # headless model without the robot (fake controllers, still no motion)
  roslaunch ranger_xarm_lifting_moveit_config demo.launch use_rviz:=false
  rosrun agx_place_tasks check_rack_reach.py --rack-file <rack.yaml> --rack-id <id> \
      --reference-frame link_base_lift0

  # one slot can be reachable only at some gripper yaw values
  rosrun agx_place_tasks check_rack_reach.py ... --slot 1 --roll-sweep 45
"""

import argparse
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ElementTree

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

ERROR_CODES = {
    1: "SUCCESS",
    0: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "START_STATE_INVALID",
    -13: "GOAL_IN_COLLISION",
    -14: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -15: "GOAL_CONSTRAINTS_VIOLATED",
    -16: "INVALID_GROUP_NAME",
    -17: "INVALID_GOAL_CONSTRAINTS",
    -18: "INVALID_ROBOT_STATE",
    -19: "INVALID_LINK_NAME",
    -21: "FRAME_TRANSFORMATION",
    -24: "COLLISION_CHECKING_UNAVAILABLE",
    -25: "ROBOT_STATE_STALE",
    -27: "COMMUNICATION_FAILURE",
    -31: "NO_IK_SOLUTION",
}


def error_name(code):
    # plan() carries a MoveItErrorCodes message, not a bare int.
    if hasattr(code, "val"):
        code = code.val
    return ERROR_CODES.get(code, "CODE_%s" % code)


def normalize(vector):
    norm = math.sqrt(sum(component * component for component in vector))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("degenerate axis")
    return [component / norm for component in vector]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def quaternion_from_axes(x_axis, y_axis, z_axis):
    m00, m10, m20 = x_axis[0], x_axis[1], x_axis[2]
    m01, m11, m21 = y_axis[0], y_axis[1], y_axis[2]
    m02, m12, m22 = z_axis[0], z_axis[1], z_axis[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    return [x, y, z, w]


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def quaternion_from_rpy_deg(rpy):
    """RPY with the sxyz convention used by the teaching record."""
    roll, pitch, yaw = [math.radians(value) for value in rpy]
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def target_pose(point, descend, slot_axis, roll_rad, taught_quaternion=None):
    """Taught tool attitude when present, otherwise derived from the rack axes."""
    if taught_quaternion is not None:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = point
        half = roll_rad / 2.0
        rotation = quaternion_multiply([0.0, 0.0, math.sin(half), math.cos(half)],
                                       taught_quaternion)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation
        return pose
    z_axis = normalize(descend)
    x_hint = normalize(slot_axis)
    x_axis = normalize([x_hint[i] - dot(x_hint, z_axis) * z_axis[i] for i in range(3)])
    y_axis = cross(z_axis, x_axis)
    cos_roll, sin_roll = math.cos(roll_rad), math.sin(roll_rad)
    rolled_x = [cos_roll * x_axis[i] - sin_roll * y_axis[i] for i in range(3)]
    rolled_y = [sin_roll * x_axis[i] + cos_roll * y_axis[i] for i in range(3)]
    quaternion = quaternion_from_axes(rolled_x, rolled_y, z_axis)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = point
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    return pose


def choose_slot_axis(rack):
    """Tool +X follows the slot row; recover it from two measured centres."""
    slots = rack["slots"]
    if len(slots) >= 2:
        first, second = slots[0]["center"], slots[1]["center"]
        axis = [second[i] - first[i] for i in range(3)]
        if sum(component * component for component in axis) > 1e-12:
            return axis
    return [1.0, 0.0, 0.0]


def load_slots(args):
    if args.slots_json:
        with open(args.slots_json) as stream:
            return json.load(stream)
    command = ["rosrun", "agx_place_tasks", "print_rack_slots", args.rack_file, args.rack_id]
    try:
        output = subprocess.check_output(command, stderr=subprocess.PIPE).decode("utf-8")
    except (OSError, subprocess.CalledProcessError) as error:
        detail = getattr(error, "stderr", b"").decode("utf-8", "replace").strip()
        raise SystemExit("cannot read rack profile (%s): %s" % (" ".join(command), detail or error))
    return json.loads(output)


def read_urdf_limits():
    text = rospy.get_param("/robot_description", None)
    if not isinstance(text, str) or not text:
        return {}
    try:
        root = ElementTree.fromstring(text)
    except ElementTree.ParseError:
        return {}
    limits = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        if joint.get("type") == "continuous":
            limits[name] = (-math.pi, math.pi)
            continue
        try:
            limits[name] = (float(limit.get("lower")), float(limit.get("upper")))
        except (TypeError, ValueError):
            continue
    return limits


def read_joint_limits(namespace):
    limits = rospy.get_param("/" + namespace + "/joint_limits", None)
    parsed = {}
    if isinstance(limits, dict):
        for name, values in limits.items():
            if not isinstance(values, dict) or not values.get("has_position_limits", False):
                continue
            try:
                parsed[name] = (float(values["min_position"]), float(values["max_position"]))
            except (KeyError, TypeError, ValueError):
                continue
    return parsed or read_urdf_limits()


def duration_seconds(value):
    # rospy exposes a genpy.Duration here; plain messages expose sec/nsec.
    if hasattr(value, "to_sec"):
        return value.to_sec()
    return value.sec + value.nsec * 1e-9


def trajectory_metrics(trajectory, joint_names, limits):
    points = trajectory.joint_trajectory.points
    if not points:
        return {}
    positions = [list(point.positions) for point in points]
    seconds = max(duration_seconds(point.time_from_start) for point in points)
    path_length = 0.0
    for previous, current in zip(positions, positions[1:]):
        path_length += math.sqrt(sum((b - a) ** 2 for a, b in zip(previous, current)))
    margin = None
    for name, value in zip(joint_names, positions[-1]):
        if name not in limits:
            continue
        low, high = limits[name]
        distance = min(value - low, high - value)
        margin = distance if margin is None else min(margin, distance)
    wrist_index = joint_names.index("joint5") if "joint5" in joint_names else 4
    wrist = None
    if all(len(point) > wrist_index for point in positions):
        wrist = min(abs(point[wrist_index]) for point in positions)
    return {
        "points": len(points),
        "duration_s": round(seconds, 3),
        "path_length_rad": round(path_length, 3),
        "min_limit_margin_rad": None if margin is None else round(margin, 4),
        "min_abs_joint5_rad": None if wrist is None else round(wrist, 4),
        "final_joints_rad": [round(value, 3) for value in positions[-1]],
    }


def plan_to(group, pose, reference_frame, tip_link, state, planning_time):
    group.set_pose_reference_frame(reference_frame)
    group.set_end_effector_link(tip_link)
    if state is not None:
        group.set_start_state(state)
    group.set_planning_time(planning_time)
    group.clear_pose_targets()
    group.set_pose_target(pose, tip_link)
    result = group.plan()
    if isinstance(result, tuple):
        success = bool(result[0])
        plan = result[1] if len(result) > 1 else None
        elapsed = result[2] if len(result) > 2 else 0.0
        error_code = result[3] if len(result) > 3 else (1 if success else 0)
    else:
        success, plan, elapsed, error_code = bool(result), None, 0.0, 1 if result else 0
    return success, plan, float(elapsed), error_code


def state_from_trajectory(plan):
    points = plan.joint_trajectory.points
    if not points:
        return None
    state = RobotState()
    state.joint_state = JointState(name=list(plan.joint_trajectory.joint_names),
                                   position=list(points[-1].positions))
    state.joint_state.header.stamp = rospy.Time.now()
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rack-file", required=True, help="onboard_rack_profiles.yaml")
    parser.add_argument("--rack-id", required=True, help="profile id inside that file")
    parser.add_argument("--slots-json", help="skip print_rack_slots and read its JSON here")
    parser.add_argument("--slots", help="comma separated 1-based slot subset, e.g. 1,3,6")
    parser.add_argument("--group", default="xarm6")
    parser.add_argument("--tip-link", default="", help="defaults to the group end effector link")
    parser.add_argument("--reference-frame", default="", help="defaults to the profile parent frame")
    parser.add_argument("--start-state", default="initial", help="named joint target to plan from")
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--roll-deg", type=float, default=0.0, help="rotation about the tool axis")
    parser.add_argument("--roll-sweep", type=float, default=0.0,
                        help="also try every yaw in this step and report what works")
    parser.add_argument("--limits-ns", default="robot_description_planning")
    parser.add_argument("--json-out", help="write the full report to this path")
    parser.add_argument("--list", action="store_true", help="print the configured slots and exit")
    args = parser.parse_args()

    rack = load_slots(args)
    if args.list:
        print(json.dumps(rack, indent=2))
        return 0

    wanted = None
    if args.slots:
        wanted = {int(item) for item in args.slots.split(",") if item.strip()}
    slots = [slot for slot in rack["slots"] if wanted is None or slot["slot"] in wanted]
    if not slots:
        raise SystemExit("no slot selected")
    reference_frame = args.reference_frame or rack["parent_frame"]
    rolls = [args.roll_deg]
    if args.roll_sweep > 0.0:
        steps = int(round(360.0 / args.roll_sweep))
        rolls = [index * args.roll_sweep for index in range(steps)]

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("check_rack_reach", anonymous=True)
    group = moveit_commander.MoveGroupCommander(args.group)
    # The taught centres refer to the recorded tip link; planning to the group
    # end effector instead silently shifts the goal by the tool offset.
    taught_tip = rack.get("tip_link") or ""
    tip_link = args.tip_link or taught_tip or group.get_end_effector_link()
    if not tip_link:
        raise SystemExit("no tip link: pass --tip-link")
    joint_names = list(group.get_active_joints())
    limits = read_joint_limits(args.limits_ns)

    warnings = []
    if taught_tip and tip_link != taught_tip:
        warnings.append("planning to tip %s but the profile was taught for %s"
                        % (tip_link, taught_tip))
    if not limits:
        warnings.append("joint limits unavailable; limit margin is not reported")
    if not rack.get("qualified", False):
        warnings.append("this rack profile is NOT qualified: the numbers below prove the tool "
                        "works, not that this robot fits")

    start_state = None
    if args.start_state:
        named = group.get_named_target_values(args.start_state)
        names = [name for name in joint_names if name in named]
        if names:
            start_state = RobotState()
            start_state.joint_state = JointState(name=names,
                                                 position=[named[name] for name in names])
            start_state.joint_state.header.stamp = rospy.Time.now()
        else:
            warnings.append("named start state '%s' is unknown; planning from the current state"
                            % args.start_state)

    print("rack profile : %s (%s) frame=%s qualified=%s"
          % (rack["profile_id"], rack.get("evidence_id", ""), reference_frame,
             rack.get("qualified", False)))
    print("group/tip    : %s / %s, planning frame %s"
          % (args.group, tip_link, group.get_planning_frame()))
    print("start state  : %s, yaw candidates %s"
          % (args.start_state or "current", [round(roll, 1) for roll in rolls]))
    for warning in warnings:
        print("warning      : " + warning)
    print("")

    slot_axis = choose_slot_axis(rack)
    taught_quaternion = None
    if rack.get("tool_rpy_deg"):
        taught_quaternion = quaternion_from_rpy_deg(rack["tool_rpy_deg"])
        offset = float(rack.get("tool_yaw_offset_deg") or 0.0)
        if offset:
            half = math.radians(offset) / 2.0
            taught_quaternion = quaternion_multiply(
                taught_quaternion, [0.0, 0.0, math.sin(half), math.cos(half)])
            print("descent yaw offset: %.1f deg about the tool axis" % offset)
        print("tool attitude: taught RPY %s deg (roll candidates rotate about the tool axis)"
              % rack["tool_rpy_deg"])
    else:
        print("tool attitude: derived from the rack axes (no tool_rpy_deg recorded)")
    report = {"profile_id": rack["profile_id"], "reference_frame": reference_frame,
              "group": args.group, "tip_link": tip_link, "qualified": rack.get("qualified", False),
              "warnings": warnings, "slots": []}
    failed_slots = []

    for slot in slots:
        attempts = []
        best = None
        for roll in rolls:
            state = start_state
            stages = []
            failure = None
            for stage_name, point in (("approach", slot["approach"]), ("insert", slot["center"])):
                pose = target_pose(point, rack["descend_axis"], slot_axis, math.radians(roll),
                                   taught_quaternion)
                success, plan, elapsed, error_code = plan_to(
                    group, pose, reference_frame, tip_link, state, args.planning_time)
                if not success or plan is None or not plan.joint_trajectory.points:
                    failure = "%s:%s" % (stage_name, error_name(error_code))
                    stages.append({"stage": stage_name, "ok": False,
                                   "error": error_name(error_code)})
                    break
                metrics = trajectory_metrics(plan, joint_names, limits)
                stages.append({"stage": stage_name, "ok": True, "error": "SUCCESS",
                               "planning_time_s": round(elapsed, 3), "metrics": metrics})
                state = state_from_trajectory(plan)
            attempt = {"roll_deg": roll, "ok": failure is None, "failure": failure,
                       "stages": stages}
            attempts.append(attempt)
            if failure is None:
                wrist = min(stage["metrics"]["min_abs_joint5_rad"] for stage in stages
                            if stage["metrics"].get("min_abs_joint5_rad") is not None) \
                    if any(stage["metrics"].get("min_abs_joint5_rad") is not None
                           for stage in stages) else 0.0
                if best is None or wrist > best["wrist"]:
                    best = {"roll": roll, "wrist": wrist, "attempt": attempt}

        entry = {"slot": slot["slot"], "ok": best is not None,
                 "best_roll_deg": None if best is None else best["roll"],
                 "attempts": attempts}
        report["slots"].append(entry)
        if best is None:
            failed_slots.append(slot["slot"])
            reasons = sorted({attempt["failure"] for attempt in attempts})
            print("slot %-3d no plan in %d yaw candidate(s): %s"
                  % (slot["slot"], len(attempts), ", ".join(reasons)))
            continue
        def stage_text(stage):
            metrics = stage["metrics"]
            margin = metrics.get("min_limit_margin_rad")
            path_wrist = metrics.get("min_abs_joint5_rad")
            final = metrics.get("final_joints_rad") or []
            final_wrist = None
            for index, name in enumerate(joint_names):
                if name == "joint5" and index < len(final):
                    final_wrist = abs(final[index])
            return ("%s ok %ss, margin=%s, min|q5|=%s, final|q5|=%s"
                    % (stage["stage"], metrics["duration_s"],
                       "n/a" if margin is None else "%.3f" % margin,
                       "n/a" if path_wrist is None else "%.3f" % path_wrist,
                       "n/a" if final_wrist is None else "%.3f" % final_wrist))

        detailed = "; ".join(stage_text(stage) for stage in best["attempt"]["stages"])
        print("slot %-3d yaw %-6.1f OK  %s" % (slot["slot"], best["roll"], detailed))
        if best["wrist"] < 0.15:
            print("         note: reaches the wrist singularity: min |joint5| = %.3f rad "
                  "(below 0.15). A vertical descent here cannot use joint-space interpolation."
                  % best["wrist"])
        if len(rolls) > 1:
            working = [attempt["roll_deg"] for attempt in attempts if attempt["ok"]]
            print("         working yaw values: %s" % ", ".join("%.0f" % value for value in working))

    print("")
    print("%d/%d slot(s) planned collision-free for at least one yaw"
          % (len(slots) - len(failed_slots), len(slots)))
    if failed_slots:
        print("no plan found for slots: %s" % ", ".join(str(slot) for slot in failed_slots))
        print("a missing plan is not proof of unreachability: raise --planning-time first, and "
              "read GOAL_IN_COLLISION separately from TIMED_OUT")
    if args.json_out:
        with open(args.json_out, "w") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
        print("report written to " + args.json_out)
    return 0 if not failed_slots else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(130)
