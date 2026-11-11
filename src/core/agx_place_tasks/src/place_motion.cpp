#include "agx_place_tasks/place_motion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <dh_gripper_msgs/GripperCtrl.h>
#include <dh_gripper_msgs/GripperState.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "agx_place_tasks/place_pose.hpp"

namespace agx_place_tasks {
namespace {

using moveit::planning_interface::MoveGroupInterface;

double SteadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool StopRequested(const std::function<bool()>& stop) { return stop && stop(); }

geometry_msgs::Pose ToRosPose(const PlacementPose& pose) {
  geometry_msgs::Pose message;
  message.position.x = pose.position.x;
  message.position.y = pose.position.y;
  message.position.z = pose.position.z;
  message.orientation.x = pose.orientation_xyzw[0];
  message.orientation.y = pose.orientation_xyzw[1];
  message.orientation.z = pose.orientation_xyzw[2];
  message.orientation.w = pose.orientation_xyzw[3];
  return message;
}

bool ExecutePlan(MoveGroupInterface& group, MoveGroupInterface::Plan& plan) {
  return group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

// q and -q describe the same rotation, so compare |dot| against one.
bool SameOrientation(const std::array<double, 4>& a, const std::array<double, 4>& b) {
  const double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  return std::fabs(dot) > 0.99995;  // about 0.5 degree
}

bool MoveToPose(MoveGroupInterface& group, const geometry_msgs::Pose& pose, std::string* detail,
                const std::string& label) {
  group.clearPoseTargets();
  group.setPoseTarget(pose);
  MoveGroupInterface::Plan plan;
  if (group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    *detail = label + ": no plan to the target pose";
    return false;
  }
  if (!ExecutePlan(group, plan)) {
    *detail = label + ": trajectory execution failed";
    return false;
  }
  return true;
}

// Straight-line motion. A fraction below 1.0 means the straight path is not
// reachable; for this rack that also means the wrist would have to leave the
// taught attitude, so it is a refusal and never a fallback to joint planning.
bool FollowStraightLine(MoveGroupInterface& group, const geometry_msgs::Pose& target,
                        const PlaceMotionConfig& config, std::string* detail,
                        const std::string& label) {
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target);
  moveit_msgs::RobotTrajectory trajectory;
  const double fraction = group.computeCartesianPath(
      waypoints, config.eef_step_m, config.cartesian_jump_threshold, trajectory);
  if (!std::isfinite(fraction) || fraction < 1.0 - 1e-6) {
    *detail = label + ": straight path reached only " + std::to_string(fraction * 100.0) + "%";
    return false;
  }
  const auto state = group.getCurrentState(2.0);
  if (!state) {
    *detail = label + ": current state unavailable for time parameterisation";
    return false;
  }
  robot_trajectory::RobotTrajectory timed(state->getRobotModel(), config.group);
  timed.setRobotTrajectoryMsg(*state, trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(timed, config.velocity_scaling, config.acceleration_scaling)) {
    *detail = label + ": Cartesian path could not be time parameterised";
    return false;
  }
  timed.getRobotTrajectoryMsg(trajectory);
  MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  if (!ExecutePlan(group, plan)) {
    *detail = label + ": straight path execution failed";
    return false;
  }
  return true;
}

// One fresh sample of the gripper state. A command that is not followed by a
// measurement proves nothing, so every gripper step reads before it writes.
bool ReadGripperState(const PlaceMotionConfig& config, double timeout_s,
                      dh_gripper_msgs::GripperState* sample,
                      const std::function<bool()>& stop) {
  ros::NodeHandle node;
  bool received = false;
  ros::Subscriber subscriber = node.subscribe<dh_gripper_msgs::GripperState>(
      config.gripper_state_topic, 2,
      [sample, &received](const dh_gripper_msgs::GripperState::ConstPtr& message) {
        *sample = *message;
        received = true;
      });
  const double deadline = SteadyNow() + timeout_s;
  while (!received && SteadyNow() < deadline) {
    if (StopRequested(stop)) return false;
    ros::spinOnce();
    ros::WallDuration(0.02).sleep();
  }
  return received && sample->is_initialized;
}

// Command a gripper position and wait until the measurement settles: either it
// reaches the command, or it stops on an object and never gets there.
bool CommandGripper(const PlaceMotionConfig& config, double position, double timeout_s,
                    double* measured, const std::function<bool()>& stop) {
  ros::NodeHandle node;
  ros::Publisher publisher =
      node.advertise<dh_gripper_msgs::GripperCtrl>(config.gripper_command_topic, 1, true);
  const double connect_deadline = SteadyNow() + 2.0;
  while (publisher.getNumSubscribers() < 1 && SteadyNow() < connect_deadline) {
    if (StopRequested(stop)) return false;
    ros::spinOnce();
    ros::WallDuration(0.05).sleep();
  }
  if (publisher.getNumSubscribers() < 1) return false;
  dh_gripper_msgs::GripperCtrl command;
  command.initialize = false;
  command.position = static_cast<float>(position);
  command.force = static_cast<float>(config.gripper_force);
  command.speed = static_cast<float>(config.gripper_speed);
  publisher.publish(command);

  dh_gripper_msgs::GripperState sample;
  double previous = 0.0, before_that = 0.0;
  bool have_two = false;
  const double deadline = SteadyNow() + timeout_s;
  while (SteadyNow() < deadline) {
    if (StopRequested(stop)) return false;
    if (!ReadGripperState(config, 0.3, &sample, stop)) continue;
    if (have_two && std::fabs(sample.position - previous) <= 3.0 &&
        std::fabs(previous - before_that) <= 3.0) {
      *measured = sample.position;
      return true;
    }
    before_that = previous;
    previous = sample.position;
    have_two = true;
    ros::WallDuration(0.05).sleep();
  }
  if (ReadGripperState(config, 0.3, &sample, stop)) {
    *measured = sample.position;
    return true;
  }
  return false;
}

// Closing to the descent width, never opening: if the measured opening is
// already narrower (the gripper holds a tube) this does nothing.
bool NarrowForDescent(const PlaceMotionConfig& config, std::string* detail,
                      const std::function<bool()>& stop) {
  dh_gripper_msgs::GripperState sample;
  if (!ReadGripperState(config, 2.0, &sample, stop)) {
    *detail = "cannot confirm the gripper opening before the descent";
    return false;
  }
  const double target = config.descent_open_position;
  if (sample.position <= target + config.descent_open_tolerance) return true;

  ros::NodeHandle node;
  ros::Publisher publisher =
      node.advertise<dh_gripper_msgs::GripperCtrl>(config.gripper_command_topic, 1, true);
  const double connect_deadline = SteadyNow() + 2.0;
  while (publisher.getNumSubscribers() < 1 && SteadyNow() < connect_deadline) {
    if (StopRequested(stop)) {
      *detail = "stopped before the gripper could be narrowed";
      return false;
    }
    ros::spinOnce();
    ros::WallDuration(0.05).sleep();
  }
  if (publisher.getNumSubscribers() < 1) {
    *detail = "gripper controller did not subscribe to " + config.gripper_command_topic;
    return false;
  }
  dh_gripper_msgs::GripperCtrl command;
  command.initialize = false;
  command.position = static_cast<float>(target);
  command.force = static_cast<float>(config.gripper_force);
  command.speed = static_cast<float>(config.gripper_speed);
  publisher.publish(command);

  const double deadline = SteadyNow() + config.descent_narrow_timeout_s;
  while (SteadyNow() < deadline) {
    if (StopRequested(stop)) {
      *detail = "stopped while narrowing the gripper";
      return false;
    }
    if (ReadGripperState(config, 0.3, &sample, stop) &&
        sample.position <= target + config.descent_open_tolerance) {
      return true;
    }
    ros::WallDuration(0.02).sleep();
  }
  *detail = "gripper did not narrow to the descent width within the timeout; refusing to descend";
  return false;
}

struct ReleaseRecord {
  bool released = false;
  bool uncertain = false;
  PlaceEvidence evidence;
  std::string detail;
};

ReleaseRecord ReleaseObject(const PlaceMotionConfig& config, const std::function<bool()>& stop) {
  ReleaseRecord record;
  dh_gripper_msgs::GripperState before;
  if (!ReadGripperState(config, 2.0, &before, stop)) {
    record.uncertain = true;
    record.detail = "no gripper state arrived on " + config.gripper_state_topic;
    return record;
  }
  if (before.position >= config.gripper_open_position - config.gripper_open_tolerance) {
    record.detail = "gripper was already open before the release command; nothing could have been released";
    return record;
  }

  double measured = 0.0;
  if (!CommandGripper(config, config.gripper_open_position, config.gripper_timeout_s, &measured,
                      stop)) {
    record.uncertain = true;
    record.detail = "gripper did not report a position after the open command";
    return record;
  }
  dh_gripper_msgs::GripperState after;
  const bool have_after = ReadGripperState(config, 0.5, &after, stop);
  if (measured < config.gripper_open_position - config.gripper_open_tolerance) {
    record.uncertain = true;
    record.detail = "gripper stopped at " + std::to_string(measured) + ", short of open";
    return record;
  }
  record.released = true;
  record.evidence.source = "measured-gripper";
  record.evidence.source_stamp_s = have_after ? after.header.stamp.toSec() : ros::Time::now().toSec();
  record.evidence.receipt_steady_s = SteadyNow();
  record.detail = "gripper reported open at " + std::to_string(measured);
  return record;
}

// Measured from the driver's own joint state, not from a planning-side copy.
bool WaitSettled(MoveGroupInterface& group, const PlaceMotionConfig& config,
                 const std::function<bool()>& stop) {
  ros::NodeHandle node;
  sensor_msgs::JointState latest;
  bool received = false;
  const auto joints = group.getActiveJoints();
  ros::Subscriber subscriber = node.subscribe<sensor_msgs::JointState>(
      config.joint_state_topic, 2,
      [&latest, &received](const sensor_msgs::JointState::ConstPtr& message) {
        latest = *message;
        received = true;
      });
  const double deadline = SteadyNow() + config.settle_timeout_s;
  int quiet_samples = 0;
  while (SteadyNow() < deadline) {
    if (StopRequested(stop)) return false;
    ros::spinOnce();
    ros::WallDuration(0.05).sleep();
    if (!received || latest.velocity.size() != latest.name.size()) continue;
    double maximum = 0.0;
    for (std::size_t index = 0; index < latest.name.size(); ++index) {
      if (std::find(joints.begin(), joints.end(), latest.name[index]) == joints.end()) continue;
      maximum = std::max(maximum, std::fabs(latest.velocity[index]));
    }
    if (maximum <= config.settle_velocity_rad_s) {
      if (++quiet_samples >= 3) return true;
    } else {
      quiet_samples = 0;
    }
  }
  return false;
}

}  // namespace

PlaceMotionOutcome RunPlaceMotion(const OnboardRackProfile& rack, std::size_t slot,
                                  const PlaceMotionConfig& config,
                                  const std::function<bool()>& stop_requested) {
  const auto approach = ApproachPose(rack, slot);
  const auto insert = SlotPose(rack, slot);
  PickTarget target;
  target.frame = rack.parent_frame;
  target.tip_link = rack.tip_link;
  target.approach_m = approach.position;
  target.grasp_m = insert.position;
  target.orientation_xyzw = approach.orientation_xyzw;
  target.approach_orientation_xyzw = approach.orientation_xyzw;
  return RunPlaceMotionAt(target, config, stop_requested);
}

PlaceMotionOutcome RunPlaceMotionAt(const PickTarget& target, const PlaceMotionConfig& config,
                                    const std::function<bool()>& stop_requested) {
  PlaceMotionOutcome outcome;
  try {
    if (!stop_requested) {
      outcome.detail = "place motion requires a stop predicate";
      return outcome;
    }
    PlacementPose approach_pose;
    approach_pose.position = target.approach_m;
    approach_pose.orientation_xyzw = target.approach_orientation_xyzw;
    PlacementPose insert_pose;
    insert_pose.position = target.grasp_m;
    insert_pose.orientation_xyzw = target.orientation_xyzw;
    const auto approach = ToRosPose(approach_pose);
    const auto insert = ToRosPose(insert_pose);

    MoveGroupInterface group(config.group);
    group.setPoseReferenceFrame(target.frame);
    if (!target.tip_link.empty()) group.setEndEffectorLink(target.tip_link);
    group.setPlanningTime(config.planning_time_s);
    group.setMaxVelocityScalingFactor(config.velocity_scaling);
    group.setMaxAccelerationScalingFactor(config.acceleration_scaling);

    if (StopRequested(stop_requested)) {
      outcome.detail = "stopped before the approach move";
      return outcome;
    }
    if (!MoveToPose(group, approach, &outcome.detail, "approach")) return outcome;

    if (StopRequested(stop_requested)) {
      outcome.detail = "stopped above the slot; nothing was released";
      return outcome;
    }
    if (!SameOrientation(target.approach_orientation_xyzw, target.orientation_xyzw)) {
      PlacementPose twist_pose;
      twist_pose.position = target.approach_m;
      twist_pose.orientation_xyzw = target.orientation_xyzw;
      if (!MoveToPose(group, ToRosPose(twist_pose), &outcome.detail, "twist")) return outcome;
    }
    // Narrow the pads before entering the rack: at full opening they sweep the
    // neighbouring caps. This only ever closes, so a held tube is untouched.
    if (config.narrow_before_descent && !NarrowForDescent(config, &outcome.detail, stop_requested))
      return outcome;
    if (StopRequested(stop_requested)) {
      outcome.detail = "stopped above the slot; nothing was released";
      return outcome;
    }
    if (!FollowStraightLine(group, insert, config, &outcome.detail, "descend")) return outcome;

    ReleaseRecord release;
    if (!config.expects_held_object) {
      release.detail = "no object was held; the release step was skipped";
    } else {
      release = ReleaseObject(config, stop_requested);
    }
    outcome.released = release.released;
    outcome.release_uncertain = release.uncertain;
    outcome.evidence = release.evidence;
    outcome.detail = release.detail;

    // Whatever happened at the slot, the tool must come back out.
    if (StopRequested(stop_requested)) {
      outcome.detail += "; stop requested, the tool was left at the slot";
    } else if (!FollowStraightLine(group, approach, config, &outcome.detail, "retreat")) {
      outcome.detail += "; retreat failed, the tool is still at the slot";
      return outcome;
    }
    if (release.released) {
      if (config.slot_occupancy_confirmed) {
        const auto measured = config.slot_occupancy_confirmed();
        if (measured) {
          outcome.slot_occupancy_measured = true;
          outcome.slot_occupancy_confirmed = *measured;
        }
      }
      if (config.return_to_start && !StopRequested(stop_requested)) {
        group.clearPoseTargets();
        group.setNamedTarget(config.start_named_target);
        MoveGroupInterface::Plan plan;
        if (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS &&
            ExecutePlan(group, plan)) {
          group.clearPoseTargets();
        } else {
          outcome.detail += "; return to '" + config.start_named_target + "' did not complete";
        }
      }
    }

    outcome.measured_stop_confirmed = WaitSettled(group, config, stop_requested);
    outcome.worker_joined = true;
    return outcome;
  } catch (const std::exception& error) {
    outcome.detail = std::string("place motion threw: ") + error.what();
    return outcome;
  }
}

PickMotionOutcome RunPickMotion(const OnboardRackProfile& rack, std::size_t slot,
                                const PlaceMotionConfig& config,
                                const std::function<bool()>& stop_requested) {
  const auto approach = ApproachPose(rack, slot);
  const auto grasp = SlotPose(rack, slot);
  PickTarget target;
  target.frame = rack.parent_frame;
  target.tip_link = rack.tip_link;
  target.approach_m = approach.position;
  target.grasp_m = grasp.position;
  target.orientation_xyzw = approach.orientation_xyzw;
  target.approach_orientation_xyzw = approach.orientation_xyzw;
  return RunPickMotionAt(target, config, stop_requested);
}

PickMotionOutcome RunPickMotionAt(const PickTarget& target, const PlaceMotionConfig& config,
                                  const std::function<bool()>& stop_requested) {
  PickMotionOutcome outcome;
  try {
    if (!stop_requested) {
      outcome.detail = "pick motion requires a stop predicate";
      return outcome;
    }
    PlacementPose approach_pose;
    approach_pose.position = target.approach_m;
    approach_pose.orientation_xyzw = target.approach_orientation_xyzw;
    PlacementPose grasp_pose;
    grasp_pose.position = target.grasp_m;
    grasp_pose.orientation_xyzw = target.orientation_xyzw;
    const auto approach = ToRosPose(approach_pose);
    const auto insert = ToRosPose(grasp_pose);

    MoveGroupInterface group(config.group);
    group.setPoseReferenceFrame(target.frame);
    if (!target.tip_link.empty()) group.setEndEffectorLink(target.tip_link);
    group.setPlanningTime(config.planning_time_s);
    group.setMaxVelocityScalingFactor(config.velocity_scaling);
    group.setMaxAccelerationScalingFactor(config.acceleration_scaling);

    if (StopRequested(stop_requested)) {
      outcome.detail = "stopped before the approach move";
      return outcome;
    }
    if (!MoveToPose(group, approach, &outcome.detail, "approach")) return outcome;

    // Approach with the travel yaw and only twist once above the target: the
    // pads are at their widest during the free-space move, so a yaw offset
    // there swings them across the neighbouring caps.
    if (!SameOrientation(target.approach_orientation_xyzw, target.orientation_xyzw)) {
      PlacementPose twist_pose;
      twist_pose.position = target.approach_m;
      twist_pose.orientation_xyzw = target.orientation_xyzw;
      if (!MoveToPose(group, ToRosPose(twist_pose), &outcome.detail, "twist")) return outcome;
    }

    // Pads wide enough to straddle the cap, narrow enough to pass the neighbours.
    if (!NarrowForDescent(config, &outcome.detail, stop_requested)) return outcome;
    if (StopRequested(stop_requested)) {
      outcome.detail = "stopped above the slot; nothing was grasped";
      return outcome;
    }
    if (!FollowStraightLine(group, insert, config, &outcome.detail, "descend")) return outcome;

    double closed = 0.0;
    if (!CommandGripper(config, config.grasp_close_position, config.grasp_timeout_s, &closed,
                        stop_requested)) {
      outcome.detail = "gripper did not report a position after the close command";
      outcome.grasp_uncertain = true;
    } else if (closed < config.grasp_close_position + config.grasp_held_margin) {
      outcome.detail = "fingers closed to " + std::to_string(closed) + " against a command of " +
                       std::to_string(config.grasp_close_position) + "; nothing was between them";
    } else {
      outcome.grasped = true;
      outcome.detail = "fingers stopped at " + std::to_string(closed) + " on an object (command " +
                       std::to_string(config.grasp_close_position) + ")";
    }
    dh_gripper_msgs::GripperState measured_state;
    if (ReadGripperState(config, 0.5, &measured_state, stop_requested)) {
      outcome.evidence.source = "measured-gripper";
      outcome.evidence.source_stamp_s = measured_state.header.stamp.toSec();
      outcome.evidence.receipt_steady_s = SteadyNow();
    }

    // Always leave the slot: a closed gripper still inside the rack drags the
    // next tube on the way out.
    if (StopRequested(stop_requested)) {
      outcome.detail += "; stop requested, the tool was left at the slot";
    } else if (!FollowStraightLine(group, approach, config, &outcome.detail, "retreat")) {
      outcome.detail += "; retreat failed, the tool is still at the slot";
      return outcome;
    }

    outcome.measured_stop_confirmed = WaitSettled(group, config, stop_requested);
    outcome.worker_joined = true;
    return outcome;
  } catch (const std::exception& error) {
    outcome.detail = std::string("pick motion threw: ") + error.what();
    return outcome;
  }
}

}  // namespace agx_place_tasks
