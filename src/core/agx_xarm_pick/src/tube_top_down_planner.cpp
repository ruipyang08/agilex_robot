#include "agx_xarm_pick/tube_top_down_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/Pose.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>

namespace agx_xarm_pick {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kBoundaryTolerance = 1e-12;

class MoveGroupTargetGuard {
 public:
  explicit MoveGroupTargetGuard(
      moveit::planning_interface::MoveGroupInterface* move_group)
      : move_group_(move_group) {}

  ~MoveGroupTargetGuard() {
    if (move_group_ != nullptr) {
      move_group_->clearPathConstraints();
      move_group_->clearPoseTargets();
    }
  }

 private:
  moveit::planning_interface::MoveGroupInterface* move_group_;
};

Eigen::Isometry3d DownwardTcpPose(const TopDownViewCandidate& candidate) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() =
      Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  pose.translation() = Eigen::Vector3d(
      candidate.tcp_x_m, candidate.tcp_y_m, candidate.tcp_z_m);
  return pose;
}

geometry_msgs::Pose ToPoseMessage(const Eigen::Isometry3d& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

}  // namespace

bool valid_top_down_motion_request(const TopDownMotionRequest& request) {
  return valid_top_down_view_region(request.region) &&
      std::isfinite(request.desired_y_m) &&
      std::isfinite(request.runtime_cap_top_z_m) &&
      std::isfinite(request.minimum_clearance_m) &&
      request.minimum_clearance_m >= request.region.clearance_min_m &&
      request.minimum_clearance_m <= request.region.clearance_max_m &&
      std::isfinite(request.downward_axis_tolerance_rad) &&
      request.downward_axis_tolerance_rad > 0.0 &&
      request.downward_axis_tolerance_rad < kPi &&
      std::isfinite(request.planning_time_per_candidate_sec) &&
      request.planning_time_per_candidate_sec > 0.0;
}

bool trajectory_sample_has_minimum_clearance(
    double tcp_z_m,
    double runtime_cap_top_z_m,
    double minimum_clearance_m) {
  return std::isfinite(tcp_z_m) &&
      std::isfinite(runtime_cap_top_z_m) &&
      std::isfinite(minimum_clearance_m) &&
      minimum_clearance_m >= 0.0 &&
      tcp_z_m + kBoundaryTolerance >=
          runtime_cap_top_z_m + minimum_clearance_m;
}

double absolute_z_axis_tolerance(const Eigen::Vector3d& tcp_z_axis) {
  if (!tcp_z_axis.allFinite() || tcp_z_axis.norm() <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double downward_alignment = std::clamp(
      -tcp_z_axis.normalized().z(), -1.0, 1.0);
  return std::acos(downward_alignment);
}

TubeTopDownPlanner::TubeTopDownPlanner(
    moveit::planning_interface::MoveGroupInterface* move_group,
    std::string planning_frame,
    std::string constrained_link,
    const Eigen::Isometry3d& link6_to_tcp)
    : move_group_(move_group),
      planning_frame_(std::move(planning_frame)),
      constrained_link_(std::move(constrained_link)),
      link6_to_tcp_(link6_to_tcp) {
  if (move_group_ == nullptr || planning_frame_.empty() ||
      constrained_link_.empty() || !link6_to_tcp_.matrix().allFinite()) {
    throw std::invalid_argument("top-down planner construction is invalid");
  }
}

std::optional<TopDownMotionResult> TubeTopDownPlanner::Plan(
    const TopDownMotionRequest& request,
    const moveit::core::RobotState& measured_start) {
  if (!valid_top_down_motion_request(request)) {
    return std::nullopt;
  }
  MoveGroupTargetGuard cleanup(move_group_);
  const auto candidates = sample_top_down_view_candidates(
      request.region, request.desired_y_m, request.runtime_cap_top_z_m);
  for (const auto& candidate : candidates) {
    const Eigen::Isometry3d planning_to_tcp = DownwardTcpPose(candidate);
    const Eigen::Isometry3d planning_to_link6 =
        planning_to_tcp * link6_to_tcp_.inverse();
    const geometry_msgs::Pose target = ToPoseMessage(planning_to_link6);

    moveit_msgs::OrientationConstraint orientation;
    orientation.header.frame_id = planning_frame_;
    orientation.link_name = constrained_link_;
    orientation.orientation = target.orientation;
    orientation.absolute_x_axis_tolerance =
        request.downward_axis_tolerance_rad;
    orientation.absolute_y_axis_tolerance =
        request.downward_axis_tolerance_rad;
    orientation.absolute_z_axis_tolerance = kPi;
    orientation.weight = 1.0;
    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(orientation);

    move_group_->setStartState(measured_start);
    move_group_->setPlanningTime(request.planning_time_per_candidate_sec);
    move_group_->setPathConstraints(constraints);
    move_group_->setPoseTarget(target, constrained_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_->clearPoseTargets();
      continue;
    }
    if (!TrajectoryIsSafe(plan, request, measured_start)) {
      move_group_->clearPoseTargets();
      continue;
    }
    return TopDownMotionResult{candidate, std::move(plan)};
  }
  return std::nullopt;
}

bool TubeTopDownPlanner::TrajectoryIsSafe(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const TopDownMotionRequest& request,
    const moveit::core::RobotState& measured_start) const {
  robot_trajectory::RobotTrajectory trajectory(
      measured_start.getRobotModel(), move_group_->getName());
  trajectory.setRobotTrajectoryMsg(measured_start, plan.trajectory_);
  if (trajectory.getWayPointCount() == 0) {
    return false;
  }
  for (std::size_t index = 0; index < trajectory.getWayPointCount(); ++index) {
    const Eigen::Isometry3d planning_to_link6 =
        trajectory.getWayPoint(index).getGlobalLinkTransform(
            constrained_link_);
    const Eigen::Isometry3d planning_to_tcp =
        planning_to_link6 * link6_to_tcp_;
    if (!trajectory_sample_has_minimum_clearance(
            planning_to_tcp.translation().z(),
            request.runtime_cap_top_z_m,
            request.minimum_clearance_m) ||
        absolute_z_axis_tolerance(planning_to_tcp.linear().col(2)) >
            request.downward_axis_tolerance_rad) {
      return false;
    }
  }
  return true;
}

}  // namespace agx_xarm_pick
