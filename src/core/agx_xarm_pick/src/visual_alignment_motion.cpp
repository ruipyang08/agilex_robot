#include "agx_xarm_pick/visual_alignment_motion.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace agx_xarm_pick {
namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d TransformToEigen(
    const geometry_msgs::Transform& transform) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  const Eigen::Quaterniond rotation(
      transform.rotation.w,
      transform.rotation.x,
      transform.rotation.y,
      transform.rotation.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() <= 1e-12) {
    result.matrix().setConstant(
        std::numeric_limits<double>::quiet_NaN());
    return result;
  }
  result.linear() = rotation.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(
      transform.translation.x,
      transform.translation.y,
      transform.translation.z);
  return result;
}

Eigen::Isometry3d PoseToEigen(const geometry_msgs::Pose& pose) {
  geometry_msgs::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
  transform.rotation = pose.orientation;
  return TransformToEigen(transform);
}

geometry_msgs::Pose EigenToPose(const Eigen::Isometry3d& transform) {
  geometry_msgs::Pose result;
  result.position.x = transform.translation().x();
  result.position.y = transform.translation().y();
  result.position.z = transform.translation().z();
  const Eigen::Quaterniond rotation(transform.rotation());
  result.orientation.x = rotation.x();
  result.orientation.y = rotation.y();
  result.orientation.z = rotation.z();
  result.orientation.w = rotation.w();
  return result;
}

double QuaternionErrorDeg(
    const geometry_msgs::Quaternion& left,
    const geometry_msgs::Quaternion& right) {
  const Eigen::Quaterniond a(left.w, left.x, left.y, left.z);
  const Eigen::Quaterniond b(right.w, right.x, right.y, right.z);
  if (!a.coeffs().allFinite() || !b.coeffs().allFinite() ||
      a.norm() <= 1e-12 || b.norm() <= 1e-12) {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = std::clamp(
      std::abs(a.normalized().dot(b.normalized())), 0.0, 1.0);
  return 2.0 * std::acos(dot) * 180.0 / kPi;
}

bool InWorkspace(
    const geometry_msgs::Point& point,
    const MoveItVisualAlignmentMotionConfig& config) {
  return finite(point.x) && finite(point.y) && finite(point.z) &&
      point.x >= config.workspace_min_xyz_m[0] &&
      point.x <= config.workspace_max_xyz_m[0] &&
      point.y >= config.workspace_min_xyz_m[1] &&
      point.y <= config.workspace_max_xyz_m[1] &&
      point.z >= config.workspace_min_xyz_m[2] &&
      point.z <= config.workspace_max_xyz_m[2];
}

std::chrono::steady_clock::duration Seconds(double seconds) {
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
}

std::string EndpointValidationDetail(
    const AlignmentEndpointCheck& endpoint) {
  if (endpoint.status == AlignmentEndpointStatus::kInvalidPose) {
    return "alignment endpoint rejected: measured pose or residual is invalid";
  }
  if (endpoint.status == AlignmentEndpointStatus::kWorkspaceRejected) {
    return "alignment endpoint rejected: measured TCP is outside workspace";
  }
  std::ostringstream detail;
  detail << std::fixed << std::setprecision(6)
         << "alignment endpoint rejected: "
         << alignment_endpoint_status_name(endpoint.status) << ' '
         << endpoint.residual_name << '=' << endpoint.residual
         << " limit=" << endpoint.limit;
  return detail.str();
}

}  // namespace

MoveItVisualAlignmentMotion::MoveItVisualAlignmentMotion(
    moveit::planning_interface::MoveGroupInterface* move_group,
    MoveItVisualAlignmentMotionConfig config,
    StateReader state_reader,
    TargetFreshness target_is_fresh)
    : move_group_(move_group),
      config_(std::move(config)),
      state_reader_(std::move(state_reader)),
      target_is_fresh_(std::move(target_is_fresh)) {
  const auto link6_to_grasp_tcp = TransformToEigen(
      config_.link6_to_grasp_tcp);
  const auto link6_to_camera = TransformToEigen(config_.link6_to_camera);
  valid_ = move_group_ != nullptr && !config_.planning_frame.empty() &&
      state_reader_ && target_is_fresh_ &&
      link6_to_grasp_tcp.matrix().allFinite() &&
      link6_to_camera.matrix().allFinite() &&
      finite(config_.maximum_step_m) && config_.maximum_step_m > 0.0 &&
      finite(config_.maximum_downward_error_deg) &&
      config_.maximum_downward_error_deg > 0.0 &&
      config_.maximum_downward_error_deg <= 90.0 &&
      finite(config_.maximum_z_error_m) &&
      config_.maximum_z_error_m > 0.0 &&
      finite(config_.vertical_relief_step_m) &&
      config_.vertical_relief_step_m >= 0.0 &&
      finite(config_.minimum_tcp_z_m) &&
      finite(config_.maximum_orientation_error_deg) &&
      config_.maximum_orientation_error_deg > 0.0 &&
      finite(config_.endpoint_position_tolerance_m) &&
      config_.endpoint_position_tolerance_m > 0.0 &&
      finite(config_.cartesian_step_m) && config_.cartesian_step_m > 0.0 &&
      finite(config_.velocity_scaling) && config_.velocity_scaling > 0.0 &&
      config_.velocity_scaling <= 1.0 &&
      finite(config_.acceleration_scaling) &&
      config_.acceleration_scaling > 0.0 &&
      config_.acceleration_scaling <= 1.0 &&
      finite(config_.state_max_age_sec) && config_.state_max_age_sec > 0.0 &&
      finite(config_.state_wait_timeout_sec) &&
      config_.state_wait_timeout_sec > 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    valid_ = valid_ && finite(config_.workspace_min_xyz_m[axis]) &&
        finite(config_.workspace_max_xyz_m[axis]) &&
        config_.workspace_min_xyz_m[axis] <
            config_.workspace_max_xyz_m[axis];
  }
  if (config_.vertical_relief_step_m > 0.0) {
    valid_ = valid_ &&
        config_.minimum_tcp_z_m >= config_.workspace_min_xyz_m[2] &&
        config_.minimum_tcp_z_m < config_.workspace_max_xyz_m[2];
  }
}

bool MoveItVisualAlignmentMotion::ReadState(
    std::chrono::steady_clock::time_point minimum_receipt_time,
    moveit::core::RobotStatePtr* state,
    geometry_msgs::Pose* grasp_tcp_pose) const {
  if (!valid_ || state == nullptr || grasp_tcp_pose == nullptr) {
    return false;
  }
  return state_reader_(
      minimum_receipt_time,
      std::chrono::steady_clock::now() +
          Seconds(config_.state_wait_timeout_sec),
      state,
      grasp_tcp_pose);
}

AlignmentPose MoveItVisualAlignmentMotion::DescribePose(
    const geometry_msgs::Pose& grasp_tcp_pose) const {
  AlignmentPose result;
  const Eigen::Isometry3d planning_to_tcp = PoseToEigen(grasp_tcp_pose);
  const Eigen::Isometry3d link6_to_grasp_tcp =
      TransformToEigen(config_.link6_to_grasp_tcp);
  const Eigen::Isometry3d link6_to_camera =
      TransformToEigen(config_.link6_to_camera);
  const Eigen::Isometry3d planning_to_link6 =
      planning_to_tcp * link6_to_grasp_tcp.inverse();
  const Eigen::Isometry3d planning_to_camera =
      planning_to_link6 * link6_to_camera;
  if (!planning_to_tcp.matrix().allFinite() ||
      !planning_to_camera.matrix().allFinite()) {
    return result;
  }
  const auto axes = projected_image_axes(
      {planning_to_camera.linear()(0, 0),
       planning_to_camera.linear()(1, 0),
       planning_to_camera.linear()(2, 0)},
      {planning_to_camera.linear()(0, 1),
       planning_to_camera.linear()(1, 1),
       planning_to_camera.linear()(2, 1)},
      {planning_to_camera.linear()(0, 2),
       planning_to_camera.linear()(1, 2),
       planning_to_camera.linear()(2, 2)},
      90.0);
  if (!axes) {
    return result;
  }
  result.valid = true;
  result.workspace_valid = InWorkspace(grasp_tcp_pose.position, config_);
  result.tcp_x_m = grasp_tcp_pose.position.x;
  result.tcp_y_m = grasp_tcp_pose.position.y;
  result.tcp_z_m = grasp_tcp_pose.position.z;
  result.qx = grasp_tcp_pose.orientation.x;
  result.qy = grasp_tcp_pose.orientation.y;
  result.qz = grasp_tcp_pose.orientation.z;
  result.qw = grasp_tcp_pose.orientation.w;
  result.camera_u_axis = {
      planning_to_camera.linear()(0, 0),
      planning_to_camera.linear()(1, 0),
      planning_to_camera.linear()(2, 0)};
  result.camera_v_axis = {
      planning_to_camera.linear()(0, 1),
      planning_to_camera.linear()(1, 1),
      planning_to_camera.linear()(2, 1)};
  result.camera_forward_axis = {
      planning_to_camera.linear()(0, 2),
      planning_to_camera.linear()(1, 2),
      planning_to_camera.linear()(2, 2)};
  result.downward_error_deg = axes->downward_error_deg;
  return result;
}

AlignmentPoseResult MoveItVisualAlignmentMotion::CurrentPose() {
  AlignmentPoseResult result;
  moveit::core::RobotStatePtr state;
  geometry_msgs::Pose tcp;
  const auto now = std::chrono::steady_clock::now();
  if (!ReadState(now - Seconds(config_.state_max_age_sec), &state, &tcp)) {
    result.status = AlignmentMotionStatus::kInvalidPose;
    result.detail = "fresh synchronized arm state is unavailable";
    return result;
  }
  result.pose = DescribePose(tcp);
  result.pose.state_stamp_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch()).count());
  if (!result.pose.valid) {
    result.status = AlignmentMotionStatus::kInvalidPose;
    result.detail = "measured TCP/camera pose is invalid";
  } else if (!result.pose.workspace_valid) {
    result.status = AlignmentMotionStatus::kWorkspaceRejected;
    result.detail = "measured TCP is outside alignment workspace";
  } else if (result.pose.downward_error_deg >
      config_.maximum_downward_error_deg) {
    result.status = AlignmentMotionStatus::kOrientationRejected;
    result.detail = "measured camera is not sufficiently downward";
  } else {
    result.status = AlignmentMotionStatus::kSuccess;
    result.detail = "fresh synchronized alignment pose";
  }
  return result;
}

bool MoveItVisualAlignmentMotion::TrajectoryIsValid(
    const moveit::core::RobotState& start_state,
    const geometry_msgs::Pose& start_tcp,
    const moveit_msgs::RobotTrajectory& trajectory_message,
    double maximum_z_change_m) const {
  robot_trajectory::RobotTrajectory trajectory(
      start_state.getRobotModel(), move_group_->getName());
  trajectory.setRobotTrajectoryMsg(start_state, trajectory_message);
  if (trajectory.getWayPointCount() == 0) {
    return false;
  }
  const auto link6_to_grasp_tcp =
      TransformToEigen(config_.link6_to_grasp_tcp);
  const auto link6_to_camera = TransformToEigen(config_.link6_to_camera);
  std::vector<HorizontalTrajectorySample> samples;
  samples.reserve(trajectory.getWayPointCount());
  for (std::size_t index = 0; index < trajectory.getWayPointCount(); ++index) {
    const auto& state = trajectory.getWayPoint(index);
    const Eigen::Isometry3d model_to_planning =
        state.getGlobalLinkTransform(config_.planning_frame);
    const Eigen::Isometry3d model_to_link6 =
        state.getGlobalLinkTransform("link6");
    const Eigen::Isometry3d planning_to_link6 =
        model_to_planning.inverse() * model_to_link6;
    const Eigen::Isometry3d planning_to_tcp =
        planning_to_link6 * link6_to_grasp_tcp;
    const Eigen::Isometry3d planning_to_camera =
        planning_to_link6 * link6_to_camera;
    if (!planning_to_tcp.matrix().allFinite() ||
        !planning_to_camera.matrix().allFinite()) {
      return false;
    }
    const auto axes = projected_image_axes(
        {planning_to_camera.linear()(0, 0),
         planning_to_camera.linear()(1, 0),
         planning_to_camera.linear()(2, 0)},
        {planning_to_camera.linear()(0, 1),
         planning_to_camera.linear()(1, 1),
         planning_to_camera.linear()(2, 1)},
        {planning_to_camera.linear()(0, 2),
         planning_to_camera.linear()(1, 2),
         planning_to_camera.linear()(2, 2)},
        90.0);
    if (!axes) {
      return false;
    }
    const Eigen::Quaterniond orientation(planning_to_tcp.rotation());
    samples.push_back({
        planning_to_tcp.translation().x(),
        planning_to_tcp.translation().y(),
        planning_to_tcp.translation().z(),
        orientation.x(), orientation.y(), orientation.z(), orientation.w(),
        axes->downward_error_deg});
  }
  const HorizontalTrajectorySample start_sample{
      start_tcp.position.x,
      start_tcp.position.y,
      start_tcp.position.z,
      start_tcp.orientation.x,
      start_tcp.orientation.y,
      start_tcp.orientation.z,
      start_tcp.orientation.w,
      DescribePose(start_tcp).downward_error_deg};
  return valid_horizontal_trajectory_samples(
      samples,
      start_sample,
      config_.workspace_min_xyz_m,
      config_.workspace_max_xyz_m,
      maximum_z_change_m,
      config_.maximum_orientation_error_deg,
      config_.maximum_downward_error_deg);
}

AlignmentEndpointCheck MoveItVisualAlignmentMotion::ValidateMeasuredEndpoint(
    const geometry_msgs::Pose& requested_tcp,
    const geometry_msgs::Pose& measured_tcp) const {
  const auto measured = DescribePose(measured_tcp);
  const double planar_position_error_m = std::hypot(
          measured_tcp.position.x - requested_tcp.position.x,
          measured_tcp.position.y - requested_tcp.position.y);
  const double height_error_m =
      std::abs(measured_tcp.position.z - requested_tcp.position.z);
  const double orientation_error_deg = QuaternionErrorDeg(
      measured_tcp.orientation, requested_tcp.orientation);
  return validate_alignment_endpoint(
      measured.valid,
      measured.workspace_valid,
      measured.downward_error_deg,
      planar_position_error_m,
      height_error_m,
      orientation_error_deg,
      config_.maximum_downward_error_deg,
      config_.endpoint_position_tolerance_m,
      config_.maximum_z_error_m,
      config_.maximum_orientation_error_deg);
}

AlignmentMotionResult MoveItVisualAlignmentMotion::AttemptCorrection(
    const VisualTargetObservation& required_fresh_target,
    double correction_x_m,
    double correction_y_m) {
  AlignmentMotionResult result;
  result.requested_x_m = correction_x_m;
  result.requested_y_m = correction_y_m;
  if (!valid_ || !valid_visual_target_observation(required_fresh_target) ||
      !finite(correction_x_m) || !finite(correction_y_m) ||
      std::hypot(correction_x_m, correction_y_m) <= 0.0 ||
      std::hypot(correction_x_m, correction_y_m) >
          config_.maximum_step_m + 1e-9) {
    result.status = AlignmentMotionStatus::kInvalidPose;
    result.detail = "invalid alignment correction request";
    return result;
  }

  moveit::core::RobotStatePtr planning_state;
  geometry_msgs::Pose start_tcp;
  const auto planning_started = std::chrono::steady_clock::now();
  if (!ReadState(
          planning_started - Seconds(config_.state_max_age_sec),
          &planning_state, &start_tcp)) {
    result.status = AlignmentMotionStatus::kInvalidPose;
    result.detail = "fresh synchronized planning state is unavailable";
    return result;
  }
  result.before = DescribePose(start_tcp);
  if (!result.before.valid || !result.before.workspace_valid) {
    result.status = result.before.workspace_valid
        ? AlignmentMotionStatus::kInvalidPose
        : AlignmentMotionStatus::kWorkspaceRejected;
    result.detail = "alignment start pose is invalid";
    return result;
  }
  if (result.before.downward_error_deg >
      config_.maximum_downward_error_deg) {
    result.status = AlignmentMotionStatus::kOrientationRejected;
    result.detail = "alignment start camera is not downward";
    return result;
  }

  geometry_msgs::Pose target_tcp = start_tcp;
  target_tcp.position.x += correction_x_m;
  target_tcp.position.y += correction_y_m;
  const auto exact_down_axes = exact_down_camera_axes(
      result.before.camera_u_axis);
  if (!exact_down_axes) {
    result.status = AlignmentMotionStatus::kInvalidPose;
    result.detail =
        "current camera U axis cannot define an exact-down endpoint";
    return result;
  }
  Eigen::Matrix3d planning_to_camera_rotation;
  planning_to_camera_rotation.col(0) = Eigen::Vector3d(
      exact_down_axes->u_axis.x,
      exact_down_axes->u_axis.y,
      exact_down_axes->u_axis.z);
  planning_to_camera_rotation.col(1) = Eigen::Vector3d(
      exact_down_axes->v_axis.x,
      exact_down_axes->v_axis.y,
      exact_down_axes->v_axis.z);
  planning_to_camera_rotation.col(2) = Eigen::Vector3d(
      exact_down_axes->forward_axis.x,
      exact_down_axes->forward_axis.y,
      exact_down_axes->forward_axis.z);
  const Eigen::Isometry3d link6_to_grasp_tcp =
      TransformToEigen(config_.link6_to_grasp_tcp);
  const Eigen::Isometry3d link6_to_camera =
      TransformToEigen(config_.link6_to_camera);
  const Eigen::Matrix3d planning_to_link6_rotation =
      planning_to_camera_rotation * link6_to_camera.rotation().transpose();
  const Eigen::Matrix3d planning_to_tcp_rotation =
      planning_to_link6_rotation * link6_to_grasp_tcp.rotation();
  const Eigen::Quaterniond target_tcp_orientation(
      planning_to_tcp_rotation);
  if (!target_tcp_orientation.coeffs().allFinite() ||
      target_tcp_orientation.norm() <= 1e-12) {
    result.status = AlignmentMotionStatus::kInvalidPose;
    result.detail = "exact-down TCP endpoint orientation is invalid";
    return result;
  }
  const Eigen::Quaterniond normalized_target_orientation =
      target_tcp_orientation.normalized();
  target_tcp.orientation.x = normalized_target_orientation.x();
  target_tcp.orientation.y = normalized_target_orientation.y();
  target_tcp.orientation.z = normalized_target_orientation.z();
  target_tcp.orientation.w = normalized_target_orientation.w();
  if (!InWorkspace(target_tcp.position, config_)) {
    result.status = AlignmentMotionStatus::kWorkspaceRejected;
    result.detail = "requested alignment endpoint is outside workspace";
    return result;
  }
  moveit_msgs::RobotTrajectory trajectory_message;
  const bool avoid_collisions = true;
  const auto plan_candidate = [
      this,
      &planning_state,
      &start_tcp,
      &link6_to_grasp_tcp,
      &trajectory_message,
      &result,
      avoid_collisions](
          const geometry_msgs::Pose& candidate_tcp,
          double maximum_z_change_m) {
    const Eigen::Isometry3d planning_to_tcp = PoseToEigen(candidate_tcp);
    const Eigen::Isometry3d planning_to_link6 =
        planning_to_tcp * link6_to_grasp_tcp.inverse();
    const geometry_msgs::Pose target_link6 = EigenToPose(planning_to_link6);
    move_group_->setStartState(*planning_state);
    trajectory_message = moveit_msgs::RobotTrajectory{};
    std::vector<geometry_msgs::Pose> waypoints{target_link6};
    moveit_msgs::MoveItErrorCodes cartesian_error;
    result.cartesian_fraction = move_group_->computeCartesianPath(
        waypoints,
        config_.cartesian_step_m,
        trajectory_message,
        avoid_collisions,
        &cartesian_error);
    return finite(result.cartesian_fraction) &&
        result.cartesian_fraction >= 1.0 - 1e-6 &&
        TrajectoryIsValid(
            *planning_state,
            start_tcp,
            trajectory_message,
            maximum_z_change_m);
  };

  bool used_vertical_relief = false;
  if (!plan_candidate(target_tcp, config_.maximum_z_error_m)) {
    geometry_msgs::Pose relieved_target_tcp = target_tcp;
    relieved_target_tcp.position.z = std::max(
        config_.minimum_tcp_z_m,
        start_tcp.position.z - config_.vertical_relief_step_m);
    const bool relief_is_available =
        config_.vertical_relief_step_m > 0.0 &&
        relieved_target_tcp.position.z < start_tcp.position.z - 1e-9 &&
        InWorkspace(relieved_target_tcp.position, config_);
    if (!relief_is_available ||
        !plan_candidate(
            relieved_target_tcp,
            config_.vertical_relief_step_m + config_.maximum_z_error_m)) {
      result.status = AlignmentMotionStatus::kPlanningIncomplete;
      result.detail = "alignment Cartesian path is incomplete";
      return result;
    }
    target_tcp = relieved_target_tcp;
    used_vertical_relief = true;
  }
  if (!TrajectoryIsValid(
          *planning_state,
          start_tcp,
          trajectory_message,
          (used_vertical_relief
               ? config_.vertical_relief_step_m
               : 0.0) + config_.maximum_z_error_m)) {
    result.status = AlignmentMotionStatus::kPlanningIncomplete;
    result.detail = "alignment Cartesian path violates top-down invariants";
    return result;
  }

  robot_trajectory::RobotTrajectory timed_trajectory(
      planning_state->getRobotModel(), move_group_->getName());
  timed_trajectory.setRobotTrajectoryMsg(
      *planning_state, trajectory_message);
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  if (!time_parameterization.computeTimeStamps(
          timed_trajectory,
          config_.velocity_scaling,
          config_.acceleration_scaling)) {
    result.status = AlignmentMotionStatus::kPlanningIncomplete;
    result.detail = "alignment Cartesian path timing failed";
    return result;
  }
  timed_trajectory.getRobotTrajectoryMsg(trajectory_message);

  if (!target_is_fresh_(required_fresh_target)) {
    result.status = AlignmentMotionStatus::kStaleTargetBeforeExecution;
    result.detail = "visual target became stale before execution";
    return result;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::robotStateToRobotStateMsg(
      *planning_state, plan.start_state_);
  plan.trajectory_ = trajectory_message;
  if (move_group_->execute(plan) !=
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    result.status = AlignmentMotionStatus::kExecutionFailed;
    result.detail = "alignment Cartesian execution failed";
    return result;
  }

  result.attempted_x_m = correction_x_m;
  result.attempted_y_m = correction_y_m;

  const auto execution_complete = std::chrono::steady_clock::now();
  moveit::core::RobotStatePtr endpoint_state;
  geometry_msgs::Pose measured_endpoint;
  if (!ReadState(execution_complete, &endpoint_state, &measured_endpoint)) {
    result.status = AlignmentMotionStatus::kInvalidEndpoint;
    result.detail =
        "alignment endpoint feedback is unavailable after execution";
    return result;
  }
  result.after = DescribePose(measured_endpoint);
  result.after.state_stamp_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          execution_complete.time_since_epoch()).count());
  const auto endpoint_validation =
      ValidateMeasuredEndpoint(target_tcp, measured_endpoint);
  if (endpoint_validation.status != AlignmentEndpointStatus::kValid) {
    result.status = AlignmentMotionStatus::kInvalidEndpoint;
    result.detail = EndpointValidationDetail(endpoint_validation);
    return result;
  }
  result.status = AlignmentMotionStatus::kSuccess;
  result.detail = used_vertical_relief
      ? "alignment Cartesian correction completed with downward reach relief"
      : "alignment Cartesian correction completed";
  return result;
}

}  // namespace agx_xarm_pick
