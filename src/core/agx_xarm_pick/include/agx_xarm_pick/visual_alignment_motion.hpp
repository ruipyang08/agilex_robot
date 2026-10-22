#ifndef AGX_XARM_PICK_VISUAL_ALIGNMENT_MOTION_HPP
#define AGX_XARM_PICK_VISUAL_ALIGNMENT_MOTION_HPP

#include <array>
#include <chrono>
#include <functional>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include "agx_xarm_pick/visual_alignment.hpp"

namespace agx_xarm_pick {

struct MoveItVisualAlignmentMotionConfig {
  std::string planning_frame;
  geometry_msgs::Transform link6_to_grasp_tcp;
  geometry_msgs::Transform link6_to_camera;
  std::array<double, 3> workspace_min_xyz_m{};
  std::array<double, 3> workspace_max_xyz_m{};
  double maximum_step_m = 0.0;
  double maximum_downward_error_deg = 0.0;
  double maximum_z_error_m = 0.0;
  double vertical_relief_step_m = 0.0;
  double minimum_tcp_z_m = 0.0;
  double maximum_orientation_error_deg = 0.0;
  double endpoint_position_tolerance_m = 0.0;
  double cartesian_step_m = 0.0;
  double velocity_scaling = 0.0;
  double acceleration_scaling = 0.0;
  double state_max_age_sec = 0.0;
  double state_wait_timeout_sec = 0.0;
};

class MoveItVisualAlignmentMotion final : public VisualAlignmentMotion {
 public:
  using StateReader = std::function<bool(
      std::chrono::steady_clock::time_point minimum_receipt_time,
      std::chrono::steady_clock::time_point deadline,
      moveit::core::RobotStatePtr* state,
      geometry_msgs::Pose* grasp_tcp_pose)>;
  using TargetFreshness =
      std::function<bool(const VisualTargetObservation& observation)>;

  MoveItVisualAlignmentMotion(
      moveit::planning_interface::MoveGroupInterface* move_group,
      MoveItVisualAlignmentMotionConfig config,
      StateReader state_reader,
      TargetFreshness target_is_fresh);

  AlignmentPoseResult CurrentPose() override;
  AlignmentMotionResult AttemptCorrection(
      const VisualTargetObservation& required_fresh_target,
      double correction_x_m,
      double correction_y_m) override;

 private:
  bool ReadState(
      std::chrono::steady_clock::time_point minimum_receipt_time,
      moveit::core::RobotStatePtr* state,
      geometry_msgs::Pose* grasp_tcp_pose) const;
  AlignmentPose DescribePose(const geometry_msgs::Pose& grasp_tcp_pose) const;
  bool TrajectoryIsValid(
      const moveit::core::RobotState& start_state,
      const geometry_msgs::Pose& start_tcp,
      const moveit_msgs::RobotTrajectory& trajectory,
      double maximum_z_change_m) const;
  AlignmentEndpointCheck ValidateMeasuredEndpoint(
      const geometry_msgs::Pose& requested_tcp,
      const geometry_msgs::Pose& measured_tcp) const;

  moveit::planning_interface::MoveGroupInterface* move_group_ = nullptr;
  MoveItVisualAlignmentMotionConfig config_;
  StateReader state_reader_;
  TargetFreshness target_is_fresh_;
  bool valid_ = false;
};

}  // namespace agx_xarm_pick

#endif  // AGX_XARM_PICK_VISUAL_ALIGNMENT_MOTION_HPP
