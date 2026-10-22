#pragma once

#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include "agx_xarm_pick/tube_view_region.hpp"

namespace agx_xarm_pick {

struct TopDownMotionRequest {
  TopDownViewRegion region;
  double desired_y_m = 0.0;
  double runtime_cap_top_z_m = 0.0;
  double minimum_clearance_m = 0.0;
  double downward_axis_tolerance_rad = 0.0;
  double planning_time_per_candidate_sec = 0.0;
};

struct TopDownMotionResult {
  TopDownViewCandidate candidate;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
};

bool valid_top_down_motion_request(const TopDownMotionRequest& request);

bool trajectory_sample_has_minimum_clearance(
    double tcp_z_m,
    double runtime_cap_top_z_m,
    double minimum_clearance_m);

double absolute_z_axis_tolerance(const Eigen::Vector3d& tcp_z_axis);

class TubeTopDownPlanner {
 public:
  TubeTopDownPlanner(
      moveit::planning_interface::MoveGroupInterface* move_group,
      std::string planning_frame,
      std::string constrained_link,
      const Eigen::Isometry3d& link6_to_tcp);

  std::optional<TopDownMotionResult> Plan(
      const TopDownMotionRequest& request,
      const moveit::core::RobotState& measured_start);

 private:
  bool TrajectoryIsSafe(
      const moveit::planning_interface::MoveGroupInterface::Plan& plan,
      const TopDownMotionRequest& request,
      const moveit::core::RobotState& measured_start) const;

  moveit::planning_interface::MoveGroupInterface* move_group_;
  std::string planning_frame_;
  std::string constrained_link_;
  Eigen::Isometry3d link6_to_tcp_;
};

}  // namespace agx_xarm_pick
