#ifndef AGX_XARM_PICK_TUBE_GRASP_CONTRACT_HPP
#define AGX_XARM_PICK_TUBE_GRASP_CONTRACT_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace agxcore {

enum class TubeMissionStatus {
  kSuccess = 0,
  kSetupUnavailable,
  kInvalidRequest,
  kProfileMismatch,
  kGripperStateUnavailable,
  kPayloadStateUnsafe,
  kLiftInterfaceUnavailable,
  kLiftRequestRejected,
  kLiftSpeedRequestRejected,
  kLiftFeedbackUnavailable,
  kLiftMotionTimedOut,
  kLiftTargetMismatch,
  kUnsafeInitialCenterline,
  kPreparationPlanFailed,
  kPreparationExecutionFailed,
  kPhaseOnePlanFailed,
  kPhaseOneExecutionFailed,
  kBaseNotStopped,
  kPhaseTwoPlanFailed,
  kPhaseTwoExecutionFailed,
  kInvalidPreparationPhase,
  kSearchWaypointPlanFailed,
  kSearchWaypointExecutionFailed,
  kSearchExhausted,
  kRecenteringExhausted,
  kObservationPlanFailed,
  kObservationExecutionFailed,
  kDetectorUnavailable,
  kTargetAbsent,
  kAmbiguousTarget,
  kStaleFrame,
  kTfUnavailable,
  kOutOfWorkspace,
  kInconsistentTarget,
  kLayoutUnderconstrained,
  kLatticeRejected,
  kClearanceFailed,
  kInsertionApertureFailed,
  kPregraspFailed,
  kDescentFailed,
  kGripperTimeout,
  kCaughtStateRequired,
  kLiftFailed,
  kRetentionLost,
  kBusy,
  kInterrupted,
};

enum class PayloadKnowledge {
  kUnknown = 0,
  kKnownEmpty,
  kInsertionAperture,
  kCloseIssuedUnknown,
  kHolding,
};

inline int tube_mission_exit_code(TubeMissionStatus status) {
  return status == TubeMissionStatus::kSuccess ? EXIT_SUCCESS : EXIT_FAILURE;
}

inline bool may_command_open_during_cleanup(PayloadKnowledge knowledge) {
  return knowledge == PayloadKnowledge::kKnownEmpty ||
      knowledge == PayloadKnowledge::kInsertionAperture;
}

inline bool may_attempt_preclose_retreat(PayloadKnowledge knowledge) {
  return knowledge == PayloadKnowledge::kInsertionAperture;
}

inline PayloadKnowledge payload_after_verified_preclose_cleanup(
    PayloadKnowledge knowledge,
    bool retreat_succeeded,
    bool open_verified) {
  return knowledge == PayloadKnowledge::kInsertionAperture &&
          retreat_succeeded &&
          open_verified
      ? PayloadKnowledge::kKnownEmpty
      : knowledge;
}

inline bool known_open_gripper_state(
    bool is_initialized,
    int grip_state,
    double position,
    double target_position,
    double open_position,
    double tolerance) {
  return is_initialized &&
      grip_state == 1 &&
      std::isfinite(position) &&
      std::isfinite(target_position) &&
      std::isfinite(open_position) &&
      std::isfinite(tolerance) &&
      tolerance >= 0.0 &&
      std::abs(position - open_position) <= tolerance &&
      std::abs(target_position - open_position) <= tolerance;
}

inline bool known_caught_gripper_state(
    bool is_initialized,
    int grip_state,
    double target_position,
    double close_position,
    double tolerance) {
  return is_initialized &&
      grip_state == 2 &&
      std::isfinite(target_position) &&
      std::isfinite(close_position) &&
      std::isfinite(tolerance) &&
      tolerance >= 0.0 &&
      std::abs(target_position - close_position) <= tolerance;
}

inline bool current_aperture_evidence_is_valid(
    bool has_state,
    bool is_initialized,
    int grip_state,
    double position,
    double target_position,
    double command_position,
    double tolerance,
    double state_receipt_time,
    double command_time,
    double current_time,
    double maximum_age) {
  return has_state &&
      is_initialized &&
      (grip_state == 0 || grip_state == 1) &&
      std::isfinite(position) &&
      std::isfinite(target_position) &&
      std::isfinite(command_position) &&
      std::isfinite(tolerance) &&
      tolerance >= 0.0 &&
      std::isfinite(state_receipt_time) &&
      std::isfinite(command_time) &&
      std::isfinite(current_time) &&
      std::isfinite(maximum_age) &&
      maximum_age >= 0.0 &&
      state_receipt_time > command_time &&
      state_receipt_time <= current_time &&
      current_time - state_receipt_time <= maximum_age &&
      std::abs(position - command_position) <= tolerance &&
      std::abs(target_position - command_position) <= tolerance;
}

inline bool vertical_xy_path_within_tolerance(
    const std::array<double, 2>& start_xy,
    const std::vector<std::array<double, 2>>& sampled_xy,
    double tolerance_m) {
  if (!std::isfinite(start_xy[0]) ||
      !std::isfinite(start_xy[1]) ||
      !std::isfinite(tolerance_m) ||
      tolerance_m < 0.0 ||
      sampled_xy.empty()) {
    return false;
  }
  for (const auto& point : sampled_xy) {
    if (!std::isfinite(point[0]) ||
        !std::isfinite(point[1]) ||
        std::hypot(
            point[0] - start_xy[0],
            point[1] - start_xy[1]) > tolerance_m) {
      return false;
    }
  }
  return true;
}

inline bool yaw_change_requires_observation_refresh(
    double previous_yaw_rad,
    double next_yaw_rad,
    double maximum_change_deg) {
  if (!std::isfinite(previous_yaw_rad) ||
      !std::isfinite(next_yaw_rad) ||
      !std::isfinite(maximum_change_deg) ||
      maximum_change_deg < 0.0 ||
      maximum_change_deg > 180.0) {
    return true;
  }
  constexpr double kPi = 3.14159265358979323846;
  const double wrapped_change = std::remainder(
      next_yaw_rad - previous_yaw_rad, 2.0 * kPi);
  return !std::isfinite(wrapped_change) ||
      std::abs(wrapped_change) >
          maximum_change_deg * kPi / 180.0 + 1e-12;
}

inline double wrapped_grasp_line_yaw_distance(
    double first_yaw_rad,
    double second_yaw_rad) {
  if (!std::isfinite(first_yaw_rad) ||
      !std::isfinite(second_yaw_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  constexpr double kPi = 3.14159265358979323846;
  return std::abs(std::remainder(
      second_yaw_rad - first_yaw_rad, kPi));
}

inline double canonical_grasp_line_yaw(
    double line_yaw_rad,
    double reference_yaw_rad) {
  const double distance = wrapped_grasp_line_yaw_distance(
      reference_yaw_rad, line_yaw_rad);
  if (!std::isfinite(distance)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  constexpr double kPi = 3.14159265358979323846;
  return reference_yaw_rad +
      std::remainder(line_yaw_rad - reference_yaw_rad, kPi);
}

inline bool grasp_line_yaw_change_requires_refresh(
    double previous_line_yaw_rad,
    double next_line_yaw_rad,
    double maximum_change_deg) {
  const double distance = wrapped_grasp_line_yaw_distance(
      previous_line_yaw_rad, next_line_yaw_rad);
  if (!std::isfinite(distance) ||
      !std::isfinite(maximum_change_deg) ||
      maximum_change_deg < 0.0 ||
      maximum_change_deg > 180.0) {
    return true;
  }
  constexpr double kPi = 3.14159265358979323846;
  return distance >
      maximum_change_deg * kPi / 180.0 + 1e-12;
}

inline bool quaternion_orientation_within_tolerance(
    const std::array<double, 4>& first,
    const std::array<double, 4>& second,
    double tolerance_deg) {
  if (!std::isfinite(tolerance_deg) ||
      tolerance_deg < 0.0 ||
      tolerance_deg > 180.0) {
    return false;
  }
  double first_norm_squared = 0.0;
  double second_norm_squared = 0.0;
  double dot = 0.0;
  for (std::size_t index = 0; index < first.size(); ++index) {
    if (!std::isfinite(first[index]) ||
        !std::isfinite(second[index])) {
      return false;
    }
    first_norm_squared += first[index] * first[index];
    second_norm_squared += second[index] * second[index];
    dot += first[index] * second[index];
  }
  if (std::abs(first_norm_squared - 1.0) > 1e-6 ||
      std::abs(second_norm_squared - 1.0) > 1e-6) {
    return false;
  }
  constexpr double kPi = 3.14159265358979323846;
  const double absolute_dot = std::min(1.0, std::abs(dot));
  const double angular_distance_rad =
      2.0 * std::acos(absolute_dot);
  return std::isfinite(angular_distance_rad) &&
      angular_distance_rad <= tolerance_deg * kPi / 180.0 + 1e-12;
}

inline bool tcp_pose_within_tolerance(
    double requested_x,
    double requested_y,
    double requested_z,
    double measured_x,
    double measured_y,
    double measured_z,
    double tolerance_m) {
  if (!std::isfinite(requested_x) ||
      !std::isfinite(requested_y) ||
      !std::isfinite(requested_z) ||
      !std::isfinite(measured_x) ||
      !std::isfinite(measured_y) ||
      !std::isfinite(measured_z) ||
      !std::isfinite(tolerance_m) ||
      tolerance_m < 0.0) {
    return false;
  }
  const double dx = requested_x - measured_x;
  const double dy = requested_y - measured_y;
  const double dz = requested_z - measured_z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) <= tolerance_m;
}

inline bool tcp_approach_axis_is_downward(
    double quaternion_x,
    double quaternion_y,
    double quaternion_z,
    double quaternion_w,
    double tolerance_deg) {
  if (!std::isfinite(quaternion_x) ||
      !std::isfinite(quaternion_y) ||
      !std::isfinite(quaternion_z) ||
      !std::isfinite(quaternion_w) ||
      !std::isfinite(tolerance_deg) ||
      tolerance_deg < 0.0 ||
      tolerance_deg > 180.0) {
    return false;
  }
  const double norm = std::sqrt(
      quaternion_x * quaternion_x +
      quaternion_y * quaternion_y +
      quaternion_z * quaternion_z +
      quaternion_w * quaternion_w);
  if (std::abs(norm - 1.0) > 1e-6) {
    return false;
  }
  constexpr double kDegreesToRadians =
      3.14159265358979323846 / 180.0;
  const double approach_axis_z =
      1.0 - 2.0 * (
          quaternion_x * quaternion_x +
          quaternion_y * quaternion_y);
  return -approach_axis_z >=
      std::cos(tolerance_deg * kDegreesToRadians);
}

inline bool in_place_yaw_trajectory_is_safe(
    const std::array<double, 3>& anchor_position,
    const std::vector<std::array<double, 3>>& sampled_positions,
    const std::vector<std::array<double, 4>>& sampled_orientations,
    double position_tolerance_m,
    double downward_axis_tolerance_deg) {
  if (sampled_positions.empty() ||
      sampled_positions.size() != sampled_orientations.size() ||
      !std::isfinite(position_tolerance_m) ||
      position_tolerance_m < 0.0) {
    return false;
  }
  for (double coordinate : anchor_position) {
    if (!std::isfinite(coordinate)) {
      return false;
    }
  }
  for (std::size_t index = 0; index < sampled_positions.size(); ++index) {
    double squared_drift = 0.0;
    for (std::size_t axis = 0; axis < anchor_position.size(); ++axis) {
      if (!std::isfinite(sampled_positions[index][axis])) {
        return false;
      }
      const double difference =
          sampled_positions[index][axis] - anchor_position[axis];
      squared_drift += difference * difference;
    }
    if (!std::isfinite(squared_drift) ||
        std::sqrt(squared_drift) > position_tolerance_m ||
        !tcp_approach_axis_is_downward(
            sampled_orientations[index][0],
            sampled_orientations[index][1],
            sampled_orientations[index][2],
            sampled_orientations[index][3],
            downward_axis_tolerance_deg)) {
      return false;
    }
  }
  return true;
}

inline double execution_position_tolerance_m(double cartesian_step_m) {
  if (!std::isfinite(cartesian_step_m) || cartesian_step_m < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(0.005, 2.0 * cartesian_step_m);
}

inline bool recenter_tcp_state_is_safe(
    const std::array<double, 3>& tcp_position,
    const std::array<double, 4>& tcp_quaternion,
    bool camera_axis_is_downward,
    const std::array<double, 3>& workspace_min,
    const std::array<double, 3>& workspace_max,
    double search_height_m,
    double position_tolerance_m,
    double downward_axis_tolerance_deg) {
  if (!camera_axis_is_downward ||
      !std::isfinite(search_height_m) ||
      !std::isfinite(position_tolerance_m) ||
      position_tolerance_m < 0.0) {
    return false;
  }
  for (std::size_t index = 0; index < tcp_position.size(); ++index) {
    if (!std::isfinite(tcp_position[index]) ||
        !std::isfinite(workspace_min[index]) ||
        !std::isfinite(workspace_max[index]) ||
        workspace_min[index] > workspace_max[index] ||
        tcp_position[index] < workspace_min[index] ||
        tcp_position[index] > workspace_max[index]) {
      return false;
    }
  }
  return std::abs(tcp_position[2] - search_height_m) <=
          position_tolerance_m &&
      tcp_approach_axis_is_downward(
          tcp_quaternion[0],
          tcp_quaternion[1],
          tcp_quaternion[2],
          tcp_quaternion[3],
          downward_axis_tolerance_deg);
}

inline bool xy_displacement_within_envelope(
    double origin_x,
    double origin_y,
    double current_x,
    double current_y,
    double maximum_displacement_m) {
  return std::isfinite(origin_x) &&
      std::isfinite(origin_y) &&
      std::isfinite(current_x) &&
      std::isfinite(current_y) &&
      std::isfinite(maximum_displacement_m) &&
      maximum_displacement_m >= 0.0 &&
      std::hypot(current_x - origin_x, current_y - origin_y) <=
          maximum_displacement_m;
}

inline bool same_target_observation(
    const std::array<double, 3>& first,
    const std::array<double, 3>& second,
    double maximum_distance_m) {
  if (!std::isfinite(maximum_distance_m) || maximum_distance_m < 0.0) {
    return false;
  }
  double squared_distance = 0.0;
  for (std::size_t index = 0; index < first.size(); ++index) {
    if (!std::isfinite(first[index]) || !std::isfinite(second[index])) {
      return false;
    }
    const double difference = first[index] - second[index];
    squared_distance += difference * difference;
  }
  return std::sqrt(squared_distance) <= maximum_distance_m;
}

inline constexpr double kMaximumInitialCenterlineToleranceM = 0.005;

inline bool valid_initial_centerline_tolerance(double tolerance_m) {
  return std::isfinite(tolerance_m) &&
      tolerance_m > 0.0 &&
      tolerance_m <= kMaximumInitialCenterlineToleranceM;
}

inline bool valid_initial_compact_z_bounds(
    double minimum_z_m,
    double maximum_z_m,
    double first_rise_height_m) {
  return std::isfinite(minimum_z_m) &&
      std::isfinite(maximum_z_m) &&
      std::isfinite(first_rise_height_m) &&
      minimum_z_m < maximum_z_m &&
      maximum_z_m < first_rise_height_m;
}

inline bool initial_compact_preparation_state_is_safe(
    double tcp_x,
    double tcp_y,
    double tcp_z,
    double joint1,
    double initial_x_min,
    double initial_x_max,
    double centerline_y,
    double maximum_center_offset_m,
    double initial_z_min_m,
    double initial_z_max_m,
    double phase_one_height_m,
    double centerline_joint,
    double centerline_joint_tolerance) {
  if (!std::isfinite(tcp_x) ||
      !std::isfinite(tcp_y) ||
      !std::isfinite(tcp_z) ||
      !std::isfinite(joint1) ||
      !std::isfinite(initial_x_min) ||
      !std::isfinite(initial_x_max) ||
      initial_x_min > initial_x_max ||
      !std::isfinite(centerline_y) ||
      !valid_initial_centerline_tolerance(maximum_center_offset_m) ||
      !valid_initial_compact_z_bounds(
          initial_z_min_m, initial_z_max_m, phase_one_height_m) ||
      !std::isfinite(centerline_joint) ||
      !std::isfinite(centerline_joint_tolerance) ||
      centerline_joint_tolerance < 0.0) {
    return false;
  }
  return tcp_x >= initial_x_min &&
      tcp_x <= initial_x_max &&
      std::abs(tcp_y - centerline_y) <= maximum_center_offset_m &&
      tcp_z >= initial_z_min_m &&
      tcp_z <= initial_z_max_m &&
      tcp_z < phase_one_height_m &&
      std::abs(joint1 - centerline_joint) <=
          centerline_joint_tolerance;
}

inline bool fresh_stamped_state(
    bool has_state,
    double stamp_sec,
    double now_sec,
    double wall_receipt_age_sec,
    double maximum_age_sec,
    bool values_finite) {
  return has_state &&
      values_finite &&
      std::isfinite(stamp_sec) &&
      stamp_sec > 0.0 &&
      std::isfinite(now_sec) &&
      now_sec >= stamp_sec &&
      std::isfinite(wall_receipt_age_sec) &&
      wall_receipt_age_sec >= 0.0 &&
      std::isfinite(maximum_age_sec) &&
      maximum_age_sec > 0.0 &&
      now_sec - stamp_sec <= maximum_age_sec &&
      wall_receipt_age_sec <= maximum_age_sec;
}

inline bool stamped_observation_is_fresh(
    bool frame_matches,
    double stamp_sec,
    double now_sec,
    double maximum_age_sec) {
  return frame_matches &&
      fresh_stamped_state(
          true,
          stamp_sec,
          now_sec,
          0.0,
          maximum_age_sec,
          true);
}

inline bool refreshed_target_allows_action(
    const std::array<double, 3>& previous_target,
    const std::array<double, 3>& refreshed_target,
    bool frame_matches,
    double refreshed_stamp_sec,
    double now_sec,
    double maximum_age_sec,
    double maximum_identity_distance_m) {
  return stamped_observation_is_fresh(
          frame_matches,
          refreshed_stamp_sec,
          now_sec,
          maximum_age_sec) &&
      same_target_observation(
          previous_target,
          refreshed_target,
          maximum_identity_distance_m);
}

inline bool refreshed_target_xy_shift_is_safe(
    double previous_x,
    double previous_y,
    double refreshed_x,
    double refreshed_y,
    double pitch_tolerance_m,
    double maximum_interframe_spread_m) {
  if (!std::isfinite(pitch_tolerance_m) ||
      !std::isfinite(maximum_interframe_spread_m) ||
      pitch_tolerance_m < 0.0 ||
      maximum_interframe_spread_m < 0.0) {
    return false;
  }
  return xy_displacement_within_envelope(
      previous_x,
      previous_y,
      refreshed_x,
      refreshed_y,
      std::min(pitch_tolerance_m, maximum_interframe_spread_m));
}

inline bool extract_required_arm_joint_positions(
    const std::vector<std::string>& names,
    const std::vector<double>& positions,
    std::vector<double>* ordered_positions) {
  if (ordered_positions == nullptr) {
    return false;
  }
  const std::array<std::string, 6> required = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  std::array<bool, 6> found{};
  std::array<double, 6> ordered{};
  for (std::size_t index = 0; index < names.size(); ++index) {
    const auto match =
        std::find(required.begin(), required.end(), names[index]);
    if (match == required.end()) {
      continue;
    }
    const std::size_t required_index =
        static_cast<std::size_t>(
            std::distance(required.begin(), match));
    if (found[required_index] ||
        index >= positions.size() ||
        !std::isfinite(positions[index])) {
      return false;
    }
    found[required_index] = true;
    ordered[required_index] = positions[index];
  }
  if (!std::all_of(found.begin(), found.end(), [](bool value) {
    return value;
  })) {
    return false;
  }
  ordered_positions->assign(ordered.begin(), ordered.end());
  return true;
}

inline bool required_arm_joint_positions_are_finite(
    const std::vector<std::string>& names,
    const std::vector<double>& positions) {
  std::vector<double> ordered_positions;
  return extract_required_arm_joint_positions(
      names, positions, &ordered_positions);
}

inline bool joint_position_snapshots_match(
    const std::vector<double>& source_positions,
    const std::vector<double>& compared_positions,
    double tolerance_rad) {
  if (source_positions.size() != 6 ||
      compared_positions.size() != 6 ||
      !std::isfinite(tolerance_rad) ||
      tolerance_rad < 0.0) {
    return false;
  }
  for (std::size_t index = 0; index < source_positions.size(); ++index) {
    if (!std::isfinite(source_positions[index]) ||
        !std::isfinite(compared_positions[index]) ||
        std::abs(
            source_positions[index] - compared_positions[index]) >
            tolerance_rad) {
      return false;
    }
  }
  return true;
}

inline bool synchronized_robot_state_evidence_is_valid(
    bool has_snapshot,
    double source_stamp_sec,
    double ros_now_sec,
    double receipt_time_sec,
    double wall_now_sec,
    double minimum_receipt_time_sec,
    double maximum_age_sec,
    const std::vector<double>& source_positions,
    const std::vector<double>& moveit_positions,
    double position_tolerance_rad) {
  if (!std::isfinite(receipt_time_sec) ||
      !std::isfinite(wall_now_sec) ||
      !std::isfinite(minimum_receipt_time_sec) ||
      wall_now_sec < receipt_time_sec ||
      receipt_time_sec < minimum_receipt_time_sec) {
    return false;
  }
  return fresh_stamped_state(
          has_snapshot,
          source_stamp_sec,
          ros_now_sec,
          wall_now_sec - receipt_time_sec,
          maximum_age_sec,
          true) &&
      joint_position_snapshots_match(
          source_positions,
          moveit_positions,
          position_tolerance_rad);
}

inline bool target_cap_observation_is_valid(
    bool frame_matches,
    double stamp_sec,
    double now_sec,
    double maximum_age_sec,
    const std::array<double, 3>& point,
    const std::array<double, 3>& workspace_min,
    const std::array<double, 3>& workspace_max,
    double expected_cap_top_z_m,
    double height_tolerance_m) {
  if (!frame_matches ||
      !fresh_stamped_state(
          true,
          stamp_sec,
          now_sec,
          0.0,
          maximum_age_sec,
          true) ||
      !std::isfinite(expected_cap_top_z_m) ||
      !std::isfinite(height_tolerance_m) ||
      height_tolerance_m < 0.0) {
    return false;
  }
  for (std::size_t index = 0; index < point.size(); ++index) {
    if (!std::isfinite(point[index]) ||
        !std::isfinite(workspace_min[index]) ||
        !std::isfinite(workspace_max[index]) ||
        workspace_min[index] > workspace_max[index] ||
        point[index] < workspace_min[index] ||
        point[index] > workspace_max[index]) {
      return false;
    }
  }
  return std::abs(point[2] - expected_cap_top_z_m) <= height_tolerance_m;
}

inline constexpr double kLatticeRmsCoherenceToleranceM = 1e-6;

inline bool measured_lattice_is_valid(
    const std::array<double, 3>& target,
    const std::vector<std::array<double, 3>>& points,
    const std::vector<std::int32_t>& grid_u,
    const std::vector<std::int32_t>& grid_v,
    const std::array<double, 3>& axis_u,
    const std::array<double, 3>& axis_v,
    int rows,
    int columns,
    int minimum_inliers,
    double center_pitch_m,
    double point_tolerance_m,
    double maximum_rms_error_m,
    double reported_rms_error_m) {
  const std::size_t point_count = points.size();
  if (point_count != grid_u.size() ||
      point_count != grid_v.size() ||
      rows <= 0 ||
      columns <= 0 ||
      minimum_inliers <= 0 ||
      point_count < static_cast<std::size_t>(minimum_inliers) ||
      point_count > static_cast<std::size_t>(
          static_cast<std::int64_t>(rows) *
          static_cast<std::int64_t>(columns)) ||
      !std::isfinite(center_pitch_m) ||
      center_pitch_m <= 0.0 ||
      !std::isfinite(point_tolerance_m) ||
      point_tolerance_m < 0.0 ||
      !std::isfinite(maximum_rms_error_m) ||
      maximum_rms_error_m < 0.0 ||
      !std::isfinite(reported_rms_error_m) ||
      reported_rms_error_m < 0.0 ||
      reported_rms_error_m > maximum_rms_error_m) {
    return false;
  }
  for (std::size_t axis = 0; axis < target.size(); ++axis) {
    if (!std::isfinite(target[axis]) ||
        !std::isfinite(axis_u[axis]) ||
        !std::isfinite(axis_v[axis])) {
      return false;
    }
  }
  const double axis_u_norm = std::sqrt(
      axis_u[0] * axis_u[0] +
      axis_u[1] * axis_u[1] +
      axis_u[2] * axis_u[2]);
  const double axis_v_norm = std::sqrt(
      axis_v[0] * axis_v[0] +
      axis_v[1] * axis_v[1] +
      axis_v[2] * axis_v[2]);
  const double axis_dot =
      axis_u[0] * axis_v[0] +
      axis_u[1] * axis_v[1] +
      axis_u[2] * axis_v[2];
  if (std::abs(axis_u[2]) > 1e-6 ||
      std::abs(axis_v[2]) > 1e-6 ||
      std::abs(axis_u_norm - 1.0) > 1e-3 ||
      std::abs(axis_v_norm - 1.0) > 1e-3 ||
      std::abs(axis_dot) > 1e-3) {
    return false;
  }

  std::set<std::pair<std::int32_t, std::int32_t>> occupied;
  std::int32_t minimum_u = std::numeric_limits<std::int32_t>::max();
  std::int32_t maximum_u = std::numeric_limits<std::int32_t>::min();
  std::int32_t minimum_v = std::numeric_limits<std::int32_t>::max();
  std::int32_t maximum_v = std::numeric_limits<std::int32_t>::min();
  std::size_t target_key_count = 0;
  for (std::size_t index = 0; index < point_count; ++index) {
    if (!occupied.emplace(grid_u[index], grid_v[index]).second) {
      return false;
    }
    minimum_u = std::min(minimum_u, grid_u[index]);
    maximum_u = std::max(maximum_u, grid_u[index]);
    minimum_v = std::min(minimum_v, grid_v[index]);
    maximum_v = std::max(maximum_v, grid_v[index]);
    if (grid_u[index] == 0 && grid_v[index] == 0) {
      ++target_key_count;
    }
  }
  if (target_key_count != 1 ||
      static_cast<std::int64_t>(maximum_u) -
              static_cast<std::int64_t>(minimum_u) >= columns ||
      static_cast<std::int64_t>(maximum_v) -
              static_cast<std::int64_t>(minimum_v) >= rows) {
    return false;
  }

  double sum_squared_residual = 0.0;
  for (std::size_t index = 0; index < point_count; ++index) {
    for (double coordinate : points[index]) {
      if (!std::isfinite(coordinate)) {
        return false;
      }
    }
    double squared_residual = 0.0;
    for (std::size_t axis = 0; axis < 2; ++axis) {
      const double expected =
          target[axis] +
          center_pitch_m * (
              static_cast<double>(grid_u[index]) * axis_u[axis] +
              static_cast<double>(grid_v[index]) * axis_v[axis]);
      const double residual = points[index][axis] - expected;
      squared_residual += residual * residual;
    }
    const double residual_norm = std::sqrt(squared_residual);
    if (!std::isfinite(residual_norm) ||
        residual_norm > point_tolerance_m) {
      return false;
    }
    if (grid_u[index] == 0 &&
        grid_v[index] == 0 &&
        !same_target_observation(
            target, points[index], point_tolerance_m)) {
      return false;
    }
    sum_squared_residual += squared_residual;
  }
  const double recomputed_rms =
      std::sqrt(sum_squared_residual / static_cast<double>(point_count));
  return std::isfinite(recomputed_rms) &&
      recomputed_rms <= maximum_rms_error_m &&
      std::abs(reported_rms_error_m - recomputed_rms) <=
          kLatticeRmsCoherenceToleranceM;
}

inline bool cap_exceeds_body(double cap_diameter_m, double body_diameter_m) {
  return std::isfinite(cap_diameter_m) &&
      std::isfinite(body_diameter_m) &&
      cap_diameter_m > body_diameter_m &&
      body_diameter_m > 0.0;
}

inline bool valid_hsv_bounds(
    const std::array<int, 3>& lower,
    const std::array<int, 3>& upper) {
  constexpr std::array<int, 3> kLimits = {179, 255, 255};
  for (std::size_t index = 0; index < lower.size(); ++index) {
    if (lower[index] < 0 ||
        upper[index] < 0 ||
        lower[index] > upper[index] ||
        upper[index] > kLimits[index]) {
      return false;
    }
  }
  return true;
}

inline bool valid_command_gap_samples(
    const std::vector<std::array<double, 2>>& samples,
    double close_command,
    double fully_open_command) {
  if (samples.size() < 2 ||
      !std::isfinite(close_command) ||
      !std::isfinite(fully_open_command) ||
      close_command >= fully_open_command) {
    return false;
  }
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const double command = samples[index][0];
    const double gap_m = samples[index][1];
    if (!std::isfinite(command) ||
        !std::isfinite(gap_m) ||
        command < close_command ||
        command > fully_open_command ||
        gap_m < 0.0 ||
        (index > 0 &&
         (samples[index - 1][0] >= command ||
          samples[index - 1][1] >= gap_m))) {
      return false;
    }
  }
  return true;
}

inline bool headerless_optional_evidence_is_safe(bool input_available) {
  return !input_available;
}

inline double tube_lift_distance(
    double body_height_m,
    double cap_height_m,
    double lift_clearance_m) {
  return body_height_m + cap_height_m + lift_clearance_m;
}

inline const char* tube_mission_status_name(TubeMissionStatus status) {
  switch (status) {
    case TubeMissionStatus::kSuccess: return "success";
    case TubeMissionStatus::kSetupUnavailable: return "setup unavailable";
    case TubeMissionStatus::kInvalidRequest: return "invalid request";
    case TubeMissionStatus::kProfileMismatch: return "fixture profile mismatch";
    case TubeMissionStatus::kGripperStateUnavailable: return "gripper state unavailable";
    case TubeMissionStatus::kPayloadStateUnsafe: return "payload state unsafe";
    case TubeMissionStatus::kLiftInterfaceUnavailable: return "lift interface unavailable";
    case TubeMissionStatus::kLiftRequestRejected: return "lift request rejected";
    case TubeMissionStatus::kLiftSpeedRequestRejected: return "lift speed request rejected";
    case TubeMissionStatus::kLiftFeedbackUnavailable: return "lift feedback unavailable";
    case TubeMissionStatus::kLiftMotionTimedOut: return "lift motion timed out";
    case TubeMissionStatus::kLiftTargetMismatch: return "lift feedback target mismatch";
    case TubeMissionStatus::kUnsafeInitialCenterline: return "unsafe initial centerline state";
    case TubeMissionStatus::kPreparationPlanFailed: return "preparation planning failed";
    case TubeMissionStatus::kPreparationExecutionFailed: return "preparation execution failed";
    case TubeMissionStatus::kPhaseOnePlanFailed: return "phase-one planning failed";
    case TubeMissionStatus::kPhaseOneExecutionFailed: return "phase-one execution failed";
    case TubeMissionStatus::kBaseNotStopped: return "rover base is not stopped";
    case TubeMissionStatus::kPhaseTwoPlanFailed: return "phase-two planning failed";
    case TubeMissionStatus::kPhaseTwoExecutionFailed: return "phase-two execution failed";
    case TubeMissionStatus::kInvalidPreparationPhase: return "invalid preparation phase";
    case TubeMissionStatus::kSearchWaypointPlanFailed: return "search waypoint planning failed";
    case TubeMissionStatus::kSearchWaypointExecutionFailed: return "search waypoint execution failed";
    case TubeMissionStatus::kSearchExhausted: return "target not found after search";
    case TubeMissionStatus::kRecenteringExhausted: return "target recentering exhausted";
    case TubeMissionStatus::kObservationPlanFailed: return "observation planning failed";
    case TubeMissionStatus::kObservationExecutionFailed: return "observation execution failed";
    case TubeMissionStatus::kDetectorUnavailable: return "detector unavailable";
    case TubeMissionStatus::kTargetAbsent: return "target absent";
    case TubeMissionStatus::kAmbiguousTarget: return "ambiguous target";
    case TubeMissionStatus::kStaleFrame: return "stale frame";
    case TubeMissionStatus::kTfUnavailable: return "TF unavailable";
    case TubeMissionStatus::kOutOfWorkspace: return "target out of workspace";
    case TubeMissionStatus::kInconsistentTarget: return "inconsistent target";
    case TubeMissionStatus::kLayoutUnderconstrained: return "layout underconstrained";
    case TubeMissionStatus::kLatticeRejected: return "lattice rejected";
    case TubeMissionStatus::kClearanceFailed: return "physical clearance failed";
    case TubeMissionStatus::kInsertionApertureFailed: return "insertion aperture failed";
    case TubeMissionStatus::kPregraspFailed: return "pregrasp failed";
    case TubeMissionStatus::kDescentFailed: return "descent failed";
    case TubeMissionStatus::kGripperTimeout: return "gripper timeout";
    case TubeMissionStatus::kCaughtStateRequired: return "caught state required";
    case TubeMissionStatus::kLiftFailed: return "lift failed";
    case TubeMissionStatus::kRetentionLost: return "retention lost";
    case TubeMissionStatus::kBusy: return "mission busy";
    case TubeMissionStatus::kInterrupted: return "interrupted";
  }
  return "unknown";
}

}  // namespace agxcore

#endif  // AGX_XARM_PICK_TUBE_GRASP_CONTRACT_HPP
