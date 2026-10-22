#include "agx_xarm_pick/tube_grasp_contract.hpp"
#include "agx_xarm_pick/tube_grasp_geometry.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

int main() {
  using agxcore::PayloadKnowledge;
  using agxcore::TubeMissionStatus;
  using agxcore::cap_exceeds_body;
  using agxcore::canonical_grasp_line_yaw;
  using agxcore::choose_grasp_diagonal;
  using agxcore::current_aperture_evidence_is_valid;
  using agxcore::extract_required_arm_joint_positions;
  using agxcore::execution_position_tolerance_m;
  using agxcore::fresh_stamped_state;
  using agxcore::grasp_line_yaw_change_requires_refresh;
  using agxcore::headerless_optional_evidence_is_safe;
  using agxcore::initial_compact_preparation_state_is_safe;
  using agxcore::in_place_yaw_trajectory_is_safe;
  using agxcore::joint_position_snapshots_match;
  using agxcore::known_caught_gripper_state;
  using agxcore::known_open_gripper_state;
  using agxcore::may_command_open_during_cleanup;
  using agxcore::may_attempt_preclose_retreat;
  using agxcore::measured_lattice_is_valid;
  using agxcore::recenter_tcp_state_is_safe;
  using agxcore::payload_after_verified_preclose_cleanup;
  using agxcore::quaternion_orientation_within_tolerance;
  using agxcore::refreshed_target_allows_action;
  using agxcore::refreshed_target_xy_shift_is_safe;
  using agxcore::required_arm_joint_positions_are_finite;
  using agxcore::same_target_observation;
  using agxcore::stamped_observation_is_fresh;
  using agxcore::synchronized_robot_state_evidence_is_valid;
  using agxcore::target_cap_observation_is_valid;
  using agxcore::tcp_approach_axis_is_downward;
  using agxcore::tcp_pose_within_tolerance;
  using agxcore::tube_lift_distance;
  using agxcore::tube_mission_exit_code;
  using agxcore::tube_mission_status_name;
  using agxcore::vertical_xy_path_within_tolerance;
  using agxcore::valid_command_gap_samples;
  using agxcore::valid_hsv_bounds;
  using agxcore::valid_initial_centerline_tolerance;
  using agxcore::valid_initial_compact_z_bounds;
  using agxcore::xy_displacement_within_envelope;
  using agxcore::wrapped_grasp_line_yaw_distance;
  using agxcore::yaw_change_requires_observation_refresh;

  if (tube_mission_exit_code(TubeMissionStatus::kSuccess) != EXIT_SUCCESS) {
    std::cerr << "tube success did not map to EXIT_SUCCESS\n";
    return EXIT_FAILURE;
  }
  constexpr std::array<TubeMissionStatus, 45> failures = {
      TubeMissionStatus::kSetupUnavailable,
      TubeMissionStatus::kInvalidRequest,
      TubeMissionStatus::kProfileMismatch,
      TubeMissionStatus::kGripperStateUnavailable,
      TubeMissionStatus::kPayloadStateUnsafe,
      TubeMissionStatus::kLiftInterfaceUnavailable,
      TubeMissionStatus::kLiftRequestRejected,
      TubeMissionStatus::kLiftSpeedRequestRejected,
      TubeMissionStatus::kLiftFeedbackUnavailable,
      TubeMissionStatus::kLiftMotionTimedOut,
      TubeMissionStatus::kLiftTargetMismatch,
      TubeMissionStatus::kUnsafeInitialCenterline,
      TubeMissionStatus::kPreparationPlanFailed,
      TubeMissionStatus::kPreparationExecutionFailed,
      TubeMissionStatus::kPhaseOnePlanFailed,
      TubeMissionStatus::kPhaseOneExecutionFailed,
      TubeMissionStatus::kBaseNotStopped,
      TubeMissionStatus::kPhaseTwoPlanFailed,
      TubeMissionStatus::kPhaseTwoExecutionFailed,
      TubeMissionStatus::kInvalidPreparationPhase,
      TubeMissionStatus::kSearchWaypointPlanFailed,
      TubeMissionStatus::kSearchWaypointExecutionFailed,
      TubeMissionStatus::kSearchExhausted,
      TubeMissionStatus::kRecenteringExhausted,
      TubeMissionStatus::kObservationPlanFailed,
      TubeMissionStatus::kObservationExecutionFailed,
      TubeMissionStatus::kDetectorUnavailable,
      TubeMissionStatus::kTargetAbsent,
      TubeMissionStatus::kAmbiguousTarget,
      TubeMissionStatus::kStaleFrame,
      TubeMissionStatus::kTfUnavailable,
      TubeMissionStatus::kOutOfWorkspace,
      TubeMissionStatus::kInconsistentTarget,
      TubeMissionStatus::kLayoutUnderconstrained,
      TubeMissionStatus::kLatticeRejected,
      TubeMissionStatus::kClearanceFailed,
      TubeMissionStatus::kInsertionApertureFailed,
      TubeMissionStatus::kPregraspFailed,
      TubeMissionStatus::kDescentFailed,
      TubeMissionStatus::kGripperTimeout,
      TubeMissionStatus::kCaughtStateRequired,
      TubeMissionStatus::kLiftFailed,
      TubeMissionStatus::kRetentionLost,
      TubeMissionStatus::kBusy,
      TubeMissionStatus::kInterrupted,
  };
  for (const auto status : failures) {
    if (tube_mission_exit_code(status) != EXIT_FAILURE ||
        std::string(tube_mission_status_name(status)) == "unknown") {
      std::cerr << "tube failure status mapping is incomplete\n";
      return EXIT_FAILURE;
    }
  }
  constexpr std::array<std::pair<TubeMissionStatus, const char*>, 7>
      preparation_status_names = {{
          {
              TubeMissionStatus::kLiftSpeedRequestRejected,
              "lift speed request rejected",
          },
          {
              TubeMissionStatus::kPhaseOnePlanFailed,
              "phase-one planning failed",
          },
          {
              TubeMissionStatus::kPhaseOneExecutionFailed,
              "phase-one execution failed",
          },
          {
              TubeMissionStatus::kBaseNotStopped,
              "rover base is not stopped",
          },
          {
              TubeMissionStatus::kPhaseTwoPlanFailed,
              "phase-two planning failed",
          },
          {
              TubeMissionStatus::kPhaseTwoExecutionFailed,
              "phase-two execution failed",
          },
          {
              TubeMissionStatus::kInvalidPreparationPhase,
              "invalid preparation phase",
          },
      }};
  for (const auto& status_and_name : preparation_status_names) {
    if (std::string(tube_mission_status_name(status_and_name.first)) !=
        status_and_name.second) {
      std::cerr << "tube preparation status name is incorrect\n";
      return EXIT_FAILURE;
    }
  }

  if (!may_command_open_during_cleanup(PayloadKnowledge::kKnownEmpty) ||
      !may_command_open_during_cleanup(
          PayloadKnowledge::kInsertionAperture)) {
    std::cerr << "known-empty cleanup unexpectedly forbids opening\n";
    return EXIT_FAILURE;
  }
  if (may_command_open_during_cleanup(PayloadKnowledge::kUnknown) ||
      may_command_open_during_cleanup(
          PayloadKnowledge::kCloseIssuedUnknown) ||
      may_command_open_during_cleanup(PayloadKnowledge::kHolding)) {
    std::cerr << "unknown or held payload may be opened\n";
    return EXIT_FAILURE;
  }
  if (!may_attempt_preclose_retreat(
          PayloadKnowledge::kInsertionAperture) ||
      may_attempt_preclose_retreat(PayloadKnowledge::kKnownEmpty) ||
      may_attempt_preclose_retreat(
          PayloadKnowledge::kCloseIssuedUnknown) ||
      may_attempt_preclose_retreat(PayloadKnowledge::kHolding)) {
    std::cerr << "pre-close retreat authorization is unsafe\n";
    return EXIT_FAILURE;
  }
  if (payload_after_verified_preclose_cleanup(
          PayloadKnowledge::kInsertionAperture, true, true) !=
          PayloadKnowledge::kKnownEmpty ||
      payload_after_verified_preclose_cleanup(
          PayloadKnowledge::kInsertionAperture, false, true) !=
          PayloadKnowledge::kInsertionAperture ||
      payload_after_verified_preclose_cleanup(
          PayloadKnowledge::kInsertionAperture, true, false) !=
          PayloadKnowledge::kInsertionAperture ||
      payload_after_verified_preclose_cleanup(
          PayloadKnowledge::kCloseIssuedUnknown, true, true) !=
          PayloadKnowledge::kCloseIssuedUnknown) {
    std::cerr << "pre-close cleanup payload transition is unsafe\n";
    return EXIT_FAILURE;
  }
  constexpr double task10_pi = 3.14159265358979323846;
  if (!yaw_change_requires_observation_refresh(
          0.0, task10_pi / 4.0, 30.0) ||
      yaw_change_requires_observation_refresh(
          0.0, task10_pi / 6.0, 30.0) ||
      !yaw_change_requires_observation_refresh(
          task10_pi - 0.1, -task10_pi + 0.1, 10.0) ||
      yaw_change_requires_observation_refresh(
          task10_pi - 0.1, -task10_pi + 0.1, 12.0)) {
    std::cerr << "yaw refresh threshold is unsafe\n";
    return EXIT_FAILURE;
  }
  const double immediately_after_aperture_command =
      std::nextafter(10.0, 11.0);
  if (!current_aperture_evidence_is_valid(
          true, true, 1, 239.0, 241.0, 240.0, 2.0,
          immediately_after_aperture_command, 10.0, 10.25, 0.25) ||
      !current_aperture_evidence_is_valid(
          true, true, 0, 239.0, 241.0, 240.0, 2.0,
          immediately_after_aperture_command, 10.0, 10.25, 0.25) ||
      current_aperture_evidence_is_valid(
          false, true, 1, 239.0, 241.0, 240.0, 2.0,
          10.0, 10.0, 10.1, 0.25) ||
      current_aperture_evidence_is_valid(
          true, true, 1, 239.0, 241.0, 240.0, 2.0,
          10.0, 10.0, 10.1, 0.25) ||
      current_aperture_evidence_is_valid(
          true, false, 1, 239.0, 241.0, 240.0, 2.0,
          10.0, 10.0, 10.1, 0.25) ||
      current_aperture_evidence_is_valid(
          true, true, 1, 239.0, 241.0, 240.0, 2.0,
          10.0, 10.0, 10.251, 0.25) ||
      current_aperture_evidence_is_valid(
          true, true, 1, 237.9, 241.0, 240.0, 2.0,
          10.0, 10.0, 10.1, 0.25) ||
      current_aperture_evidence_is_valid(
          true, true, 1, 239.0, 242.1, 240.0, 2.0,
          10.0, 10.0, 10.1, 0.25) ||
      current_aperture_evidence_is_valid(
          true, true, 1, 239.0, 241.0, 240.0, 2.0,
          9.999, 10.0, 10.1, 0.25) ||
      current_aperture_evidence_is_valid(
          true, true, 2, 239.0, 241.0, 240.0, 2.0,
          10.0, 10.0, 10.1, 0.25) ||
      current_aperture_evidence_is_valid(
          true, true, 3, 239.0, 241.0, 240.0, 2.0,
          10.0, 10.0, 10.1, 0.25)) {
    std::cerr << "current aperture evidence verification is unsafe\n";
    return EXIT_FAILURE;
  }
  const double line_yaw = -task10_pi / 4.0;
  if (std::abs(wrapped_grasp_line_yaw_distance(
          line_yaw, line_yaw + task10_pi)) > 1e-12 ||
      std::abs(wrapped_grasp_line_yaw_distance(
          line_yaw, line_yaw + task10_pi / 2.0) -
          task10_pi / 2.0) > 1e-12 ||
      grasp_line_yaw_change_requires_refresh(
          line_yaw, line_yaw + task10_pi / 2.0, 180.0) ||
      std::abs(canonical_grasp_line_yaw(
          line_yaw, 3.0 * task10_pi / 4.0) -
          3.0 * task10_pi / 4.0) > 1e-12) {
    std::cerr << "parallel-jaw line yaw symmetry is unsafe\n";
    return EXIT_FAILURE;
  }
  const std::vector<agxcore::Point2> sparse_asymmetric_occupancy = {
      {0.0, 0.0},
      {0.012, 0.012},
  };
  const std::array<
      std::array<agxcore::Point2, 2>, 4> equivalent_lattice_labels = {{
      {{{1.0, 0.0}, {0.0, 1.0}}},
      {{{-1.0, 0.0}, {0.0, -1.0}}},
      {{{0.0, 1.0}, {-1.0, 0.0}}},
      {{{0.0, -1.0}, {1.0, 0.0}}},
  }};
  const auto baseline_diagonal = choose_grasp_diagonal(
      {0.0, 0.0},
      sparse_asymmetric_occupancy,
      equivalent_lattice_labels[0][0],
      equivalent_lattice_labels[0][1],
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  if (!baseline_diagonal) {
    std::cerr << "baseline sparse diagonal was rejected\n";
    return EXIT_FAILURE;
  }
  for (const auto& axes : equivalent_lattice_labels) {
    const auto relabeled_diagonal = choose_grasp_diagonal(
        {0.0, 0.0},
        sparse_asymmetric_occupancy,
        axes[0],
        axes[1],
        0.024,
        0.008,
        0.012,
        0.020,
        0.003);
    if (!relabeled_diagonal ||
        grasp_line_yaw_change_requires_refresh(
            baseline_diagonal->yaw_rad,
            relabeled_diagonal->yaw_rad,
            2.0) ||
        std::abs(
            canonical_grasp_line_yaw(
                relabeled_diagonal->yaw_rad, task10_pi / 3.0) -
            canonical_grasp_line_yaw(
                baseline_diagonal->yaw_rad, task10_pi / 3.0)) >
            1e-12) {
      std::cerr << "equivalent lattice relabeling changed grasp line\n";
      return EXIT_FAILURE;
    }
  }
  const auto orthogonal_diagonal = choose_grasp_diagonal(
      {0.0, 0.0},
      {{0.0, 0.0}, {0.012, -0.012}},
      {1.0, 0.0},
      {0.0, 1.0},
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  if (!orthogonal_diagonal ||
      !grasp_line_yaw_change_requires_refresh(
          baseline_diagonal->yaw_rad,
          orthogonal_diagonal->yaw_rad,
          2.0)) {
    std::cerr << "orthogonal refreshed grasp line was accepted\n";
    return EXIT_FAILURE;
  }
  if (!vertical_xy_path_within_tolerance(
          {0.35, -0.15},
          {{0.35, -0.15}, {0.3509, -0.1509}},
          0.0013) ||
      vertical_xy_path_within_tolerance(
          {0.35, -0.15},
          {{0.35, -0.15}, {0.3510, -0.1510}},
          0.0013) ||
      vertical_xy_path_within_tolerance(
          {0.35, -0.15},
          {
              {0.35, -0.15},
              {std::numeric_limits<double>::quiet_NaN(), -0.15},
          },
          0.0013)) {
    std::cerr << "vertical Cartesian XY drift validation is unsafe\n";
    return EXIT_FAILURE;
  }
  const double half_ten_degrees = 5.0 * task10_pi / 180.0;
  const std::array<double, 4> identity_orientation = {0.0, 0.0, 0.0, 1.0};
  const std::array<double, 4> ten_degree_yaw = {
      0.0,
      0.0,
      std::sin(half_ten_degrees),
      std::cos(half_ten_degrees),
  };
  if (!quaternion_orientation_within_tolerance(
          identity_orientation, ten_degree_yaw, 10.1) ||
      quaternion_orientation_within_tolerance(
          identity_orientation, ten_degree_yaw, 9.9) ||
      !quaternion_orientation_within_tolerance(
          identity_orientation, {0.0, 0.0, 0.0, -1.0}, 0.0) ||
      quaternion_orientation_within_tolerance(
          identity_orientation, {0.0, 0.0, 0.0, 2.0}, 180.0)) {
    std::cerr << "quaternion orientation tolerance is unsafe\n";
    return EXIT_FAILURE;
  }
  constexpr double task10_sqrt_half = 0.70710678118654752440;
  const std::vector<std::array<double, 3>> yaw_positions = {
      {0.35, -0.15, 0.48},
      {0.3505, -0.1505, 0.4805},
      {0.35, -0.15, 0.48},
  };
  const std::vector<std::array<double, 4>> yaw_orientations = {
      {1.0, 0.0, 0.0, 0.0},
      {task10_sqrt_half, task10_sqrt_half, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0},
  };
  if (!in_place_yaw_trajectory_is_safe(
          {0.35, -0.15, 0.48},
          yaw_positions,
          yaw_orientations,
          0.001,
          2.0) ||
      in_place_yaw_trajectory_is_safe(
          {0.35, -0.15, 0.48},
          {
              {0.35, -0.15, 0.48},
              {0.35, -0.15, 0.482},
          },
          {
              {1.0, 0.0, 0.0, 0.0},
              {task10_sqrt_half, task10_sqrt_half, 0.0, 0.0},
          },
          0.001,
          2.0) ||
      in_place_yaw_trajectory_is_safe(
          {0.35, -0.15, 0.48},
          {
              {0.35, -0.15, 0.48},
              {0.352, -0.15, 0.48},
          },
          {
              {1.0, 0.0, 0.0, 0.0},
              {task10_sqrt_half, task10_sqrt_half, 0.0, 0.0},
          },
          0.001,
          2.0) ||
      in_place_yaw_trajectory_is_safe(
          {0.35, -0.15, 0.48},
          yaw_positions,
          {
              {1.0, 0.0, 0.0, 0.0},
              {0.0, 0.0, 0.0, 1.0},
              {0.0, 1.0, 0.0, 0.0},
          },
          0.001,
          2.0) ||
      in_place_yaw_trajectory_is_safe(
          {0.35, -0.15, 0.48},
          {},
          {},
          0.001,
          2.0)) {
    std::cerr << "in-place yaw trajectory validation is unsafe\n";
    return EXIT_FAILURE;
  }
  if (!known_open_gripper_state(
          true, 1, 995.0, 1000.0, 1000.0, 20.0) ||
      known_open_gripper_state(
          true, 1, 0.0, 0.0, 1000.0, 20.0) ||
      known_open_gripper_state(
          false, 1, 1000.0, 1000.0, 1000.0, 20.0) ||
      known_open_gripper_state(
          true, 2, 1000.0, 1000.0, 1000.0, 20.0)) {
    std::cerr << "known-open gripper classification is unsafe\n";
    return EXIT_FAILURE;
  }
  if (!known_caught_gripper_state(true, 2, 5.0, 0.0, 20.0) ||
      known_caught_gripper_state(false, 2, 0.0, 0.0, 20.0) ||
      known_caught_gripper_state(true, 1, 0.0, 0.0, 20.0) ||
      known_caught_gripper_state(true, 2, 1000.0, 0.0, 20.0)) {
    std::cerr << "known-caught gripper classification is unsafe\n";
    return EXIT_FAILURE;
  }
  if (tube_lift_distance(0.080, 0.014, 0.015) < 0.109 - 1e-12) {
    std::cerr << "lift does not clear the complete tube\n";
    return EXIT_FAILURE;
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::array<double, 3> recenter_workspace_min =
      {0.20, -0.50, 0.05};
  const std::array<double, 3> recenter_workspace_max =
      {0.70, 0.50, 0.65};
  const std::array<double, 4> top_down = {1.0, 0.0, 0.0, 0.0};
  const double recenter_position_tolerance =
      execution_position_tolerance_m(0.010);
  if (std::abs(recenter_position_tolerance - 0.020) > 1e-12 ||
      execution_position_tolerance_m(0.001) != 0.005 ||
      std::isfinite(execution_position_tolerance_m(nan)) ||
      !recenter_tcp_state_is_safe(
          {0.35, 0.0, 0.60},
          top_down,
          true,
          recenter_workspace_min,
          recenter_workspace_max,
          0.60,
          recenter_position_tolerance,
          2.0) ||
      recenter_tcp_state_is_safe(
          {0.35, 0.0, 0.60},
          top_down,
          false,
          recenter_workspace_min,
          recenter_workspace_max,
          0.60,
          recenter_position_tolerance,
          2.0) ||
      recenter_tcp_state_is_safe(
          {0.35, 0.0, 0.60},
          {0.0, 0.0, 0.0, 1.0},
          true,
          recenter_workspace_min,
          recenter_workspace_max,
          0.60,
          recenter_position_tolerance,
          2.0) ||
      recenter_tcp_state_is_safe(
          {0.35, 0.0, 0.579},
          top_down,
          true,
          recenter_workspace_min,
          recenter_workspace_max,
          0.60,
          recenter_position_tolerance,
          2.0) ||
      recenter_tcp_state_is_safe(
          {0.35, 0.0, 0.621},
          top_down,
          true,
          recenter_workspace_min,
          recenter_workspace_max,
          0.60,
          recenter_position_tolerance,
          2.0) ||
      recenter_tcp_state_is_safe(
          {0.71, 0.0, 0.60},
          top_down,
          true,
          recenter_workspace_min,
          recenter_workspace_max,
          0.60,
          recenter_position_tolerance,
          2.0)) {
    std::cerr << "recenter TCP safety did not fail closed\n";
    return EXIT_FAILURE;
  }

  double cumulative_recenter_x = 0.0;
  bool individual_recenters_valid = true;
  for (int move = 0; move < 3; ++move) {
    const double next_x = cumulative_recenter_x + 0.08;
    individual_recenters_valid =
        individual_recenters_valid &&
        xy_displacement_within_envelope(
            cumulative_recenter_x,
            0.0,
            next_x,
            0.0,
            0.08);
    cumulative_recenter_x = next_x;
  }
  if (!individual_recenters_valid ||
      !xy_displacement_within_envelope(
          0.0, 0.0, 0.08, 0.0, 0.08) ||
      xy_displacement_within_envelope(
          0.0, 0.0, 0.080001, 0.0, 0.08) ||
      xy_displacement_within_envelope(
          0.0, 0.0, cumulative_recenter_x, 0.0, 0.08) ||
      xy_displacement_within_envelope(
          0.0, 0.0, nan, 0.0, 0.08)) {
    std::cerr << "cumulative recenter envelope is unsafe\n";
    return EXIT_FAILURE;
  }
  if (!same_target_observation(
          {0.0, 0.0, 0.0},
          {0.012, 0.0, 0.0},
          0.012) ||
      same_target_observation(
          {0.0, 0.0, 0.0},
          {0.012001, 0.0, 0.0},
          0.012) ||
      same_target_observation(
          {0.35, -0.15, 0.40},
          {0.43, -0.15, 0.40},
          0.012) ||
      same_target_observation(
          {0.35, -0.15, 0.40},
          {nan, -0.15, 0.40},
          0.012)) {
    std::cerr << "target identity continuity is unsafe\n";
    return EXIT_FAILURE;
  }
  if (!stamped_observation_is_fresh(true, 9.5, 10.0, 0.5) ||
      stamped_observation_is_fresh(true, 9.499, 10.0, 0.5) ||
      stamped_observation_is_fresh(true, 10.001, 10.0, 0.5) ||
      stamped_observation_is_fresh(false, 9.9, 10.0, 0.5) ||
      stamped_observation_is_fresh(true, 0.0, 10.0, 0.5)) {
    std::cerr << "observation freshness boundary is unsafe\n";
    return EXIT_FAILURE;
  }
  if (stamped_observation_is_fresh(true, 9.0, 10.0, 0.5) ||
      !refreshed_target_allows_action(
          {0.35, -0.15, 0.40},
          {0.355, -0.15, 0.40},
          true,
          9.9,
          10.0,
          0.5,
          0.012) ||
      refreshed_target_allows_action(
          {0.35, -0.15, 0.40},
          {0.43, -0.15, 0.40},
          true,
          9.9,
          10.0,
          0.5,
          0.012) ||
      refreshed_target_allows_action(
          {0.35, -0.15, 0.40},
          {0.355, -0.15, 0.40},
          true,
          9.0,
          10.0,
          0.5,
          0.012)) {
    std::cerr << "post-motion refreshed target gate is unsafe\n";
    return EXIT_FAILURE;
  }
  if (!refreshed_target_xy_shift_is_safe(
          0.0, 0.0, 0.004, 0.0, 0.004, 0.012) ||
      refreshed_target_xy_shift_is_safe(
          0.0, 0.0, 0.004001, 0.0, 0.004, 0.012) ||
      !refreshed_target_xy_shift_is_safe(
          0.0, 0.0, 0.012, 0.0, 0.020, 0.012) ||
      refreshed_target_xy_shift_is_safe(
          0.0, 0.0, 0.012001, 0.0, 0.020, 0.012)) {
    std::cerr << "post-motion XY shift did not use the tighter bound\n";
    return EXIT_FAILURE;
  }

  constexpr double sqrt_half = 0.70710678118654752440;
  const std::array<double, 3> lattice_target = {0.35, -0.15, 0.40};
  const std::array<double, 3> lattice_axis_u =
      {sqrt_half, sqrt_half, 0.0};
  const std::array<double, 3> lattice_axis_v =
      {-sqrt_half, sqrt_half, 0.0};
  std::vector<std::int32_t> lattice_u = {0, 1, -1, 0, 0, 1};
  std::vector<std::int32_t> lattice_v = {0, 0, 0, 1, -1, 1};
  const auto lattice_point =
      [&](std::int32_t u, std::int32_t v) {
        return std::array<double, 3>{
            lattice_target[0] +
                0.040 * (
                    u * lattice_axis_u[0] +
                    v * lattice_axis_v[0]),
            lattice_target[1] +
                0.040 * (
                    u * lattice_axis_u[1] +
                    v * lattice_axis_v[1]),
            lattice_target[2]};
      };
  std::vector<std::array<double, 3>> lattice_points;
  for (std::size_t index = 0; index < lattice_u.size(); ++index) {
    lattice_points.push_back(
        lattice_point(lattice_u[index], lattice_v[index]));
  }
  if (!measured_lattice_is_valid(
          lattice_target,
          lattice_points,
          lattice_u,
          lattice_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0)) {
    std::cerr << "valid sparse rotated lattice was rejected\n";
    return EXIT_FAILURE;
  }
  auto height_varied_points = lattice_points;
  height_varied_points[1][2] += 0.005;
  height_varied_points[2][2] -= 0.005;
  if (!measured_lattice_is_valid(
          lattice_target,
          height_varied_points,
          lattice_u,
          lattice_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0)) {
    std::cerr << "XY lattice rejected allowed cap-height variation\n";
    return EXIT_FAILURE;
  }
  auto near_limit_points = lattice_points;
  for (auto& point : near_limit_points) {
    point[0] += 0.0035;
  }
  if (!measured_lattice_is_valid(
          lattice_target,
          near_limit_points,
          lattice_u,
          lattice_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0035) ||
      measured_lattice_is_valid(
          lattice_target,
          near_limit_points,
          lattice_u,
          lattice_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0)) {
    std::cerr << "recomputed XY RMS was incoherent with producer metadata\n";
    return EXIT_FAILURE;
  }
  auto impossible_points = lattice_points;
  impossible_points.back()[0] += 0.020;
  auto missing_zero_u = lattice_u;
  auto missing_zero_v = lattice_v;
  missing_zero_u[0] = -1;
  missing_zero_v[0] = -1;
  auto mismatched_zero_points = lattice_points;
  mismatched_zero_points[0][0] += 0.005;
  auto overwide_u = lattice_u;
  auto overwide_points = lattice_points;
  overwide_u.back() = 2;
  overwide_points.back() = lattice_point(2, lattice_v.back());
  auto overwide_v = lattice_v;
  auto overhigh_points = lattice_points;
  overwide_v.back() = 2;
  overhigh_points.back() = lattice_point(lattice_u.back(), 2);
  auto duplicate_u = lattice_u;
  auto duplicate_v = lattice_v;
  auto duplicate_points = lattice_points;
  duplicate_u.back() = 0;
  duplicate_v.back() = 0;
  duplicate_points.back() = lattice_target;
  auto short_grid_v = lattice_v;
  short_grid_v.pop_back();
  if (measured_lattice_is_valid(
          lattice_target,
          impossible_points,
          lattice_u,
          lattice_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0) ||
      measured_lattice_is_valid(
          lattice_target,
          lattice_points,
          lattice_u,
          short_grid_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0) ||
      measured_lattice_is_valid(
          lattice_target,
          overhigh_points,
          lattice_u,
          overwide_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0) ||
      measured_lattice_is_valid(
          lattice_target,
          lattice_points,
          missing_zero_u,
          missing_zero_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0) ||
      measured_lattice_is_valid(
          lattice_target,
          mismatched_zero_points,
          lattice_u,
          lattice_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0) ||
      measured_lattice_is_valid(
          lattice_target,
          overwide_points,
          overwide_u,
          lattice_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0) ||
      measured_lattice_is_valid(
          lattice_target,
          lattice_points,
          lattice_u,
          lattice_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.005) ||
      measured_lattice_is_valid(
          lattice_target,
          duplicate_points,
          duplicate_u,
          duplicate_v,
          lattice_axis_u,
          lattice_axis_v,
          3,
          3,
          6,
          0.040,
          0.004,
          0.004,
          0.0)) {
    std::cerr << "malformed lattice was accepted from claimed RMS metadata\n";
    return EXIT_FAILURE;
  }

  if (!tcp_pose_within_tolerance(
          0.20, -0.10, 0.40,
          0.23, -0.14, 0.40,
          0.05) ||
      tcp_pose_within_tolerance(
          0.20, -0.10, 0.40,
          0.230001, -0.14, 0.40,
          0.05) ||
      tcp_pose_within_tolerance(
          nan, -0.10, 0.40,
          0.20, -0.10, 0.40,
          0.05) ||
      tcp_pose_within_tolerance(
          0.20, -0.10, 0.40,
          0.20, nan, 0.40,
          0.05) ||
      tcp_pose_within_tolerance(
          0.20, -0.10, 0.40,
          0.20, -0.10, 0.40,
          nan) ||
      tcp_pose_within_tolerance(
          0.20, -0.10, 0.40,
          0.20, -0.10, 0.40,
          -0.01)) {
    std::cerr << "TCP translation tolerance is not fail closed\n";
    return EXIT_FAILURE;
  }

  constexpr double pi = 3.14159265358979323846;
  const auto tilted_from_downward =
      [](double tilt_deg, double tolerance_deg) {
        const double rotation =
            pi - tilt_deg * pi / 180.0;
        return tcp_approach_axis_is_downward(
            std::sin(rotation / 2.0),
            0.0,
            0.0,
            std::cos(rotation / 2.0),
            tolerance_deg);
      };
  if (!tcp_approach_axis_is_downward(1.0, 0.0, 0.0, 0.0, 2.0) ||
      tcp_approach_axis_is_downward(0.0, 0.0, 0.0, 1.0, 2.0) ||
      tilted_from_downward(10.001, 10.0) ||
      !tilted_from_downward(10.0, 10.0) ||
      tcp_approach_axis_is_downward(2.0, 0.0, 0.0, 0.0, 2.0) ||
      tcp_approach_axis_is_downward(nan, 0.0, 0.0, 1.0, 2.0) ||
      tcp_approach_axis_is_downward(1.0, 0.0, 0.0, 0.0, -0.1) ||
      tcp_approach_axis_is_downward(1.0, 0.0, 0.0, 0.0, 180.1)) {
    std::cerr << "TCP downward-axis tolerance is not fail closed\n";
    return EXIT_FAILURE;
  }

  const auto initial_rise_state_safe =
      [](double x, double y, double z, double joint1) {
        return initial_compact_preparation_state_is_safe(
            x, y, z,
            joint1,
            0.10, 0.30,
            0.0, 0.005,
            -0.10, 0.05,
            0.30,
            0.0, 0.020);
      };
  if (!valid_initial_centerline_tolerance(0.005) ||
      valid_initial_centerline_tolerance(0.005001) ||
      valid_initial_centerline_tolerance(0.050) ||
      valid_initial_centerline_tolerance(0.0) ||
      valid_initial_centerline_tolerance(-0.001) ||
      valid_initial_centerline_tolerance(nan) ||
      !valid_initial_compact_z_bounds(-0.10, 0.05, 0.30) ||
      valid_initial_compact_z_bounds(-0.10, -0.10, 0.30) ||
      valid_initial_compact_z_bounds(0.05, -0.10, 0.30) ||
      valid_initial_compact_z_bounds(-0.10, 0.30, 0.30) ||
      valid_initial_compact_z_bounds(-0.10, 0.31, 0.30) ||
      valid_initial_compact_z_bounds(nan, 0.05, 0.30) ||
      valid_initial_compact_z_bounds(-0.10, nan, 0.30) ||
      valid_initial_compact_z_bounds(-0.10, 0.05, nan) ||
      !initial_rise_state_safe(
          0.201655, 0.000018, -0.069883, 0.000100) ||
      !initial_rise_state_safe(0.10, 0.005, -0.10, 0.020) ||
      !initial_rise_state_safe(0.30, -0.005, 0.05, -0.020) ||
      initial_rise_state_safe(0.099999, 0.0, 0.30, 0.0) ||
      initial_rise_state_safe(0.300001, 0.0, 0.30, 0.0) ||
      initial_rise_state_safe(0.20, 0.005001, 0.30, 0.0) ||
      initial_rise_state_safe(0.20, 0.0, -0.100001, 0.0) ||
      initial_rise_state_safe(0.20, 0.0, 0.050001, 0.0) ||
      initial_rise_state_safe(0.20, 0.0, 0.30, 0.020001) ||
      initial_rise_state_safe(nan, 0.0, 0.30, 0.0) ||
      initial_rise_state_safe(0.20, nan, 0.30, 0.0) ||
      initial_rise_state_safe(0.20, 0.0, nan, 0.0) ||
      initial_rise_state_safe(0.20, 0.0, 0.30, nan) ||
      initial_rise_state_safe(
          std::numeric_limits<double>::infinity(), 0.0, 0.30, 0.0) ||
      initial_rise_state_safe(
          0.20, 0.0, -std::numeric_limits<double>::infinity(), 0.0) ||
      initial_compact_preparation_state_is_safe(
          0.20, 0.0, 0.30, 0.0,
          0.10, 0.30,
          0.0, 0.005,
          -0.10, 0.05,
          0.30,
          0.0, nan) ||
      initial_compact_preparation_state_is_safe(
          0.20, 0.0, 0.30, 0.0,
          0.10, 0.30,
          0.0, 0.005001,
          -0.10, 0.05,
          0.30,
          0.0, 0.020)) {
    std::cerr << "initial centerline rise validation is not fail closed\n";
    return EXIT_FAILURE;
  }

  const std::array<double, 3> workspace_min = {0.20, -0.50, 0.05};
  const std::array<double, 3> workspace_max = {0.70, 0.50, 0.65};
  const auto target_valid =
      [&](double stamp, double now, double x, double y, double z) {
        return target_cap_observation_is_valid(
            true,
            stamp,
            now,
            0.50,
            {x, y, z},
            workspace_min,
            workspace_max,
            0.402564,
            0.013);
      };
  if (!target_valid(
          9.8, 10.0, 0.351485 + 0.040, 0.000869 - 0.040, 0.402564) ||
      target_valid(9.4, 10.0, 0.35, -0.15, 0.402564) ||
      target_valid(10.1, 10.0, 0.35, -0.15, 0.402564) ||
      target_valid(9.8, 10.0, 0.35, -0.15, 0.416) ||
      target_valid(9.8, 10.0, nan, -0.15, 0.402564) ||
      target_valid(9.8, 10.0, 0.71, -0.15, 0.402564) ||
      target_cap_observation_is_valid(
          false,
          9.8,
          10.0,
          0.50,
          {0.35, -0.15, 0.402564},
          workspace_min,
          workspace_max,
          0.402564,
          0.013)) {
    std::cerr << "target-only observation validation is unsafe\n";
    return EXIT_FAILURE;
  }

  if (!fresh_stamped_state(true, 9.8, 10.0, 0.20, 0.25, true) ||
      fresh_stamped_state(false, 9.8, 10.0, 0.20, 0.25, true) ||
      fresh_stamped_state(true, 0.0, 10.0, 0.20, 0.25, true) ||
      fresh_stamped_state(true, 10.1, 10.0, 0.20, 0.25, true) ||
      fresh_stamped_state(true, 9.7, 10.0, 0.20, 0.25, true) ||
      fresh_stamped_state(true, 9.8, 10.0, 0.30, 0.25, true) ||
      fresh_stamped_state(true, 9.8, 10.0, 0.20, 0.25, false)) {
    std::cerr << "fresh stamped state validation is unsafe\n";
    return EXIT_FAILURE;
  }
  if (!required_arm_joint_positions_are_finite(
          {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"},
          {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}) ||
      required_arm_joint_positions_are_finite(
          {"joint1", "joint2", "joint3", "joint4", "joint5"},
          {0.0, 0.1, 0.2, 0.3, 0.4}) ||
      required_arm_joint_positions_are_finite(
          {"joint1", "joint2", "joint3", "joint4", "joint5", "joint5"},
          {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}) ||
      required_arm_joint_positions_are_finite(
          {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"},
          {0.0, 0.1, 0.2, 0.3, 0.4, nan}) ||
      required_arm_joint_positions_are_finite(
          {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"},
          {0.0, 0.1, 0.2})) {
    std::cerr << "required arm joint validation is unsafe\n";
    return EXIT_FAILURE;
  }
  std::vector<double> extracted_positions;
  if (!extract_required_arm_joint_positions(
          {"joint6", "joint4", "joint2", "joint5", "joint1", "joint3"},
          {0.5, 0.3, 0.1, 0.4, 0.0, 0.2},
          &extracted_positions) ||
      extracted_positions !=
          std::vector<double>({0.0, 0.1, 0.2, 0.3, 0.4, 0.5}) ||
      extract_required_arm_joint_positions(
          {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"},
          {0.0, 0.1, 0.2, 0.3, 0.4, 0.5},
          nullptr)) {
    std::cerr << "joint-state extraction did not preserve joint1..joint6\n";
    return EXIT_FAILURE;
  }
  const std::vector<double> joint_snapshot =
      {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  const std::vector<double> close_joint_snapshot =
      {0.0, 0.1, 0.2, 0.3, 0.4, 0.5000005};
  const std::vector<double> mismatched_joint_snapshot =
      {0.0, 0.1, 0.2, 0.3, 0.4, 0.51};
  if (!joint_position_snapshots_match(
          joint_snapshot, close_joint_snapshot, 1e-6) ||
      joint_position_snapshots_match(
          joint_snapshot, mismatched_joint_snapshot, 1e-6) ||
      joint_position_snapshots_match(
          {0.0, 0.1}, {0.0, 0.1}, 1e-6) ||
      joint_position_snapshots_match(
          joint_snapshot, {0.0, 0.1, 0.2, 0.3, 0.4, nan}, 1e-6) ||
      joint_position_snapshots_match(
          joint_snapshot, joint_snapshot, -1e-6) ||
      joint_position_snapshots_match(
          joint_snapshot, joint_snapshot, nan)) {
    std::cerr << "joint snapshot matching is not fail closed\n";
    return EXIT_FAILURE;
  }
  if (!synchronized_robot_state_evidence_is_valid(
          true,
          9.9,
          10.0,
          4.9,
          5.0,
          4.8,
          0.25,
          joint_snapshot,
          close_joint_snapshot,
          1e-6) ||
      synchronized_robot_state_evidence_is_valid(
          false,
          9.9,
          10.0,
          4.9,
          5.0,
          4.8,
          0.25,
          joint_snapshot,
          joint_snapshot,
          1e-6) ||
      synchronized_robot_state_evidence_is_valid(
          true,
          0.0,
          0.0,
          4.9,
          5.0,
          4.8,
          0.25,
          joint_snapshot,
          joint_snapshot,
          1e-6) ||
      synchronized_robot_state_evidence_is_valid(
          true,
          9.9,
          10.0,
          4.7,
          5.0,
          4.8,
          0.25,
          joint_snapshot,
          joint_snapshot,
          1e-6) ||
      synchronized_robot_state_evidence_is_valid(
          true,
          9.9,
          10.0,
          4.9,
          5.0,
          4.900000001,
          0.25,
          joint_snapshot,
          joint_snapshot,
          1e-6) ||
      synchronized_robot_state_evidence_is_valid(
          true,
          9.7,
          10.0,
          4.9,
          5.0,
          4.8,
          0.25,
          joint_snapshot,
          joint_snapshot,
          1e-6) ||
      synchronized_robot_state_evidence_is_valid(
          true,
          9.9,
          10.0,
          4.9,
          5.0,
          4.8,
          0.25,
          joint_snapshot,
          mismatched_joint_snapshot,
          1e-6)) {
    std::cerr << "synchronized robot-state evidence is unsafe\n";
    return EXIT_FAILURE;
  }
  if (!headerless_optional_evidence_is_safe(false) ||
      headerless_optional_evidence_is_safe(true)) {
    std::cerr << "headerless optional evidence was accepted\n";
    return EXIT_FAILURE;
  }

  if (!cap_exceeds_body(0.020, 0.016) ||
      cap_exceeds_body(0.016, 0.016) ||
      cap_exceeds_body(nan, 0.016) ||
      !valid_hsv_bounds({40, 80, 50}, {85, 255, 255}) ||
      !valid_hsv_bounds({179, 255, 255}, {179, 255, 255}) ||
      valid_hsv_bounds({180, 80, 50}, {180, 255, 255}) ||
      valid_hsv_bounds({86, 80, 50}, {85, 255, 255})) {
    std::cerr << "profile physical or HSV validation diverged\n";
    return EXIT_FAILURE;
  }
  if (!valid_command_gap_samples(
          {{0.0, 0.000}, {500.0, 0.050}, {1000.0, 0.100}},
          0.0,
          1000.0) ||
      !valid_command_gap_samples(
          {{100.0, 0.010}, {500.0, 0.050}, {900.0, 0.090}},
          0.0,
          1000.0) ||
      valid_command_gap_samples(
          {{-1.0, 0.010}, {500.0, 0.050}, {900.0, 0.090}},
          0.0,
          1000.0)) {
    std::cerr << "command-gap validation diverged from canonical schema\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
