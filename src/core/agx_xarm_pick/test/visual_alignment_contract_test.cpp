#include "agx_xarm_pick/visual_alignment_contract.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>

namespace {

bool Near(double actual, double expected, double tolerance = 1e-9) {
  return std::abs(actual - expected) <= tolerance;
}

int Fail(const char* detail) {
  std::cerr << detail << '\n';
  return EXIT_FAILURE;
}

}  // namespace

int main() {
  using agx_xarm_pick::AlignmentEndpointStatus;
  using agx_xarm_pick::HorizontalTrajectorySample;
  using agx_xarm_pick::Vec3;
  using agx_xarm_pick::VisualAlignmentConfig;
  using agx_xarm_pick::VisualTargetObservation;
  using agx_xarm_pick::bounded_visual_correction;
  using agx_xarm_pick::exact_down_camera_axes;
  using agx_xarm_pick::next_planning_retry_magnitude;
  using agx_xarm_pick::projected_image_axes;
  using agx_xarm_pick::valid_visual_alignment_config;
  using agx_xarm_pick::validate_alignment_endpoint;
  using agx_xarm_pick::valid_horizontal_trajectory_samples;
  using agx_xarm_pick::visual_pixel_error;

  const VisualAlignmentConfig valid{};
  if (!valid_visual_alignment_config(valid)) {
    return Fail("default commissioned alignment config is invalid");
  }
  auto invalid = valid;
  invalid.u_axis_gain_m_per_px = 0.000009;
  if (valid_visual_alignment_config(invalid)) {
    return Fail("gain below 0.00001 m/px was accepted");
  }
  invalid = valid;
  invalid.v_axis_gain_m_per_px = 0.005001;
  if (valid_visual_alignment_config(invalid)) {
    return Fail("gain above 0.005 m/px was accepted");
  }
  invalid = valid;
  invalid.minimum_retry_step_m = 0.051;
  if (valid_visual_alignment_config(invalid)) {
    return Fail("retry step larger than maximum step was accepted");
  }
  invalid = valid;
  invalid.maximum_total_displacement_m = 0.049;
  if (valid_visual_alignment_config(invalid)) {
    return Fail("maximum step larger than displacement budget was accepted");
  }

  VisualTargetObservation target;
  target.u_px = 323.5;
  target.v_px = 242.5;
  target.reference_u_px = 319.5;
  target.reference_v_px = 239.5;
  if (!Near(visual_pixel_error(target), 5.0)) {
    return Fail("pixel error is not Euclidean");
  }

  const auto yaw_zero = projected_image_axes(
      Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0},
      Vec3{0.0, 0.0, -1.0}, 2.0);
  const auto yaw_plus_90 = projected_image_axes(
      Vec3{0.0, 1.0, 0.0}, Vec3{-1.0, 0.0, 0.0},
      Vec3{0.0, 0.0, -1.0}, 2.0);
  const auto yaw_minus_90 = projected_image_axes(
      Vec3{0.0, -1.0, 0.0}, Vec3{1.0, 0.0, 0.0},
      Vec3{0.0, 0.0, -1.0}, 2.0);
  if (!yaw_zero || !yaw_plus_90 || !yaw_minus_90) {
    return Fail("valid image axes were rejected");
  }

  VisualTargetObservation axis_target;
  axis_target.u_px = 329.5;
  axis_target.v_px = 259.5;
  axis_target.reference_u_px = 319.5;
  axis_target.reference_v_px = 239.5;
  auto correction = bounded_visual_correction(
      valid, axis_target, *yaw_zero, valid.maximum_total_displacement_m,
      valid.maximum_step_m);
  if (!correction || !Near(correction->x_m, 0.0075) ||
      !Near(correction->y_m, 0.015)) {
    return Fail("yaw-zero correction did not follow U/V image axes");
  }
  correction = bounded_visual_correction(
      valid, axis_target, *yaw_plus_90,
      valid.maximum_total_displacement_m, valid.maximum_step_m);
  if (!correction || !Near(correction->x_m, -0.015) ||
      !Near(correction->y_m, 0.0075)) {
    return Fail("positive-yaw correction did not rotate with the camera");
  }
  correction = bounded_visual_correction(
      valid, axis_target, *yaw_minus_90,
      valid.maximum_total_displacement_m, valid.maximum_step_m);
  if (!correction || !Near(correction->x_m, 0.015) ||
      !Near(correction->y_m, -0.0075)) {
    return Fail("negative-yaw correction did not rotate with the camera");
  }

  if (projected_image_axes(
          Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0},
          Vec3{0.0, std::sin(3.0 * M_PI / 180.0),
               -std::cos(3.0 * M_PI / 180.0)},
          2.0)) {
    return Fail("camera with excessive downward error was accepted");
  }
  if (projected_image_axes(
          Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 1.0, 0.0},
          Vec3{0.0, 0.0, -1.0}, 2.0)) {
    return Fail("degenerate projected image axis was accepted");
  }
  if (projected_image_axes(
          Vec3{1.0, 0.0, 0.0}, Vec3{1.0, 0.01, 0.0},
          Vec3{0.0, 0.0, -1.0}, 2.0)) {
    return Fail("non-orthogonal image axes were accepted");
  }

  const auto exact_down_zero = exact_down_camera_axes(
      Vec3{0.98, 0.0, 0.20});
  if (!exact_down_zero ||
      !Near(exact_down_zero->u_axis.x, 1.0) ||
      !Near(exact_down_zero->u_axis.y, 0.0) ||
      !Near(exact_down_zero->u_axis.z, 0.0) ||
      !Near(exact_down_zero->v_axis.x, 0.0) ||
      !Near(exact_down_zero->v_axis.y, -1.0) ||
      !Near(exact_down_zero->v_axis.z, 0.0) ||
      !Near(exact_down_zero->forward_axis.x, 0.0) ||
      !Near(exact_down_zero->forward_axis.y, 0.0) ||
      !Near(exact_down_zero->forward_axis.z, -1.0)) {
    return Fail("zero-yaw exact-down camera basis is invalid");
  }
  const auto exact_down_rotated = exact_down_camera_axes(
      Vec3{0.0, 2.0, -0.25});
  if (!exact_down_rotated ||
      !Near(exact_down_rotated->u_axis.x, 0.0) ||
      !Near(exact_down_rotated->u_axis.y, 1.0) ||
      !Near(exact_down_rotated->v_axis.x, 1.0) ||
      !Near(exact_down_rotated->v_axis.y, 0.0) ||
      !Near(exact_down_rotated->forward_axis.z, -1.0)) {
    return Fail("rotated exact-down camera basis did not preserve U yaw");
  }
  if (exact_down_camera_axes(Vec3{0.0, 0.0, 1.0}) ||
      exact_down_camera_axes(Vec3{
          std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0})) {
    return Fail("degenerate exact-down camera basis input was accepted");
  }

  axis_target.u_px = 419.5;
  axis_target.v_px = 239.5;
  correction = bounded_visual_correction(
      valid, axis_target, *yaw_zero, 0.20, 0.05);
  if (!correction || !Near(std::hypot(correction->x_m, correction->y_m), 0.05)) {
    return Fail("correction was not clamped to 50 mm");
  }
  correction = bounded_visual_correction(
      valid, axis_target, *yaw_zero, 0.010, 0.05);
  if (!correction || !Near(std::hypot(correction->x_m, correction->y_m), 0.010)) {
    return Fail("correction was not clamped to remaining displacement");
  }

  const auto retry0 = next_planning_retry_magnitude(0.05, 0, 0.00625);
  const auto retry1 = next_planning_retry_magnitude(0.05, 1, 0.00625);
  const auto retry2 = next_planning_retry_magnitude(0.05, 2, 0.00625);
  const auto retry3 = next_planning_retry_magnitude(0.05, 3, 0.00625);
  const auto retry4 = next_planning_retry_magnitude(0.05, 4, 0.00625);
  if (!retry0 || !retry1 || !retry2 || !retry3 || retry4 ||
      !Near(*retry0, 0.05) || !Near(*retry1, 0.025) ||
      !Near(*retry2, 0.0125) || !Near(*retry3, 0.00625)) {
    return Fail("planning retry magnitudes are not 50/25/12.5/6.25 mm");
  }
  const auto near_floor_original =
      next_planning_retry_magnitude(0.00975, 0, 0.00625);
  const auto near_floor_retry =
      next_planning_retry_magnitude(0.00975, 1, 0.00625);
  const auto near_floor_exhausted =
      next_planning_retry_magnitude(0.00975, 2, 0.00625);
  if (!near_floor_original || !near_floor_retry || near_floor_exhausted ||
      !Near(*near_floor_original, 0.00975) ||
      !Near(*near_floor_retry, 0.00625)) {
    return Fail(
        "planning retry did not attempt the minimum step exactly once");
  }
  const auto small_original =
      next_planning_retry_magnitude(0.004, 0, 0.00625);
  const auto small_retry =
      next_planning_retry_magnitude(0.004, 1, 0.00625);
  if (!small_original || !Near(*small_original, 0.004) || small_retry) {
    return Fail("sub-floor original correction was not attempted exactly once");
  }

  const auto live_endpoint = validate_alignment_endpoint(
      true, true,
      2.189109, 0.000496, 0.004098, 0.54519,
      2.0, 0.005, 0.005, 2.0);
  if (live_endpoint.status != AlignmentEndpointStatus::kDownwardRejected ||
      std::string(live_endpoint.residual_name) != "downward_error_deg" ||
      !Near(live_endpoint.residual, 2.189109) ||
      !Near(live_endpoint.limit, 2.0)) {
    return Fail("live endpoint did not identify the downward residual");
  }
  const auto valid_endpoint = validate_alignment_endpoint(
      true, true,
      1.9, 0.005, 0.005, 2.0,
      2.0, 0.005, 0.005, 2.0);
  if (valid_endpoint.status != AlignmentEndpointStatus::kValid) {
    return Fail("endpoint values at their exact limits were rejected");
  }
  if (validate_alignment_endpoint(
          false, true, 0.0, 0.0, 0.0, 0.0,
          2.0, 0.005, 0.005, 2.0).status !=
          AlignmentEndpointStatus::kInvalidPose ||
      validate_alignment_endpoint(
          true, false, 0.0, 0.0, 0.0, 0.0,
          2.0, 0.005, 0.005, 2.0).status !=
          AlignmentEndpointStatus::kWorkspaceRejected ||
      validate_alignment_endpoint(
          true, true, 1.0, 0.0051, 0.0, 0.0,
          2.0, 0.005, 0.005, 2.0).status !=
          AlignmentEndpointStatus::kPlanarPositionRejected ||
      validate_alignment_endpoint(
          true, true, 1.0, 0.0, 0.0051, 0.0,
          2.0, 0.005, 0.005, 2.0).status !=
          AlignmentEndpointStatus::kHeightRejected ||
      validate_alignment_endpoint(
          true, true, 1.0, 0.0, 0.0, 2.1,
          2.0, 0.005, 0.005, 2.0).status !=
          AlignmentEndpointStatus::kOrientationRejected ||
      validate_alignment_endpoint(
          true, true, std::numeric_limits<double>::quiet_NaN(),
          0.0, 0.0, 0.0,
          2.0, 0.005, 0.005, 2.0).status !=
          AlignmentEndpointStatus::kInvalidPose) {
    return Fail("endpoint residual statuses are not distinct and fail closed");
  }

  const HorizontalTrajectorySample start{
      0.50, 0.00, 0.35, 0.0, 0.0, 0.0, 1.0, 0.0};
  const HorizontalTrajectorySample middle{
      0.51, 0.01, 0.35, 0.0, 0.0, 0.0, 1.0, 0.5};
  const HorizontalTrajectorySample end{
      0.52, 0.02, 0.35, 0.0, 0.0, 0.0, 1.0, 1.0};
  if (!valid_horizontal_trajectory_samples(
          {start, middle, end}, start,
          {0.09, -0.50, 0.05}, {0.70, 0.50, 0.65},
          0.002, 2.0, 2.0)) {
    return Fail("valid horizontal trajectory samples were rejected");
  }
  auto invalid_sample = middle;
  invalid_sample.z_m += 0.0021;
  if (valid_horizontal_trajectory_samples(
          {start, invalid_sample, end}, start,
          {0.09, -0.50, 0.05}, {0.70, 0.50, 0.65},
          0.002, 2.0, 2.0)) {
    return Fail("trajectory Z violation was accepted");
  }
  invalid_sample = middle;
  invalid_sample.qz = std::sin(2.1 * M_PI / 360.0);
  invalid_sample.qw = std::cos(2.1 * M_PI / 360.0);
  if (valid_horizontal_trajectory_samples(
          {start, invalid_sample, end}, start,
          {0.09, -0.50, 0.05}, {0.70, 0.50, 0.65},
          0.002, 2.0, 2.0)) {
    return Fail("trajectory orientation violation was accepted");
  }
  invalid_sample = middle;
  invalid_sample.x_m = 0.71;
  if (valid_horizontal_trajectory_samples(
          {start, invalid_sample, end}, start,
          {0.09, -0.50, 0.05}, {0.70, 0.50, 0.65},
          0.002, 2.0, 2.0)) {
    return Fail("trajectory workspace violation was accepted");
  }
  invalid_sample = middle;
  invalid_sample.downward_error_deg = 2.1;
  if (valid_horizontal_trajectory_samples(
          {start, invalid_sample, end}, start,
          {0.09, -0.50, 0.05}, {0.70, 0.50, 0.65},
          0.002, 2.0, 2.0)) {
    return Fail("trajectory downward-angle violation was accepted");
  }
  return EXIT_SUCCESS;
}
