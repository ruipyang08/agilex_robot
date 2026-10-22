#ifndef AGX_XARM_PICK_VISUAL_ALIGNMENT_CONTRACT_HPP
#define AGX_XARM_PICK_VISUAL_ALIGNMENT_CONTRACT_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace agx_xarm_pick {

constexpr double kMinimumVisualGainMPerPx = 0.00001;
constexpr double kMaximumVisualGainMPerPx = 0.005;
constexpr double kMaximumImageAxisDot = 0.01;

struct VisualAlignmentConfig {
  double u_axis_gain_m_per_px = 0.00075;
  double v_axis_gain_m_per_px = 0.00075;
  double target_center_tolerance_px = 8.0;
  int required_centered_observations = 2;
  double maximum_step_m = 0.05;
  double minimum_retry_step_m = 0.00625;
  double maximum_total_displacement_m = 0.20;
  int maximum_iterations = 12;
  int stationary_target_retries = 2;
  double minimum_pixel_error_improvement_px = 1.0;
};

struct VisualTargetObservation {
  std::uint64_t stamp_ns = 0;
  std::string frame_id;
  double u_px = 0.0;
  double v_px = 0.0;
  double reference_u_px = 0.0;
  double reference_v_px = 0.0;
  std::uint32_t image_width = 0;
  std::uint32_t image_height = 0;
};

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ImageAxesXY {
  double u_x = 0.0;
  double u_y = 0.0;
  double v_x = 0.0;
  double v_y = 0.0;
  double downward_error_deg = 0.0;
};

struct ExactDownCameraAxes {
  Vec3 u_axis;
  Vec3 v_axis;
  Vec3 forward_axis;
};

struct PlanarCorrection {
  double x_m = 0.0;
  double y_m = 0.0;
};

struct HorizontalTrajectorySample {
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  double downward_error_deg = 0.0;
};

enum class AlignmentEndpointStatus {
  kValid,
  kInvalidPose,
  kWorkspaceRejected,
  kDownwardRejected,
  kPlanarPositionRejected,
  kHeightRejected,
  kOrientationRejected,
};

struct AlignmentEndpointCheck {
  AlignmentEndpointStatus status = AlignmentEndpointStatus::kInvalidPose;
  const char* residual_name = "invalid_pose";
  double residual = 0.0;
  double limit = 0.0;
};

inline const char* alignment_endpoint_status_name(
    AlignmentEndpointStatus status) {
  switch (status) {
    case AlignmentEndpointStatus::kValid:
      return "valid";
    case AlignmentEndpointStatus::kInvalidPose:
      return "invalid_pose";
    case AlignmentEndpointStatus::kWorkspaceRejected:
      return "workspace_rejected";
    case AlignmentEndpointStatus::kDownwardRejected:
      return "downward_rejected";
    case AlignmentEndpointStatus::kPlanarPositionRejected:
      return "planar_position_rejected";
    case AlignmentEndpointStatus::kHeightRejected:
      return "height_rejected";
    case AlignmentEndpointStatus::kOrientationRejected:
      return "orientation_rejected";
  }
  return "invalid_pose";
}

inline AlignmentEndpointCheck validate_alignment_endpoint(
    bool measured_pose_valid,
    bool workspace_valid,
    double downward_error_deg,
    double planar_position_error_m,
    double height_error_m,
    double orientation_error_deg,
    double maximum_downward_error_deg,
    double maximum_planar_position_error_m,
    double maximum_height_error_m,
    double maximum_orientation_error_deg) {
  const bool residuals_valid =
      std::isfinite(downward_error_deg) && downward_error_deg >= 0.0 &&
      std::isfinite(planar_position_error_m) &&
      planar_position_error_m >= 0.0 &&
      std::isfinite(height_error_m) && height_error_m >= 0.0 &&
      std::isfinite(orientation_error_deg) &&
      orientation_error_deg >= 0.0;
  const bool limits_valid =
      std::isfinite(maximum_downward_error_deg) &&
      maximum_downward_error_deg >= 0.0 &&
      std::isfinite(maximum_planar_position_error_m) &&
      maximum_planar_position_error_m >= 0.0 &&
      std::isfinite(maximum_height_error_m) &&
      maximum_height_error_m >= 0.0 &&
      std::isfinite(maximum_orientation_error_deg) &&
      maximum_orientation_error_deg >= 0.0;
  if (!measured_pose_valid || !residuals_valid || !limits_valid) {
    return {};
  }
  if (!workspace_valid) {
    return {
        AlignmentEndpointStatus::kWorkspaceRejected,
        "workspace_valid", 0.0, 1.0};
  }
  if (downward_error_deg > maximum_downward_error_deg + 1e-12) {
    return {
        AlignmentEndpointStatus::kDownwardRejected,
        "downward_error_deg", downward_error_deg,
        maximum_downward_error_deg};
  }
  if (planar_position_error_m >
      maximum_planar_position_error_m + 1e-12) {
    return {
        AlignmentEndpointStatus::kPlanarPositionRejected,
        "planar_position_error_m", planar_position_error_m,
        maximum_planar_position_error_m};
  }
  if (height_error_m > maximum_height_error_m + 1e-12) {
    return {
        AlignmentEndpointStatus::kHeightRejected,
        "height_error_m", height_error_m, maximum_height_error_m};
  }
  if (orientation_error_deg > maximum_orientation_error_deg + 1e-12) {
    return {
        AlignmentEndpointStatus::kOrientationRejected,
        "orientation_error_deg", orientation_error_deg,
        maximum_orientation_error_deg};
  }
  return {AlignmentEndpointStatus::kValid, "none", 0.0, 0.0};
}

inline bool finite(double value) {
  return std::isfinite(value);
}

inline std::optional<ExactDownCameraAxes> exact_down_camera_axes(
    const Vec3& current_camera_u_axis) {
  if (!finite(current_camera_u_axis.x) ||
      !finite(current_camera_u_axis.y) ||
      !finite(current_camera_u_axis.z)) {
    return std::nullopt;
  }
  const double projected_norm = std::hypot(
      current_camera_u_axis.x, current_camera_u_axis.y);
  if (!finite(projected_norm) || projected_norm <= 1e-9) {
    return std::nullopt;
  }
  const double u_x = current_camera_u_axis.x / projected_norm;
  const double u_y = current_camera_u_axis.y / projected_norm;
  return ExactDownCameraAxes{
      Vec3{u_x, u_y, 0.0},
      Vec3{u_y, -u_x, 0.0},
      Vec3{0.0, 0.0, -1.0}};
}

inline bool valid_visual_alignment_config(
    const VisualAlignmentConfig& config) {
  return finite(config.u_axis_gain_m_per_px) &&
      config.u_axis_gain_m_per_px >= kMinimumVisualGainMPerPx &&
      config.u_axis_gain_m_per_px <= kMaximumVisualGainMPerPx &&
      finite(config.v_axis_gain_m_per_px) &&
      config.v_axis_gain_m_per_px >= kMinimumVisualGainMPerPx &&
      config.v_axis_gain_m_per_px <= kMaximumVisualGainMPerPx &&
      finite(config.target_center_tolerance_px) &&
      config.target_center_tolerance_px > 0.0 &&
      config.required_centered_observations >= 1 &&
      finite(config.maximum_step_m) && config.maximum_step_m > 0.0 &&
      finite(config.minimum_retry_step_m) &&
      config.minimum_retry_step_m > 0.0 &&
      config.minimum_retry_step_m <= config.maximum_step_m &&
      finite(config.maximum_total_displacement_m) &&
      config.maximum_total_displacement_m >= config.maximum_step_m &&
      config.maximum_iterations >= 1 &&
      config.stationary_target_retries >= 0 &&
      finite(config.minimum_pixel_error_improvement_px) &&
      config.minimum_pixel_error_improvement_px > 0.0;
}

inline bool valid_visual_target_observation(
    const VisualTargetObservation& observation) {
  return observation.stamp_ns != 0 && !observation.frame_id.empty() &&
      observation.image_width > 0 && observation.image_height > 0 &&
      finite(observation.u_px) && finite(observation.v_px) &&
      finite(observation.reference_u_px) &&
      finite(observation.reference_v_px) &&
      observation.u_px >= 0.0 && observation.v_px >= 0.0 &&
      observation.reference_u_px >= 0.0 &&
      observation.reference_v_px >= 0.0 &&
      observation.u_px < static_cast<double>(observation.image_width) &&
      observation.v_px < static_cast<double>(observation.image_height) &&
      observation.reference_u_px <
          static_cast<double>(observation.image_width) &&
      observation.reference_v_px <
          static_cast<double>(observation.image_height);
}

inline double visual_pixel_error(
    const VisualTargetObservation& observation) {
  return std::hypot(
      observation.u_px - observation.reference_u_px,
      observation.v_px - observation.reference_v_px);
}

inline std::optional<ImageAxesXY> projected_image_axes(
    const Vec3& camera_u_axis,
    const Vec3& camera_v_axis,
    const Vec3& camera_forward_axis,
    double maximum_downward_error_deg) {
  const auto finite_vec = [](const Vec3& value) {
    return finite(value.x) && finite(value.y) && finite(value.z);
  };
  if (!finite_vec(camera_u_axis) || !finite_vec(camera_v_axis) ||
      !finite_vec(camera_forward_axis) ||
      !finite(maximum_downward_error_deg) ||
      maximum_downward_error_deg < 0.0 ||
      maximum_downward_error_deg > 90.0) {
    return std::nullopt;
  }

  const double forward_norm = std::sqrt(
      camera_forward_axis.x * camera_forward_axis.x +
      camera_forward_axis.y * camera_forward_axis.y +
      camera_forward_axis.z * camera_forward_axis.z);
  if (!finite(forward_norm) || forward_norm <= 1e-12) {
    return std::nullopt;
  }
  const double down_dot = std::clamp(
      -camera_forward_axis.z / forward_norm, -1.0, 1.0);
  const double downward_error_deg =
      std::acos(down_dot) * 180.0 / 3.14159265358979323846;
  if (!finite(downward_error_deg) ||
      downward_error_deg > maximum_downward_error_deg + 1e-9) {
    return std::nullopt;
  }

  const double u_norm = std::hypot(camera_u_axis.x, camera_u_axis.y);
  const double v_norm = std::hypot(camera_v_axis.x, camera_v_axis.y);
  if (!finite(u_norm) || !finite(v_norm) ||
      u_norm <= 1e-9 || v_norm <= 1e-9) {
    return std::nullopt;
  }
  ImageAxesXY axes;
  axes.u_x = camera_u_axis.x / u_norm;
  axes.u_y = camera_u_axis.y / u_norm;
  axes.v_x = camera_v_axis.x / v_norm;
  axes.v_y = camera_v_axis.y / v_norm;
  axes.downward_error_deg = downward_error_deg;
  const double dot = axes.u_x * axes.v_x + axes.u_y * axes.v_y;
  if (!finite(dot) || std::abs(dot) > kMaximumImageAxisDot) {
    return std::nullopt;
  }
  return axes;
}

inline std::optional<PlanarCorrection> bounded_visual_correction(
    const VisualAlignmentConfig& config,
    const VisualTargetObservation& observation,
    const ImageAxesXY& axes,
    double remaining_displacement_m,
    double correction_cap_m) {
  if (!valid_visual_alignment_config(config) ||
      !finite(remaining_displacement_m) || remaining_displacement_m <= 0.0 ||
      !finite(correction_cap_m) || correction_cap_m <= 0.0 ||
      !finite(axes.u_x) || !finite(axes.u_y) ||
      !finite(axes.v_x) || !finite(axes.v_y)) {
    return std::nullopt;
  }
  const double error_u_px = observation.u_px - observation.reference_u_px;
  const double error_v_px = observation.v_px - observation.reference_v_px;
  if (!finite(error_u_px) || !finite(error_v_px)) {
    return std::nullopt;
  }

  PlanarCorrection correction;
  correction.x_m =
      error_u_px * config.u_axis_gain_m_per_px * axes.u_x +
      error_v_px * config.v_axis_gain_m_per_px * axes.v_x;
  correction.y_m =
      error_u_px * config.u_axis_gain_m_per_px * axes.u_y +
      error_v_px * config.v_axis_gain_m_per_px * axes.v_y;
  const double magnitude = std::hypot(correction.x_m, correction.y_m);
  if (!finite(magnitude)) {
    return std::nullopt;
  }
  if (magnitude <= 1e-12) {
    return correction;
  }
  const double allowed = std::min(
      {magnitude, config.maximum_step_m, remaining_displacement_m,
       correction_cap_m});
  if (!finite(allowed) || allowed <= 0.0) {
    return std::nullopt;
  }
  const double scale = allowed / magnitude;
  correction.x_m *= scale;
  correction.y_m *= scale;
  return correction;
}

inline std::optional<double> next_planning_retry_magnitude(
    double original_magnitude_m,
    int retry_index,
    double minimum_retry_step_m) {
  if (!finite(original_magnitude_m) || original_magnitude_m <= 0.0 ||
      retry_index < 0 || !finite(minimum_retry_step_m) ||
      minimum_retry_step_m <= 0.0) {
    return std::nullopt;
  }
  if (retry_index == 0) {
    return original_magnitude_m;
  }
  const double divisor = std::ldexp(1.0, retry_index);
  const double candidate = original_magnitude_m / divisor;
  if (!finite(candidate) || candidate + 1e-12 < minimum_retry_step_m) {
    const double previous_divisor = std::ldexp(1.0, retry_index - 1);
    const double previous_candidate =
        original_magnitude_m / previous_divisor;
    if (!finite(previous_candidate) ||
        previous_candidate <= minimum_retry_step_m + 1e-12) {
      return std::nullopt;
    }
    return minimum_retry_step_m;
  }
  return std::max(candidate, minimum_retry_step_m);
}

inline bool valid_horizontal_trajectory_samples(
    const std::vector<HorizontalTrajectorySample>& samples,
    const HorizontalTrajectorySample& start,
    const std::array<double, 3>& workspace_min_xyz_m,
    const std::array<double, 3>& workspace_max_xyz_m,
    double maximum_z_error_m,
    double maximum_orientation_error_deg,
    double maximum_downward_error_deg) {
  if (samples.empty() || !finite(maximum_z_error_m) ||
      maximum_z_error_m < 0.0 ||
      !finite(maximum_orientation_error_deg) ||
      maximum_orientation_error_deg < 0.0 ||
      maximum_orientation_error_deg > 180.0 ||
      !finite(maximum_downward_error_deg) ||
      maximum_downward_error_deg < 0.0 ||
      maximum_downward_error_deg > 90.0) {
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!finite(workspace_min_xyz_m[axis]) ||
        !finite(workspace_max_xyz_m[axis]) ||
        workspace_min_xyz_m[axis] >= workspace_max_xyz_m[axis]) {
      return false;
    }
  }
  const double start_norm = std::sqrt(
      start.qx * start.qx + start.qy * start.qy +
      start.qz * start.qz + start.qw * start.qw);
  if (!finite(start.x_m) || !finite(start.y_m) || !finite(start.z_m) ||
      !finite(start_norm) || start_norm <= 1e-12) {
    return false;
  }
  for (const auto& sample : samples) {
    const double sample_norm = std::sqrt(
        sample.qx * sample.qx + sample.qy * sample.qy +
        sample.qz * sample.qz + sample.qw * sample.qw);
    if (!finite(sample.x_m) || !finite(sample.y_m) ||
        !finite(sample.z_m) || !finite(sample.qx) || !finite(sample.qy) ||
        !finite(sample.qz) || !finite(sample.qw) ||
        !finite(sample.downward_error_deg) || sample_norm <= 1e-12 ||
        sample.x_m < workspace_min_xyz_m[0] ||
        sample.x_m > workspace_max_xyz_m[0] ||
        sample.y_m < workspace_min_xyz_m[1] ||
        sample.y_m > workspace_max_xyz_m[1] ||
        sample.z_m < workspace_min_xyz_m[2] ||
        sample.z_m > workspace_max_xyz_m[2] ||
        std::abs(sample.z_m - start.z_m) > maximum_z_error_m + 1e-12 ||
        sample.downward_error_deg > maximum_downward_error_deg + 1e-12) {
      return false;
    }
    const double dot = std::clamp(
        std::abs(
            (sample.qx * start.qx + sample.qy * start.qy +
             sample.qz * start.qz + sample.qw * start.qw) /
            (sample_norm * start_norm)),
        0.0, 1.0);
    const double orientation_error_deg =
        2.0 * std::acos(dot) * 180.0 / 3.14159265358979323846;
    if (!finite(orientation_error_deg) ||
        orientation_error_deg > maximum_orientation_error_deg + 1e-12) {
      return false;
    }
  }
  return true;
}

}  // namespace agx_xarm_pick

#endif  // AGX_XARM_PICK_VISUAL_ALIGNMENT_CONTRACT_HPP
