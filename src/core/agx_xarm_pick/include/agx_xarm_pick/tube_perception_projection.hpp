#pragma once
#include "agx_xarm_pick/resolved_tube_config.hpp"
#include <algorithm>

namespace agx_xarm_pick {
// Version 1 wire subset; excludes paths and arm/gripper control settings.
struct TubePerceptionProjection {
  std::uint32_t schema_version{};
  std::string planning_frame{};
  std::string color_frame{};
  std::string lift_state_topic{};
  double table_surface_height_above_floor_m{};
  double table_height_tolerance_m{};
  double arm_base_height_at_zero_lift_above_floor_m{};
  double lift_minimum_position_m{};
  double lift_maximum_position_m{};
  double lift_target_tolerance_m{};
  double lift_state_max_age_sec{};
  double body_diameter_m{};
  double body_height_m{};
  double cap_diameter_m{};
  double cap_height_m{};
  double diameter_tolerance_m{};
  double height_tolerance_m{};
  double rack_height_above_surface_m{};
  std::int32_t rows{};
  std::int32_t columns{};
  std::int32_t maximum_missing_slots{};
  std::int32_t minimum_layout_inliers{};
  double center_pitch_m{};
  double pitch_tolerance_m{};
  double maximum_lattice_rms_error_m{};
  std::array<double, 4> roi_normalized{};
  std::int32_t minimum_layout_saturation{};
  std::array<std::int32_t, 3> target_hsv_lower{};
  std::array<std::int32_t, 3> target_hsv_upper{};
  double cap_top_height_band_m{};
  double minimum_cap_circularity{};
  double maximum_cap_axis_ratio{};
  double maximum_target_temporal_spread_px{};
  double maximum_target_center_error_px{};
  std::int32_t maximum_recentering_iterations{};
  std::int32_t required_consistent_frames{};
  std::int32_t synchronizer_queue_size{};
  double max_sync_skew_sec{};
  double detection_timeout_sec{};
  double maximum_frame_age_sec{};
  double max_interframe_spread_m{};
  std::array<double, 3> workspace_min_xyz_m{};
  std::array<double, 3> workspace_max_xyz_m{};
};
TubePerceptionProjection MakeTubePerceptionProjection(
    const ResolvedTubeTaskConfig& config, const std::string& target_cap_color);
std::string TubePerceptionProjectionBytes(const TubePerceptionProjection& projection,
    const std::string& fixture_instance_id, const std::string& fixture_profile_id,
    const std::string& target_cap_color);
std::string TubePerceptionProjectionSha256(const TubePerceptionProjection& projection,
    const std::string& fixture_instance_id, const std::string& fixture_profile_id,
    const std::string& target_cap_color);

// Compile-time ROS message adapter keeps the pure library free of ROS headers.
template <typename Message>
void CopyTubePerceptionProjection(const TubePerceptionProjection& p, Message* message) {
  message->schema_version = p.schema_version;
  message->planning_frame = p.planning_frame;
  message->color_frame = p.color_frame;
  message->lift_state_topic = p.lift_state_topic;
  message->table_surface_height_above_floor_m = p.table_surface_height_above_floor_m;
  message->table_height_tolerance_m = p.table_height_tolerance_m;
  message->arm_base_height_at_zero_lift_above_floor_m = p.arm_base_height_at_zero_lift_above_floor_m;
  message->lift_minimum_position_m = p.lift_minimum_position_m;
  message->lift_maximum_position_m = p.lift_maximum_position_m;
  message->lift_target_tolerance_m = p.lift_target_tolerance_m;
  message->lift_state_max_age_sec = p.lift_state_max_age_sec;
  message->body_diameter_m = p.body_diameter_m;
  message->body_height_m = p.body_height_m;
  message->cap_diameter_m = p.cap_diameter_m;
  message->cap_height_m = p.cap_height_m;
  message->diameter_tolerance_m = p.diameter_tolerance_m;
  message->height_tolerance_m = p.height_tolerance_m;
  message->rack_height_above_surface_m = p.rack_height_above_surface_m;
  message->rows = p.rows;
  message->columns = p.columns;
  message->maximum_missing_slots = p.maximum_missing_slots;
  message->minimum_layout_inliers = p.minimum_layout_inliers;
  message->center_pitch_m = p.center_pitch_m;
  message->pitch_tolerance_m = p.pitch_tolerance_m;
  message->maximum_lattice_rms_error_m = p.maximum_lattice_rms_error_m;
  std::copy(p.roi_normalized.begin(), p.roi_normalized.end(), message->roi_normalized.begin());
  message->minimum_layout_saturation = p.minimum_layout_saturation;
  std::copy(p.target_hsv_lower.begin(), p.target_hsv_lower.end(), message->target_hsv_lower.begin());
  std::copy(p.target_hsv_upper.begin(), p.target_hsv_upper.end(), message->target_hsv_upper.begin());
  message->cap_top_height_band_m = p.cap_top_height_band_m;
  message->minimum_cap_circularity = p.minimum_cap_circularity;
  message->maximum_cap_axis_ratio = p.maximum_cap_axis_ratio;
  message->maximum_target_temporal_spread_px = p.maximum_target_temporal_spread_px;
  message->maximum_target_center_error_px = p.maximum_target_center_error_px;
  message->maximum_recentering_iterations = p.maximum_recentering_iterations;
  message->required_consistent_frames = p.required_consistent_frames;
  message->synchronizer_queue_size = p.synchronizer_queue_size;
  message->max_sync_skew_sec = p.max_sync_skew_sec;
  message->detection_timeout_sec = p.detection_timeout_sec;
  message->maximum_frame_age_sec = p.maximum_frame_age_sec;
  message->max_interframe_spread_m = p.max_interframe_spread_m;
  std::copy(p.workspace_min_xyz_m.begin(), p.workspace_min_xyz_m.end(), message->workspace_min_xyz_m.begin());
  std::copy(p.workspace_max_xyz_m.begin(), p.workspace_max_xyz_m.end(), message->workspace_max_xyz_m.begin());
}

template <typename Request>
void FillConfiguredTubePerceptionRequest(const ResolvedTubeTaskConfig& config,
    const std::string& target_cap_color, Request* request) {
  const auto projection = MakeTubePerceptionProjection(config, target_cap_color);
  request->config_id = config.config_id;
  request->fixture_instance_id = config.fixture_instance_id;
  request->fixture_profile_id = config.fixture_profile_id;
  request->target_cap_color = target_cap_color;
  request->projection_sha256 = TubePerceptionProjectionSha256(projection,
      config.fixture_instance_id, config.fixture_profile_id, target_cap_color);
  CopyTubePerceptionProjection(projection, &request->perception);
}
}  // namespace agx_xarm_pick
