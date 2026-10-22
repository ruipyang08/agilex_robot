#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "agx_xarm_pick/arm_execution.hpp"
#include "agx_xarm_pick/tube_view_region.hpp"
#include "agx_xarm_pick/visual_alignment_contract.hpp"

namespace agx_xarm_pick {

struct ConfigPoint { double x = 0, y = 0, z = 0; };
struct ConfigQuaternion { double x = 0, y = 0, z = 0, w = 1; };
struct ConfigTransform { ConfigPoint translation; ConfigQuaternion rotation; };

// Pure projection of validated domain facts into the existing mission's units.
// Empty values are not operational defaults: only the resolver validates this type.
struct TubeTaskMotionConfig {
  std::string active_profile_id;
  std::string task_type;
  std::string target_cap_color;
  std::string profile_target_cap_color;

  double table_surface_height_above_floor_m = 0.0;
  double table_height_tolerance_m = 0.0;
  double body_diameter_m = 0.0;
  double body_height_m = 0.0;
  double cap_diameter_m = 0.0;
  double cap_height_m = 0.0;
  double diameter_tolerance_m = 0.0;
  double height_tolerance_m = 0.0;
  int rows = 0;
  int columns = 0;
  double center_pitch_m = 0.0;
  double rack_height_above_surface_m = 0.0;
  double pitch_tolerance_m = 0.0;
  int maximum_missing_slots = 0;
  int minimum_layout_inliers = 0;
  double maximum_lattice_rms_error_m = 0.0;
  std::string color_frame;
  std::array<int, 3> target_hsv_lower{};
  std::array<int, 3> target_hsv_upper{};
  double cap_top_height_band_m = 0.0;
  double minimum_cap_circularity = 0.0;
  double maximum_cap_axis_ratio = 0.0;
  double maximum_target_center_error_px = 0.0;
  int maximum_recentering_iterations = 0;
  double fully_open_command = 0.0;
  double close_command = 0.0;
  double insertion_radial_clearance_m = 0.0;
  double finger_thickness_m = 0.0;
  double finger_width_m = 0.0;
  double tube_clearance_margin_m = 0.0;
  std::vector<std::array<double, 2>> command_gap_samples;
  double preparation_clearance_above_cap_m = 0.0;
  double pregrasp_clearance_m = 0.0;
  double grasp_tcp_offset_below_cap_top_m = 0.0;
  double lift_clearance_m = 0.0;
  double profile_downward_axis_tolerance_deg = 0.0;
  double lift_distance_m = 0.0;

  std::string context_source;
  std::string planning_frame;
  std::vector<double> nominal_fixture_center_xyz_m;
  double maximum_translation_error_m = 0.0;
  double maximum_yaw_error_deg = 0.0;
  bool tag_registration_required = false;

  std::string lift_service_name;
  std::string lift_state_topic;
  double arm_base_height_at_zero_lift_above_floor_m = 0.0;
  double lift_minimum_position_m = 0.0;
  double lift_maximum_position_m = 0.0;
  double lift_target_tolerance_m = 0.0;
  std::int64_t lift_target_tolerance_mm = 0;
  std::int64_t lift_maximum_speed_request_mm_s = 0;
  double lift_state_max_age_sec = 0.0;
  double lift_motion_timeout_sec = 0.0;

  double max_interframe_spread_m = 0.0;
  int required_consistent_frames = 0;
  int synchronizer_queue_size = 0;
  double max_sync_skew_sec = 0.0;
  double detection_timeout_sec = 0.0;
  double maximum_frame_age_sec = 0.0;
  std::vector<double> workspace_min_xyz_m;
  std::vector<double> workspace_max_xyz_m;

  ConfigTransform link6_to_grasp_tcp;
  ConfigTransform link6_to_camera;

  std::vector<double> phase_one_joint_positions_rad;
  ConfigPoint phase_one_expected_tcp;
  double phase_one_downward_axis_tolerance_deg = 0.0;
  double phase_two_centerline_y_m = 0.0;
  double phase_two_downward_axis_tolerance_deg = 0.0;

  double search_centerline_y_m = 0.0;
  double initial_centerline_tolerance_m = 0.0;
  double centerline_joint_position_rad = 0.0;
  double centerline_joint_tolerance_rad = 0.0;
  std::vector<double> initial_tcp_x_bounds_m;
  std::vector<double> initial_tcp_z_bounds_m;
  agx_xarm_pick::TopDownViewRegion view_region;
  double view_entry_y_m = 0.0;
  double view_planning_time_per_candidate_sec = 0.0;
  agx_xarm_pick::VisualAlignmentConfig visual_alignment;
  double maximum_recentering_step_m = 0.0;
  double minimum_pixel_error_improvement_px = 0.0;
  std::vector<double> observation_y_positions_m;
  double downward_axis_tolerance_deg = 0.0;
  double minimum_adjacent_view_overlap = 0.0;
  double search_settle_duration_sec = 0.0;
  int detector_attempts_per_waypoint = 0;

  double gripper_force = 0.0;
  std::array<double, 3> gripper_speeds{};
  double gripper_command_tolerance = 0.0;
  double gripper_state_max_age_sec = 0.0;
  double robot_state_max_age_sec = 0.0;
  double planning_timeout_sec = 0.0;
  double gripper_timeout_sec = 0.0;
  double hold_duration_sec = 0.0;
  double cartesian_step_m = 0.0;
  agx_xarm_pick::ArmExecutionProfile arm_execution{};
};

struct ConfigSourceIdentity {
  std::string kind;
  std::string id;
  std::string sha256;
  std::string absolute_path;  // Provenance only; never included in config_id.
};
struct FixtureInstallation {
  std::string id, station_id, parent_frame, fixture_frame;
  ConfigTransform measured_transform;
  bool registration_required = false;
  std::optional<std::string> tag_id;
};
struct RackSlot { std::string id; int row = 0, column = 0; std::array<double, 2> center_xy_m{}; };
struct RackProfile {
  std::string id;
  std::array<double, 3> dimensions_xyz_m{};
  std::vector<RackSlot> slots;
  std::vector<std::string> supported_container_profiles;
};
struct SupportSurface {
  std::string id, station_id, frame_id;
  std::array<double, 3> dimensions_xyz_m{};
  double surface_height_above_floor_m = 0;
  double height_tolerance_m = 0;
};
struct ContainerProfile {
  std::string id;
  double body_diameter_m = 0, body_height_m = 0;
  double cap_diameter_m = 0, cap_height_m = 0;
  double diameter_tolerance_m = 0, height_tolerance_m = 0;
};
struct ColorClassifier { std::array<int, 3> hsv_lower{}, hsv_upper{}; };
struct TubeBehavior {
  std::string id, task_type;
  std::map<std::string, ColorClassifier> color_classifiers;
  std::array<double, 4> roi_normalized{};
  int minimum_layout_saturation = 0;
  double maximum_target_temporal_spread_px = 0;
};
struct TubeRobotProfile {
  std::string id, planning_frame, color_frame;
  std::string detector_service, joint_state_topic, gripper_command_topic, gripper_state_topic;
};
struct ResolvedTubeTaskConfig {
  std::string config_id, task_config_id, fixture_instance_id, fixture_profile_id;
  FixtureInstallation installation;
  RackProfile rack;
  SupportSurface support;
  ContainerProfile container;
  TubeBehavior behavior;
  TubeRobotProfile robot;
  TubeTaskMotionConfig motion;
  std::vector<ConfigSourceIdentity> sources;
};

// Throws std::invalid_argument with source/key context. Loads each canonical
// file once, validates the complete graph, and returns a frozen value snapshot.
std::shared_ptr<const ResolvedTubeTaskConfig> ResolveTubeTaskConfig(
    const std::string& task_file);

// Bind invocation intent to a validated snapshot; never mutate the shared graph.
// Rejects adapters/registration modes not implemented by the current mission.
TubeTaskMotionConfig BindTubeTaskMotion(const ResolvedTubeTaskConfig&,
                                       const std::string& target_cap_color);

}  // namespace agx_xarm_pick
