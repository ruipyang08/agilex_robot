#pragma once
#include "agx_xarm_pick/resolved_tube_config.hpp"
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

namespace tube_config_test {
// Test-only integration comparison, not runtime normalization. Keep these
// grouped fields complete when TubeTaskMotionConfig or its nested types grow.
// The two loaders use different angle units; allow only floating-point roundoff.
inline bool Equal(double a, double b) {
  return std::isfinite(a) && std::isfinite(b) &&
      std::abs(a-b) <= 1e-12 * (1.0 + std::max(std::abs(a), std::abs(b)));
}
template<class T> bool Equal(const T& a, const T& b) { return a==b; }
template<class T, std::size_t... I>
bool TupleEqual(const T& a, const T& b, std::index_sequence<I...>) {
  return (Equal(std::get<I>(a), std::get<I>(b)) && ...);
}
template<class T> bool TupleEqual(const T& a, const T& b) {
  return TupleEqual(a,b,std::make_index_sequence<std::tuple_size<T>::value>{});
}
inline auto PointFields(const agx_xarm_pick::ConfigPoint& p) {
  return std::tie(p.x,p.y,p.z);
}
inline auto TransformFields(const agx_xarm_pick::ConfigTransform& t) {
  return std::tuple_cat(PointFields(t.translation),
      std::tie(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w));
}

inline std::vector<std::string> MotionDifferences(
    const agx_xarm_pick::TubeTaskMotionConfig& a,
    const agx_xarm_pick::TubeTaskMotionConfig& b) {
  std::vector<std::string> differences;
  const auto check=[&](const char* group, auto fields) {
    if (!TupleEqual(fields(a),fields(b))) differences.emplace_back(group);
  };
  check("identity/context",[](const auto& c) {
    return std::tie(c.active_profile_id,c.task_type,c.target_cap_color,
        c.profile_target_cap_color,c.context_source,c.planning_frame,
        c.nominal_fixture_center_xyz_m,c.maximum_translation_error_m,
        c.maximum_yaw_error_deg,c.tag_registration_required);
  });
  check("table/tube/layout",[](const auto& c) {
    return std::tie(c.table_surface_height_above_floor_m,c.table_height_tolerance_m,
        c.body_diameter_m,c.body_height_m,c.cap_diameter_m,c.cap_height_m,
        c.diameter_tolerance_m,c.height_tolerance_m,c.rows,c.columns,
        c.center_pitch_m,c.rack_height_above_surface_m,c.pitch_tolerance_m,
        c.maximum_missing_slots,c.minimum_layout_inliers,c.maximum_lattice_rms_error_m);
  });
  check("perception",[](const auto& c) {
    return std::tie(c.color_frame,c.target_hsv_lower,c.target_hsv_upper,
        c.cap_top_height_band_m,c.minimum_cap_circularity,c.maximum_cap_axis_ratio,
        c.maximum_target_center_error_px,c.maximum_recentering_iterations,
        c.max_interframe_spread_m,c.required_consistent_frames,c.synchronizer_queue_size,
        c.max_sync_skew_sec,c.detection_timeout_sec,c.maximum_frame_age_sec,
        c.workspace_min_xyz_m,c.workspace_max_xyz_m);
  });
  check("gripper",[](const auto& c) {
    return std::tie(c.fully_open_command,c.close_command,c.insertion_radial_clearance_m,
        c.finger_thickness_m,c.finger_width_m,c.tube_clearance_margin_m,
        c.command_gap_samples,c.gripper_force,c.gripper_speeds,
        c.gripper_command_tolerance,c.gripper_state_max_age_sec);
  });
  check("motion clearances/timeouts",[](const auto& c) {
    return std::tie(c.preparation_clearance_above_cap_m,c.pregrasp_clearance_m,
        c.grasp_tcp_offset_below_cap_top_m,c.lift_clearance_m,
        c.profile_downward_axis_tolerance_deg,c.lift_distance_m,
        c.robot_state_max_age_sec,c.planning_timeout_sec,c.gripper_timeout_sec,
        c.hold_duration_sec,c.cartesian_step_m);
  });
  check("lift",[](const auto& c) {
    return std::tie(c.lift_service_name,c.lift_state_topic,
        c.arm_base_height_at_zero_lift_above_floor_m,c.lift_minimum_position_m,
        c.lift_maximum_position_m,c.lift_target_tolerance_m,c.lift_target_tolerance_mm,
        c.lift_maximum_speed_request_mm_s,c.lift_state_max_age_sec,c.lift_motion_timeout_sec);
  });
  check("transforms",[](const auto& c) {
    return std::tuple_cat(TransformFields(c.link6_to_grasp_tcp),TransformFields(c.link6_to_camera));
  });
  check("preparation",[](const auto& c) {
    return std::tuple_cat(std::tie(c.phase_one_joint_positions_rad,
        c.phase_one_downward_axis_tolerance_deg,c.phase_two_centerline_y_m,
        c.phase_two_downward_axis_tolerance_deg),PointFields(c.phase_one_expected_tcp));
  });
  check("search",[](const auto& c) {
    return std::tie(c.search_centerline_y_m,c.initial_centerline_tolerance_m,
        c.centerline_joint_position_rad,c.centerline_joint_tolerance_rad,
        c.initial_tcp_x_bounds_m,c.initial_tcp_z_bounds_m,c.view_entry_y_m,
        c.view_planning_time_per_candidate_sec,c.maximum_recentering_step_m,
        c.minimum_pixel_error_improvement_px,c.observation_y_positions_m,
        c.downward_axis_tolerance_deg,c.minimum_adjacent_view_overlap,
        c.search_settle_duration_sec,c.detector_attempts_per_waypoint);
  });
  check("view region",[](const auto& c) {
    const auto& v=c.view_region;
    return std::tie(v.x_min_m,v.x_max_m,v.preferred_x_m,v.clearance_min_m,
        v.clearance_max_m,v.preferred_clearance_m,v.sampling_step_m,
        v.y_tolerance_m,v.maximum_candidate_attempts);
  });
  check("visual alignment",[](const auto& c) {
    const auto& v=c.visual_alignment;
    return std::tie(v.u_axis_gain_m_per_px,v.v_axis_gain_m_per_px,
        v.target_center_tolerance_px,v.required_centered_observations,
        v.maximum_step_m,v.minimum_retry_step_m,v.maximum_total_displacement_m,
        v.maximum_iterations,v.stationary_target_retries,v.minimum_pixel_error_improvement_px);
  });
  for (std::size_t i=0;i<a.arm_execution.size();++i) {
    if (!Equal(a.arm_execution[i].velocity,b.arm_execution[i].velocity) ||
        !Equal(a.arm_execution[i].acceleration,b.arm_execution[i].acceleration))
      differences.push_back(std::string("arm execution/")+agx_xarm_pick::kArmMotionNames[i]);
  }
  return differences;
}
}  // namespace tube_config_test
