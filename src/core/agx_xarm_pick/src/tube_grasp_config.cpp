#include "agx_xarm_pick/tube_grasp_config.hpp"

#include "agx_xarm_pick/tube_grasp_geometry.hpp"
#include "agx_xarm_pick/tube_search_contract.hpp"
#include "agx_xarm_pick/visual_alignment_contract.hpp"
#include "agx_xarm_pick/tube_lift_contract.hpp"
#include "agx_xarm_pick/tube_preparation_contract.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <vector>
#include <sstream>
#include <iomanip>
#include <openssl/sha.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace agxcore {
namespace {

constexpr double kPi = 3.14159265358979323846;


// A read-only view of the three parameter subtrees captured at admission.
// Every helper below reads this value, never the mutable parameter server.
class SnapshotNode {
 public:
  SnapshotNode(std::shared_ptr<const XmlRpc::XmlRpcValue> root, std::string prefix = {})
      : root_(std::move(root)), prefix_(std::move(prefix)) {}
  SnapshotNode Root() const { return SnapshotNode(root_); }
  bool getParam(const std::string& key, XmlRpc::XmlRpcValue& output) const {
    const std::string path = !key.empty() && key.front() == '/' ? key : prefix_ + "/" + key;
    const auto* value = root_.get();
    std::istringstream stream(path);
    std::string part;
    while (std::getline(stream, part, '/')) {
      if (part.empty()) continue;
      if (value->getType() != XmlRpc::XmlRpcValue::TypeStruct || !value->hasMember(part)) return false;
      value = &(*value)[part];
    }
    output = *value;
    return true;
  }
  bool getParam(const std::string& key, std::string& output) const {
    XmlRpc::XmlRpcValue value;
    if (!getParam(key, value) || value.getType() != XmlRpc::XmlRpcValue::TypeString) return false;
    output = static_cast<std::string>(value);
    return true;
  }
  bool hasParam(const std::string& key) const { XmlRpc::XmlRpcValue value; return getParam(key, value); }
 private:
  std::shared_ptr<const XmlRpc::XmlRpcValue> root_;
  std::string prefix_;
};

void Fingerprint(std::ostream& out, const XmlRpc::XmlRpcValue& value) {
  using V = XmlRpc::XmlRpcValue;
  out << int(value.getType()) << ':';
  switch (value.getType()) {
    case V::TypeBoolean: out << bool(value); break;
    case V::TypeInt: out << int(value); break;
    case V::TypeDouble: out << std::hexfloat << double(value) << std::defaultfloat; break;
    case V::TypeString: {const auto text=std::string(value);out << text.size() << ':' << text;break;}
    case V::TypeArray:
      out << value.size() << ':';
      for (int i=0;i<value.size();++i) Fingerprint(out,value[i]);
      break;
    case V::TypeStruct:
      out << value.size() << ':';
      for (auto it=value.begin();it!=value.end();++it) {
        out << it->first.size() << ':' << it->first;
        Fingerprint(out,it->second);
      }
      break;
    default: throw std::invalid_argument("unsupported legacy configuration value");
  }
  out << ';';
}

std::string Digest(const XmlRpc::XmlRpcValue& value) {
  std::ostringstream canonical; Fingerprint(canonical,value);
  const auto bytes=canonical.str(); unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(bytes.data()),bytes.size(),digest);
  std::ostringstream out;
  for (auto byte:digest) out << std::hex << std::setfill('0') << std::setw(2) << int(byte);
  return out.str();
}

bool GetString(
    const SnapshotNode& node,
    const std::string& key,
    std::string* value) {
  return node.getParam(key, *value) && !value->empty();
}

bool GetNumber(
    const SnapshotNode& node,
    const std::string& key,
    double* value) {
  XmlRpc::XmlRpcValue raw;
  if (!node.getParam(key, raw)) {
    return false;
  }
  if (raw.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    *value = static_cast<int>(raw);
  } else if (raw.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    *value = static_cast<double>(raw);
  } else {
    return false;
  }
  return std::isfinite(*value);
}

bool GetInteger(
    const SnapshotNode& node,
    const std::string& key,
    int* value) {
  XmlRpc::XmlRpcValue raw;
  if (!node.getParam(key, raw) ||
      raw.getType() != XmlRpc::XmlRpcValue::TypeInt) {
    return false;
  }
  *value = static_cast<int>(raw);
  return true;
}

bool GetBoolean(
    const SnapshotNode& node,
    const std::string& key,
    bool* value) {
  XmlRpc::XmlRpcValue raw;
  if (!node.getParam(key, raw) ||
      raw.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    return false;
  }
  *value = static_cast<bool>(raw);
  return true;
}

bool GetVector(
    const SnapshotNode& node,
    const std::string& key,
    std::size_t expected_size,
    std::vector<double>* values) {
  XmlRpc::XmlRpcValue raw;
  if (!node.getParam(key, raw) ||
      raw.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      raw.size() != static_cast<int>(expected_size)) {
    return false;
  }
  values->clear();
  values->reserve(expected_size);
  for (int index = 0; index < raw.size(); ++index) {
    double value = 0.0;
    if (raw[index].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      value = static_cast<int>(raw[index]);
    } else if (raw[index].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      value = static_cast<double>(raw[index]);
    } else {
      return false;
    }
    if (!std::isfinite(value)) {
      return false;
    }
    values->push_back(value);
  }
  return true;
}

bool GetHsv(
    const SnapshotNode& node,
    const std::string& key,
    std::array<int, 3>* values) {
  XmlRpc::XmlRpcValue raw;
  if (!node.getParam(key, raw) ||
      raw.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      raw.size() != static_cast<int>(values->size())) {
    return false;
  }
  for (int index = 0; index < raw.size(); ++index) {
    if (raw[index].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      return false;
    }
    (*values)[static_cast<std::size_t>(index)] =
        static_cast<int>(raw[index]);
  }
  return true;
}

bool GetNumberVector(
    const SnapshotNode& node,
    const std::string& key,
    std::vector<double>* values) {
  XmlRpc::XmlRpcValue raw;
  if (!node.getParam(key, raw) ||
      raw.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      raw.size() == 0) {
    return false;
  }
  values->clear();
  values->reserve(raw.size());
  for (int index = 0; index < raw.size(); ++index) {
    double value = 0.0;
    if (raw[index].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      value = static_cast<int>(raw[index]);
    } else if (raw[index].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      value = static_cast<double>(raw[index]);
    } else {
      return false;
    }
    if (!std::isfinite(value)) {
      return false;
    }
    values->push_back(value);
  }
  return true;
}

bool GetPairs(
    const SnapshotNode& node,
    const std::string& key,
    std::vector<std::array<double, 2>>* pairs) {
  XmlRpc::XmlRpcValue raw;
  if (!node.getParam(key, raw) ||
      raw.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      raw.size() < 2) {
    return false;
  }
  pairs->clear();
  pairs->reserve(raw.size());
  for (int index = 0; index < raw.size(); ++index) {
    const XmlRpc::XmlRpcValue& row = raw[index];
    if (row.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        row.size() != 2) {
      return false;
    }
    std::array<double, 2> pair{};
    for (int column = 0; column < 2; ++column) {
      if (row[column].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        pair[static_cast<std::size_t>(column)] =
            static_cast<int>(row[column]);
      } else if (
          row[column].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        pair[static_cast<std::size_t>(column)] =
            static_cast<double>(row[column]);
      } else {
        return false;
      }
      if (!std::isfinite(pair[static_cast<std::size_t>(column)])) {
        return false;
      }
    }
    pairs->push_back(pair);
  }
  return true;
}

bool HasExactKeys(
    const XmlRpc::XmlRpcValue& mapping,
    std::initializer_list<const char*> keys) {
  if (mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      mapping.size() != static_cast<int>(keys.size())) {
    return false;
  }
  return std::all_of(
      keys.begin(), keys.end(),
      [&mapping](const char* key) {
        return mapping.hasMember(key);
      });
}

bool MappingHasExactKeys(
    const SnapshotNode& node,
    const std::string& key,
    std::initializer_list<const char*> keys) {
  XmlRpc::XmlRpcValue mapping;
  return node.getParam(key, mapping) && HasExactKeys(mapping, keys);
}

bool MotionHasExactKeys(const SnapshotNode& config) {
  XmlRpc::XmlRpcValue mapping;
  if (!config.getParam("motion", mapping) ||
      mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct) return false;
  std::vector<std::string> keys{
      "gripper_force", "gripper_command_tolerance",
      "gripper_state_max_age_sec", "robot_state_max_age_sec", "planning_timeout_sec",
      "gripper_timeout_sec", "hold_duration_sec", "cartesian_step_m", "preparation", "search"};
  keys.push_back(mapping.hasMember("gripper_speeds") ? "gripper_speeds" : "gripper_speed");
  if (mapping.hasMember("arm_speed")) keys.push_back("arm_speed");
  else {
    keys.push_back("velocity_scaling");
    keys.push_back("acceleration_scaling");
  }
  return mapping.size() == static_cast<int>(keys.size()) &&
      std::all_of(keys.begin(), keys.end(),
                  [&mapping](const std::string& key) { return mapping.hasMember(key); });
}

bool LoadGripperSpeeds(const SnapshotNode& config,
                       std::array<double, 3>* speeds) {
  constexpr std::array<const char*, 3> names{{"preclose", "final_close", "recovery"}};
  if (!config.hasParam("motion/gripper_speeds")) {
    double legacy;
    if (!GetNumber(config, "motion/gripper_speed", &legacy)) return false;
    speeds->fill(legacy);
    ROS_WARN("tube gripper speed: legacy uniform speed");
  } else {
    if (config.hasParam("motion/gripper_speed") ||
        !MappingHasExactKeys(config, "motion/gripper_speeds",
                             {"preclose", "final_close", "recovery"})) return false;
    for (std::size_t i=0; i<speeds->size(); ++i)
      if (!GetNumber(config, std::string("motion/gripper_speeds/") + names[i],
                     &(*speeds)[i])) return false;
  }
  for (std::size_t i=0; i<speeds->size(); ++i) {
    if (!std::isfinite((*speeds)[i]) || (*speeds)[i] < 1.0 || (*speeds)[i] > 100.0)
      return false;
    ROS_INFO("tube resolved gripper speed: %s speed=%.1f", names[i], (*speeds)[i]);
  }
  return true;
}

bool LoadArmExecution(const SnapshotNode& config,
                      agx_xarm_pick::ArmExecutionProfile* profile) {
  using namespace agx_xarm_pick;
  ArmExecutionScales global{};
  try {
    if (!config.hasParam("motion/arm_speed")) {
      if (!GetNumber(config, "motion/velocity_scaling", &global.velocity) ||
          !GetNumber(config, "motion/acceleration_scaling", &global.acceleration)) return false;
      profile->fill(resolve_arm_execution(global, 1.0, std::nullopt));
      ROS_WARN("tube arm execution: legacy uniform scaling");
    } else {
      if (config.hasParam("motion/velocity_scaling") ||
          config.hasParam("motion/acceleration_scaling")) return false;
      const auto root = config.Root();
      if (!MappingHasExactKeys(root, "/arm_execution", {"velocity_scale", "acceleration_scale"}) ||
          !GetNumber(root, "/arm_execution/velocity_scale", &global.velocity) ||
          !GetNumber(root, "/arm_execution/acceleration_scale", &global.acceleration)) return false;
      XmlRpc::XmlRpcValue overrides;
      double task_default;
      if (!config.getParam("motion/arm_speed", overrides) ||
          overrides.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          !GetNumber(config, "motion/arm_speed/default_coefficient", &task_default)) return false;
      for (auto it=overrides.begin(); it!=overrides.end(); ++it) {
        if (it->first != "default_coefficient" &&
            std::find(kArmMotionNames.begin(), kArmMotionNames.end(), it->first) == kArmMotionNames.end()) return false;
      }
      for (std::size_t i=0; i<profile->size(); ++i) {
        const std::string key = std::string("motion/arm_speed/") + kArmMotionNames[i];
        std::optional<double> coefficient;
        if (config.hasParam(key)) {
          double value;
          if (!GetNumber(config, key, &value)) return false;
          coefficient = value;
        }
        (*profile)[i] = resolve_arm_execution(global, task_default, coefficient);
      }
    }
  } catch (const std::invalid_argument& error) {
    ROS_ERROR("invalid arm execution scaling: %s", error.what());
    return false;
  }
  for (std::size_t i=0; i<profile->size(); ++i)
    ROS_INFO("tube resolved arm speed: %s velocity=%.3f acceleration=%.3f",
             kArmMotionNames[i], (*profile)[i].velocity, (*profile)[i].acceleration);
  return true;
}

bool Positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool UnitScaling(double value) {
  return Positive(value) && value <= 1.0;
}

bool FinitePoint(const agx_xarm_pick::ConfigPoint& point) {
  return std::isfinite(point.x) &&
      std::isfinite(point.y) &&
      std::isfinite(point.z);
}


}  // namespace

bool LoadLegacyMotionConfig(std::shared_ptr<const XmlRpc::XmlRpcValue> snapshot,
                            agx_xarm_pick::TubeTaskMotionConfig* config_) {
  SnapshotNode root(snapshot);
  SnapshotNode config(snapshot, "/tube_grasp");
  if (!GetString(
          config, "active_profile_id", &config_->active_profile_id)) {
    ROS_ERROR("missing /tube_grasp/active_profile_id");
    return false;
  }

  SnapshotNode profiles(snapshot, "/tube_fixture_profiles");
  SnapshotNode profile(snapshot, "/tube_fixture_profiles/" + config_->active_profile_id);
  std::vector<double> grasp_tcp_xyz;
  std::vector<double> grasp_tcp_quaternion;
  std::vector<double> camera_xyz;
  std::vector<double> camera_quaternion;
  std::vector<double> phase_one_expected_tcp_xyz_m;
  std::vector<double> view_x_bounds_m;
  std::vector<double> view_clearance_bounds_m;
  int maximum_candidate_attempts = 0;
  int lift_maximum_speed_request_mm_s = 0;
  bool loaded =
      MappingHasExactKeys(
          profiles, config_->active_profile_id,
          {
              "task_type",
              "target_cap_color",
              "table",
              "tube",
              "layout",
              "perception",
              "gripper",
              "motion",
              "simulation",
          }) &&
      MappingHasExactKeys(
          profile, "table",
          {"surface_height_above_floor_m", "height_tolerance_m"}) &&
      MappingHasExactKeys(
          profile, "tube",
          {
              "body_diameter_m",
              "body_height_m",
              "cap_diameter_m",
              "cap_height_m",
              "diameter_tolerance_m",
              "height_tolerance_m",
          }) &&
      MappingHasExactKeys(
          profile, "layout",
          {
              "rows",
              "columns",
              "center_pitch_m",
              "rack_height_above_surface_m",
              "pitch_tolerance_m",
              "maximum_missing_slots",
              "minimum_layout_inliers",
              "maximum_lattice_rms_error_m",
          }) &&
      MappingHasExactKeys(
          profile, "perception",
          {
              "color_frame",
              "roi_normalized",
              "minimum_layout_saturation",
              "target_hsv_lower",
              "target_hsv_upper",
              "cap_top_height_band_m",
              "minimum_cap_circularity",
              "maximum_cap_axis_ratio",
              "maximum_target_temporal_spread_px",
              "maximum_target_center_error_px",
              "maximum_recentering_iterations",
          }) &&
      MappingHasExactKeys(
          profile, "gripper",
          {
              "fully_open_command",
              "close_command",
              "insertion_radial_clearance_m",
              "finger_thickness_m",
              "finger_width_m",
              "tube_clearance_margin_m",
              "command_gap_samples",
          }) &&
      MappingHasExactKeys(
          profile, "motion",
          {
              "preparation_clearance_above_cap_m",
              "pregrasp_clearance_m",
              "grasp_offset_below_cap_top_m",
              "lift_clearance_m",
              "downward_axis_tolerance_deg",
          }) &&
      MappingHasExactKeys(
          root, "/tube_grasp",
          {
              "active_profile_id",
              "target_cap_color",
              "context",
              "lift",
              "perception",
              "transforms",
              "motion",
          }) &&
      MappingHasExactKeys(
          config, "context",
          {
              "source",
              "planning_frame",
              "nominal_fixture_center_xyz_m",
              "maximum_translation_error_m",
              "maximum_yaw_error_deg",
              "tag_registration_required",
          }) &&
      MappingHasExactKeys(
          config, "lift",
          {
              "service_name",
              "state_topic",
              "arm_base_height_at_zero_lift_above_floor_m",
              "minimum_position_m",
              "maximum_position_m",
              "target_tolerance_m",
              "maximum_speed_request_mm_s",
              "state_max_age_sec",
              "motion_timeout_sec",
          }) &&
      MappingHasExactKeys(
          config, "perception",
          {
              "required_consistent_frames",
              "synchronizer_queue_size",
              "max_sync_skew_sec",
              "detection_timeout_sec",
              "maximum_frame_age_sec",
              "max_interframe_spread_m",
              "workspace_min_xyz_m",
              "workspace_max_xyz_m",
          }) &&
      MappingHasExactKeys(
          config, "transforms",
          {
              "link6_to_grasp_tcp_xyz_m",
              "link6_to_grasp_tcp_quaternion_xyzw",
              "link6_to_camera_xyz_m",
              "link6_to_camera_quaternion_xyzw",
          }) &&
      MotionHasExactKeys(config) &&
      MappingHasExactKeys(
          config, "motion/preparation",
          {
              "phase_one",
              "phase_two",
          }) &&
      MappingHasExactKeys(
          config, "motion/preparation/phase_one",
          {
              "joint_positions_rad",
              "expected_tcp_xyz_m",
              "downward_axis_tolerance_deg",
          }) &&
      MappingHasExactKeys(
          config, "motion/preparation/phase_two",
          {
              "centerline_y_m",
              "downward_axis_tolerance_deg",
          }) &&
      MappingHasExactKeys(
          config, "motion/search",
          {
              "centerline_y_m",
              "initial_centerline_tolerance_m",
              "centerline_joint_position_rad",
              "centerline_joint_tolerance_rad",
              "initial_tcp_x_bounds_m",
              "initial_tcp_z_bounds_m",
              "view_region",
              "visual_alignment",
              "observation_y_positions_m",
              "downward_axis_tolerance_deg",
              "minimum_adjacent_view_overlap",
              "settle_duration_sec",
              "detector_attempts_per_waypoint",
          }) &&
      MappingHasExactKeys(
          config, "motion/search/view_region",
          {
              "x_bounds_m",
              "preferred_x_m",
              "clearance_above_cap_bounds_m",
              "preferred_clearance_above_cap_m",
              "sampling_step_m",
              "entry_y_m",
              "waypoint_y_tolerance_m",
              "maximum_candidate_attempts",
              "planning_time_per_candidate_sec",
          }) &&
      MappingHasExactKeys(
          config, "motion/search/visual_alignment",
          {
              "u_axis_gain_m_per_px",
              "v_axis_gain_m_per_px",
              "target_center_tolerance_px",
              "required_centered_observations",
              "maximum_step_m",
              "minimum_retry_step_m",
              "maximum_total_displacement_m",
              "maximum_iterations",
              "stationary_target_retries",
              "minimum_pixel_error_improvement_px",
          }) &&
      GetString(profile, "task_type", &config_->task_type) &&
      GetString(
          profile, "target_cap_color",
          &config_->profile_target_cap_color) &&
      GetNumber(
          profile, "table/surface_height_above_floor_m",
          &config_->table_surface_height_above_floor_m) &&
      GetNumber(
          profile, "table/height_tolerance_m",
          &config_->table_height_tolerance_m) &&
      GetNumber(
          profile, "tube/body_diameter_m", &config_->body_diameter_m) &&
      GetNumber(
          profile, "tube/body_height_m", &config_->body_height_m) &&
      GetNumber(
          profile, "tube/cap_diameter_m", &config_->cap_diameter_m) &&
      GetNumber(
          profile, "tube/cap_height_m", &config_->cap_height_m) &&
      GetNumber(
          profile, "tube/diameter_tolerance_m",
          &config_->diameter_tolerance_m) &&
      GetNumber(
          profile, "tube/height_tolerance_m",
          &config_->height_tolerance_m) &&
      GetInteger(profile, "layout/rows", &config_->rows) &&
      GetInteger(profile, "layout/columns", &config_->columns) &&
      GetNumber(
          profile, "layout/center_pitch_m", &config_->center_pitch_m) &&
      GetNumber(
          profile, "layout/rack_height_above_surface_m",
          &config_->rack_height_above_surface_m) &&
      GetNumber(
          profile, "layout/pitch_tolerance_m",
          &config_->pitch_tolerance_m) &&
      GetInteger(
          profile, "layout/maximum_missing_slots",
          &config_->maximum_missing_slots) &&
      GetInteger(
          profile, "layout/minimum_layout_inliers",
          &config_->minimum_layout_inliers) &&
      GetNumber(
          profile, "layout/maximum_lattice_rms_error_m",
          &config_->maximum_lattice_rms_error_m) &&
      GetString(
          profile, "perception/color_frame", &config_->color_frame) &&
      GetHsv(
          profile, "perception/target_hsv_lower",
          &config_->target_hsv_lower) &&
      GetHsv(
          profile, "perception/target_hsv_upper",
          &config_->target_hsv_upper) &&
      GetNumber(
          profile, "perception/cap_top_height_band_m",
          &config_->cap_top_height_band_m) &&
      GetNumber(
          profile, "perception/minimum_cap_circularity",
          &config_->minimum_cap_circularity) &&
      GetNumber(
          profile, "perception/maximum_cap_axis_ratio",
          &config_->maximum_cap_axis_ratio) &&
      GetNumber(
          profile, "perception/maximum_target_center_error_px",
          &config_->maximum_target_center_error_px) &&
      GetInteger(
          profile, "perception/maximum_recentering_iterations",
          &config_->maximum_recentering_iterations) &&
      GetNumber(
          profile, "gripper/fully_open_command",
          &config_->fully_open_command) &&
      GetNumber(
          profile, "gripper/close_command", &config_->close_command) &&
      GetNumber(
          profile, "gripper/insertion_radial_clearance_m",
          &config_->insertion_radial_clearance_m) &&
      GetNumber(
          profile, "gripper/finger_thickness_m",
          &config_->finger_thickness_m) &&
      GetNumber(
          profile, "gripper/finger_width_m", &config_->finger_width_m) &&
      GetNumber(
          profile, "gripper/tube_clearance_margin_m",
          &config_->tube_clearance_margin_m) &&
      GetPairs(
          profile, "gripper/command_gap_samples",
          &config_->command_gap_samples) &&
      GetNumber(
          profile, "motion/preparation_clearance_above_cap_m",
          &config_->preparation_clearance_above_cap_m) &&
      GetNumber(
          profile, "motion/pregrasp_clearance_m",
          &config_->pregrasp_clearance_m) &&
      GetNumber(
          profile, "motion/grasp_offset_below_cap_top_m",
          &config_->grasp_tcp_offset_below_cap_top_m) &&
      GetNumber(
          profile, "motion/lift_clearance_m",
          &config_->lift_clearance_m) &&
      GetNumber(
          profile, "motion/downward_axis_tolerance_deg",
          &config_->profile_downward_axis_tolerance_deg) &&
      GetString(config, "target_cap_color", &config_->target_cap_color) &&
      GetString(config, "context/source", &config_->context_source) &&
      GetString(
          config, "context/planning_frame", &config_->planning_frame) &&
      GetVector(
          config, "context/nominal_fixture_center_xyz_m", 3,
          &config_->nominal_fixture_center_xyz_m) &&
      GetNumber(
          config, "context/maximum_translation_error_m",
          &config_->maximum_translation_error_m) &&
      GetNumber(
          config, "context/maximum_yaw_error_deg",
          &config_->maximum_yaw_error_deg) &&
      GetBoolean(
          config, "context/tag_registration_required",
          &config_->tag_registration_required) &&
      GetString(
          config, "lift/service_name",
          &config_->lift_service_name) &&
      GetString(
          config, "lift/state_topic",
          &config_->lift_state_topic) &&
      GetNumber(
          config, "lift/arm_base_height_at_zero_lift_above_floor_m",
          &config_->arm_base_height_at_zero_lift_above_floor_m) &&
      GetNumber(
          config, "lift/minimum_position_m",
          &config_->lift_minimum_position_m) &&
      GetNumber(
          config, "lift/maximum_position_m",
          &config_->lift_maximum_position_m) &&
      GetNumber(
          config, "lift/target_tolerance_m",
          &config_->lift_target_tolerance_m) &&
      GetInteger(
          config, "lift/maximum_speed_request_mm_s",
          &lift_maximum_speed_request_mm_s) &&
      GetNumber(
          config, "lift/state_max_age_sec",
          &config_->lift_state_max_age_sec) &&
      GetNumber(
          config, "lift/motion_timeout_sec",
          &config_->lift_motion_timeout_sec) &&
      GetNumber(
          config, "perception/max_interframe_spread_m",
          &config_->max_interframe_spread_m) &&
      GetInteger(
          config, "perception/required_consistent_frames",
          &config_->required_consistent_frames) &&
      GetInteger(
          config, "perception/synchronizer_queue_size",
          &config_->synchronizer_queue_size) &&
      GetNumber(
          config, "perception/max_sync_skew_sec",
          &config_->max_sync_skew_sec) &&
      GetNumber(
          config, "perception/detection_timeout_sec",
          &config_->detection_timeout_sec) &&
      GetNumber(
          config, "perception/maximum_frame_age_sec",
          &config_->maximum_frame_age_sec) &&
      GetVector(
          config, "perception/workspace_min_xyz_m", 3,
          &config_->workspace_min_xyz_m) &&
      GetVector(
          config, "perception/workspace_max_xyz_m", 3,
          &config_->workspace_max_xyz_m) &&
      GetVector(
          config, "transforms/link6_to_grasp_tcp_xyz_m", 3,
          &grasp_tcp_xyz) &&
      GetVector(
          config, "transforms/link6_to_grasp_tcp_quaternion_xyzw", 4,
          &grasp_tcp_quaternion) &&
      GetVector(
          config, "transforms/link6_to_camera_xyz_m", 3,
          &camera_xyz) &&
      GetVector(
          config, "transforms/link6_to_camera_quaternion_xyzw", 4,
          &camera_quaternion) &&
      GetVector(
          config, "motion/preparation/phase_one/joint_positions_rad", 6,
          &config_->phase_one_joint_positions_rad) &&
      GetVector(
          config, "motion/preparation/phase_one/expected_tcp_xyz_m", 3,
          &phase_one_expected_tcp_xyz_m) &&
      GetNumber(
          config,
          "motion/preparation/phase_one/downward_axis_tolerance_deg",
          &config_->phase_one_downward_axis_tolerance_deg) &&
      GetNumber(
          config, "motion/preparation/phase_two/centerline_y_m",
          &config_->phase_two_centerline_y_m) &&
      GetNumber(
          config,
          "motion/preparation/phase_two/downward_axis_tolerance_deg",
          &config_->phase_two_downward_axis_tolerance_deg) &&
      GetNumber(
          config, "motion/search/centerline_y_m",
          &config_->search_centerline_y_m) &&
      GetNumber(
          config, "motion/search/initial_centerline_tolerance_m",
          &config_->initial_centerline_tolerance_m) &&
      GetNumber(
          config, "motion/search/centerline_joint_position_rad",
          &config_->centerline_joint_position_rad) &&
      GetNumber(
          config, "motion/search/centerline_joint_tolerance_rad",
          &config_->centerline_joint_tolerance_rad) &&
      GetVector(
          config, "motion/search/initial_tcp_x_bounds_m", 2,
          &config_->initial_tcp_x_bounds_m) &&
      GetVector(
          config, "motion/search/initial_tcp_z_bounds_m", 2,
          &config_->initial_tcp_z_bounds_m) &&
      GetVector(
          config, "motion/search/view_region/x_bounds_m", 2,
          &view_x_bounds_m) &&
      GetNumber(
          config, "motion/search/view_region/preferred_x_m",
          &config_->view_region.preferred_x_m) &&
      GetVector(
          config,
          "motion/search/view_region/clearance_above_cap_bounds_m",
          2,
          &view_clearance_bounds_m) &&
      GetNumber(
          config,
          "motion/search/view_region/preferred_clearance_above_cap_m",
          &config_->view_region.preferred_clearance_m) &&
      GetNumber(
          config, "motion/search/view_region/sampling_step_m",
          &config_->view_region.sampling_step_m) &&
      GetNumber(
          config, "motion/search/view_region/entry_y_m",
          &config_->view_entry_y_m) &&
      GetNumber(
          config, "motion/search/view_region/waypoint_y_tolerance_m",
          &config_->view_region.y_tolerance_m) &&
      GetInteger(
          config, "motion/search/view_region/maximum_candidate_attempts",
          &maximum_candidate_attempts) &&
      GetNumber(
          config,
          "motion/search/view_region/planning_time_per_candidate_sec",
          &config_->view_planning_time_per_candidate_sec) &&
      GetNumber(
          config, "motion/search/visual_alignment/u_axis_gain_m_per_px",
          &config_->visual_alignment.u_axis_gain_m_per_px) &&
      GetNumber(
          config, "motion/search/visual_alignment/v_axis_gain_m_per_px",
          &config_->visual_alignment.v_axis_gain_m_per_px) &&
      GetNumber(
          config, "motion/search/visual_alignment/target_center_tolerance_px",
          &config_->visual_alignment.target_center_tolerance_px) &&
      GetInteger(
          config,
          "motion/search/visual_alignment/required_centered_observations",
          &config_->visual_alignment.required_centered_observations) &&
      GetNumber(
          config, "motion/search/visual_alignment/maximum_step_m",
          &config_->visual_alignment.maximum_step_m) &&
      GetNumber(
          config, "motion/search/visual_alignment/minimum_retry_step_m",
          &config_->visual_alignment.minimum_retry_step_m) &&
      GetNumber(
          config,
          "motion/search/visual_alignment/maximum_total_displacement_m",
          &config_->visual_alignment.maximum_total_displacement_m) &&
      GetInteger(
          config, "motion/search/visual_alignment/maximum_iterations",
          &config_->visual_alignment.maximum_iterations) &&
      GetInteger(
          config, "motion/search/visual_alignment/stationary_target_retries",
          &config_->visual_alignment.stationary_target_retries) &&
      GetNumber(
          config,
          "motion/search/visual_alignment/minimum_pixel_error_improvement_px",
          &config_->visual_alignment.minimum_pixel_error_improvement_px) &&
      GetNumberVector(
          config, "motion/search/observation_y_positions_m",
          &config_->observation_y_positions_m) &&
      GetNumber(
          config, "motion/search/downward_axis_tolerance_deg",
          &config_->downward_axis_tolerance_deg) &&
      GetNumber(
          config, "motion/search/minimum_adjacent_view_overlap",
          &config_->minimum_adjacent_view_overlap) &&
      GetNumber(
          config, "motion/search/settle_duration_sec",
          &config_->search_settle_duration_sec) &&
      GetInteger(
          config, "motion/search/detector_attempts_per_waypoint",
          &config_->detector_attempts_per_waypoint) &&
      GetNumber(
          config, "motion/gripper_force", &config_->gripper_force) &&
      LoadGripperSpeeds(config, &config_->gripper_speeds) &&
      GetNumber(
          config, "motion/gripper_command_tolerance",
          &config_->gripper_command_tolerance) &&
      GetNumber(
          config, "motion/gripper_state_max_age_sec",
          &config_->gripper_state_max_age_sec) &&
      GetNumber(
          config, "motion/robot_state_max_age_sec",
          &config_->robot_state_max_age_sec) &&
      GetNumber(
          config, "motion/planning_timeout_sec",
          &config_->planning_timeout_sec) &&
      GetNumber(
          config, "motion/gripper_timeout_sec",
          &config_->gripper_timeout_sec) &&
      GetNumber(
          config, "motion/hold_duration_sec",
          &config_->hold_duration_sec) &&
      GetNumber(
          config, "motion/cartesian_step_m",
          &config_->cartesian_step_m) &&
      LoadArmExecution(config, &config_->arm_execution);
  if (!loaded) {
    ROS_ERROR(
        "missing, unexpected, or malformed fixture profile/mission parameter");
    return false;
  }
  if (maximum_candidate_attempts <= 0) {
    ROS_ERROR("tube view region requires candidate attempts");
    return false;
  }

  config_->lift_maximum_speed_request_mm_s =
      lift_maximum_speed_request_mm_s;
  config_->view_region.x_min_m = view_x_bounds_m[0];
  config_->view_region.x_max_m = view_x_bounds_m[1];
  config_->view_region.clearance_min_m = view_clearance_bounds_m[0];
  config_->view_region.clearance_max_m = view_clearance_bounds_m[1];
  config_->view_region.maximum_candidate_attempts =
      static_cast<std::size_t>(maximum_candidate_attempts);
  config_->maximum_recentering_step_m =
      config_->visual_alignment.maximum_step_m;
  config_->minimum_pixel_error_improvement_px =
      config_->visual_alignment.minimum_pixel_error_improvement_px;
  config_->phase_one_expected_tcp.x = phase_one_expected_tcp_xyz_m[0];
  config_->phase_one_expected_tcp.y = phase_one_expected_tcp_xyz_m[1];
  config_->phase_one_expected_tcp.z = phase_one_expected_tcp_xyz_m[2];

  const bool lift_tolerance_roundable =
      std::isfinite(config_->lift_target_tolerance_m) &&
      config_->lift_target_tolerance_m >= 0.0 &&
      config_->lift_target_tolerance_m <=
          static_cast<double>(std::numeric_limits<std::int64_t>::max()) /
              1000.0;
  config_->lift_target_tolerance_mm = lift_tolerance_roundable
      ? std::llround(config_->lift_target_tolerance_m * 1000.0)
      : -1;

  config_->link6_to_grasp_tcp.translation.x = grasp_tcp_xyz[0];
  config_->link6_to_grasp_tcp.translation.y = grasp_tcp_xyz[1];
  config_->link6_to_grasp_tcp.translation.z = grasp_tcp_xyz[2];
  config_->link6_to_grasp_tcp.rotation.x = grasp_tcp_quaternion[0];
  config_->link6_to_grasp_tcp.rotation.y = grasp_tcp_quaternion[1];
  config_->link6_to_grasp_tcp.rotation.z = grasp_tcp_quaternion[2];
  config_->link6_to_grasp_tcp.rotation.w = grasp_tcp_quaternion[3];
  config_->link6_to_camera.translation.x = camera_xyz[0];
  config_->link6_to_camera.translation.y = camera_xyz[1];
  config_->link6_to_camera.translation.z = camera_xyz[2];
  config_->link6_to_camera.rotation.x = camera_quaternion[0];
  config_->link6_to_camera.rotation.y = camera_quaternion[1];
  config_->link6_to_camera.rotation.z = camera_quaternion[2];
  config_->link6_to_camera.rotation.w = camera_quaternion[3];
  config_->lift_distance_m = tube_lift_distance(
      config_->body_height_m,
      config_->cap_height_m,
      config_->lift_clearance_m);

  return ValidateTubeMotionConfig(*config_);
}

bool ValidateTubeMotionConfig(const agx_xarm_pick::TubeTaskMotionConfig& config) {
  const auto* config_ = &config;
  if (config.nominal_fixture_center_xyz_m.size()!=3 ||
      config.workspace_min_xyz_m.size()!=3 || config.workspace_max_xyz_m.size()!=3 ||
      config.initial_tcp_x_bounds_m.size()!=2 || config.initial_tcp_z_bounds_m.size()!=2 ||
      config.lift_service_name.empty() || config.lift_state_topic.empty()) return false;
  const auto lift_target_ = agx_xarm_pick::compute_lift_target(
      config.table_surface_height_above_floor_m,
      config.arm_base_height_at_zero_lift_above_floor_m,
      config.lift_minimum_position_m, config.lift_maximum_position_m);
  const bool lift_tolerance_roundable = std::isfinite(config.lift_target_tolerance_m) &&
      config.lift_target_tolerance_m >= 0 && config.lift_target_tolerance_m <=
      static_cast<double>(std::numeric_limits<std::int64_t>::max())/1000.0;
  const auto& tcp = config.link6_to_grasp_tcp.rotation;
  const auto& camera = config.link6_to_camera.rotation;
  const std::vector<double> grasp_tcp_quaternion{tcp.x,tcp.y,tcp.z,tcp.w};
  const std::vector<double> camera_quaternion{camera.x,camera.y,camera.z,camera.w};
  const auto quaternion_norm = [](const std::vector<double>& quaternion) {
    return std::sqrt(
        quaternion[0] * quaternion[0] +
        quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] +
        quaternion[3] * quaternion[3]);
  };
  tf2::Quaternion link6_to_grasp_tcp_rotation(
      grasp_tcp_quaternion[0],
      grasp_tcp_quaternion[1],
      grasp_tcp_quaternion[2],
      grasp_tcp_quaternion[3]);
  tf2::Quaternion link6_to_camera_rotation(
      camera_quaternion[0],
      camera_quaternion[1],
      camera_quaternion[2],
      camera_quaternion[3]);
  const tf2::Vector3 grasp_approach_in_link6 =
      tf2::quatRotate(
          link6_to_grasp_tcp_rotation, tf2::Vector3(0.0, 0.0, 1.0));
  const tf2::Vector3 camera_optical_axis_in_link6 =
      tf2::quatRotate(
          link6_to_camera_rotation, tf2::Vector3(0.0, 0.0, 1.0));
  const double camera_axis_alignment =
      grasp_approach_in_link6.normalized().dot(
          camera_optical_axis_in_link6.normalized());
  const bool camera_mount_valid =
      std::isfinite(camera_axis_alignment) &&
      camera_axis_alignment >= std::cos(
          config_->profile_downward_axis_tolerance_deg * kPi / 180.0);
  const bool vectors_ordered =
      std::equal(
          config_->workspace_min_xyz_m.begin(),
          config_->workspace_min_xyz_m.end(),
          config_->workspace_max_xyz_m.begin(),
          [](double minimum, double maximum) {
            return minimum < maximum;
          });
  const std::int64_t slot_count =
      static_cast<std::int64_t>(config_->rows) *
      static_cast<std::int64_t>(config_->columns);
  const bool geometry_valid =
      Positive(config_->table_surface_height_above_floor_m) &&
      Positive(config_->table_height_tolerance_m) &&
      Positive(config_->body_diameter_m) &&
      Positive(config_->body_height_m) &&
      Positive(config_->cap_diameter_m) &&
      Positive(config_->cap_height_m) &&
      Positive(config_->diameter_tolerance_m) &&
      Positive(config_->height_tolerance_m) &&
      cap_exceeds_body(
          config_->cap_diameter_m, config_->body_diameter_m) &&
      Positive(config_->center_pitch_m) &&
      std::isfinite(config_->rack_height_above_surface_m) &&
      config_->rack_height_above_surface_m >= 0.0 &&
      Positive(config_->pitch_tolerance_m) &&
      config_->pitch_tolerance_m < config_->center_pitch_m &&
      config_->rows > 0 &&
      config_->columns > 0 &&
      config_->maximum_missing_slots >= 0 &&
      config_->maximum_missing_slots < slot_count &&
      static_cast<std::int64_t>(config_->minimum_layout_inliers) ==
          slot_count -
              static_cast<std::int64_t>(
                  config_->maximum_missing_slots) &&
      Positive(config_->maximum_lattice_rms_error_m) &&
      config_->maximum_lattice_rms_error_m <=
          config_->pitch_tolerance_m &&
      config_->center_pitch_m >
          config_->cap_diameter_m +
              2.0 * config_->insertion_radial_clearance_m &&
      config_->center_pitch_m - config_->cap_diameter_m >
          config_->tube_clearance_margin_m &&
      config_->grasp_tcp_offset_below_cap_top_m <
          config_->cap_height_m;
  const bool perception_valid =
      config_->color_frame == "cam1_color_optical_frame" &&
      valid_hsv_bounds(
          config_->target_hsv_lower, config_->target_hsv_upper) &&
      Positive(config_->cap_top_height_band_m) &&
      Positive(config_->minimum_cap_circularity) &&
      config_->minimum_cap_circularity <= 1.0 &&
      config_->maximum_cap_axis_ratio >= 1.0 &&
      Positive(config_->maximum_target_center_error_px) &&
      config_->maximum_recentering_iterations > 0 &&
      Positive(config_->max_interframe_spread_m) &&
      config_->required_consistent_frames > 0 &&
      config_->synchronizer_queue_size > 0 &&
      Positive(config_->max_sync_skew_sec) &&
      Positive(config_->detection_timeout_sec) &&
      Positive(config_->maximum_frame_age_sec) &&
      vectors_ordered;
  const bool commands_valid =
      config_->fully_open_command >= 0.0 &&
      config_->fully_open_command <= 1000.0 &&
      config_->close_command >= 0.0 &&
      config_->close_command <= 1000.0 &&
      config_->fully_open_command > config_->close_command &&
      Positive(config_->insertion_radial_clearance_m) &&
      Positive(config_->finger_thickness_m) &&
      Positive(config_->finger_width_m) &&
      Positive(config_->tube_clearance_margin_m) &&
      valid_command_gap_samples(
          config_->command_gap_samples,
          config_->close_command,
          config_->fully_open_command) &&
      config_->cap_diameter_m +
              2.0 * config_->insertion_radial_clearance_m <=
          config_->command_gap_samples.back()[1] &&
      config_->gripper_force >= 20.0 &&
      config_->gripper_force <= 100.0 &&
      std::all_of(config_->gripper_speeds.begin(), config_->gripper_speeds.end(),
                  [](double speed) { return std::isfinite(speed) && speed >= 1.0 && speed <= 100.0; }) &&
      Positive(config_->gripper_command_tolerance) &&
      Positive(config_->gripper_state_max_age_sec);
  const double zero_lift_cap_top_m = lift_target_.has_value()
      ? lift_target_->metres +
          config_->rack_height_above_surface_m +
          config_->body_height_m +
          config_->cap_height_m
      : std::numeric_limits<double>::quiet_NaN();
  const bool search_points_in_workspace =
      config_->initial_tcp_x_bounds_m[0] >=
          config_->workspace_min_xyz_m[0] &&
      config_->initial_tcp_x_bounds_m[1] <=
          config_->workspace_max_xyz_m[0] &&
      config_->view_region.x_min_m >=
          config_->workspace_min_xyz_m[0] &&
      config_->view_region.x_max_m <=
          config_->workspace_max_xyz_m[0] &&
      config_->search_centerline_y_m >= config_->workspace_min_xyz_m[1] &&
      config_->search_centerline_y_m <= config_->workspace_max_xyz_m[1] &&
      config_->view_entry_y_m >= config_->workspace_min_xyz_m[1] &&
      config_->view_entry_y_m <= config_->workspace_max_xyz_m[1] &&
      lift_target_.has_value() &&
      search_height_envelope_in_workspace(
          zero_lift_cap_top_m,
          lift_target_->metres,
          config_->lift_target_tolerance_m,
          config_->lift_minimum_position_m,
          config_->lift_maximum_position_m,
          config_->view_region.clearance_min_m,
          config_->view_region.clearance_max_m,
          config_->workspace_min_xyz_m[2],
          config_->workspace_max_xyz_m[2]) &&
      std::all_of(
          config_->observation_y_positions_m.begin(),
          config_->observation_y_positions_m.end(),
          [config_](double lateral) {
            return lateral >= config_->workspace_min_xyz_m[1] &&
                lateral <= config_->workspace_max_xyz_m[1];
          });
  const bool search_valid =
      std::isfinite(config_->search_centerline_y_m) &&
      valid_initial_centerline_tolerance(
          config_->initial_centerline_tolerance_m) &&
      std::isfinite(config_->centerline_joint_position_rad) &&
      Positive(config_->centerline_joint_tolerance_rad) &&
      config_->centerline_joint_tolerance_rad <= 0.1 &&
      config_->initial_tcp_x_bounds_m[0] <
          config_->initial_tcp_x_bounds_m[1] &&
      valid_initial_compact_z_bounds(
          config_->initial_tcp_z_bounds_m[0],
          config_->initial_tcp_z_bounds_m[1],
          config_->phase_one_expected_tcp.z) &&
      std::abs(
          config_->phase_two_centerline_y_m -
          config_->search_centerline_y_m) <=
          config_->initial_centerline_tolerance_m &&
      std::abs(
          config_->view_entry_y_m - config_->search_centerline_y_m) <=
          config_->view_region.y_tolerance_m &&
      agx_xarm_pick::valid_top_down_view_region(config_->view_region) &&
      Positive(config_->view_planning_time_per_candidate_sec) &&
      agx_xarm_pick::valid_visual_alignment_config(config_->visual_alignment) &&
      Positive(config_->maximum_recentering_step_m) &&
      Positive(config_->minimum_pixel_error_improvement_px) &&
      config_->observation_y_positions_m.size() >= 3 &&
      strictly_decreasing(config_->observation_y_positions_m) &&
      Positive(config_->downward_axis_tolerance_deg) &&
      config_->downward_axis_tolerance_deg <=
          config_->profile_downward_axis_tolerance_deg &&
      config_->minimum_adjacent_view_overlap > 0.0 &&
      config_->minimum_adjacent_view_overlap < 1.0 &&
      Positive(config_->search_settle_duration_sec) &&
      config_->detector_attempts_per_waypoint >= 1 &&
      search_points_in_workspace;
  const bool preparation_valid =
      agx_xarm_pick::valid_phase_one_joint_positions(
          config_->phase_one_joint_positions_rad) &&
      FinitePoint(config_->phase_one_expected_tcp) &&
      Positive(config_->phase_one_downward_axis_tolerance_deg) &&
      config_->phase_one_downward_axis_tolerance_deg <= 90.0 &&
      std::isfinite(config_->phase_two_centerline_y_m) &&
      Positive(config_->phase_two_downward_axis_tolerance_deg) &&
      config_->phase_two_downward_axis_tolerance_deg <= 90.0;
  const bool motion_valid =
      preparation_valid &&
      search_valid &&
      std::abs(quaternion_norm(grasp_tcp_quaternion) - 1.0) <= 1e-6 &&
      std::abs(quaternion_norm(camera_quaternion) - 1.0) <= 1e-6 &&
      camera_mount_valid &&
      Positive(config_->preparation_clearance_above_cap_m) &&
      Positive(config_->pregrasp_clearance_m) &&
      Positive(config_->grasp_tcp_offset_below_cap_top_m) &&
      Positive(config_->lift_clearance_m) &&
      Positive(config_->profile_downward_axis_tolerance_deg) &&
      config_->profile_downward_axis_tolerance_deg <= 90.0 &&
      Positive(config_->planning_timeout_sec) &&
      Positive(config_->gripper_timeout_sec) &&
      Positive(config_->hold_duration_sec) &&
      Positive(config_->robot_state_max_age_sec) &&
      Positive(config_->cartesian_step_m) &&
      std::all_of(config_->arm_execution.begin(), config_->arm_execution.end(),
                  [](const agx_xarm_pick::ArmExecutionScales& scales) {
                    return UnitScaling(scales.velocity) && UnitScaling(scales.acceleration);
                  }) &&
      std::abs(
          config_->lift_distance_m -
          (config_->body_height_m +
           config_->cap_height_m +
           config_->lift_clearance_m)) <= 1e-12;
  const bool lift_valid =
      config_->lift_service_name.front() == '/' &&
      config_->lift_state_topic.front() == '/' &&
      Positive(config_->arm_base_height_at_zero_lift_above_floor_m) &&
      std::isfinite(config_->lift_minimum_position_m) &&
      config_->lift_minimum_position_m >= 0.0 &&
      std::isfinite(config_->lift_maximum_position_m) &&
      config_->lift_maximum_position_m >=
          config_->lift_minimum_position_m &&
      lift_tolerance_roundable &&
      config_->lift_target_tolerance_mm >= 0 &&
      config_->lift_maximum_speed_request_mm_s > 0 &&
      Positive(config_->lift_state_max_age_sec) &&
      Positive(config_->lift_motion_timeout_sec) &&
      lift_target_.has_value();
  const bool protocol_valid =
      config_->task_type == "test_tube_pickup" &&
      !config_->target_cap_color.empty() &&
      config_->profile_target_cap_color == config_->target_cap_color &&
      (config_->context_source == "configured_demo" ||
       config_->context_source == "resolved_installation") &&
      config_->planning_frame == "link_base" &&
      !config_->tag_registration_required &&
      std::all_of(
          config_->nominal_fixture_center_xyz_m.begin(),
          config_->nominal_fixture_center_xyz_m.end(),
          [](double value) { return std::isfinite(value); }) &&
      config_->nominal_fixture_center_xyz_m[0] >=
          config_->workspace_min_xyz_m[0] &&
      config_->nominal_fixture_center_xyz_m[0] <=
          config_->workspace_max_xyz_m[0] &&
      config_->nominal_fixture_center_xyz_m[1] >=
          config_->workspace_min_xyz_m[1] &&
      config_->nominal_fixture_center_xyz_m[1] <=
          config_->workspace_max_xyz_m[1] &&
      lift_target_.has_value() &&
      std::abs(
          config_->nominal_fixture_center_xyz_m[2] -
          lift_target_->metres) <= config_->table_height_tolerance_m &&
      zero_lift_cap_top_m >=
          config_->workspace_min_xyz_m[2] &&
      zero_lift_cap_top_m <=
          config_->workspace_max_xyz_m[2] &&
      Positive(config_->maximum_translation_error_m) &&
      Positive(config_->maximum_yaw_error_deg) &&
      config_->maximum_yaw_error_deg <= 180.0;

  if (!geometry_valid || !perception_valid || !commands_valid ||
      !motion_valid || !lift_valid || !protocol_valid) {
    ROS_ERROR("invalid /tube_grasp protocol values");
    return false;
  }
  return true;
}

std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig>
SnapshotLegacyTubeTaskConfig(const ros::NodeHandle& node) {
  auto raw = std::make_shared<XmlRpc::XmlRpcValue>();
  XmlRpc::XmlRpcValue mission, profiles, arm;
  if (!node.getParam("/tube_grasp", mission) ||
      !node.getParam("/tube_fixture_profiles", profiles))
    throw std::runtime_error("invalid /tube_grasp configuration");
  (*raw)["tube_grasp"] = mission;
  (*raw)["tube_fixture_profiles"] = profiles;
  if (node.getParam("/arm_execution", arm)) (*raw)["arm_execution"] = arm;
  auto result = std::make_shared<agx_xarm_pick::ResolvedTubeTaskConfig>();
  if (!LoadLegacyMotionConfig(raw, &result->motion))
    throw std::runtime_error("invalid /tube_grasp configuration");
  auto& m = result->motion;
  result->config_id = "sha256:" + Digest(*raw);
  result->task_config_id = "legacy/" + m.active_profile_id;
  result->fixture_profile_id = m.active_profile_id;
  result->fixture_instance_id = result->task_config_id + "/installed_fixture";
  result->installation.id = result->fixture_instance_id;
  result->installation.parent_frame = m.planning_frame;
  result->installation.fixture_frame = result->fixture_instance_id;
  result->installation.measured_transform.translation = {
      m.nominal_fixture_center_xyz_m[0], m.nominal_fixture_center_xyz_m[1],
      m.nominal_fixture_center_xyz_m[2]};
  result->rack.id = m.active_profile_id;
  result->container.id = result->task_config_id + "/tube";
  result->container.body_diameter_m=m.body_diameter_m;
  result->container.body_height_m=m.body_height_m;
  result->container.cap_diameter_m=m.cap_diameter_m;
  result->container.cap_height_m=m.cap_height_m;
  result->container.diameter_tolerance_m=m.diameter_tolerance_m;
  result->container.height_tolerance_m=m.height_tolerance_m;
  result->rack.supported_container_profiles={result->container.id};
  result->support.id=result->task_config_id+"/table";
  result->support.frame_id=m.planning_frame;
  result->support.surface_height_above_floor_m=m.table_surface_height_above_floor_m;
  result->support.height_tolerance_m=m.table_height_tolerance_m;
  result->behavior.id=result->task_config_id+"/behavior";
  result->behavior.task_type=m.task_type;
  result->behavior.color_classifiers.emplace(m.target_cap_color,
      agx_xarm_pick::ColorClassifier{m.target_hsv_lower,m.target_hsv_upper});
  SnapshotNode profile(raw,"/tube_fixture_profiles/"+m.active_profile_id);
  std::vector<double> roi;
  if (!GetVector(profile,"perception/roi_normalized",4,&roi) ||
      !GetInteger(profile,"perception/minimum_layout_saturation",&result->behavior.minimum_layout_saturation) ||
      !GetNumber(profile,"perception/maximum_target_temporal_spread_px",&result->behavior.maximum_target_temporal_spread_px))
    throw std::runtime_error("invalid /tube_grasp configuration");
  std::copy(roi.begin(),roi.end(),result->behavior.roi_normalized.begin());
  result->robot={result->task_config_id+"/robot",m.planning_frame,m.color_frame,
      "/detect_tube_cap","/joint_states","/gripper/ctrl","/gripper/states"};
  result->sources.push_back({"legacy_parameters",result->task_config_id,
      result->config_id,"rosparam:/tube_grasp+/tube_fixture_profiles+/arm_execution"});
  return result;
}

}  // namespace agxcore
