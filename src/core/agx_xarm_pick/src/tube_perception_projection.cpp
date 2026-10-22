#include "agx_xarm_pick/tube_perception_projection.hpp"
#include <openssl/evp.h>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace agx_xarm_pick {
namespace {
void AppendUnsigned(std::uint64_t value, std::size_t bytes, std::string* out) {
  for (std::size_t i = 0; i < bytes; ++i) out->push_back(static_cast<char>((value >> (8*i)) & 0xff));
}
void AppendDouble(double value, std::string* out) {
  static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559, "IEEE754 binary64 required");
  if (!std::isfinite(value)) throw std::invalid_argument("perception field must be finite");
  if (value == 0) value = 0;  // Normalize signed zero.
  std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits));
  AppendUnsigned(bits, 8, out);
}
void AppendString(const std::string& value, std::string* out) {
  if (value.empty() || value.size() > 4096 || value.find('\0') != std::string::npos)
    throw std::invalid_argument("perception string is empty, oversized or contains NUL");
  // Validate UTF8 including overlong forms, surrogate codepoints and range.
  for (std::size_t i = 0; i < value.size();) {
    const auto first = static_cast<unsigned char>(value[i++]);
    if (first < 0x80) continue;
    unsigned count = 0; std::uint32_t point = 0, minimum = 0;
    if (first >= 0xc2 && first <= 0xdf) { count = 1; point = first & 0x1f; minimum = 0x80; }
    else if (first >= 0xe0 && first <= 0xef) { count = 2; point = first & 0x0f; minimum = 0x800; }
    else if (first >= 0xf0 && first <= 0xf4) { count = 3; point = first & 0x07; minimum = 0x10000; }
    else throw std::invalid_argument("invalid UTF8 perception string");
    if (i + count > value.size()) throw std::invalid_argument("truncated UTF8 perception string");
    for (unsigned j = 0; j < count; ++j) {
      const auto next = static_cast<unsigned char>(value[i++]);
      if ((next & 0xc0) != 0x80) throw std::invalid_argument("invalid UTF8 continuation");
      point = (point << 6) | (next & 0x3f);
    }
    if (point < minimum || point > 0x10ffff || (point >= 0xd800 && point <= 0xdfff))
      throw std::invalid_argument("invalid UTF8 codepoint");
  }
  AppendUnsigned(value.size(), 4, out); out->append(value);
}
}  // namespace
TubePerceptionProjection MakeTubePerceptionProjection(
    const ResolvedTubeTaskConfig& config, const std::string& target_cap_color) {
  const auto classifier = config.behavior.color_classifiers.find(target_cap_color);
  if (classifier == config.behavior.color_classifiers.end())
    throw std::invalid_argument("target color has no qualified classifier");
  TubePerceptionProjection p;
  p.schema_version = 1;
  p.planning_frame = config.motion.planning_frame;
  p.color_frame = config.motion.color_frame;
  p.lift_state_topic = config.motion.lift_state_topic;
  p.table_surface_height_above_floor_m = config.motion.table_surface_height_above_floor_m;
  p.table_height_tolerance_m = config.motion.table_height_tolerance_m;
  p.arm_base_height_at_zero_lift_above_floor_m = config.motion.arm_base_height_at_zero_lift_above_floor_m;
  p.lift_minimum_position_m = config.motion.lift_minimum_position_m;
  p.lift_maximum_position_m = config.motion.lift_maximum_position_m;
  p.lift_target_tolerance_m = config.motion.lift_target_tolerance_m;
  p.lift_state_max_age_sec = config.motion.lift_state_max_age_sec;
  p.body_diameter_m = config.motion.body_diameter_m;
  p.body_height_m = config.motion.body_height_m;
  p.cap_diameter_m = config.motion.cap_diameter_m;
  p.cap_height_m = config.motion.cap_height_m;
  p.diameter_tolerance_m = config.motion.diameter_tolerance_m;
  p.height_tolerance_m = config.motion.height_tolerance_m;
  p.rack_height_above_surface_m = config.motion.rack_height_above_surface_m;
  p.rows = config.motion.rows;
  p.columns = config.motion.columns;
  p.maximum_missing_slots = config.motion.maximum_missing_slots;
  p.minimum_layout_inliers = config.motion.minimum_layout_inliers;
  p.center_pitch_m = config.motion.center_pitch_m;
  p.pitch_tolerance_m = config.motion.pitch_tolerance_m;
  p.maximum_lattice_rms_error_m = config.motion.maximum_lattice_rms_error_m;
  p.roi_normalized = config.behavior.roi_normalized;
  p.minimum_layout_saturation = config.behavior.minimum_layout_saturation;
  p.target_hsv_lower = classifier->second.hsv_lower;
  p.target_hsv_upper = classifier->second.hsv_upper;
  p.cap_top_height_band_m = config.motion.cap_top_height_band_m;
  p.minimum_cap_circularity = config.motion.minimum_cap_circularity;
  p.maximum_cap_axis_ratio = config.motion.maximum_cap_axis_ratio;
  p.maximum_target_temporal_spread_px = config.behavior.maximum_target_temporal_spread_px;
  p.maximum_target_center_error_px = config.motion.maximum_target_center_error_px;
  p.maximum_recentering_iterations = config.motion.maximum_recentering_iterations;
  p.required_consistent_frames = config.motion.required_consistent_frames;
  p.synchronizer_queue_size = config.motion.synchronizer_queue_size;
  p.max_sync_skew_sec = config.motion.max_sync_skew_sec;
  p.detection_timeout_sec = config.motion.detection_timeout_sec;
  p.maximum_frame_age_sec = config.motion.maximum_frame_age_sec;
  p.max_interframe_spread_m = config.motion.max_interframe_spread_m;
  if (config.motion.workspace_min_xyz_m.size() != p.workspace_min_xyz_m.size()) throw std::invalid_argument("invalid workspace vector size");
  std::copy(config.motion.workspace_min_xyz_m.begin(), config.motion.workspace_min_xyz_m.end(), p.workspace_min_xyz_m.begin());
  if (config.motion.workspace_max_xyz_m.size() != p.workspace_max_xyz_m.size()) throw std::invalid_argument("invalid workspace vector size");
  std::copy(config.motion.workspace_max_xyz_m.begin(), config.motion.workspace_max_xyz_m.end(), p.workspace_max_xyz_m.begin());
  return p;
}
std::string TubePerceptionProjectionBytes(const TubePerceptionProjection& p,
    const std::string& fixture_instance_id, const std::string& fixture_profile_id,
    const std::string& target_cap_color) {
  if (p.schema_version != 1) throw std::invalid_argument("unsupported perception schema_version");
  static constexpr char prefix[] = "agx_tube_perception_projection_v1";
  std::string out(prefix, sizeof(prefix));
  AppendString(fixture_instance_id, &out);
  AppendString(fixture_profile_id, &out);
  AppendString(target_cap_color, &out);
  AppendUnsigned(p.schema_version, 4, &out);
  AppendString(p.planning_frame, &out);
  AppendString(p.color_frame, &out);
  AppendString(p.lift_state_topic, &out);
  AppendDouble(p.table_surface_height_above_floor_m, &out);
  AppendDouble(p.table_height_tolerance_m, &out);
  AppendDouble(p.arm_base_height_at_zero_lift_above_floor_m, &out);
  AppendDouble(p.lift_minimum_position_m, &out);
  AppendDouble(p.lift_maximum_position_m, &out);
  AppendDouble(p.lift_target_tolerance_m, &out);
  AppendDouble(p.lift_state_max_age_sec, &out);
  AppendDouble(p.body_diameter_m, &out);
  AppendDouble(p.body_height_m, &out);
  AppendDouble(p.cap_diameter_m, &out);
  AppendDouble(p.cap_height_m, &out);
  AppendDouble(p.diameter_tolerance_m, &out);
  AppendDouble(p.height_tolerance_m, &out);
  AppendDouble(p.rack_height_above_surface_m, &out);
  AppendUnsigned(p.rows, 4, &out);
  AppendUnsigned(p.columns, 4, &out);
  AppendUnsigned(p.maximum_missing_slots, 4, &out);
  AppendUnsigned(p.minimum_layout_inliers, 4, &out);
  AppendDouble(p.center_pitch_m, &out);
  AppendDouble(p.pitch_tolerance_m, &out);
  AppendDouble(p.maximum_lattice_rms_error_m, &out);
  for (auto value : p.roi_normalized) AppendDouble(value, &out);
  AppendUnsigned(p.minimum_layout_saturation, 4, &out);
  for (auto value : p.target_hsv_lower) AppendUnsigned(value, 4, &out);
  for (auto value : p.target_hsv_upper) AppendUnsigned(value, 4, &out);
  AppendDouble(p.cap_top_height_band_m, &out);
  AppendDouble(p.minimum_cap_circularity, &out);
  AppendDouble(p.maximum_cap_axis_ratio, &out);
  AppendDouble(p.maximum_target_temporal_spread_px, &out);
  AppendDouble(p.maximum_target_center_error_px, &out);
  AppendUnsigned(p.maximum_recentering_iterations, 4, &out);
  AppendUnsigned(p.required_consistent_frames, 4, &out);
  AppendUnsigned(p.synchronizer_queue_size, 4, &out);
  AppendDouble(p.max_sync_skew_sec, &out);
  AppendDouble(p.detection_timeout_sec, &out);
  AppendDouble(p.maximum_frame_age_sec, &out);
  AppendDouble(p.max_interframe_spread_m, &out);
  for (auto value : p.workspace_min_xyz_m) AppendDouble(value, &out);
  for (auto value : p.workspace_max_xyz_m) AppendDouble(value, &out);
  return out;
}
std::string TubePerceptionProjectionSha256(const TubePerceptionProjection& p,
    const std::string& fixture_instance_id, const std::string& fixture_profile_id,
    const std::string& target_cap_color) {
  const auto bytes = TubePerceptionProjectionBytes(p, fixture_instance_id, fixture_profile_id, target_cap_color);
  unsigned char digest[EVP_MAX_MD_SIZE]; unsigned size = 0;
  if (EVP_Digest(bytes.data(), bytes.size(), digest, &size, EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("perception SHA256 failure");
  std::ostringstream out; out << "sha256:" << std::hex << std::setfill('0');
  for (unsigned i = 0; i < size; ++i) out << std::setw(2) << static_cast<unsigned>(digest[i]);
  return out.str();
}
}  // namespace agx_xarm_pick
