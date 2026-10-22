#include "agx_xarm_pick/resolved_tube_config.hpp"

#include "agx_xarm_pick/tube_grasp_contract.hpp"
#include "agx_xarm_pick/tube_lift_contract.hpp"
#include "agx_xarm_pick/tube_preparation_contract.hpp"
#include "agx_xarm_pick/tube_search_contract.hpp"

#include <yaml-cpp/yaml.h>
#include <openssl/evp.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace agx_xarm_pick {
namespace {
constexpr double kPi = 3.14159265358979323846;
using Node = YAML::Node;
void Require(bool valid, const std::string& detail) {
  if (!valid) throw std::invalid_argument(detail);
}
std::string String(const Node& node) {
  Require(node.IsScalar() && !node.Scalar().empty(), "expected nonempty string");
  const auto value = node.Scalar();
  if (node.Tag() != "!") {
    Require(value != "true" && value != "false" && value != "null" && value != "~", "expected string, not boolean/null");
    try { (void)node.as<double>(); throw std::invalid_argument("expected string, not number"); }
    catch (const YAML::BadConversion&) {}
  }
  return value;
}
double Number(const Node& node) {
  const auto location = " at line " + std::to_string(node.Mark().line + 1);
  Require(node.IsScalar() && node.Tag() != "!", "expected numeric scalar" + location);
  double value = node.as<double>();
  Require(std::isfinite(value), "numeric value must be finite" + location);
  return value;
}
int Integer(const Node& node) {
  Require(node.IsScalar() && node.Tag() != "!", "expected integer scalar");
  const std::string value = node.Scalar();
  std::size_t first = (value[0] == '+' || value[0] == '-') ? 1 : 0;
  Require(first < value.size() && value.find_first_not_of("0123456789", first) == std::string::npos,
          "expected decimal integer");
  const auto number = std::stoll(value, nullptr, 10);
  Require(number >= std::numeric_limits<int>::min() && number <= std::numeric_limits<int>::max(), "integer out of range");
  return static_cast<int>(number);
}
bool Boolean(const Node& node) {
  Require(node.IsScalar() && node.Tag() != "!" &&
          (node.Scalar() == "true" || node.Scalar() == "false"), "expected boolean true/false");
  return node.Scalar() == "true";
}
void Keys(const Node& node, std::initializer_list<const char*> required,
          std::initializer_list<const char*> optional = {}) {
  Require(node.IsMap(), "expected mapping");
  std::set<std::string> allowed;
  for (const auto* key : required) {
    Require(node[key].IsDefined(), std::string("missing key: ") + key);
    allowed.insert(key);
  }
  for (const auto* key : optional) allowed.insert(key);
  for (const auto& item : node)
    Require(allowed.count(String(item.first)), "unexpected key: " + item.first.Scalar());
}
void Inspect(const Node& node, std::vector<Node>* ancestors, std::size_t depth = 0) {
  Require(depth < 64, "YAML nesting exceeds 64 levels or contains alias cycle");
  for (const auto& seen : *ancestors) Require(!node.is(seen), "YAML alias cycle is forbidden");
  ancestors->push_back(node);
  if (node.IsMap()) {
    std::set<std::string> keys;
    for (const auto& item : node) {
      const auto key = String(item.first);
      Require(key != "<<", "YAML merges are forbidden");
      Require(keys.insert(key).second, "duplicate YAML key: " + key);
      Inspect(item.second, ancestors, depth + 1);
    }
  } else if (node.IsSequence()) {
    for (const auto& item : node) Inspect(item, ancestors, depth + 1);
  } else Require(node.IsScalar(), "null YAML values are forbidden");
  ancestors->pop_back();
}
std::vector<double> Vector(const Node& node, std::size_t size = 0) {
  Require(node.IsSequence() && node.size() > 0 && (size == 0 || node.size() == size), "invalid numeric vector size");
  std::vector<double> result;
  for (const auto& value : node) result.push_back(Number(value));
  return result;
}
template <std::size_t N> std::array<double, N> Array(const Node& node) {
  const auto values = Vector(node, N);
  std::array<double, N> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}
std::array<int, 3> Hsv(const Node& node) {
  Require(node.IsSequence() && node.size() == 3, "HSV needs three integers");
  return {{Integer(node[0]), Integer(node[1]), Integer(node[2])}};
}
ConfigTransform Transform(const Node& xyz, const Node& quaternion) {
  const auto p = Array<3>(xyz); const auto q = Array<4>(quaternion);
  double norm = 0; for (double value : q) norm += value * value;
  Require(std::abs(std::sqrt(norm) - 1.0) <= 1e-6, "transform quaternion must be normalized");
  return {{p[0], p[1], p[2]}, {q[0], q[1], q[2], q[3]}};
}
std::string Sha256(const std::string& input) {
  unsigned char digest[EVP_MAX_MD_SIZE]; unsigned int size = 0;
  Require(EVP_Digest(input.data(), input.size(), digest, &size, EVP_sha256(), nullptr) == 1,
          "SHA256 digest failure");
  std::ostringstream output; output << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < size; ++i) output << std::setw(2) << static_cast<int>(digest[i]);
  return output.str();
}
std::string Canonical(const Node& node) {
  if (node.IsSequence()) {
    std::string result = "[";
    for (const auto& item : node) result += Canonical(item) + ";";
    return result + "]";
  }
  if (node.IsMap()) {
    std::map<std::string, Node> sorted;
    for (const auto& item : node) sorted.emplace(String(item.first), item.second);
    std::string result = "{";
    for (const auto& item : sorted)
      result += std::to_string(item.first.size()) + ":" + item.first + Canonical(item.second);
    return result + "}";
  }
  if (node.Tag() != "!") {
    if (node.Scalar() == "true" || node.Scalar() == "false") return "b" + node.Scalar();
    try {
      double value = node.as<double>(); if (value == 0) value = 0;
      std::ostringstream out; out.imbue(std::locale::classic());
      out << "n" << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
      return out.str();
    } catch (const YAML::BadConversion&) {}
  }
  return "s" + std::to_string(node.Scalar().size()) + ":" + node.Scalar();
}
struct Document {
  std::string path, kind, id, content;
  Node node;
  std::map<std::string, std::string> references;
};
class Snapshot {
 public:
  std::map<std::string, Document> documents;
  Document& Read(const std::filesystem::path& path, const std::string& expected) {
    std::error_code error;
    const auto canonical = std::filesystem::canonical(path, error).string();
    Require(!error, "missing configuration file: " + path.string());
    Require(!active_.count(canonical), "configuration reference cycle: " + canonical);
    const auto found = documents.find(canonical);
    if (found != documents.end()) {
      Require(found->second.kind == expected, "mixed ownership: expected " + expected + " at " + canonical);
      return found->second;
    }
    active_.insert(canonical);
    std::ifstream file(canonical, std::ios::binary);
    Require(file.good(), "cannot read configuration: " + canonical);
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    Require(!file.bad(), "cannot finish reading configuration: " + canonical);
    const auto nodes = YAML::LoadAll(content);
    Require(nodes.size() == 1, "configuration must contain exactly one YAML document");
    Node node = nodes.front(); std::vector<Node> ancestors; Inspect(node, &ancestors);
    Require(node.IsMap() && node["schema_version"] && Integer(node["schema_version"]) == 1,
            "unsupported or missing schema_version: " + canonical);
    const auto kind = String(node["kind"]); const auto id = String(node["id"]);
    Require(kind == expected, "mixed ownership: expected " + expected + ", got " + kind);
    for (const auto& entry : documents)
      Require(entry.second.kind != kind || entry.second.id != id, "duplicate configuration identity: " + kind + ":" + id);
    Document& document = documents.emplace(canonical, Document{canonical, kind, id, content, node, {}}).first->second;
    if (kind == "tube_task") {
      Reference(document, "installation", "fixture_installation");
      Reference(document, "container", "container_profile");
      Reference(document, "behavior", "tube_behavior");
      Reference(document, "robot", "robot_profile");
    } else if (kind == "fixture_installation") {
      Reference(document, "fixture_profile", "rack_profile");
      Reference(document, "support_surface", "support_surface");
    }
    active_.erase(canonical);
    return document;
  }
  Document& Child(Document& parent, const std::string& key) { return documents.at(parent.references.at(key)); }
 private:
  std::set<std::string> active_;
  void Reference(Document& parent, const std::string& key, const std::string& kind) {
    const auto ref = String(parent.node[key]);
    auto& child = Read(std::filesystem::path(parent.path).parent_path() / ref, kind);
    parent.references.emplace(key, child.path);
  }
};
template <typename Parse>
void WithSource(const Document& document, Parse parse) {
  try { parse(); }
  catch (const std::exception& error) {
    throw std::invalid_argument(document.path + ": " + error.what());
  }
}
void ParseInstallation(const Document& doc, ResolvedTubeTaskConfig* result) {
  const auto n = doc.node;
  Keys(n, {"schema_version", "kind", "id", "fixture_profile", "support_surface", "station_id", "parent_frame", "fixture_frame", "measured_transform", "registration"});
  auto& i = result->installation; auto& c = result->motion;
  i.id = doc.id; i.station_id = String(n["station_id"]); i.parent_frame = String(n["parent_frame"]);
  i.fixture_frame = String(n["fixture_frame"]);
  Require(i.parent_frame != i.fixture_frame, "fixture frame must differ from parent frame");
  const auto t = n["measured_transform"];
  Keys(t, {"translation_xyz_m", "quaternion_xyzw"});
  i.measured_transform = Transform(t["translation_xyz_m"], t["quaternion_xyzw"]);
  const auto r = n["registration"];
  Keys(r, {"required", "maximum_translation_error_m", "maximum_yaw_error_rad"}, {"tag_id"});
  i.registration_required = Boolean(r["required"]);
  if (r["tag_id"]) i.tag_id = String(r["tag_id"]);
  Require(!i.registration_required || i.tag_id.has_value(), "tag registration requires tag_id");
  c.maximum_translation_error_m = Number(r["maximum_translation_error_m"]);
  c.maximum_yaw_error_deg = Number(r["maximum_yaw_error_rad"]) * 180.0 / kPi;
  c.nominal_fixture_center_xyz_m = Vector(t["translation_xyz_m"], 3);
  c.context_source = "resolved_installation"; c.tag_registration_required = i.registration_required;
  result->fixture_instance_id = i.id;
}
void ParseSupport(const Document& doc, ResolvedTubeTaskConfig* result) {
  auto n = doc.node;
  Keys(n, {"schema_version", "kind", "id", "station_id", "frame_id", "dimensions_xyz_m", "surface_height_above_floor_m", "height_tolerance_m"});
  auto& s = result->support; s.id = doc.id; s.station_id = String(n["station_id"]); s.frame_id = String(n["frame_id"]);
  s.dimensions_xyz_m = Array<3>(n["dimensions_xyz_m"]);
  s.surface_height_above_floor_m = Number(n["surface_height_above_floor_m"]); s.height_tolerance_m = Number(n["height_tolerance_m"]);
  result->motion.table_surface_height_above_floor_m = s.surface_height_above_floor_m;
  result->motion.table_height_tolerance_m = s.height_tolerance_m;
}
void ParseRack(const Document& doc, ResolvedTubeTaskConfig* result) {
  const auto n = doc.node;
  Keys(n, {"schema_version", "kind", "id", "dimensions_xyz_m", "layout", "slots", "supported_container_profiles"});
  auto& r = result->rack; auto& c = result->motion;
  r.id = doc.id; r.dimensions_xyz_m = Array<3>(n["dimensions_xyz_m"]);
  result->fixture_profile_id = r.id; c.active_profile_id = r.id;
  const auto l = n["layout"];
  Keys(l, {"rows", "columns", "center_pitch_m", "rack_height_above_surface_m", "pitch_tolerance_m", "maximum_missing_slots", "minimum_layout_inliers", "maximum_lattice_rms_error_m"});
  c.rows = Integer(l["rows"]); c.columns = Integer(l["columns"]);
  c.maximum_missing_slots = Integer(l["maximum_missing_slots"]); c.minimum_layout_inliers = Integer(l["minimum_layout_inliers"]);
#define READ(field) c.field = Number(l[#field])
  READ(center_pitch_m); READ(rack_height_above_surface_m); READ(pitch_tolerance_m); READ(maximum_lattice_rms_error_m);
#undef READ
  Require(n["slots"].IsSequence() && n["slots"].size() > 0, "rack slots are required");
  std::set<std::string> ids; std::set<std::pair<int,int>> coordinates;
  for (const auto& slot : n["slots"]) {
    Keys(slot, {"id", "row", "column", "center_xy_m"});
    RackSlot value{String(slot["id"]), Integer(slot["row"]), Integer(slot["column"]), Array<2>(slot["center_xy_m"])};
    Require(ids.insert(value.id).second && coordinates.emplace(value.row,value.column).second, "duplicate rack slot identity or coordinates");
    Require(value.row >= 0 && value.row < c.rows && value.column >= 0 && value.column < c.columns, "rack slot outside layout");
    r.slots.push_back(value);
  }
  Require(static_cast<std::int64_t>(r.slots.size()) == static_cast<std::int64_t>(c.rows) * c.columns, "rack slots must cover every layout coordinate");
  const auto& origin = r.slots.front();
  for (const auto& slot : r.slots) {
    Require(std::abs(slot.center_xy_m[0] - origin.center_xy_m[0] - (slot.column-origin.column)*c.center_pitch_m) <= 1e-9 &&
            std::abs(slot.center_xy_m[1] - origin.center_xy_m[1] - (slot.row-origin.row)*c.center_pitch_m) <= 1e-9, "rack slot centers disagree with lattice geometry");
  }
  const auto supported = n["supported_container_profiles"];
  Require(supported.IsSequence() && supported.size() > 0, "supported container profiles required");
  std::set<std::string> unique;
  for (const auto& id : supported) {
    const auto value = String(id); Require(unique.insert(value).second, "duplicate supported container profile");
    r.supported_container_profiles.push_back(value);
  }
}
void ParseContainer(const Document& doc, ResolvedTubeTaskConfig* result) {
  auto n = doc.node;
  Keys(n, {"schema_version", "kind", "id", "body_diameter_m", "body_height_m", "cap_diameter_m", "cap_height_m", "diameter_tolerance_m", "height_tolerance_m"});
  auto& c = result->motion; auto& t = result->container; t.id = doc.id;
#define READ(field) t.field = c.field = Number(n[#field])
  READ(body_diameter_m); READ(body_height_m); READ(cap_diameter_m); READ(cap_height_m); READ(diameter_tolerance_m); READ(height_tolerance_m);
#undef READ
}
void ParseBehavior(const Document& doc, ResolvedTubeTaskConfig* result) {
  auto n = doc.node;
  Keys(n, {"schema_version", "kind", "id", "task_type", "perception", "motion"});
  auto& b = result->behavior; auto& c = result->motion;
  b.id = doc.id; b.task_type = c.task_type = String(n["task_type"]);
  const auto p = n["perception"];
  Keys(p, {"roi_normalized", "minimum_layout_saturation", "color_classifiers", "cap_top_height_band_m", "minimum_cap_circularity", "maximum_cap_axis_ratio", "maximum_target_temporal_spread_px", "maximum_target_center_error_px", "maximum_recentering_iterations", "required_consistent_frames", "synchronizer_queue_size", "max_sync_skew_sec", "detection_timeout_sec", "maximum_frame_age_sec", "max_interframe_spread_m", "workspace_min_xyz_m", "workspace_max_xyz_m"});
  b.roi_normalized = Array<4>(p["roi_normalized"]);
  b.minimum_layout_saturation = Integer(p["minimum_layout_saturation"]);
  b.maximum_target_temporal_spread_px = Number(p["maximum_target_temporal_spread_px"]);
#define READ(field) c.field = Number(p[#field])
  READ(cap_top_height_band_m); READ(minimum_cap_circularity); READ(maximum_cap_axis_ratio);
  READ(maximum_target_center_error_px); READ(max_sync_skew_sec); READ(detection_timeout_sec);
  READ(maximum_frame_age_sec); READ(max_interframe_spread_m);
#undef READ
  c.maximum_recentering_iterations = Integer(p["maximum_recentering_iterations"]);
  c.required_consistent_frames = Integer(p["required_consistent_frames"]);
  c.synchronizer_queue_size = Integer(p["synchronizer_queue_size"]);
  c.workspace_min_xyz_m = Vector(p["workspace_min_xyz_m"], 3); c.workspace_max_xyz_m = Vector(p["workspace_max_xyz_m"], 3);
  Require(p["color_classifiers"].IsMap() && p["color_classifiers"].size() > 0, "color classifiers required");
  for (const auto& classifier : p["color_classifiers"]) {
    Keys(classifier.second, {"hsv_lower", "hsv_upper"});
    ColorClassifier value{Hsv(classifier.second["hsv_lower"]), Hsv(classifier.second["hsv_upper"])};
    Require(agxcore::valid_hsv_bounds(value.hsv_lower, value.hsv_upper), "invalid HSV classifier bounds");
    b.color_classifiers.emplace(String(classifier.first), value);
  }
  const auto m = n["motion"];
  Keys(m, {"preparation_clearance_above_cap_m", "pregrasp_clearance_m", "grasp_offset_below_cap_top_m", "lift_clearance_m", "downward_axis_tolerance_rad", "planning_timeout_sec", "gripper_timeout_sec", "hold_duration_sec", "cartesian_step_m", "arm_speed", "preparation", "search"});
#define READ(field) c.field = Number(m[#field])
  READ(preparation_clearance_above_cap_m); READ(pregrasp_clearance_m); READ(lift_clearance_m);
  READ(planning_timeout_sec); READ(gripper_timeout_sec); READ(hold_duration_sec); READ(cartesian_step_m);
#undef READ
  c.grasp_tcp_offset_below_cap_top_m = Number(m["grasp_offset_below_cap_top_m"]);
  c.profile_downward_axis_tolerance_deg = Number(m["downward_axis_tolerance_rad"]) * 180.0 / kPi;
  const auto prep = m["preparation"]; Keys(prep, {"phase_one", "phase_two"});
  const auto one = prep["phase_one"];
  Keys(one, {"joint_positions_rad", "expected_tcp_xyz_m", "downward_axis_tolerance_rad"});
  c.phase_one_joint_positions_rad = Vector(one["joint_positions_rad"], 6);
  const auto point = Array<3>(one["expected_tcp_xyz_m"]); c.phase_one_expected_tcp = {point[0], point[1], point[2]};
  c.phase_one_downward_axis_tolerance_deg = Number(one["downward_axis_tolerance_rad"]) * 180.0 / kPi;
  const auto two = prep["phase_two"]; Keys(two, {"centerline_y_m", "downward_axis_tolerance_rad"});
  c.phase_two_centerline_y_m = Number(two["centerline_y_m"]);
  c.phase_two_downward_axis_tolerance_deg = Number(two["downward_axis_tolerance_rad"]) * 180.0 / kPi;
  const auto s = m["search"];
  Keys(s, {"centerline_y_m", "initial_centerline_tolerance_m", "centerline_joint_position_rad", "centerline_joint_tolerance_rad", "initial_tcp_x_bounds_m", "initial_tcp_z_bounds_m", "view_region", "visual_alignment", "observation_y_positions_m", "downward_axis_tolerance_rad", "minimum_adjacent_view_overlap", "settle_duration_sec", "detector_attempts_per_waypoint"});
#define READ(field) c.field = Number(s[#field])
  READ(initial_centerline_tolerance_m); READ(centerline_joint_position_rad); READ(centerline_joint_tolerance_rad);
  READ(minimum_adjacent_view_overlap);
#undef READ
  c.search_centerline_y_m = Number(s["centerline_y_m"]);
  c.search_settle_duration_sec = Number(s["settle_duration_sec"]);
  c.downward_axis_tolerance_deg = Number(s["downward_axis_tolerance_rad"]) * 180.0 / kPi;
  c.detector_attempts_per_waypoint = Integer(s["detector_attempts_per_waypoint"]);
  c.initial_tcp_x_bounds_m = Vector(s["initial_tcp_x_bounds_m"], 2); c.initial_tcp_z_bounds_m = Vector(s["initial_tcp_z_bounds_m"], 2);
  c.observation_y_positions_m = Vector(s["observation_y_positions_m"]);
  const auto v = s["view_region"];
  Keys(v, {"x_bounds_m", "preferred_x_m", "clearance_above_cap_bounds_m", "preferred_clearance_above_cap_m", "sampling_step_m", "entry_y_m", "waypoint_y_tolerance_m", "maximum_candidate_attempts", "planning_time_per_candidate_sec"});
  const auto x = Array<2>(v["x_bounds_m"]), clearance = Array<2>(v["clearance_above_cap_bounds_m"]);
  const int attempts = Integer(v["maximum_candidate_attempts"]); Require(attempts > 0, "view candidate attempts must be positive");
  c.view_region = {x[0], x[1], Number(v["preferred_x_m"]), clearance[0], clearance[1], Number(v["preferred_clearance_above_cap_m"]), Number(v["sampling_step_m"]), Number(v["waypoint_y_tolerance_m"]), static_cast<std::size_t>(attempts)};
  c.view_entry_y_m = Number(v["entry_y_m"]); c.view_planning_time_per_candidate_sec = Number(v["planning_time_per_candidate_sec"]);
  const auto a = s["visual_alignment"];
  Keys(a, {"u_axis_gain_m_per_px", "v_axis_gain_m_per_px", "target_center_tolerance_px", "required_centered_observations", "maximum_step_m", "minimum_retry_step_m", "maximum_total_displacement_m", "maximum_iterations", "stationary_target_retries", "minimum_pixel_error_improvement_px"});
#define READ(field) c.visual_alignment.field = Number(a[#field])
  READ(u_axis_gain_m_per_px); READ(v_axis_gain_m_per_px); READ(target_center_tolerance_px);
  READ(maximum_step_m); READ(minimum_retry_step_m); READ(maximum_total_displacement_m); READ(minimum_pixel_error_improvement_px);
#undef READ
  c.visual_alignment.required_centered_observations = Integer(a["required_centered_observations"]);
  c.visual_alignment.maximum_iterations = Integer(a["maximum_iterations"]);
  c.visual_alignment.stationary_target_retries = Integer(a["stationary_target_retries"]);
  c.maximum_recentering_step_m = c.visual_alignment.maximum_step_m;
  c.minimum_pixel_error_improvement_px = c.visual_alignment.minimum_pixel_error_improvement_px;
}
void ParseRobot(const Document& doc, const Document& behavior, ResolvedTubeTaskConfig* result) {
  const auto n = doc.node; auto& c = result->motion; auto& r = result->robot;
  Keys(n, {"schema_version", "kind", "id", "planning_frame", "color_frame", "interfaces", "lift", "transforms", "gripper", "robot_state_max_age_sec", "arm_execution"});
  r.id = doc.id; r.planning_frame = c.planning_frame = String(n["planning_frame"]);
  r.color_frame = c.color_frame = String(n["color_frame"]);
  c.robot_state_max_age_sec = Number(n["robot_state_max_age_sec"]);
  const auto i = n["interfaces"];
  Keys(i, {"detector_service", "joint_state_topic", "gripper_command_topic", "gripper_state_topic"});
  r.detector_service = String(i["detector_service"]); r.joint_state_topic = String(i["joint_state_topic"]);
  r.gripper_command_topic = String(i["gripper_command_topic"]); r.gripper_state_topic = String(i["gripper_state_topic"]);
  const auto l = n["lift"];
  Keys(l, {"service_name", "state_topic", "arm_base_height_at_zero_lift_above_floor_m", "minimum_position_m", "maximum_position_m", "target_tolerance_m", "state_max_age_sec", "motion_timeout_sec", "maximum_speed_request_m_s"});
  c.lift_service_name = String(l["service_name"]); c.lift_state_topic = String(l["state_topic"]);
  c.arm_base_height_at_zero_lift_above_floor_m = Number(l["arm_base_height_at_zero_lift_above_floor_m"]);
  c.lift_minimum_position_m = Number(l["minimum_position_m"]); c.lift_maximum_position_m = Number(l["maximum_position_m"]);
  c.lift_target_tolerance_m = Number(l["target_tolerance_m"]);
  c.lift_state_max_age_sec = Number(l["state_max_age_sec"]); c.lift_motion_timeout_sec = Number(l["motion_timeout_sec"]);
  const auto millimetres = [](double value) {
    const long double scaled = static_cast<long double>(value) * 1000.0L;
    Require(scaled >= 0 && scaled < static_cast<long double>(std::numeric_limits<std::int64_t>::max()), "lift value cannot be represented in integer millimetres");
    return static_cast<std::int64_t>(std::llround(scaled));
  };
  c.lift_target_tolerance_mm = millimetres(c.lift_target_tolerance_m);
  const double speed_m_s = Number(l["maximum_speed_request_m_s"]);
  c.lift_maximum_speed_request_mm_s = millimetres(speed_m_s);
  Require(std::abs(speed_m_s * 1000.0 - c.lift_maximum_speed_request_mm_s) < 1e-9, "lift speed must map exactly to native integer millimetres per second");
  const auto t = n["transforms"];
  Keys(t, {"link6_to_grasp_tcp_xyz_m", "link6_to_grasp_tcp_quaternion_xyzw", "link6_to_camera_xyz_m", "link6_to_camera_quaternion_xyzw"});
  c.link6_to_grasp_tcp = Transform(t["link6_to_grasp_tcp_xyz_m"], t["link6_to_grasp_tcp_quaternion_xyzw"]);
  c.link6_to_camera = Transform(t["link6_to_camera_xyz_m"], t["link6_to_camera_quaternion_xyzw"]);
  const auto g = n["gripper"];
  Keys(g, {"fully_open_command", "close_command", "insertion_radial_clearance_m", "finger_thickness_m", "finger_width_m", "tube_clearance_margin_m", "command_gap_samples", "gripper_force", "gripper_speeds", "gripper_command_tolerance", "gripper_state_max_age_sec"});
#define READ(field) c.field = Number(g[#field])
  READ(fully_open_command); READ(close_command); READ(insertion_radial_clearance_m); READ(finger_thickness_m);
  READ(finger_width_m); READ(tube_clearance_margin_m); READ(gripper_force); READ(gripper_command_tolerance); READ(gripper_state_max_age_sec);
#undef READ
  Require(g["command_gap_samples"].IsSequence(), "command gap samples must be a sequence");
  for (const auto& pair : g["command_gap_samples"]) c.command_gap_samples.push_back(Array<2>(pair));
  const auto speeds = g["gripper_speeds"]; Keys(speeds, {"preclose", "final_close", "recovery"});
  c.gripper_speeds = {{Number(speeds["preclose"]), Number(speeds["final_close"]), Number(speeds["recovery"])}};
  const auto arm = n["arm_execution"]; Keys(arm, {"velocity_scale", "acceleration_scale"});
  const ArmExecutionScales global{Number(arm["velocity_scale"]), Number(arm["acceleration_scale"])};
  const auto coefficients = behavior.node["motion"]["arm_speed"];
  Require(coefficients.IsMap() && coefficients["default_coefficient"], "arm speed default_coefficient required");
  for (const auto& pair : coefficients) {
    const auto key = String(pair.first);
    Require(key == "default_coefficient" || std::find(kArmMotionNames.begin(), kArmMotionNames.end(), key) != kArmMotionNames.end(), "unexpected arm speed key: " + key);
  }
  const auto default_coefficient = Number(coefficients["default_coefficient"]);
  for (std::size_t index = 0; index < kArmMotionNames.size(); ++index) {
    std::optional<double> value;
    if (coefficients[kArmMotionNames[index]]) value = Number(coefficients[kArmMotionNames[index]]);
    c.arm_execution[index] = resolve_arm_execution(global, default_coefficient, value);
  }
}
bool Positive(double value) { return std::isfinite(value) && value > 0; }
bool UnitScaling(double value) { return Positive(value) && value <= 1; }
void Validate(ResolvedTubeTaskConfig* result) {
  using namespace agxcore;
  auto& c = result->motion;
  c.lift_distance_m = tube_lift_distance(c.body_height_m, c.cap_height_m, c.lift_clearance_m);
  const auto lift_target = compute_lift_target(c.table_surface_height_above_floor_m,
      c.arm_base_height_at_zero_lift_above_floor_m, c.lift_minimum_position_m, c.lift_maximum_position_m);
  const auto axis = [](const ConfigQuaternion& q) {
    return std::array<double, 3>{{2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y)}};
  };
  const auto grasp = axis(c.link6_to_grasp_tcp.rotation), camera = axis(c.link6_to_camera.rotation);
  const double dot = grasp[0]*camera[0]+grasp[1]*camera[1]+grasp[2]*camera[2];
  const bool camera_mount_valid = std::isfinite(dot) && dot >= std::cos(c.profile_downward_axis_tolerance_deg*kPi/180);
  const bool vectors_ordered = std::equal(c.workspace_min_xyz_m.begin(), c.workspace_min_xyz_m.end(),
      c.workspace_max_xyz_m.begin(), [](double low, double high) { return low < high; });
  const std::int64_t slot_count = static_cast<std::int64_t>(c.rows) * c.columns;
  const bool geometry_valid =
      Positive(c.table_surface_height_above_floor_m) &&
      Positive(c.table_height_tolerance_m) &&
      Positive(c.body_diameter_m) &&
      Positive(c.body_height_m) &&
      Positive(c.cap_diameter_m) &&
      Positive(c.cap_height_m) &&
      Positive(c.diameter_tolerance_m) &&
      Positive(c.height_tolerance_m) &&
      cap_exceeds_body(
          c.cap_diameter_m, c.body_diameter_m) &&
      Positive(c.center_pitch_m) &&
      std::isfinite(c.rack_height_above_surface_m) &&
      c.rack_height_above_surface_m >= 0.0 &&
      Positive(c.pitch_tolerance_m) &&
      c.pitch_tolerance_m < c.center_pitch_m &&
      c.rows > 0 &&
      c.columns > 0 &&
      c.maximum_missing_slots >= 0 &&
      c.maximum_missing_slots < slot_count &&
      static_cast<std::int64_t>(c.minimum_layout_inliers) ==
          slot_count -
              static_cast<std::int64_t>(
                  c.maximum_missing_slots) &&
      Positive(c.maximum_lattice_rms_error_m) &&
      c.maximum_lattice_rms_error_m <=
          c.pitch_tolerance_m &&
      c.center_pitch_m >
          c.cap_diameter_m +
              2.0 * c.insertion_radial_clearance_m &&
      c.center_pitch_m - c.cap_diameter_m >
          c.tube_clearance_margin_m &&
      c.grasp_tcp_offset_below_cap_top_m <
          c.cap_height_m;
  const bool perception_valid =
      Positive(c.cap_top_height_band_m) &&
      Positive(c.minimum_cap_circularity) &&
      c.minimum_cap_circularity <= 1.0 &&
      c.maximum_cap_axis_ratio >= 1.0 &&
      Positive(c.maximum_target_center_error_px) &&
      c.maximum_recentering_iterations > 0 &&
      Positive(c.max_interframe_spread_m) &&
      c.required_consistent_frames > 0 &&
      c.synchronizer_queue_size > 0 &&
      Positive(c.max_sync_skew_sec) &&
      Positive(c.detection_timeout_sec) &&
      Positive(c.maximum_frame_age_sec) &&
      vectors_ordered;
  const bool commands_valid =
      c.fully_open_command >= 0.0 &&
      c.fully_open_command <= 1000.0 &&
      c.close_command >= 0.0 &&
      c.close_command <= 1000.0 &&
      c.fully_open_command > c.close_command &&
      Positive(c.insertion_radial_clearance_m) &&
      Positive(c.finger_thickness_m) &&
      Positive(c.finger_width_m) &&
      Positive(c.tube_clearance_margin_m) &&
      valid_command_gap_samples(
          c.command_gap_samples,
          c.close_command,
          c.fully_open_command) &&
      c.cap_diameter_m +
              2.0 * c.insertion_radial_clearance_m <=
          c.command_gap_samples.back()[1] &&
      c.gripper_force >= 20.0 &&
      c.gripper_force <= 100.0 &&
      std::all_of(c.gripper_speeds.begin(), c.gripper_speeds.end(),
                  [](double speed) { return std::isfinite(speed) && speed >= 1.0 && speed <= 100.0; }) &&
      Positive(c.gripper_command_tolerance) &&
      Positive(c.gripper_state_max_age_sec);
  const double zero_lift_cap_top_m = lift_target.has_value()
      ? lift_target->metres +
          c.rack_height_above_surface_m +
          c.body_height_m +
          c.cap_height_m
      : std::numeric_limits<double>::quiet_NaN();
  const bool search_points_in_workspace =
      c.initial_tcp_x_bounds_m[0] >=
          c.workspace_min_xyz_m[0] &&
      c.initial_tcp_x_bounds_m[1] <=
          c.workspace_max_xyz_m[0] &&
      c.view_region.x_min_m >=
          c.workspace_min_xyz_m[0] &&
      c.view_region.x_max_m <=
          c.workspace_max_xyz_m[0] &&
      c.search_centerline_y_m >= c.workspace_min_xyz_m[1] &&
      c.search_centerline_y_m <= c.workspace_max_xyz_m[1] &&
      c.view_entry_y_m >= c.workspace_min_xyz_m[1] &&
      c.view_entry_y_m <= c.workspace_max_xyz_m[1] &&
      lift_target.has_value() &&
      search_height_envelope_in_workspace(
          zero_lift_cap_top_m,
          lift_target->metres,
          c.lift_target_tolerance_m,
          c.lift_minimum_position_m,
          c.lift_maximum_position_m,
          c.view_region.clearance_min_m,
          c.view_region.clearance_max_m,
          c.workspace_min_xyz_m[2],
          c.workspace_max_xyz_m[2]) &&
      std::all_of(
          c.observation_y_positions_m.begin(),
          c.observation_y_positions_m.end(),
          [&c](double lateral) {
            return lateral >= c.workspace_min_xyz_m[1] &&
                lateral <= c.workspace_max_xyz_m[1];
          });
  const bool search_valid =
      std::isfinite(c.search_centerline_y_m) &&
      valid_initial_centerline_tolerance(
          c.initial_centerline_tolerance_m) &&
      std::isfinite(c.centerline_joint_position_rad) &&
      Positive(c.centerline_joint_tolerance_rad) &&
      c.centerline_joint_tolerance_rad <= 0.1 &&
      c.initial_tcp_x_bounds_m[0] <
          c.initial_tcp_x_bounds_m[1] &&
      valid_initial_compact_z_bounds(
          c.initial_tcp_z_bounds_m[0],
          c.initial_tcp_z_bounds_m[1],
          c.phase_one_expected_tcp.z) &&
      std::abs(
          c.phase_two_centerline_y_m -
          c.search_centerline_y_m) <=
          c.initial_centerline_tolerance_m &&
      std::abs(
          c.view_entry_y_m - c.search_centerline_y_m) <=
          c.view_region.y_tolerance_m &&
      agx_xarm_pick::valid_top_down_view_region(c.view_region) &&
      Positive(c.view_planning_time_per_candidate_sec) &&
      agx_xarm_pick::valid_visual_alignment_config(c.visual_alignment) &&
      Positive(c.maximum_recentering_step_m) &&
      Positive(c.minimum_pixel_error_improvement_px) &&
      c.observation_y_positions_m.size() >= 3 &&
      strictly_decreasing(c.observation_y_positions_m) &&
      Positive(c.downward_axis_tolerance_deg) &&
      c.downward_axis_tolerance_deg <=
          c.profile_downward_axis_tolerance_deg &&
      c.minimum_adjacent_view_overlap > 0.0 &&
      c.minimum_adjacent_view_overlap < 1.0 &&
      Positive(c.search_settle_duration_sec) &&
      c.detector_attempts_per_waypoint >= 1 &&
      search_points_in_workspace;
  const bool preparation_valid =
      agx_xarm_pick::valid_phase_one_joint_positions(
          c.phase_one_joint_positions_rad) &&
      (std::isfinite(c.phase_one_expected_tcp.x) && std::isfinite(c.phase_one_expected_tcp.y) && std::isfinite(c.phase_one_expected_tcp.z)) &&
      Positive(c.phase_one_downward_axis_tolerance_deg) &&
      c.phase_one_downward_axis_tolerance_deg <= 90.0 &&
      std::isfinite(c.phase_two_centerline_y_m) &&
      Positive(c.phase_two_downward_axis_tolerance_deg) &&
      c.phase_two_downward_axis_tolerance_deg <= 90.0;
  const bool motion_valid =
      preparation_valid &&
      search_valid &&
      camera_mount_valid &&
      Positive(c.preparation_clearance_above_cap_m) &&
      Positive(c.pregrasp_clearance_m) &&
      Positive(c.grasp_tcp_offset_below_cap_top_m) &&
      Positive(c.lift_clearance_m) &&
      Positive(c.profile_downward_axis_tolerance_deg) &&
      c.profile_downward_axis_tolerance_deg <= 90.0 &&
      Positive(c.planning_timeout_sec) &&
      Positive(c.gripper_timeout_sec) &&
      Positive(c.hold_duration_sec) &&
      Positive(c.robot_state_max_age_sec) &&
      Positive(c.cartesian_step_m) &&
      std::all_of(c.arm_execution.begin(), c.arm_execution.end(),
                  [](const agx_xarm_pick::ArmExecutionScales& scales) {
                    return UnitScaling(scales.velocity) && UnitScaling(scales.acceleration);
                  }) &&
      std::abs(
          c.lift_distance_m -
          (c.body_height_m +
           c.cap_height_m +
           c.lift_clearance_m)) <= 1e-12;
  const bool lift_valid =
      c.lift_service_name.front() == '/' &&
      c.lift_state_topic.front() == '/' &&
      Positive(c.arm_base_height_at_zero_lift_above_floor_m) &&
      std::isfinite(c.lift_minimum_position_m) &&
      c.lift_minimum_position_m >= 0.0 &&
      std::isfinite(c.lift_maximum_position_m) &&
      c.lift_maximum_position_m >=
          c.lift_minimum_position_m &&
      c.lift_target_tolerance_mm >= 0 &&
      c.lift_maximum_speed_request_mm_s > 0 &&
      Positive(c.lift_state_max_age_sec) &&
      Positive(c.lift_motion_timeout_sec) &&
      lift_target.has_value();

  Require(geometry_valid, "invalid rack/container/gripper geometry");
  Require(perception_valid, "invalid perception settings");
  Require(commands_valid, "invalid gripper command calibration");
  Require(valid_visual_alignment_config(c.visual_alignment), "invalid visual alignment settings");
  Require(search_valid, "invalid search bounds or workspace envelope");
  Require(preparation_valid, "invalid preparation settings");
  Require(motion_valid, "invalid motion settings or camera transform");
  Require(lift_valid, "invalid lift geometry or settings");
  Require(result->installation.parent_frame == result->support.frame_id &&
          result->installation.parent_frame == result->robot.planning_frame, "inconsistent installation/support/robot frame");
  Require(result->installation.station_id == result->support.station_id, "inconsistent installation/support station");
  Require(result->behavior.task_type == "test_tube_pickup", "unsupported behavior task_type");
  Require(std::find(result->rack.supported_container_profiles.begin(), result->rack.supported_container_profiles.end(),
                   result->container.id) != result->rack.supported_container_profiles.end(), "rack does not support selected container profile");
  for (const auto& name : {result->robot.detector_service, result->robot.joint_state_topic,
                         result->robot.gripper_command_topic, result->robot.gripper_state_topic})
    Require(name.front() == '/', "robot interfaces must use absolute ROS names");
  for (double value : result->rack.dimensions_xyz_m) Require(Positive(value), "rack dimensions must be positive");
  for (double value : result->support.dimensions_xyz_m) Require(Positive(value), "support dimensions must be positive");
  Require((c.columns-1)*c.center_pitch_m+c.cap_diameter_m <= result->rack.dimensions_xyz_m[0] &&
          (c.rows-1)*c.center_pitch_m+c.cap_diameter_m <= result->rack.dimensions_xyz_m[1], "rack dimensions cannot contain slot lattice");
  const auto& roi = result->behavior.roi_normalized;
  Require(roi[0] >= 0 && roi[1] >= 0 && roi[2] <= 1 && roi[3] <= 1 && roi[0] < roi[2] && roi[1] < roi[3], "invalid normalized perception ROI");
  Require(result->behavior.minimum_layout_saturation >= 1 && result->behavior.minimum_layout_saturation <= 255 &&
          Positive(result->behavior.maximum_target_temporal_spread_px), "invalid layout perception settings");
  Require(c.nominal_fixture_center_xyz_m[0] >= c.workspace_min_xyz_m[0] &&
          c.nominal_fixture_center_xyz_m[0] <= c.workspace_max_xyz_m[0] &&
          c.nominal_fixture_center_xyz_m[1] >= c.workspace_min_xyz_m[1] &&
          c.nominal_fixture_center_xyz_m[1] <= c.workspace_max_xyz_m[1] &&
          std::abs(c.nominal_fixture_center_xyz_m[2] - lift_target->metres) <= c.table_height_tolerance_m &&
          zero_lift_cap_top_m >= c.workspace_min_xyz_m[2] && zero_lift_cap_top_m <= c.workspace_max_xyz_m[2],
          "installation transform inconsistent with support height or workspace");
  Require(Positive(c.maximum_translation_error_m) && Positive(c.maximum_yaw_error_deg) && c.maximum_yaw_error_deg <= 180,
          "invalid registration tolerances");
}
}  // namespace

std::shared_ptr<const ResolvedTubeTaskConfig> ResolveTubeTaskConfig(const std::string& task_file) {
  try {
    Snapshot snapshot;
    auto& task = snapshot.Read(task_file, "tube_task");
    Keys(task.node, {"schema_version", "kind", "id", "installation", "container", "behavior", "robot"});
    auto result = std::make_shared<ResolvedTubeTaskConfig>();
    result->task_config_id = task.id;
    auto& installation = snapshot.Child(task, "installation");
    auto& behavior = snapshot.Child(task, "behavior");
    auto& support = snapshot.Child(installation, "support_surface");
    auto& rack = snapshot.Child(installation, "fixture_profile");
    auto& container = snapshot.Child(task, "container");
    auto& robot = snapshot.Child(task, "robot");
    WithSource(installation, [&] { ParseInstallation(installation, result.get()); });
    WithSource(support, [&] { ParseSupport(support, result.get()); });
    WithSource(rack, [&] { ParseRack(rack, result.get()); });
    WithSource(container, [&] { ParseContainer(container, result.get()); });
    WithSource(behavior, [&] { ParseBehavior(behavior, result.get()); });
    WithSource(robot, [&] { ParseRobot(robot, behavior, result.get()); });
    Validate(result.get());
    std::map<std::string, std::string> facts;
    for (const auto& entry : snapshot.documents) {
      const auto& doc = entry.second;
      result->sources.push_back({doc.kind, doc.id, Sha256(doc.content), doc.path});
      auto normalized = YAML::Clone(doc.node);
      for (const auto& reference : doc.references) {
        const auto& child = snapshot.documents.at(reference.second);
        normalized[reference.first] = child.kind + ":" + child.id;
      }
      facts.emplace(doc.kind + ":" + doc.id, Canonical(normalized));
    }
    std::sort(result->sources.begin(), result->sources.end(), [](const ConfigSourceIdentity& a, const ConfigSourceIdentity& b) {
      return std::tie(a.kind, a.id) < std::tie(b.kind, b.id);
    });
    std::string canonical = "resolved_tube_task:1;";
    for (const auto& fact : facts) canonical += std::to_string(fact.second.size()) + ":" + fact.second;
    result->config_id = "sha256:" + Sha256(canonical);
    return result;
  } catch (const std::exception& error) {
    throw std::invalid_argument("tube task configuration " + task_file + ": " + error.what());
  }
}
}  // namespace agx_xarm_pick
