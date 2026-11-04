#include "agx_place_tasks/onboard_rack_profile.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>

#include "agx_task_workflow/config.hpp"

namespace agx_place_tasks {
namespace {

constexpr std::size_t kMaximumSlots = 64;

// yaml-cpp tags every explicitly quoted scalar "!"; those are strings, not
// numbers, even when the text looks numeric.
bool Quoted(const YAML::Node& node) {
  return node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str";
}

void RequireUnit(const Point3& value, const std::string& label) {
  const double norm = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  if (!std::isfinite(norm) || std::fabs(norm - 1.0) > 1e-3)
    throw std::invalid_argument(label + " must be a unit vector");
}

Point3 ReadPoint(const YAML::Node& node, const std::string& label) {
  if (!node.IsSequence() || node.size() != 3)
    throw std::invalid_argument(label + " must be a sequence of three finite numbers");
  double values[3] = {0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 3; ++i) {
    const auto item = node[i];
    if (!item.IsScalar() || Quoted(item))
      throw std::invalid_argument(label + " must be a sequence of three finite numbers");
    try {
      values[i] = item.as<double>();
    } catch (const YAML::Exception&) {
      throw std::invalid_argument(label + " must be a sequence of three finite numbers");
    }
    if (!std::isfinite(values[i]))
      throw std::invalid_argument(label + " must be a sequence of three finite numbers");
  }
  return Point3{values[0], values[1], values[2]};
}

double ReadNumber(const YAML::Node& node, const std::string& label) {
  if (!node.IsScalar() || Quoted(node)) throw std::invalid_argument(label + " must be a finite number");
  double value = 0.0;
  try {
    value = node.as<double>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(label + " must be a finite number");
  }
  if (!std::isfinite(value)) throw std::invalid_argument(label + " must be a finite number");
  return value;
}

bool ReadBool(const YAML::Node& node, const std::string& label) {
  if (!node.IsScalar() || Quoted(node)) throw std::invalid_argument(label + " must be a boolean");
  try {
    return node.as<bool>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(label + " must be a boolean");
  }
}

std::size_t ReadSlotId(const YAML::Node& node, const std::string& label) {
  if (!node.IsScalar() || Quoted(node))
    throw std::invalid_argument(label + " must be a positive slot number");
  long long value = 0;
  try {
    value = node.as<long long>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(label + " must be a positive slot number");
  }
  if (value < 1 || value > static_cast<long long>(kMaximumSlots))
    throw std::invalid_argument(label + " must be a slot number between 1 and " +
                                std::to_string(kMaximumSlots));
  return static_cast<std::size_t>(value);
}

bool InRange(double value, double low, double high) {
  return std::isfinite(value) && value >= low && value <= high;
}

}  // namespace

const RackSlot* OnboardRackProfile::Find(std::size_t slot) const {
  for (const auto& entry : slots)
    if (entry.slot == slot) return &entry;
  return nullptr;
}

Point3 OnboardRackProfile::SlotCenter(std::size_t slot) const {
  const auto* entry = Find(slot);
  if (!entry) throw std::out_of_range("slot " + std::to_string(slot) + " is not taught in this rack");
  return entry->center_m;
}

Point3 OnboardRackProfile::ApproachPoint(std::size_t slot) const {
  const auto center = SlotCenter(slot);
  return Point3{center.x - approach_rise_m * descend_axis.x,
                center.y - approach_rise_m * descend_axis.y,
                center.z - approach_rise_m * descend_axis.z};
}

bool Validate(const OnboardRackProfile& profile, std::string* detail) {
  auto fail = [&](const std::string& reason) {
    if (detail) *detail = reason;
    return false;
  };
  if (profile.profile_id.empty()) return fail("profile_id must be nonempty");
  if (profile.parent_frame.empty()) return fail("parent_frame must be nonempty");
  if (profile.tip_link.empty()) return fail("tip_link must be nonempty");
  if (profile.slots.empty()) return fail("at least one taught slot is required");
  if (profile.slots.size() > kMaximumSlots)
    return fail("at most " + std::to_string(kMaximumSlots) + " slots are supported");
  std::set<std::size_t> seen;
  for (const auto& slot : profile.slots) {
    if (slot.slot < 1 || slot.slot > kMaximumSlots)
      return fail("slot numbers must be between 1 and " + std::to_string(kMaximumSlots));
    if (!seen.insert(slot.slot).second)
      return fail("slot " + std::to_string(slot.slot) + " is listed twice");
    if (slot.name.empty()) return fail("slot " + std::to_string(slot.slot) + " has no name");
    const auto& center = slot.center_m;
    if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z))
      return fail("slot " + std::to_string(slot.slot) + " centre must be finite");
    if (std::fabs(center.x) > 2.0 || std::fabs(center.y) > 2.0 || std::fabs(center.z) > 2.0)
      return fail("slot " + std::to_string(slot.slot) + " centre is outside the recorded envelope");
  }
  // Gaps would make "slot: N" ambiguous between the teaching record and the
  // recipe, so the numbering must be exactly 1..N.
  for (std::size_t expected = 1; expected <= profile.slots.size(); ++expected)
    if (!seen.count(expected))
      return fail("slot numbering must be contiguous from 1; missing slot " + std::to_string(expected));
  if (!InRange(profile.approach_rise_m, 0.005, 0.50))
    return fail("approach_rise_m must be between 0.005 and 0.50 m");
  if (profile.slot_tolerance_m && !InRange(*profile.slot_tolerance_m, 0.0, 0.05))
    return fail("slot_tolerance_m must be between 0 and 0.05 m");
  if (profile.tool_yaw_offset_deg && !InRange(*profile.tool_yaw_offset_deg, -180.0, 180.0))
    return fail("tool_yaw_offset_deg must be between -180 and 180 degrees");
  if (profile.tool_rpy_deg) {
    for (double angle : *profile.tool_rpy_deg)
      if (!InRange(angle, -720.0, 720.0)) return fail("tool_rpy_deg must be three finite angles");
  }
  try {
    RequireUnit(profile.descend_axis, "descend_axis");
  } catch (const std::invalid_argument& error) {
    return fail(error.what());
  }
  if (profile.qualified && (profile.evidence_id.empty() || profile.evidence_id == "unqualified"))
    return fail("a qualified rack profile must name its qualification evidence");
  if (detail) detail->clear();
  return true;
}

OnboardRackProfile ParseRackProfile(const YAML::Node& document, const std::string& profile_id) {
  if (!document.IsMap()) throw std::invalid_argument("onboard rack file must be a mapping");
  agx_task_workflow::RequireKeys(document, {"schema_version", "onboard_rack_profiles"},
                                 "onboard rack file");
  const auto version = document["schema_version"];
  if (!version.IsScalar() || Quoted(version) || version.as<long long>() != 2)
    throw std::invalid_argument("onboard rack file requires schema_version: 2");
  const auto profiles = document["onboard_rack_profiles"];
  if (!profiles.IsMap()) throw std::invalid_argument("onboard_rack_profiles must be a mapping");
  const auto node = profiles[profile_id];
  if (!node || !node.IsMap())
    throw std::invalid_argument("onboard rack profile '" + profile_id + "' is not defined");
  const auto label = "onboard rack profile '" + profile_id + "'";
  std::set<std::string> fields{"parent_frame", "tip_link", "slots", "descend_axis", "approach_rise_m",
                               "qualified", "evidence_id"};
  if (node["tool_rpy_deg"]) fields.insert("tool_rpy_deg");
  if (node["slot_tolerance_m"]) fields.insert("slot_tolerance_m");
  if (node["tool_yaw_offset_deg"]) fields.insert("tool_yaw_offset_deg");
  agx_task_workflow::RequireKeys(node, fields, label);

  OnboardRackProfile profile;
  profile.profile_id = profile_id;
  profile.parent_frame = agx_task_workflow::RequireString(node["parent_frame"], label + ".parent_frame");
  profile.tip_link = agx_task_workflow::RequireString(node["tip_link"], label + ".tip_link");
  profile.descend_axis = ReadPoint(node["descend_axis"], label + ".descend_axis");
  profile.approach_rise_m = ReadNumber(node["approach_rise_m"], label + ".approach_rise_m");

  const auto slots = node["slots"];
  if (!slots.IsMap() || slots.size() == 0)
    throw std::invalid_argument(label + ".slots must be a nonempty mapping of taught slots");
  for (const auto& entry : slots) {
    const auto slot_id = ReadSlotId(entry.first, label + ".slots key");
    const auto slot_label = label + ".slots." + std::to_string(slot_id);
    if (!entry.second.IsMap())
      throw std::invalid_argument(slot_label + " must be a mapping");
    agx_task_workflow::RequireKeys(entry.second, {"name", "center_m"}, slot_label);
    RackSlot slot;
    slot.slot = slot_id;
    slot.name = agx_task_workflow::RequireString(entry.second["name"], slot_label + ".name");
    slot.center_m = ReadPoint(entry.second["center_m"], slot_label + ".center_m");
    profile.slots.push_back(slot);
  }
  std::sort(profile.slots.begin(), profile.slots.end(),
            [](const RackSlot& a, const RackSlot& b) { return a.slot < b.slot; });

  if (node["tool_rpy_deg"]) {
    const auto rpy = ReadPoint(node["tool_rpy_deg"], label + ".tool_rpy_deg");
    profile.tool_rpy_deg = std::array<double, 3>{{rpy.x, rpy.y, rpy.z}};
  }
  if (node["slot_tolerance_m"])
    profile.slot_tolerance_m = ReadNumber(node["slot_tolerance_m"], label + ".slot_tolerance_m");
  if (node["tool_yaw_offset_deg"])
    profile.tool_yaw_offset_deg =
        ReadNumber(node["tool_yaw_offset_deg"], label + ".tool_yaw_offset_deg");
  profile.qualified = ReadBool(node["qualified"], label + ".qualified");
  profile.evidence_id = agx_task_workflow::RequireString(node["evidence_id"], label + ".evidence_id");

  std::string detail;
  if (!Validate(profile, &detail)) throw std::invalid_argument(label + ": " + detail);
  return profile;
}

OnboardRackProfile LoadRackProfile(const std::string& path, const std::string& profile_id) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read onboard rack file: " + path);
  const auto documents = YAML::LoadAll(std::string(std::istreambuf_iterator<char>(stream), {}));
  if (documents.size() != 1) throw std::invalid_argument("exactly one onboard rack YAML document is required");
  return ParseRackProfile(documents.front(), profile_id);
}

}  // namespace agx_place_tasks
