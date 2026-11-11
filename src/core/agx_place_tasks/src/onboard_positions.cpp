#include "agx_place_tasks/onboard_positions.hpp"

#include <cmath>
#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>

namespace agx_place_tasks {
namespace {

bool Quoted(const YAML::Node& node) {
  return node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str";
}

double Number(const YAML::Node& node, const std::string& label) {
  if (!node.IsScalar() || Quoted(node)) throw std::invalid_argument(label + " must be a number");
  double value = 0.0;
  try {
    value = node.as<double>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(label + " must be a number");
  }
  if (!std::isfinite(value)) throw std::invalid_argument(label + " must be finite");
  return value;
}

std::array<double, 2> Pair(const YAML::Node& node, const std::string& label) {
  if (!node.IsSequence() || node.size() != 2)
    throw std::invalid_argument(label + " must be a pair of numbers");
  return {{Number(node[0], label), Number(node[1], label)}};
}

std::array<double, 3> Triple(const YAML::Node& node, const std::string& label) {
  if (!node.IsSequence() || node.size() != 3)
    throw std::invalid_argument(label + " must be three numbers");
  return {{Number(node[0], label), Number(node[1], label), Number(node[2], label)}};
}

std::string Text(const YAML::Node& node, const std::string& label) {
  if (!node.IsScalar() || !Quoted(node) == false) {
    // yaml-cpp marks plain scalars without a tag; accept any non-empty scalar.
  }
  if (!node.IsScalar()) throw std::invalid_argument(label + " must be a string");
  const auto value = node.Scalar();
  if (value.empty()) throw std::invalid_argument(label + " must be a non-empty string");
  return value;
}

}  // namespace

const PositionTarget* PositionTable::Find(const std::string& name) const {
  const auto found = positions.find(name);
  return found == positions.end() ? nullptr : &found->second;
}

bool ValidatePositions(const PositionTable& table, std::string* detail) {
  auto fail = [&](const std::string& reason) {
    if (detail) *detail = reason;
    return false;
  };
  if (table.frame.empty()) return fail("frame must be nonempty");
  if (table.positions.empty()) return fail("at least one position is required");
  if (!std::isfinite(table.grasp_z)) return fail("grasp_z must be finite");
  if (!std::isfinite(table.approach_z)) return fail("approach_z must be finite");
  if (table.approach_z <= table.grasp_z)
    return fail("approach_z must be above grasp_z");
  if (table.velocity_scaling <= 0.0 || table.velocity_scaling > 1.0)
    return fail("velocity_scaling must be inside (0, 1]");
  if (table.acceleration_scaling <= 0.0 || table.acceleration_scaling > 1.0)
    return fail("acceleration_scaling must be inside (0, 1]");
  if (table.eef_step_m <= 0.0 || table.eef_step_m > 0.05)
    return fail("eef_step_m must be inside (0, 0.05]");
  if (table.close_on_cap <= 0.0 || table.close_on_cap > 1000.0)
    return fail("close_on_cap must be inside (0, 1000]");
  if (table.held_margin <= 0.0)
    return fail("held_margin must be positive");
  if (!table.default_name.empty() && !table.Find(table.default_name))
    return fail("default names a position that is not listed");
  for (const auto& entry : table.positions) {
    const auto& target = entry.second;
    if (entry.first.empty()) return fail("position names must be nonempty");
    if (!std::isfinite(target.x) || !std::isfinite(target.y))
      return fail(entry.first + ": xy must be finite");
    if (std::fabs(target.x) > 2.0 || std::fabs(target.y) > 2.0)
      return fail(entry.first + ": xy is outside the recorded envelope");
    if (!std::isfinite(target.grasp_z) || !std::isfinite(target.approach_z))
      return fail(entry.first + ": z must be finite");
    if (target.approach_z <= target.grasp_z)
      return fail(entry.first + ": approach_z must be above grasp_z");
    if (target.place_z > target.approach_z)
      return fail(entry.first + ": place_z must not be above approach_z");
  }
  if (detail) detail->clear();
  return true;
}

PositionTable ParsePositions(const YAML::Node& document, const std::string& source) {
  // yaml-cpp throws YAML::Exception for missing or mistyped nodes; callers must
  // see a configuration error instead of an uncaught library exception.
  auto parse = [&]() -> PositionTable {
  if (!document.IsMap()) throw std::invalid_argument(source + " must be a mapping");
  const auto root_node = document["green_tube"];
  if (!root_node || !root_node.IsMap())
    throw std::invalid_argument(source + " requires a green_tube mapping");

  PositionTable table;
  table.frame = Text(root_node["frame"], source + ".frame");
  table.grasp_z = Number(root_node["grasp_z_m"], source + ".grasp_z_m");
  table.approach_z = Number(root_node["approach_z_m"], source + ".approach_z_m");
  if (root_node["default"]) table.default_name = Text(root_node["default"], source + ".default");
  if (root_node["tool_rpy_deg"])
    table.tool_rpy_deg = Triple(root_node["tool_rpy_deg"], source + ".tool_rpy_deg");
  if (root_node["tool_yaw_offset_deg"])
    table.tool_yaw_offset_deg = Number(root_node["tool_yaw_offset_deg"], source + ".tool_yaw_offset_deg");
  if (root_node["motion"]) {
    const auto motion = root_node["motion"];
    if (motion["velocity_scaling"])
      table.velocity_scaling = Number(motion["velocity_scaling"], source + ".motion.velocity_scaling");
    if (motion["acceleration_scaling"])
      table.acceleration_scaling =
          Number(motion["acceleration_scaling"], source + ".motion.acceleration_scaling");
    if (motion["eef_step_m"]) table.eef_step_m = Number(motion["eef_step_m"], source + ".motion.eef_step_m");
  }
  if (root_node["gripper"]) {
    const auto gripper = root_node["gripper"];
    if (gripper["open_before_descent"])
      table.open_before_descent =
          Number(gripper["open_before_descent"], source + ".gripper.open_before_descent");
    if (gripper["close_on_cap"])
      table.close_on_cap = Number(gripper["close_on_cap"], source + ".gripper.close_on_cap");
    if (gripper["held_margin"])
      table.held_margin = Number(gripper["held_margin"], source + ".gripper.held_margin");
    if (gripper["force"]) table.gripper_force = Number(gripper["force"], source + ".gripper.force");
    if (gripper["speed"]) table.gripper_speed = Number(gripper["speed"], source + ".gripper.speed");
  }

  const auto positions = root_node["positions"];
  if (!positions || !positions.IsMap() || positions.size() == 0)
    throw std::invalid_argument(source + ".positions must be a nonempty mapping");
  for (const auto& entry : positions) {
    const auto name = entry.first.as<std::string>();
    const auto node = entry.second;
    PositionTarget target;
    target.name = name;
    target.grasp_z = table.grasp_z;
    target.approach_z = table.approach_z;
    target.place_z = table.grasp_z;
    if (node.IsSequence()) {
      const auto xy = Pair(node, source + ".positions." + name);
      target.x = xy[0];
      target.y = xy[1];
    } else if (node.IsMap()) {
      std::set<std::string> allowed{"xy", "approach_z_m", "grasp_z_m", "place_z_m"};
      for (const auto& key : node)
        if (!allowed.count(key.first.as<std::string>()))
          throw std::invalid_argument(source + ".positions." + name + " has an unknown key");
      const auto xy = Pair(node["xy"], source + ".positions." + name + ".xy");
      target.x = xy[0];
      target.y = xy[1];
      if (node["approach_z_m"])
        target.approach_z = Number(node["approach_z_m"], source + ".positions." + name + ".approach_z_m");
      if (node["grasp_z_m"])
        target.grasp_z = Number(node["grasp_z_m"], source + ".positions." + name + ".grasp_z_m");
      if (node["place_z_m"])
        target.place_z = Number(node["place_z_m"], source + ".positions." + name + ".place_z_m");
    } else {
      throw std::invalid_argument(source + ".positions." + name + " must be [x, y] or a mapping");
    }
    table.positions.emplace(name, target);
  }

  std::string detail;
  if (!ValidatePositions(table, &detail)) throw std::invalid_argument(source + ": " + detail);
  return table;
  };
  try {
    return parse();
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument(source + ": " + error.what());
  }
}

PositionTable LoadPositions(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read positions file: " + path);
  const auto documents = YAML::LoadAll(std::string(std::istreambuf_iterator<char>(stream), {}));
  if (documents.size() != 1) throw std::invalid_argument("exactly one positions document is required");
  return ParsePositions(documents.front(), path);
}

}  // namespace agx_place_tasks
