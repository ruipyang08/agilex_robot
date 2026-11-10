#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>

#include <yaml-cpp/yaml.h>

namespace agx_place_tasks {

// One measured pick-and-place pose. The approach height is per position
// because the row nearest the robot reaches the wrist limit and needs a
// shorter straight descent.
struct PositionTarget {
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double grasp_z = 0.0;
  double approach_z = 0.0;
};

// Table of measured positions for the rack mounted on this robot. Loaded from
// config/green_tube_grasp.yaml; every entry was confirmed on hardware.
struct PositionTable {
  std::string frame;
  std::string default_name;
  double grasp_z = 0.0;
  double approach_z = 0.0;
  std::array<double, 3> tool_rpy_deg{{180.0, 0.0, 0.0}};
  double tool_yaw_offset_deg = 0.0;
  double velocity_scaling = 0.05;
  double acceleration_scaling = 0.05;
  double eef_step_m = 0.005;
  double open_before_descent = 750.0;
  double close_on_cap = 300.0;
  double gripper_force = 50.0;
  double gripper_speed = 50.0;
  std::map<std::string, PositionTarget> positions;

  const PositionTarget* Find(const std::string& name) const;
};

bool ValidatePositions(const PositionTable& table, std::string* detail);
PositionTable ParsePositions(const YAML::Node& document, const std::string& source);
PositionTable LoadPositions(const std::string& path);

}  // namespace agx_place_tasks
