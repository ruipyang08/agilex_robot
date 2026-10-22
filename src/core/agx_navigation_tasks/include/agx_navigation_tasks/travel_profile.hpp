#pragma once
#include <array>
#include <map>
#include <string>
#include <vector>
#include "agx_xarm_pick/tube_live_observer.hpp"

namespace YAML { class Node; }
namespace agx_navigation_tasks {
enum class TravelMode { Simulation, Hardware };
struct TravelQualification { bool enabled = false; std::string evidence_id; };
struct TravelJointTarget {
  std::string name;
  double position_rad = 0, tolerance_rad = 0, stopped_velocity_rad_s = 0;
};
struct TravelStartPose {
  std::string id, path_id;
  std::map<std::string, double> arm_positions_rad;
  double lift_position_m = 0;
  bool empty_allowed = false, single_held_allowed = false;
  std::map<std::string, double> arm_tolerances_rad;
  double lift_tolerance_m = 0;
};
struct TravelRobotConfig {
  std::string arm_group, arm_joint_state_topic, gripper_state_topic, lift_service, lift_state_topic;
  agxcore::TubeObserverConfig observer;
};
struct TravelPoint { double x_m = 0, y_m = 0; };
struct TravelBaseEnvelope {
  std::string id;
  std::vector<TravelPoint> footprint;
  double linear_velocity_mps = 0, angular_velocity_rad_s = 0;
  double linear_acceleration_mps2 = 0, angular_acceleration_rad_s2 = 0;
};
struct TravelProfile {
  std::string profile_id, config_id;
  std::string source_path;  // Provenance only; never included in the digest.
  TravelRobotConfig robot;
  TravelQualification simulation, hardware;
  std::vector<TravelJointTarget> arm;
  double lift_position_m = 0, lift_tolerance_m = 0, lift_stopped_velocity_mps = 0;
  double arm_motion_velocity_rad_s = 0, arm_motion_acceleration_rad_s2 = 0;
  double lift_motion_velocity_mps = 0;
  std::string container_profile_id;
  double held_max_mass_kg = 0, held_max_radius_m = 0, held_max_length_m = 0;
  std::string held_attachment_link;
  std::array<double,3> held_center_xyz_m{};
  std::array<double,4> held_orientation_xyzw{};
  std::vector<std::string> held_touch_links;
  TravelBaseEnvelope base;
  double source_max_age_s = 0, receipt_max_age_s = 0, settle_s = 0, preparation_timeout_s = 0;
  std::vector<TravelStartPose> starts;
};
// Parsing/validation perform no I/O and never enable an unqualified mode.
TravelProfile LoadTravelProfile(const YAML::Node& node);
TravelProfile ParseTravelProfile(const std::string& text, const std::string& source_path = {});
void ValidateTravelProfile(const TravelProfile& profile);
bool TravelEnabled(const TravelProfile& profile, TravelMode mode);
}  // namespace agx_navigation_tasks
