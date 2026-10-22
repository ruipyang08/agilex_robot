#include "agx_navigation_tasks/travel_profile.hpp"
#include <cmath>
#include <cctype>
#include <set>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>
#include "agx_task_workflow/config.hpp"
namespace agx_navigation_tasks {
namespace {
void Require(bool value, const std::string& detail) {
  if (!value) throw std::invalid_argument("travel profile: " + detail);
}
bool Text(const std::string& value) { return value.find_first_not_of(" \t\r\n") != std::string::npos; }
bool Positive(double value) { return std::isfinite(value) && value > 0; }
bool LinkName(const std::string& name) {
  if (name.empty() || !(std::isalpha(static_cast<unsigned char>(name.front())) || name.front() == '_')) return false;
  for (const unsigned char c : name)
    if (!(std::isalnum(c) || c == '_' || c == '/')) return false;
  return name.back() != '/' && name.find("//") == std::string::npos;
}
double Number(const YAML::Node& node, const std::string& label) {
  Require(node.IsScalar() && node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str", label + " must be numeric");
  const auto value = node.as<double>();
  Require(std::isfinite(value), label + " must be finite");
  return value;
}
bool Boolean(const YAML::Node& node) {
  Require(node.IsScalar() && (node.Tag() == "?" || node.Tag() == "tag:yaml.org,2002:bool") &&
      (node.Scalar() == "true" || node.Scalar() == "false"), "boolean must be true or false");
  return node.Scalar() == "true";
}
TravelQualification Qualification(const YAML::Node& node) {
  agx_task_workflow::RequireKeys(node,{"enabled","evidence_id"},"travel qualification");
  return {Boolean(node["enabled"]),agx_task_workflow::RequireString(node["evidence_id"],"evidence_id")};
}
std::string Digest(const std::string& text) {
  unsigned char digest[EVP_MAX_MD_SIZE]; unsigned int size=0;
  Require(EVP_Digest(text.data(),text.size(),digest,&size,EVP_sha256(),nullptr) == 1,"SHA256 failed");
  std::ostringstream out; out << "sha256:" << std::hex << std::setfill('0');
  for (unsigned int i=0;i<size;++i) out << std::setw(2) << static_cast<int>(digest[i]);
  return out.str();
}
}
bool TravelEnabled(const TravelProfile& p, TravelMode mode) {
  switch (mode) {
    case TravelMode::Simulation: return p.simulation.enabled;
    case TravelMode::Hardware: return p.hardware.enabled;
  }
  throw std::invalid_argument("unknown travel execution mode");
}
void ValidateTravelProfile(const TravelProfile& p) {
  Require(Text(p.profile_id) && Text(p.config_id), "profile_id and config_id are required");
  Require(!p.simulation.enabled || Text(p.simulation.evidence_id), "simulation qualification evidence is required");
  Require(!p.hardware.enabled || Text(p.hardware.evidence_id), "hardware qualification evidence is required");
  if (p.arm.empty() && !p.simulation.enabled && !p.hardware.enabled) return;
  Require(Text(p.robot.arm_group) && Text(p.robot.arm_joint_state_topic) && Text(p.robot.gripper_state_topic) &&
      Text(p.robot.lift_service) && Text(p.robot.lift_state_topic),"all frozen robot interfaces are required");
  const auto& observer=p.robot.observer;
  for (double value : {observer.gripper_max_age_s,observer.arm_max_age_s,observer.lift_max_age_s,
      observer.command_tolerance,observer.stopped_velocity_rad_s,observer.stop_settle_s,
      observer.stopped_arm_range_rad,observer.stopped_lift_range_m})
    Require(Positive(value),"observer freshness, command and measured stop limits must be explicit and positive");
  Require(std::isfinite(observer.fully_open_command) && std::isfinite(observer.close_command) &&
      std::abs(observer.fully_open_command-observer.close_command) > 2*observer.command_tolerance,
      "gripper open/closed observations must be distinct");
  Require(!p.arm.empty(), "qualified named arm targets are required");
  std::set<std::string> names;
  for (const auto& joint : p.arm) {
    Require(Text(joint.name) && names.insert(joint.name).second, "arm names must be unique and nonempty");
    Require(std::isfinite(joint.position_rad) && Positive(joint.tolerance_rad) &&
        Positive(joint.stopped_velocity_rad_s), "invalid arm target/tolerance/stopped speed");
  }
  Require(std::isfinite(p.lift_position_m) && p.lift_position_m >= 0, "invalid lift target");
  for (double value : {p.lift_tolerance_m,p.lift_stopped_velocity_mps,p.arm_motion_velocity_rad_s,
      p.arm_motion_acceleration_rad_s2,p.lift_motion_velocity_mps,p.held_max_mass_kg,
      p.held_max_radius_m,p.held_max_length_m,p.source_max_age_s,p.receipt_max_age_s,
      p.settle_s,p.preparation_timeout_s,p.base.linear_velocity_mps,p.base.angular_velocity_rad_s,
      p.base.linear_acceleration_mps2,p.base.angular_acceleration_rad_s2})
    Require(Positive(value), "all motion, load, freshness and timing bounds must be finite and positive");
  Require(p.settle_s < p.preparation_timeout_s, "settle must be shorter than preparation deadline");
  Require(p.preparation_timeout_s <= 3600.0,"preparation deadline exceeds the bounded worker software limit");
  Require(Text(p.container_profile_id) && Text(p.base.id), "load and shared base envelope IDs are required");
  Require(LinkName(p.held_attachment_link), "held attachment_link must be a valid named robot link");
  for (const auto value : p.held_center_xyz_m) Require(std::isfinite(value),"held cylinder center must be finite");
  double orientation_norm=0;
  for (const auto value : p.held_orientation_xyzw) {
    Require(std::isfinite(value),"held cylinder orientation must be finite"); orientation_norm+=value*value;
  }
  Require(std::isfinite(orientation_norm) && std::abs(orientation_norm-1.0) <= 1e-6,
      "held cylinder orientation must be a normalized quaternion");
  std::set<std::string> touch_links;
  Require(!p.held_touch_links.empty(),"explicit gripper-only touch_links are required");
  for (const auto& link : p.held_touch_links)
    Require(LinkName(link) && touch_links.insert(link).second,"touch_links must be unique valid robot link names");
  Require(p.base.footprint.size() >= 3, "shared base footprint requires at least three vertices");
  double orientation = 0;
  for (std::size_t i = 0; i < p.base.footprint.size(); ++i) {
    const auto a=p.base.footprint[i], b=p.base.footprint[(i+1)%p.base.footprint.size()];
    const auto c=p.base.footprint[(i+2)%p.base.footprint.size()];
    Require(std::isfinite(a.x_m) && std::isfinite(a.y_m), "footprint must be finite");
    const double turn=(b.x_m-a.x_m)*(c.y_m-b.y_m)-(b.y_m-a.y_m)*(c.x_m-b.x_m);
    Require(std::isfinite(turn) && turn != 0, "footprint must be strictly convex");
    if (orientation == 0) orientation=turn;
    Require((turn > 0) == (orientation > 0), "footprint must be convex and ordered");
    const double origin_side=(b.x_m-a.x_m)*(-a.y_m)-(b.y_m-a.y_m)*(-a.x_m);
    Require(origin_side != 0 && ((origin_side > 0) == (orientation > 0)), "footprint must contain base origin");
    for (std::size_t k=0;k<p.base.footprint.size();++k) {
      if (k == i || k == (i+1)%p.base.footprint.size()) continue;
      const auto point=p.base.footprint[k];
      const double side=(b.x_m-a.x_m)*(point.y_m-a.y_m)-(b.y_m-a.y_m)*(point.x_m-a.x_m);
      Require(std::isfinite(side) && side != 0 && ((side > 0) == (orientation > 0)),"footprint may not intersect or repeat vertices");
    }
  }
  std::set<std::string> starts;
  bool empty=false, held=false;
  for (const auto& start : p.starts) {
    Require(Text(start.id) && starts.insert(start.id).second && Text(start.path_id), "unique named starts and qualified path IDs are required");
    Require(start.arm_positions_rad.size() == names.size() && start.arm_tolerances_rad.size() == names.size(),
        "start must specify every named arm joint and tolerance");
    for (const auto& name : names)
      Require(start.arm_positions_rad.count(name) && std::isfinite(start.arm_positions_rad.at(name)) &&
          start.arm_tolerances_rad.count(name) && Positive(start.arm_tolerances_rad.at(name)), "invalid named starting arm pose/tolerance");
    Require(std::isfinite(start.lift_position_m) && start.lift_position_m >= 0 && Positive(start.lift_tolerance_m), "invalid starting lift pose/tolerance");
    Require(start.empty_allowed || start.single_held_allowed, "starting path must support a declared load");
    empty=empty || start.empty_allowed; held=held || start.single_held_allowed;
  }
  Require(empty || held, "profile must qualify at least one empty or single-held start");
}
TravelProfile LoadTravelProfile(const YAML::Node& node) {
  agx_task_workflow::RequireKeys(node,{"schema_version","profile_id","robot","simulation","hardware","parameters"},"travel profile");
  Require(node["schema_version"].IsScalar() && node["schema_version"].Scalar() == "1" &&
      (node["schema_version"].Tag() == "?" || node["schema_version"].Tag() == "tag:yaml.org,2002:int"), "schema_version must be 1");
  TravelProfile p;
  p.profile_id=agx_task_workflow::RequireString(node["profile_id"],"profile_id"); p.config_id=Digest(YAML::Dump(node));
  p.simulation=Qualification(node["simulation"]); p.hardware=Qualification(node["hardware"]);
  const auto n=node["parameters"];
  if (n.IsNull()) {
    Require(node["robot"].IsNull(),"disabled placeholder robot and parameters must both be null");
    ValidateTravelProfile(p); return p;
  }
  auto robot=node["robot"];
  agx_task_workflow::RequireKeys(robot,{"arm_group","arm_joint_state_topic","gripper_state_topic","lift_service","lift_state_topic","observer"},"travel robot");
  p.robot.arm_group=agx_task_workflow::RequireString(robot["arm_group"],"arm_group");
  p.robot.arm_joint_state_topic=agx_task_workflow::RequireString(robot["arm_joint_state_topic"],"arm_joint_state_topic");
  p.robot.gripper_state_topic=agx_task_workflow::RequireString(robot["gripper_state_topic"],"gripper_state_topic");
  p.robot.lift_service=agx_task_workflow::RequireString(robot["lift_service"],"lift_service");
  p.robot.lift_state_topic=agx_task_workflow::RequireString(robot["lift_state_topic"],"lift_state_topic");
  auto observer=robot["observer"];
  agx_task_workflow::RequireKeys(observer,{"gripper_max_age_s","arm_max_age_s","lift_max_age_s","fully_open_command","close_command",
      "command_tolerance","stopped_velocity_rad_s","stop_settle_s","stopped_arm_range_rad","stopped_lift_range_m"},"travel observer");
  p.robot.observer={Number(observer["gripper_max_age_s"],"gripper_max_age_s"),Number(observer["arm_max_age_s"],"arm_max_age_s"),
      Number(observer["lift_max_age_s"],"lift_max_age_s"),Number(observer["fully_open_command"],"fully_open_command"),
      Number(observer["close_command"],"close_command"),Number(observer["command_tolerance"],"command_tolerance"),
      Number(observer["stopped_velocity_rad_s"],"stopped_velocity_rad_s"),Number(observer["stop_settle_s"],"stop_settle_s"),
      Number(observer["stopped_arm_range_rad"],"stopped_arm_range_rad"),Number(observer["stopped_lift_range_m"],"stopped_lift_range_m")};
  agx_task_workflow::RequireKeys(n,{"arm","lift","motion","held_load","base","timing","starts"},"travel parameters");
  Require(n["arm"].IsSequence(),"arm must be a sequence");
  for (const auto& joint : n["arm"]) {
    agx_task_workflow::RequireKeys(joint,{"name","position_rad","tolerance_rad","stopped_velocity_rad_s"},"travel arm joint");
    p.arm.push_back({agx_task_workflow::RequireString(joint["name"],"joint name"),Number(joint["position_rad"],"position_rad"),
        Number(joint["tolerance_rad"],"tolerance_rad"),Number(joint["stopped_velocity_rad_s"],"stopped_velocity_rad_s")});
  }
  auto lift=n["lift"];
  agx_task_workflow::RequireKeys(lift,{"position_m","tolerance_m","stopped_velocity_mps"},"travel lift");
  p.lift_position_m=Number(lift["position_m"],"lift position"); p.lift_tolerance_m=Number(lift["tolerance_m"],"lift tolerance");
  p.lift_stopped_velocity_mps=Number(lift["stopped_velocity_mps"],"lift stopped speed");
  auto motion=n["motion"];
  agx_task_workflow::RequireKeys(motion,{"arm_velocity_rad_s","arm_acceleration_rad_s2","lift_velocity_mps"},"travel motion");
  p.arm_motion_velocity_rad_s=Number(motion["arm_velocity_rad_s"],"arm motion speed");
  p.arm_motion_acceleration_rad_s2=Number(motion["arm_acceleration_rad_s2"],"arm motion acceleration");
  p.lift_motion_velocity_mps=Number(motion["lift_velocity_mps"],"lift motion speed");
  auto load=n["held_load"];
  agx_task_workflow::RequireKeys(load,{"container_profile_id","max_mass_kg","max_radius_m","max_length_m",
      "attachment_link","center_xyz_m","orientation_xyzw","touch_links"},"travel held load");
  p.container_profile_id=agx_task_workflow::RequireString(load["container_profile_id"],"container_profile_id");
  p.held_max_mass_kg=Number(load["max_mass_kg"],"max_mass_kg"); p.held_max_radius_m=Number(load["max_radius_m"],"max_radius_m");
  p.held_max_length_m=Number(load["max_length_m"],"max_length_m");
  p.held_attachment_link=agx_task_workflow::RequireString(load["attachment_link"],"attachment_link");
  Require(load["center_xyz_m"].IsSequence() && load["center_xyz_m"].size() == 3,"center_xyz_m must contain exactly three numbers");
  Require(load["orientation_xyzw"].IsSequence() && load["orientation_xyzw"].size() == 4,"orientation_xyzw must contain exactly four numbers");
  for (std::size_t i=0;i<3;++i) p.held_center_xyz_m[i]=Number(load["center_xyz_m"][i],"center_xyz_m");
  for (std::size_t i=0;i<4;++i) p.held_orientation_xyzw[i]=Number(load["orientation_xyzw"][i],"orientation_xyzw");
  Require(load["touch_links"].IsSequence(),"touch_links must be a sequence");
  for (const auto& link : load["touch_links"]) p.held_touch_links.push_back(agx_task_workflow::RequireString(link,"touch link"));
  auto base=n["base"];
  agx_task_workflow::RequireKeys(base,{"id","footprint","linear_velocity_mps","angular_velocity_rad_s","linear_acceleration_mps2","angular_acceleration_rad_s2"},"travel base envelope");
  p.base.id=agx_task_workflow::RequireString(base["id"],"base envelope id");
  Require(base["footprint"].IsSequence(),"footprint must be a sequence");
  for (const auto& point : base["footprint"]) {
    agx_task_workflow::RequireKeys(point,{"x_m","y_m"},"footprint vertex");
    p.base.footprint.push_back({Number(point["x_m"],"x_m"),Number(point["y_m"],"y_m")});
  }
  p.base.linear_velocity_mps=Number(base["linear_velocity_mps"],"linear velocity");
  p.base.angular_velocity_rad_s=Number(base["angular_velocity_rad_s"],"angular velocity");
  p.base.linear_acceleration_mps2=Number(base["linear_acceleration_mps2"],"linear acceleration");
  p.base.angular_acceleration_rad_s2=Number(base["angular_acceleration_rad_s2"],"angular acceleration");
  auto timing=n["timing"];
  agx_task_workflow::RequireKeys(timing,{"source_max_age_s","receipt_max_age_s","settle_s","preparation_timeout_s"},"travel timing");
  p.source_max_age_s=Number(timing["source_max_age_s"],"source_max_age_s");
  p.receipt_max_age_s=Number(timing["receipt_max_age_s"],"receipt_max_age_s");
  p.settle_s=Number(timing["settle_s"],"settle_s"); p.preparation_timeout_s=Number(timing["preparation_timeout_s"],"preparation_timeout_s");
  Require(n["starts"].IsSequence(),"starts must be a sequence");
  for (const auto& node_start : n["starts"]) {
    agx_task_workflow::RequireKeys(node_start,{"id","path_id","arm_positions_rad","arm_tolerances_rad","lift_position_m","lift_tolerance_m","empty_allowed","single_held_allowed"},"travel starting pose");
    TravelStartPose start;
    start.id=agx_task_workflow::RequireString(node_start["id"],"start id"); start.path_id=agx_task_workflow::RequireString(node_start["path_id"],"path_id");
    Require(node_start["arm_positions_rad"].IsMap(),"starting arm positions must be a mapping");
    for (const auto& entry : node_start["arm_positions_rad"])
      Require(start.arm_positions_rad.emplace(agx_task_workflow::RequireString(entry.first,"starting joint"),Number(entry.second,"starting joint position")).second,"duplicate starting joint");
    Require(node_start["arm_tolerances_rad"].IsMap(),"starting arm tolerances must be a mapping");
    for (const auto& entry : node_start["arm_tolerances_rad"])
      Require(start.arm_tolerances_rad.emplace(agx_task_workflow::RequireString(entry.first,"starting joint"),Number(entry.second,"starting joint tolerance")).second,"duplicate starting joint tolerance");
    start.lift_position_m=Number(node_start["lift_position_m"],"starting lift position");
    start.lift_tolerance_m=Number(node_start["lift_tolerance_m"],"starting lift tolerance");
    start.empty_allowed=Boolean(node_start["empty_allowed"]); start.single_held_allowed=Boolean(node_start["single_held_allowed"]);
    p.starts.push_back(std::move(start));
  }
  ValidateTravelProfile(p); return p;
}
TravelProfile ParseTravelProfile(const std::string& text,const std::string& source_path) {
  const auto docs=YAML::LoadAll(text);
  Require(docs.size() == 1,"exactly one YAML document is required");
  auto result=LoadTravelProfile(docs.front()); result.config_id=Digest(text); result.source_path=source_path;
  return result;
}
}
