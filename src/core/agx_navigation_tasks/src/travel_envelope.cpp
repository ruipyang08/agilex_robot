#include "agx_navigation_tasks/travel_envelope.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>
#include "agx_task_workflow/config.hpp"
namespace agx_navigation_tasks {
namespace {
void Require(bool ok,const std::string& why) {
  if (!ok) throw std::invalid_argument("travel envelope: " + why);
}
double Number(const YAML::Node& node) {
  Require(node.IsScalar() && node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str","numeric value required");
  const double value=node.as<double>(); Require(std::isfinite(value),"nonfinite limit"); return value;
}
double Positive(const YAML::Node& node) {
  const auto value=Number(node); Require(value>0,"positive limit required"); return value;
}
std::vector<TravelPoint> Polygon(const YAML::Node& node) {
  Require(node.IsSequence() && node.size()>=3 && node.size()<=256,"bounded polygon required");
  std::vector<TravelPoint> result;
  for (const auto& point:node) {
    Require(point.IsSequence() && point.size()==2,"footprint point must have two coordinates");
    result.push_back({Number(point[0]),Number(point[1])});
  }
  double orientation=0;
  for (std::size_t i=0;i<result.size();++i) {
    const auto a=result[i],b=result[(i+1)%result.size()];
    for (std::size_t j=0;j<result.size();++j) {
      if (j==i || j==(i+1)%result.size()) continue;
      const auto p=result[j];
      const double cross=(b.x_m-a.x_m)*(p.y_m-a.y_m)-(b.y_m-a.y_m)*(p.x_m-a.x_m);
      Require(std::isfinite(cross) && cross!=0,"strict convex polygon required");
      if (!orientation) orientation=cross;
      Require((cross>0)==(orientation>0),"polygon is not convex and ordered");
    }
  }
  return result;
}
void Contains(const std::vector<TravelPoint>& outer,const std::vector<TravelPoint>& inner,const std::string& label) {
  const auto a=outer[0],b=outer[1],c=outer[2];
  const double orientation=(b.x_m-a.x_m)*(c.y_m-a.y_m)-(b.y_m-a.y_m)*(c.x_m-a.x_m);
  for (std::size_t i=0;i<outer.size();++i) {
    const auto x=outer[i],y=outer[(i+1)%outer.size()];
    for (const auto& point:inner) {
      const double cross=(y.x_m-x.x_m)*(point.y_m-x.y_m)-(y.y_m-x.y_m)*(point.x_m-x.x_m);
      Require(std::isfinite(cross) && (orientation>0 ? cross>=-1e-10 : cross<=1e-10),label+" does not contain qualified footprint");
    }
  }
}
std::vector<double> Scales(const YAML::Node& node) {
  agx_task_workflow::RequireKeys(node,{"velocity_scale","acceleration_scale","default_coefficient","task_coefficients"},"rover_execution");
  std::vector<double> result;
  for (const auto& key:{"velocity_scale","acceleration_scale","default_coefficient"}) {
    const double value=Positive(node[key]); Require(value<=1,"rover_execution cannot raise limits"); result.push_back(value);
  }
  const std::set<std::string> tasks={"coarse_navigation","visual_docking","final_approach","rollback"};
  const auto coefficients=node["task_coefficients"];
  Require(coefficients.IsMap(),"task_coefficients must be a map");
  std::set<std::string> seen;
  for (const auto& entry:coefficients) {
    const auto key=entry.first.as<std::string>();
    Require(tasks.count(key) && seen.insert(key).second,"unknown/duplicate rover execution task");
  }
  for (const auto& task:tasks) {
    const double value=coefficients[task] ? Positive(coefficients[task]) : result[2];
    Require(value<=1 && result[0]*value>0 && result[1]*value>0,"invalid rover execution coefficient");
    result.push_back(value);
  }
  return result;
}
std::string Sha256(const std::string& text) {
  unsigned char bytes[EVP_MAX_MD_SIZE]; unsigned int count=0;
  Require(EVP_Digest(text.data(),text.size(),bytes,&count,EVP_sha256(),nullptr)==1,"SHA256 failed");
  std::ostringstream out; out<<"sha256:"<<std::hex<<std::setfill('0');
  for (unsigned i=0;i<count;++i) out<<std::setw(2)<<static_cast<int>(bytes[i]);
  return out.str();
}
}
const std::vector<std::string>& TravelEnvelopeComponents() {
  static const std::vector<std::string> names={"local_planner","global_planner","base_arbiter","docking","lidar"}; return names;
}
TravelEnvelopeSnapshot EvaluateTravelEnvelope(const TravelProfile& profile,const TravelEnvelopeReports& reports) {
  ValidateTravelProfile(profile);
  Require(!profile.arm.empty(),"enabled/qualified travel parameters required");
  Require(reports.size()==TravelEnvelopeComponents().size(),"exactly five startup reports required");
  std::map<std::string,YAML::Node> configs;
  YAML::Node snapshot;
  snapshot["travel_profile_id"]=profile.profile_id; snapshot["travel_config_id"]=profile.config_id;
  for (const auto& name:TravelEnvelopeComponents()) {
    const auto found=reports.find(name); Require(found!=reports.end(),"missing "+name+" report");
    Require(found->second.size()<=65536,"report too large");
    const auto report=YAML::Load(found->second);
    agx_task_workflow::RequireKeys(report,{"schema_version","component","session_id","configuration"},"effective report");
    Require(Number(report["schema_version"])==1,"unknown report version");
    Require(agx_task_workflow::RequireString(report["component"],"component")==name,"report component mismatch");
    agx_task_workflow::RequireString(report["session_id"],"session_id");
    configs[name]=report["configuration"];
    snapshot["reports"][name]=found->second;
  }
  const auto& base=profile.base;
  const auto limit=[&](double actual,double maximum,const std::string& name) {
    Require(std::isfinite(actual) && actual>0 && actual<=maximum+1e-12,name+" exceeds qualified limit");
  };
  const auto local=configs.at("local_planner");
  agx_task_workflow::RequireKeys(local,{"footprint","collision_footprint","max_linear_speed_mps","max_spin_speed_radps","min_turn_radius_m","max_linear_accel_mps2","max_linear_decel_mps2","max_lateral_accel_mps2","max_angular_accel_radps2","rover_execution","navigation_progress"},"local planner report");
  const auto progress=local["navigation_progress"];
  agx_task_workflow::RequireKeys(progress,{"policy","clock","timeout_s","translation_m","rotation_rad","parent_frame","body_frame","stock_oscillation_timeout_requested_s"},"navigation progress report");
  Require(agx_task_workflow::RequireString(progress["policy"],"progress policy")=="operation_pose_displacement", "mandatory navigation progress policy missing");
  Require(agx_task_workflow::RequireString(progress["clock"],"progress clock")=="ros_source", "unsupported navigation progress clock");
  Positive(progress["timeout_s"]);Positive(progress["translation_m"]);
  Require(Positive(progress["rotation_rad"])<std::acos(-1.),"navigation progress rotation must be below pi");
  Require(Number(progress["stock_oscillation_timeout_requested_s"])==0,"workbench stock watchdog replacement was not requested");
  Require(agx_task_workflow::RequireString(progress["parent_frame"],"progress parent frame")=="odom" &&
          agx_task_workflow::RequireString(progress["body_frame"],"progress body frame")=="base_link", "unsupported navigation progress pose frames");
  Contains(Polygon(local["footprint"]),base.footprint,"local costmap");
  Contains(Polygon(local["collision_footprint"]),base.footprint,"local collision checker");
  const double speed=Positive(local["max_linear_speed_mps"]);
  limit(speed,base.linear_velocity_mps,"coarse speed");
  limit(std::max(Positive(local["max_spin_speed_radps"]),speed/Positive(local["min_turn_radius_m"])),base.angular_velocity_rad_s,"coarse angular speed");
  limit(std::hypot(std::max(Positive(local["max_linear_accel_mps2"]),Positive(local["max_linear_decel_mps2"])),Positive(local["max_lateral_accel_mps2"])),base.linear_acceleration_mps2,"coarse planar acceleration");
  limit(Positive(local["max_angular_accel_radps2"]),base.angular_acceleration_rad_s2,"coarse angular acceleration");
  const auto local_scales=Scales(local["rover_execution"]);
  const auto global=configs.at("global_planner");
  agx_task_workflow::RequireKeys(global,{"footprint"},"global planner report");
  Contains(Polygon(global["footprint"]),base.footprint,"global costmap (including circle approximation)");
  const auto arbiter=configs.at("base_arbiter");
  agx_task_workflow::RequireKeys(arbiter,{"max_linear_x_mps","max_linear_y_mps","max_angular_z_radps",
      "final_command_policy","normal_slew_clock","odometry_receipt_s",
      "odometry_header_s","maximum_header_future_skew_s",
      "max_linear_speed_lead_mps","max_angular_speed_lead_radps",
      "pose_feedback","min_turn_radius_m"},"arbiter report");
  Require(Positive(local["min_turn_radius_m"])>=Positive(arbiter["min_turn_radius_m"]),
      "planner turning radius is below the arbiter's supported base geometry");
  limit(std::hypot(Positive(arbiter["max_linear_x_mps"]),Positive(arbiter["max_linear_y_mps"])),base.linear_velocity_mps,"arbiter planar speed");
  limit(Positive(arbiter["max_angular_z_radps"]),base.angular_velocity_rad_s,"arbiter angular speed");
  Positive(arbiter["odometry_receipt_s"]);
  Positive(arbiter["odometry_header_s"]);
  Require(Number(arbiter["maximum_header_future_skew_s"])>=0,"negative feedback future tolerance");
  Positive(arbiter["max_linear_speed_lead_mps"]);
  Positive(arbiter["max_angular_speed_lead_radps"]);
  Require(agx_task_workflow::RequireString(arbiter["final_command_policy"],"final_command_policy")==
      "measured_progress_contraction","unsupported final command policy");
  Require(agx_task_workflow::RequireString(arbiter["normal_slew_clock"],"normal_slew_clock")==
      "producer_source_time","unsupported normal command shaping clock");
  const auto pose=arbiter["pose_feedback"];
  agx_task_workflow::RequireKeys(pose,{"parent_frame","child_frame","command_frame","min_interval_s","max_interval_s",
      "max_translation_step_m","max_rotation_step_rad","max_linear_speed_mps","max_angular_speed_radps",
      "quaternion_norm_tolerance","extrinsic_translation_tolerance_m","extrinsic_rotation_tolerance_rad"},
      "arbiter pose feedback report");
  for (const auto& key:{"parent_frame","child_frame","command_frame"}) agx_task_workflow::RequireString(pose[key],key);
  Require(pose["command_frame"].as<std::string>()=="base_link","arbiter command frame must match vendor base_link origin");
  Require(pose["parent_frame"].as<std::string>()==progress["parent_frame"].as<std::string>() &&
          pose["command_frame"].as<std::string>()==progress["body_frame"].as<std::string>(),
          "navigation progress and canonical pose frames differ");
  for (const auto& key:{"min_interval_s","max_interval_s","max_translation_step_m","max_rotation_step_rad",
      "max_linear_speed_mps","max_angular_speed_radps","quaternion_norm_tolerance",
      "extrinsic_translation_tolerance_m","extrinsic_rotation_tolerance_rad"}) Positive(pose[key]);
  Require(Number(pose["min_interval_s"])<Number(pose["max_interval_s"]),"invalid pose feedback source interval");
  const auto docking=configs.at("docking");
  agx_task_workflow::RequireKeys(docking,{"max_yaw_speed_radps","max_lateral_speed_mps","max_visual_forward_speed_mps","max_blind_speed_mps","max_rollback_speed_mps","max_visual_linear_acceleration_mps2","max_visual_angular_acceleration_radps2","max_blind_acceleration_mps2","max_rollback_acceleration_mps2","rover_execution"},"docking report");
  const double visual_speed=std::hypot(Positive(docking["max_lateral_speed_mps"]),Positive(docking["max_visual_forward_speed_mps"]));
  const double yaw_speed=Positive(docking["max_yaw_speed_radps"]);
  // Normal interpolation is performed by each producer before collision checks.
  // These source-time bounds do not certify publication-time slew or actuator
  // acceleration; the profile still requires recorded physical qualification.
  limit(std::max({visual_speed,Positive(docking["max_blind_speed_mps"]),Positive(docking["max_rollback_speed_mps"])}),base.linear_velocity_mps,"docking planar speed");
  limit(yaw_speed,base.angular_velocity_rad_s,"docking yaw speed");
  // Bound the generated command derivative, including the rotating body frame.
  limit(std::max({std::sqrt(2.0)*Positive(docking["max_visual_linear_acceleration_mps2"])+visual_speed*yaw_speed,Positive(docking["max_blind_acceleration_mps2"]),Positive(docking["max_rollback_acceleration_mps2"])}),base.linear_acceleration_mps2,"docking planar acceleration");
  limit(Positive(docking["max_visual_angular_acceleration_radps2"]),base.angular_acceleration_rad_s2,"docking angular acceleration");
  Require(local_scales==Scales(docking["rover_execution"]),"planner and docking loaded different execution policies");
  const auto lidar=configs.at("lidar");
  std::set<std::string> lidar_keys={"front_x_m","rear_x_m","left_y_m","right_y_m","min_obstacle_z_m","max_obstacle_z_m","self_padding_m"};
  if(lidar["self_exclusion_xy_m"])lidar_keys.insert("self_exclusion_xy_m");
  agx_task_workflow::RequireKeys(lidar,lidar_keys,"lidar report");
  const double front=Number(lidar["front_x_m"]),rear=Number(lidar["rear_x_m"]),left=Number(lidar["left_y_m"]),right=Number(lidar["right_y_m"]),padding=Number(lidar["self_padding_m"]);
  Require(front>rear && left>right && padding>=0 && rear+padding<front-padding && right+padding<left-padding,"invalid lidar footprint/self inset");
  Require(Number(lidar["min_obstacle_z_m"])<=0 && Number(lidar["max_obstacle_z_m"])>0,"lidar height filter excludes base plane");
  Contains({{rear,right},{front,right},{front,left},{rear,left}},base.footprint,"lidar protected envelope");
  if(lidar["self_exclusion_xy_m"])
    Contains(base.footprint,Polygon(lidar["self_exclusion_xy_m"]),"qualified footprint for lidar self-return exclusion");
  else Contains(base.footprint,{{rear+padding,right+padding},{front-padding,right+padding},{front-padding,left-padding},{rear+padding,left-padding}},"qualified footprint for lidar self-return exclusion");
  const auto text=YAML::Dump(snapshot); return {Sha256(text),text};
}
TravelEnvelopeMonitor::TravelEnvelopeMonitor(TravelProfile p):profile_(std::move(p)) {}
void TravelEnvelopeMonitor::Fail(const std::string& detail) { if (!faulted_) { faulted_=true; detail_=detail; } }
void TravelEnvelopeMonitor::Observe(const std::string& component,const std::string& report,double receipt_s) {
  if (faulted_) return;
  const auto& names=TravelEnvelopeComponents();
  if (!std::isfinite(receipt_s) || receipt_s<0 || report.size()>65536 || std::find(names.begin(),names.end(),component)==names.end()) { Fail("invalid effective report receipt"); return; }
  if (reports_.count(component) && (reports_.at(component)!=report || receipt_s<receipts_.at(component) || (admitted_ && receipt_s-receipts_.at(component)>2.0))) { Fail("effective report changed/restarted or receipt continuity lost: "+component); return; }
  reports_[component]=report; receipts_[component]=receipt_s;
}
bool TravelEnvelopeMonitor::TryAdmit(double now_s) {
  if (faulted_) return false;
  if (admitted_) return Guard(now_s);
  if (reports_.size()!=TravelEnvelopeComponents().size()) return false;
  try { snapshot_=EvaluateTravelEnvelope(profile_,reports_); }
  catch (const std::exception& error) { Fail(error.what()); return false; }
  admitted_=true; detail_="effective startup envelope admitted"; return Guard(now_s);
}
bool TravelEnvelopeMonitor::Guard(double now_s) {
  if (faulted_ || !admitted_) return false;
  if (!std::isfinite(now_s)) { Fail("invalid effective report clock"); return false; }
  for (const auto& item:receipts_) if (now_s<item.second || now_s-item.second>2.0) { Fail("effective report missing/stale: "+item.first); return false; }
  return true;
}
}  // namespace agx_navigation_tasks
