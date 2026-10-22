#include <cassert>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>
#include "travel_test_support.hpp"
#include "agx_navigation_tasks/travel_envelope.hpp"
using namespace agx_navigation_tasks;
namespace {
TravelEnvelopeReports Reports() {
  TravelEnvelopeReports reports;
  const std::map<std::string,std::string> configurations = {
    {"local_planner", "{footprint: [[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]], collision_footprint: [[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]], max_linear_speed_mps: .05, max_spin_speed_radps: .05, min_turn_radius_m: 1, max_linear_accel_mps2: .04, max_linear_decel_mps2: .05, max_lateral_accel_mps2: .02, max_angular_accel_radps2: .05, navigation_progress: {policy: operation_pose_displacement, clock: ros_source, timeout_s: 45, translation_m: .2, rotation_rad: .1, parent_frame: odom, body_frame: base_link, stock_oscillation_timeout_requested_s: 0}, rover_execution: {velocity_scale: 1, acceleration_scale: 1, default_coefficient: 1, task_coefficients: {}}}"},
    {"global_planner", "{footprint: [[-1,-1],[1,-1],[1,1],[-1,1]]}"},
    {"base_arbiter", "{max_linear_x_mps: .06, max_linear_y_mps: .06, max_angular_z_radps: .08, max_linear_speed_lead_mps: .002, max_angular_speed_lead_radps: .004, final_command_policy: measured_progress_contraction, normal_slew_clock: producer_source_time, odometry_receipt_s: .25, odometry_header_s: .35, maximum_header_future_skew_s: .05, min_turn_radius_m: .4764, pose_feedback: {parent_frame: odom, child_frame: rslidar, command_frame: base_link, min_interval_s: .001, max_interval_s: .25, max_translation_step_m: .1, max_rotation_step_rad: .2, max_linear_speed_mps: 2, max_angular_speed_radps: 3, quaternion_norm_tolerance: .00001, extrinsic_translation_tolerance_m: .000001, extrinsic_rotation_tolerance_rad: .000001}}"},
    {"docking", "{max_yaw_speed_radps: .05, max_lateral_speed_mps: .03, max_visual_forward_speed_mps: .03, max_blind_speed_mps: .05, max_rollback_speed_mps: .04, max_visual_linear_acceleration_mps2: .02, max_visual_angular_acceleration_radps2: .05, max_blind_acceleration_mps2: .05, max_rollback_acceleration_mps2: .04, rover_execution: {velocity_scale: 1, acceleration_scale: 1, default_coefficient: 1, task_coefficients: {}}}"},
    {"lidar", "{front_x_m: .5, rear_x_m: -.5, left_y_m: .5, right_y_m: -.5, min_obstacle_z_m: -.2, max_obstacle_z_m: 2, self_padding_m: .03}"}
  };
  for (const auto& item : configurations)
    reports[item.first] = "{schema_version: 1, component: " + item.first +
        ", session_id: test-" + item.first + ", configuration: " + item.second + "}";
  return reports;
}
void Rejects(const TravelEnvelopeReports& reports) {
  bool rejected=false;
  try { EvaluateTravelEnvelope(travel_test::Profile(),reports); }
  catch (const std::exception&) { rejected=true; }
  assert(rejected);
}
void Mutation(const std::string& component, const std::string& key, double value) {
  auto reports=Reports(); auto node=YAML::Load(reports.at(component));
  node["configuration"][key]=value; reports[component]=YAML::Dump(node); Rejects(reports);
}
}
int main() {
  const auto profile=travel_test::Profile(); const auto reports=Reports();
  const auto accepted=EvaluateTravelEnvelope(profile,reports);
  assert(accepted.digest.rfind("sha256:",0)==0 && accepted.digest.size()==71);
  assert(!accepted.snapshot_yaml.empty());
  auto narrowed=reports;auto lidar=YAML::Load(narrowed.at("lidar"));
  lidar["configuration"]["self_exclusion_xy_m"]=YAML::Load("[[-.4,-.4],[.4,-.4],[.4,.4],[-.4,.4]]");
  narrowed["lidar"]=YAML::Dump(lidar);
  assert(!EvaluateTravelEnvelope(profile,narrowed).digest.empty());
  lidar["configuration"]["self_exclusion_xy_m"]=YAML::Load("[[-.6,-.4],[.4,-.4],[.4,.4],[-.6,.4]]");
  narrowed["lidar"]=YAML::Dump(lidar);Rejects(narrowed);
  auto changed=reports; changed.erase("lidar"); Rejects(changed);
  changed=reports; changed["extra"]="{}"; Rejects(changed);
  for (const auto& component : {"local_planner","global_planner"}) {
    changed=reports; auto node=YAML::Load(changed.at(component));
    node["configuration"]["footprint"]=YAML::Load("[[-.49,-.5],[.49,-.5],[.49,.5],[-.49,.5]]");
    changed[component]=YAML::Dump(node); Rejects(changed);
  }
  changed=reports; auto node=YAML::Load(changed.at("local_planner"));
  node["configuration"]["collision_footprint"]=YAML::Load("[[-.4,-.5],[.4,-.5],[.4,.5],[-.4,.5]]");
  changed["local_planner"]=YAML::Dump(node); Rejects(changed);
  // Concave or self-intersecting envelopes cannot certify containment.
  for (const auto& polygon : {"[[0,0],[1,0],[0,1]]", "[[-1,-1],[1,1],[1,-1],[-1,1]]"}) {
    changed=reports; node=YAML::Load(changed.at("global_planner"));
    node["configuration"]["footprint"]=YAML::Load(polygon);
    changed["global_planner"]=YAML::Dump(node); Rejects(changed);
  }
  // The stock-zero composition requires this exact finite replacement policy.
  for(const auto& key:{"policy","clock","timeout_s","translation_m","rotation_rad","parent_frame","body_frame","stock_oscillation_timeout_requested_s"}) {
    changed=reports;auto missing=YAML::Load(changed.at("local_planner"));
    missing["configuration"]["navigation_progress"].remove(key);
    changed["local_planner"]=YAML::Dump(missing);Rejects(changed);
  }
  for(const auto& key:{"timeout_s","translation_m","rotation_rad"}) {
    for(double bad:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
      changed=reports;auto invalid=YAML::Load(changed.at("local_planner"));
      invalid["configuration"]["navigation_progress"][key]=bad;
      changed["local_planner"]=YAML::Dump(invalid);Rejects(changed);
    }
  }
  for(const auto& key:{"policy","clock","parent_frame","body_frame"}) {
    changed=reports;auto invalid=YAML::Load(changed.at("local_planner"));
    invalid["configuration"]["navigation_progress"][key]="wrong";
    changed["local_planner"]=YAML::Dump(invalid);Rejects(changed);
  }
  for(const auto& setting:std::vector<std::pair<std::string,double>>{{"rotation_rad",std::acos(-1.)},{"stock_oscillation_timeout_requested_s",45}}) {
    changed=reports;auto invalid=YAML::Load(changed.at("local_planner"));
    invalid["configuration"]["navigation_progress"][setting.first]=setting.second;
    changed["local_planner"]=YAML::Dump(invalid);Rejects(changed);
  }
  Mutation("local_planner","max_linear_speed_mps",.11);
  Mutation("local_planner","min_turn_radius_m",.1); // Angular arc speed exceeds profile.
  Mutation("local_planner","max_linear_decel_mps2",.099); // Combined planar acceleration.
  Mutation("local_planner","max_angular_accel_radps2",.11);
  Mutation("base_arbiter","min_turn_radius_m",1.1); // Planner cannot turn inside the declared base limit.
  Mutation("base_arbiter","max_linear_y_mps",.09); // Vector, not each axis alone.
  Mutation("base_arbiter","odometry_receipt_s",0);
  Mutation("base_arbiter","odometry_header_s",-.1);
  Mutation("base_arbiter","maximum_header_future_skew_s",-.001);
  Mutation("base_arbiter","max_linear_speed_lead_mps",0);
  Mutation("base_arbiter","max_angular_speed_lead_radps",-.001);
  for (const auto& key : {"final_command_policy", "normal_slew_clock", "pose_feedback"}) {
    changed=reports; auto incomplete=YAML::Load(changed.at("base_arbiter"));
    incomplete["configuration"].remove(key);
    changed["base_arbiter"]=YAML::Dump(incomplete); Rejects(changed);
  }
  for (const auto& key : {"final_command_policy", "normal_slew_clock"}) {
    changed=reports; auto incompatible=YAML::Load(changed.at("base_arbiter"));
    incompatible["configuration"][key]="final_publication_slew";
    changed["base_arbiter"]=YAML::Dump(incompatible); Rejects(changed);
  }
  // Legacy slew fields are rejected, even if numerically conservative.
  Mutation("base_arbiter","max_linear_acceleration_mps2",.001);
  changed=reports; auto modified=YAML::Load(changed.at("base_arbiter"));
  modified["configuration"]["pose_feedback"]["min_interval_s"]=.3;
  changed["base_arbiter"]=YAML::Dump(modified); Rejects(changed);
  changed=reports; modified=YAML::Load(changed.at("base_arbiter"));
  modified["configuration"]["pose_feedback"]["command_frame"]="";
  changed["base_arbiter"]=YAML::Dump(modified); Rejects(changed);
  changed=reports; modified=YAML::Load(changed.at("base_arbiter"));
  modified["configuration"]["pose_feedback"]["command_frame"]="rslidar";
  changed["base_arbiter"]=YAML::Dump(modified); Rejects(changed);
  changed=reports; modified=YAML::Load(changed.at("base_arbiter"));
  modified["configuration"]["max_linear_speed_lead_mps"]=.001;
  changed["base_arbiter"]=YAML::Dump(modified);
  assert(EvaluateTravelEnvelope(profile,changed).digest!=accepted.digest);
  for (const auto& key : {"odometry_receipt_s", "odometry_header_s", "maximum_header_future_skew_s"}) {
    changed=reports; modified=YAML::Load(changed.at("base_arbiter"));
    modified["configuration"][key]=.1;
    changed["base_arbiter"]=YAML::Dump(modified);
    assert(EvaluateTravelEnvelope(profile,changed).digest!=accepted.digest);
    modified["configuration"].remove(key);
    changed["base_arbiter"]=YAML::Dump(modified); Rejects(changed);
  }
  changed=reports; modified=YAML::Load(changed.at("base_arbiter"));
  modified["configuration"]["pose_feedback"]["child_frame"]="registered_lidar";
  changed["base_arbiter"]=YAML::Dump(modified);
  assert(EvaluateTravelEnvelope(profile,changed).digest!=accepted.digest);
  Mutation("docking","max_visual_linear_acceleration_mps2",.08);
  Mutation("docking","max_rollback_acceleration_mps2",.11);
  Mutation("lidar","front_x_m",.49);
  Mutation("lidar","front_x_m",.6); // Self-return exclusion reaches outside qualified volume.
  Mutation("lidar","self_padding_m",-.01);
  Mutation("lidar","max_obstacle_z_m",-.3);
  Mutation("docking","max_yaw_speed_radps",std::numeric_limits<double>::quiet_NaN());
  for (const auto& component : {"local_planner","docking"}) {
    changed=reports; node=YAML::Load(changed.at(component));
    node["configuration"]["rover_execution"]["acceleration_scale"]=1.1;
    changed[component]=YAML::Dump(node); Rejects(changed);
    node["configuration"]["rover_execution"]["acceleration_scale"]=1;
    node["configuration"]["rover_execution"]["task_coefficients"]["unknown_task"]=1;
    changed[component]=YAML::Dump(node); Rejects(changed);
  }
  changed=reports; node=YAML::Load(changed.at("lidar")); node["schema_version"]=2;
  changed["lidar"]=YAML::Dump(node); Rejects(changed);
  changed=reports; node=YAML::Load(changed.at("lidar")); node["session_id"]="";
  changed["lidar"]=YAML::Dump(node); Rejects(changed);

  TravelEnvelopeMonitor monitor(profile);
  assert(!monitor.TryAdmit(10));
  for (const auto& item : reports) monitor.Observe(item.first,item.second,10);
  assert(monitor.TryAdmit(10)); assert(monitor.Guard(11.9));
  assert(monitor.Digest()==accepted.digest);
  for (const auto& item : reports) monitor.Observe(item.first,item.second,11.9);
  assert(monitor.Guard(12));
  assert(!monitor.Guard(14));
  for (const auto& item : reports) monitor.Observe(item.first,item.second,14);
  assert(!monitor.Guard(14)); // Loss latches, never re-admit silently.
  TravelEnvelopeMonitor restart(profile);
  for (const auto& item : reports) restart.Observe(item.first,item.second,10);
  assert(restart.TryAdmit(10));
  node=YAML::Load(reports.at("lidar")); node["session_id"]="restarted";
  restart.Observe("lidar",YAML::Dump(node),10.1);
  assert(!restart.Guard(10.1));
  TravelEnvelopeMonitor reversed(profile);
  for (const auto& item : reports) reversed.Observe(item.first,item.second,10);
  assert(reversed.TryAdmit(10)); assert(!reversed.Guard(9));
  TravelEnvelopeMonitor gap(profile);
  for (const auto& item : reports) gap.Observe(item.first,item.second,10);
  assert(gap.TryAdmit(10));
  for (const auto& item : reports) gap.Observe(item.first,item.second,13);
  assert(!gap.Guard(13)); // Returning reports cannot hide a supervision gap.
  changed=reports; node=YAML::Load(changed.at("docking"));
  node["configuration"]["rover_execution"]["velocity_scale"]=.5;
  changed["docking"]=YAML::Dump(node); Rejects(changed);
}
