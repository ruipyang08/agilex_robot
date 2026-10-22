#include <agx_workbench_navigation/effective_planner_config.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <cassert>
#include <limits>
#include <sstream>

using namespace agx_workbench_navigation;
boost::property_tree::ptree Parse(const std::string& text) {
  boost::property_tree::ptree result; std::istringstream stream(text);
  boost::property_tree::read_json(stream,result); return result;
}
int main() {
  RangerPlannerConfig limits{};
  limits.max_linear_speed_mps=.2; limits.max_spin_speed_radps=.3;
  limits.min_turn_radius_m=.4764; limits.max_linear_accel_mps2=.1;
  limits.max_linear_decel_mps2=.15; limits.max_lateral_accel_mps2=.08;
  limits.max_angular_accel_radps2=.16;
  RoverExecutionSnapshot policy;
  policy.velocity_scale=.8;policy.acceleration_scale=.5;
  policy.default_coefficient=.75; policy.task_coefficients["coarse_navigation"]=.5;
  EffectiveFootprint collision{{{-.5,-.3}},{{.5,-.3}},{{.5,.3}},{{-.5,.3}}};
  auto live=collision;
  const auto session=NewEffectivePlannerSession();
  assert(!session.empty() && session!=NewEffectivePlannerSession());
  const auto text=LocalPlannerEffectiveReport(session,limits,policy,live,collision);
  auto parsed=Parse(text);
  assert(parsed.get<int>("schema_version")==1);
  assert(parsed.get<std::string>("component")=="local_planner");
  assert(parsed.get<std::string>("session_id")==session);
  assert(parsed.get<double>("configuration.max_angular_accel_radps2")==.16);
  assert(parsed.get<double>("configuration.max_linear_speed_mps")==.2);
  assert(parsed.get<std::string>("configuration.navigation_progress.policy")=="operation_pose_displacement");
  assert(parsed.get<std::string>("configuration.navigation_progress.clock")=="ros_source");
  assert(parsed.get<double>("configuration.navigation_progress.timeout_s")==45);
  assert(parsed.get<double>("configuration.navigation_progress.translation_m")==.2);
  assert(parsed.get<double>("configuration.navigation_progress.rotation_rad")==.1);
  assert(parsed.get<double>("configuration.navigation_progress.stock_oscillation_timeout_requested_s")==0);
  assert(parsed.get<double>("configuration.rover_execution.task_coefficients.coarse_navigation")==.5);
  assert(parsed.get<double>("configuration.rover_execution.default_coefficient")==.75);
  assert(parsed.get_child("configuration.footprint").size()==4);
  assert(parsed.get_child("configuration.collision_footprint").size()==4);
  assert(LocalPlannerEffectiveReport(session,limits,policy,live,collision)==text);
  live.front()[0]=-.6;
  const auto changed=LocalPlannerEffectiveReport(session,limits,policy,live,collision);
  assert(changed!=text);
  parsed=Parse(changed);
  assert(parsed.get_child("configuration.footprint").front().second.front().second.get_value<double>()==-.6);
  assert(parsed.get_child("configuration.collision_footprint").front().second.front().second.get_value<double>()==-.5);
  auto global=Parse(GlobalPlannerEffectiveReport(session,live));
  assert(global.get<std::string>("component")=="global_planner");
  assert(global.get_child("configuration").size()==1);
  const RoverExecutionSnapshot identity;
  parsed=Parse(LocalPlannerEffectiveReport(session,limits,identity,live,collision));
  assert(parsed.get<double>("configuration.rover_execution.velocity_scale")==1.0);
  assert(parsed.get_child("configuration.rover_execution.task_coefficients").empty());
  live.front()[0]=std::numeric_limits<double>::quiet_NaN();
  bool rejected=false;try {GlobalPlannerEffectiveReport(session,live);} catch(const std::invalid_argument&) {rejected=true;}
  assert(rejected); // invalid geometry must not produce a fresh usable report
  return 0;
}
