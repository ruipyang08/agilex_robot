#include "agx_robot_task_runtime/robot_sequence_plan.hpp"
#include <cassert>
#include <stdexcept>
#include "registration_fixture.hpp"
#include "agx_xarm_pick/resolved_tube_config.hpp"
#include "../../agx_navigation_tasks/test/travel_test_support.hpp"
using namespace robot_workflow_test;
const std::string nav="{id: approach, type: navigate, target: wb-100}";
const std::string grasp="{id: pick, type: grasp_test_tube, fixture_profile: rack, cap_color: green}";
agx_task_workflow::Config recipe(const std::string& list) {return agx_task_workflow::ParseConfig("schema_version: 2\nworkflow_id: test\ntasks: ["+list+"]\n");}
int main() {
  std::vector<std::string> calls,phases; bool cancel=false; std::string fail;
  auto call=[&](const std::string& name) {calls.push_back(name);return agx_task_workflow::StepResult{name!=fail,"native "+name};};
  BuiltinPorts p{[&](const NavigateParameters& n){assert(n.target=="wb-100");return call("navigate");},
    [&]{return call("verify");},[&]{return call("hold");},
    [&](const GraspParameters& g){assert(g.fixture_profile=="rack" && g.cap_color=="green");return call("grasp");},[&]{return cancel;}};
  agx_task_workflow::TaskRegistry registry; RegisterBuiltinTasks(registry,{{"wb-100"},"rack","green"},p);
  const auto nav_description=agx_task_workflow::DescribeTask(registry,"navigate");
  const auto grasp_description=agx_task_workflow::DescribeTask(registry,"grasp_test_tube");
  assert(nav_description.metadata_complete && grasp_description.metadata_complete);
  assert(nav_description.inputs.size()==1 && grasp_description.inputs.size()==2);
  assert(!nav_description.interfaces.empty() && grasp_description.resources.size()==5);
  assert(calls.empty());
  auto prepared=agx_task_workflow::PrepareTasks(recipe(nav+","+grasp),registry); assert(calls.empty());
  assert(prepared[0].entry_check && prepared[0].stop && prepared[0].cleanup);
  assert(!prepared[0].entry_check().ok); // Unconnected guards must never claim readiness.
  const auto missing_stop=prepared[0].stop(1.0);
  assert(!missing_stop.ok && missing_stop.failure==agx_task_workflow::FailureKind::UnconfirmedStop && missing_stop.stop==agx_task_workflow::StopState::NotConfirmed);
  assert(prepared[0].entry_phase=="NAVIGATE" && prepared[1].entry_phase=="GRASP");
  auto phase=[&](const std::string& s){phases.push_back(s);};
  for(const auto& t:prepared) assert(t.execute(phase).ok);
  assert((calls==std::vector<std::string>{"navigate","verify","hold","grasp"}));
  assert((phases==std::vector<std::string>{"VERIFY_STOP","HOLD_BASE"}));
  for(const auto& f: {"navigate","verify","hold"}) {
    fail=f;calls.clear();auto result=prepared[0].execute(phase);
    assert(!result.ok && result.detail=="native "+fail && calls.back()==fail);
  }
  fail.clear(); calls.clear(); cancel=false;
  auto result=prepared[0].execute([&](const std::string& s){if(s=="HOLD_BASE")cancel=true;});
  assert(!result.ok && (calls==std::vector<std::string>{"navigate","verify"}));
  cancel=false;
  const auto fails=[&](const std::string& list){
    calls.clear();try {agx_task_workflow::PrepareTasks(recipe(list),registry);assert(false);}catch(const agx_task_workflow::PreparationError&){}
    assert(calls.empty());
  };
  fails(grasp);fails(nav+",{id: second, type: navigate, target: wb-100}");
  fails(nav+","+grasp+",{id: last, type: navigate, target: wb-100}");
  fails(nav+","+grasp+",{id: twice, type: grasp_test_tube, fixture_profile: rack, cap_color: green}");
  for (const auto& value:{"missing","true","12","null","[]","{}","''"})
    fails(std::string("{id: approach, type: navigate, target: ")+value+"}");
  fails("{id: approach, type: navigate}");
  fails("{id: approach, type: navigate, target: wb-100, speed: 1}");
  fails("{id: approach, type: navigate, target: wb-100, config: /opt/lab/task.yaml}");
  fails(nav+",{id: pick, type: grasp_test_tube, fixture_profile: rack, cap_color: green, config: /opt/lab/task.yaml}");
  fails("{id: approach, type: navigate, target: wb-100, on_failure: continue},"+grasp);
  fails(nav+",{id: first, type: grasp_test_tube, fixture_profile: rack, cap_color: green, on_failure: continue},"+grasp);
  for(const auto& args:{"fixture_profile: other, cap_color: green","fixture_profile: rack, cap_color: blue",
                        "fixture_profile: rack, cap_color: true","fixture_profile: rack", "fixture_profile: rack, cap_color: green, extra: 1"})
    fails(nav+",{id: pick, type: grasp_test_tube, "+args+"}");
  // An extension can follow the held/payload state without changing dispatch.
  agx_task_workflow::RegisterTask(registry,"record_result",[&](const agx_task_workflow::TaskSpec& t,agx_task_workflow::PreparationState& s){
    const auto plan=s.planning.Get<agx_robot_task_runtime::RobotSequencePlan>(agx_robot_task_runtime::kRobotSequencePlan);
    assert(plan.held_station=="wb-100" && plan.payload_may_be_retained);
    return agx_task_workflow::PreparedTask{t.id,t.type,"TASK_RUNNING",[&](const agx_task_workflow::PhaseSink&){return call("record");}};
  });
  prepared=agx_task_workflow::PrepareTasks(recipe(nav+","+grasp+",{id: record, type: record_result}"),registry);
  calls.clear();for(const auto& t:prepared)assert(t.execute(phase).ok);
  assert(calls.back()=="record" && calls.size()==5);

  // Admitted composed navigation may depart repeatedly with a retained payload.
  const auto profile=std::make_shared<const TravelProfile>(travel_test::Profile());
  InstalledSelections composed{{"wb-100","wb-200"},"rack","green"};
  for (const auto& id:{"approach","next","return"}) composed.admitted_navigation[id]=profile;
  const auto tube_fixture=std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/
      "agx_xarm_pick/test/fixtures/tube_task_config/taskA.yaml";
  auto grasp_config=std::make_shared<agx_xarm_pick::ResolvedTubeTaskConfig>(
      *agx_xarm_pick::ResolveTubeTaskConfig(tube_fixture.string()));
  grasp_config->fixture_profile_id="rack";
  grasp_config->robot.joint_state_topic=profile->robot.arm_joint_state_topic;
  grasp_config->robot.gripper_state_topic=profile->robot.gripper_state_topic;
  grasp_config->motion.lift_service_name=profile->robot.lift_service;
  grasp_config->motion.lift_state_topic=profile->robot.lift_state_topic;
  composed.admitted_grasps["pick"]={"rack","green",grasp_config,"wb-100"};
  std::vector<std::string> destinations;
  auto composed_ports=p;
  composed_ports.grasp=[&](const GraspParameters& request) {
    assert(request.config==grasp_config);return call("grasp");
  };
  composed_ports.composed_navigation=[&](const NavigateParameters& request,const agx_task_workflow::PhaseSink& sink) {
    assert(request.travel_profile==profile); destinations.push_back(request.target);
    sink("PREPARE_TRAVEL"); return agx_task_workflow::StepResult{true,"composed"};
  };
  agx_task_workflow::TaskRegistry composed_registry; RegisterBuiltinTasks(composed_registry,composed,composed_ports);
  const auto repeated=recipe(nav+","+grasp+",{id: next,type: navigate,target: wb-200},{id: return,type: navigate,target: wb-100}");
  const auto composed_recipe=agx_task_workflow::PrepareRecipe(repeated,composed_registry);
  assert(destinations.empty());
  assert(composed_recipe.tasks[0].entry_phase=="NAVIGATE");
  calls.clear();
  for (const auto& task:composed_recipe.tasks) assert(task.execute(phase).ok);
  assert((calls==std::vector<std::string>{"grasp"}));
  assert((destinations==std::vector<std::string>{"wb-100","wb-200","wb-100"}));
  const auto dependency=[&](const std::string& key)->const agx_task_workflow::InterfaceRequirement& {
    for (const auto& item:composed_recipe.required_interfaces) if (item.key==key) return item;
    assert(false);throw std::runtime_error("missing dependency");
  };
  assert(dependency("arm_joint_state").endpoint==profile->robot.arm_joint_state_topic);
  assert(dependency("gripper_state").endpoint==profile->robot.gripper_state_topic);
  assert(dependency("lift_control").endpoint==profile->robot.lift_service);
  assert(dependency("lift_state").endpoint==profile->robot.lift_state_topic);
  assert(dependency("base_acquire_hold").ros_type=="agx_workbench_navigation/AcquireBaseHold");
  assert(dependency("base_prepare_departure").ros_type=="agx_workbench_navigation/PrepareBaseDeparture");
  assert(dependency("base_release_hold").ros_type=="agx_workbench_navigation/ReleaseBaseHold");
  assert(dependency("travel_readiness").direction==agx_task_workflow::InterfaceDirection::Command);
  assert(dependency("arm_planning_scene").ros_type=="moveit_msgs/GetPlanningScene");
  for (const auto& component:{"local_planner","global_planner","base_arbiter","docking","lidar"}) {
    const auto& report=dependency(std::string("effective_")+component);
    assert(report.endpoint==std::string("/workbench_nav/effective_config/")+component);
    assert(report.ros_type=="std_msgs/String" && report.direction==agx_task_workflow::InterfaceDirection::Observe);
  }
  assert(composed_recipe.tasks[0].resources.size()==5);
  auto wrong=composed;wrong.admitted_grasps.at("pick").station_id="station-b";
  agx_task_workflow::TaskRegistry wrong_station;RegisterBuiltinTasks(wrong_station,wrong,composed_ports);
  bool station_rejected=false;
  try {agx_task_workflow::PrepareRecipe(repeated,wrong_station);}
  catch(const agx_task_workflow::PreparationError&){station_rejected=true;}
  assert(station_rejected);
  // Full metadata remains unbound during description and has no preparation I/O.
  const auto full_description=agx_task_workflow::DescribeTask(composed_registry,"navigate");
  for (const auto& item:full_description.interfaces)
    if (item.key=="arm_joint_state") assert(item.endpoint.empty());
  destinations.clear(); cancel=true;
  assert(composed_recipe.tasks[0].execute(phase).failure==agx_task_workflow::FailureKind::Cancelled && destinations.empty());
  cancel=false;
  composed.admitted_navigation.erase("next");
  agx_task_workflow::TaskRegistry missing_admission;RegisterBuiltinTasks(missing_admission,composed,composed_ports);
  bool admission_rejected=false;
  try {agx_task_workflow::PrepareRecipe(repeated,missing_admission);} catch(const agx_task_workflow::PreparationError&) {admission_rejected=true;}
  assert(admission_rejected && destinations.empty()); // Production cannot fall back to legacy navigation.
}
