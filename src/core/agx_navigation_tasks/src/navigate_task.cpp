#include "agx_robot_task_runtime/robot_sequence_plan.hpp"
#include "agx_robot_task_runtime/task_metadata.hpp"
#include "agx_navigation_tasks/navigate_task.hpp"
#include "agx_navigation_tasks/travel_profile.hpp"

namespace agx_navigation_tasks {
namespace {
bool Possible(const agx_task_workflow::PreparationState& state, agx_task_workflow::ExpectedEffectKind kind) {
  for (const auto& effect:state.conditional_effects) if (effect.effect.kind==kind) return true;
  return false;
}
void BindLifecycle(agx_task_workflow::PreparedTask& task, const NavigateTaskPorts& ports) {
  task.entry_check=[ports] {
    if (ports.entry_check) return ports.entry_check();
    return agx_task_workflow::StepResult{false,"measured entry guard adapter is not connected"};
  };
  task.stop=[ports](double timeout) {
    if (ports.stop) return ports.stop(timeout);
    agx_task_workflow::StepResult result{false,"bounded stop adapter is not connected"};
    result.failure=agx_task_workflow::FailureKind::UnconfirmedStop;return result;
  };
  task.cleanup=[ports](double timeout) {
    if (ports.cleanup) return ports.cleanup(timeout);
    return agx_task_workflow::StepResult{false,"bounded cleanup adapter is not connected"};
  };
}
std::vector<agx_task_workflow::InterfaceRequirement> BaseInterfaces() {
  return {{agx_task_workflow::InterfaceKind::Topic,"base_authority","/workbench_nav/authority_state","agx_workbench_navigation/BaseAuthorityState"},
          {agx_task_workflow::InterfaceKind::Topic,"base_odometry","/odom","nav_msgs/Odometry"},
          {agx_task_workflow::InterfaceKind::Topic,"base_command","/cmd_vel","geometry_msgs/Twist"},
          {agx_task_workflow::InterfaceKind::Service,"base_hold","/workbench_nav/hold_for_manipulation","std_srvs/Trigger"},
          {agx_task_workflow::InterfaceKind::Topic,"workflow_watchdog","/workbench_workflow/watchdog_ready","std_msgs/Bool"}};
}
}  // namespace
void RegisterNavigationTasks(agx_task_workflow::TaskRegistry& registry, const NavigationInstallation& installed, const NavigateTaskPorts& ports) {
  agx_task_workflow::TaskDefinition navigation;
  navigation.type="navigate";
  navigation.description="Prepare qualified empty or single-tube travel, depart under a readiness lease, and establish a fresh destination hold";
  navigation.inputs={{"target",agx_task_workflow::InputType::String,true,"","Registered workbench ID"}};
  navigation.interfaces=BaseInterfaces();
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Action,"navigate","/workbench_nav/navigate","agx_workbench_navigation/NavigateToWorkbenchAction"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"navigation_route","/workbench_nav/prepare_navigation_route","agx_workbench_navigation/PrepareNavigationRoute"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"navigation_goal","/move_base/goal","move_base_msgs/MoveBaseActionGoal"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"navigation_status","/move_base/status","actionlib_msgs/GoalStatusArray"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"base_acquire_hold","/workbench_nav/acquire_base_hold","agx_workbench_navigation/AcquireBaseHold"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"base_prepare_departure","/workbench_nav/prepare_base_departure","agx_workbench_navigation/PrepareBaseDeparture"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"base_release_hold","/workbench_nav/release_base_hold","agx_workbench_navigation/ReleaseBaseHold"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"travel_readiness","/workbench_nav/travel_readiness","agx_workbench_navigation/TravelReadiness",agx_task_workflow::InterfaceDirection::Command});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"lift_control","","bt_task_msgs/LiftMotorSrv"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"lift_state","","bt_task_msgs/LiftMotorMsg"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"arm_joint_state","","sensor_msgs/JointState"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"gripper_state","","dh_gripper_msgs/GripperState"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Action,"arm_execution","/execute_trajectory","moveit_msgs/ExecuteTrajectoryAction"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Action,"arm_planning","/move_group","moveit_msgs/MoveGroupAction"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"arm_planning_scene","/get_planning_scene","moveit_msgs/GetPlanningScene"});
  navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"arm_stop","/trajectory_execution_event","std_msgs/String",agx_task_workflow::InterfaceDirection::Command});
  for (const auto& component:{"local_planner","global_planner","base_arbiter","docking","lidar"})
    navigation.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,std::string("effective_")+component,
        std::string("/workbench_nav/effective_config/")+component,"std_msgs/String"});
  navigation.resources={agx_robot_task_runtime::resources::Base,agx_robot_task_runtime::resources::Arm,agx_robot_task_runtime::resources::Lift,agx_robot_task_runtime::resources::Gripper,agx_robot_task_runtime::resources::Camera};
  navigation.requirements={{agx_robot_task_runtime::requirements::SupportedTransport,agx_task_workflow::RequirementTiming::Entry,"fresh known empty or one supported held tube, with no unresolved transfer"},
                           {agx_robot_task_runtime::requirements::SafeClearance,agx_task_workflow::RequirementTiming::Entry,"measured named starting posture admits the qualified path to default"},
                           {agx_robot_task_runtime::requirements::StationCapability,agx_task_workflow::RequirementTiming::Entry,"destination is registered for navigation"},
                           {agx_task_workflow::requirements::CommonSupervision,agx_task_workflow::RequirementTiming::Continuous,"fresh base authority, localization, posture, retained payload, clearance and watchdog"}};
  navigation.effects={{agx_robot_task_runtime::effects::TransportReady,agx_task_workflow::EffectCondition::VerifiedPhysicalEvent,"measured default posture and supported custody establish transport readiness"},
                      {agx_robot_task_runtime::effects::HeldArrival,agx_task_workflow::EffectCondition::TaskSucceeded,"fresh destination hold after measured stop"}};
  auto configs=std::make_shared<NavigationConfigResolver>(installed);
  navigation.prepare=[configs,ports,interfaces=navigation.interfaces](const agx_task_workflow::TaskSpec& task,agx_task_workflow::PreparationState& state) {
    auto plan=state.planning.Get<agx_robot_task_runtime::RobotSequencePlan>(agx_robot_task_runtime::kRobotSequencePlan);
    const auto request=configs->Resolve(task);
    if (request.travel_profile) {
      if (!ports.composed_navigation || !ports.cancelled)
        throw std::invalid_argument("reusable navigation profile or physical handoff adapter is missing");
      ValidateTravelProfile(*request.travel_profile);
      if (!request.travel_profile->simulation.enabled && !request.travel_profile->hardware.enabled)
        throw std::invalid_argument("navigation profile has no qualified execution mode");
      plan.held_station=request.target;
      state.planning.Set(agx_robot_task_runtime::kRobotSequencePlan,plan);
      agx_task_workflow::PreparedTask bound{task.id,task.type,"NAVIGATE",[ports,request](const agx_task_workflow::PhaseSink& phase) {
        if (ports.cancelled()) {
          agx_task_workflow::StepResult result{false,"navigation cancelled before physical handoff"};
          result.failure=agx_task_workflow::FailureKind::Cancelled; return result;
        }
        return ports.composed_navigation(request,phase);
      }};
      bound.required_interfaces=interfaces;
      for (auto& dependency:bound.required_interfaces) {
        const auto& robot=request.travel_profile->robot;
        if (dependency.key=="lift_control") dependency.endpoint=robot.lift_service;
        else if (dependency.key=="lift_state") dependency.endpoint=robot.lift_state_topic;
        else if (dependency.key=="arm_joint_state") dependency.endpoint=robot.arm_joint_state_topic;
        else if (dependency.key=="gripper_state") dependency.endpoint=robot.gripper_state_topic;
      }
      BindLifecycle(bound,ports);
      if(ports.admit)bound.admission_check=[ports,request,id=task.id]{return ports.admit(id,request);};
      if(ports.begin)bound.begin=[ports,request]{ports.begin(request);};
      bound.continuation_check=ports.continuation_check;
      return bound;
    }
    if (task.config_path || ports.composed_navigation)
      throw std::invalid_argument("navigation requires an installed qualified travel_profile");
    if (plan.held_station || plan.payload_may_be_retained || Possible(state,agx_robot_task_runtime::effects::HeldArrival) ||
        Possible(state,agx_robot_task_runtime::effects::GripperHolding))
      throw std::invalid_argument("navigation after manipulation hold requires an implemented hold-release/transport task");
    plan.held_station=request.target;
    state.planning.Set(agx_robot_task_runtime::kRobotSequencePlan,plan);
    if (!ports.navigate || !ports.verify_stop || !ports.hold_base || !ports.cancelled)
      throw std::invalid_argument("navigation execution ports are not bound");
    agx_task_workflow::PreparedTask bound{task.id,task.type,"NAVIGATE",[ports,request](const agx_task_workflow::PhaseSink& phase) {
      if (ports.cancelled()) return agx_task_workflow::StepResult{false,"cancelled"};
      auto result=ports.navigate(request);
      if (!result.ok) return result;
      if (ports.cancelled()) return agx_task_workflow::StepResult{false,"cancelled"};
      phase("VERIFY_STOP");
      if (ports.cancelled()) return agx_task_workflow::StepResult{false,"cancelled"};
      result=ports.verify_stop();
      if (!result.ok) return result;
      if (ports.cancelled()) return agx_task_workflow::StepResult{false,"cancelled"};
      phase("HOLD_BASE");
      if (ports.cancelled()) return agx_task_workflow::StepResult{false,"cancelled"};
      return ports.hold_base();
    }};
    BindLifecycle(bound,ports);
    if(ports.admit)bound.admission_check=[ports,request,id=task.id]{return ports.admit(id,request);};
    if(ports.begin)bound.begin=[ports,request]{ports.begin(request);};
    bound.continuation_check=ports.continuation_check;
    return bound;
  };
  agx_task_workflow::RegisterTask(registry,std::move(navigation));

}
}  // namespace agx_navigation_tasks
