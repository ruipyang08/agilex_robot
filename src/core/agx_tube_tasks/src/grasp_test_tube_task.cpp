#include "agx_robot_task_runtime/robot_sequence_plan.hpp"
#include "agx_robot_task_runtime/task_metadata.hpp"
#include "agx_tube_tasks/grasp_test_tube_task.hpp"
#include "agx_xarm_pick/resolved_tube_config.hpp"

namespace agx_tube_tasks {
namespace {
bool Possible(const agx_task_workflow::PreparationState& state, agx_task_workflow::ExpectedEffectKind kind) {
  for (const auto& effect:state.conditional_effects) if (effect.effect.kind==kind) return true;
  return false;
}
void BindLifecycle(agx_task_workflow::PreparedTask& task, const TubeTaskPorts& ports) {
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
void RegisterTubeTasks(agx_task_workflow::TaskRegistry& registry, const TubeInstallation& installed, const TubeTaskPorts& ports) {
  agx_task_workflow::TaskDefinition grasp;
  grasp.type="grasp_test_tube";
  grasp.description="Grasp one supported test tube while supervising a held base";
  grasp.inputs={{"fixture_profile",agx_task_workflow::InputType::String,false,"","Legacy fixture selection; mutually exclusive with common config path"},
                {"cap_color",agx_task_workflow::InputType::String,true,"","Supported target selection color; not object identity"}};
  grasp.interfaces=BaseInterfaces();
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"tube_detector","","agx_pick_msg/DetectTubeCapConfigured"});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"tube_alignment_target","/select_tube_target_2d_configured","agx_pick_msg/SelectTubeTarget2DConfigured"});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"lift_control","","bt_task_msgs/LiftMotorSrv"});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"lift_state","","bt_task_msgs/LiftMotorMsg"});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"arm_joint_state","","sensor_msgs/JointState"});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Action,"arm_execution","/execute_trajectory","moveit_msgs/ExecuteTrajectoryAction"});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Action,"arm_planning","/move_group","moveit_msgs/MoveGroupAction"});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Service,"arm_cartesian_path","/compute_cartesian_path","moveit_msgs/GetCartesianPath"});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"arm_stop","/trajectory_execution_event","std_msgs/String",agx_task_workflow::InterfaceDirection::Command});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"gripper_control","","dh_gripper_msgs/GripperCtrl",agx_task_workflow::InterfaceDirection::Command});
  grasp.interfaces.push_back({agx_task_workflow::InterfaceKind::Topic,"gripper_state","","dh_gripper_msgs/GripperState"});
  grasp.resources={agx_robot_task_runtime::resources::Base,agx_robot_task_runtime::resources::Arm,agx_robot_task_runtime::resources::Lift,agx_robot_task_runtime::resources::Gripper,agx_robot_task_runtime::resources::Camera};
  grasp.requirements={{agx_robot_task_runtime::requirements::HeldArrival,agx_task_workflow::RequirementTiming::Continuous,"fresh held arrival and measured stopped base"},
                      {agx_robot_task_runtime::requirements::GripperEmpty,agx_task_workflow::RequirementTiming::Entry,"fresh confirmed empty gripper"},
                      {agx_robot_task_runtime::requirements::FixtureCapability,agx_task_workflow::RequirementTiming::Entry,"loaded supported fixture and tube profile"},
                      {agx_task_workflow::requirements::CommonSupervision,agx_task_workflow::RequirementTiming::Continuous,"fresh actuator feedback and watchdog"}};
  grasp.effects={{agx_robot_task_runtime::effects::GripperHolding,agx_task_workflow::EffectCondition::VerifiedPhysicalEvent,"verified capture survives later extraction/return failure"}};
  auto configs=std::make_shared<TubeConfigResolver>(installed);
  grasp.prepare=[configs,ports,interfaces=grasp.interfaces](const agx_task_workflow::TaskSpec& task,agx_task_workflow::PreparationState& state) {
    auto plan=state.planning.Get<agx_robot_task_runtime::RobotSequencePlan>(agx_robot_task_runtime::kRobotSequencePlan);
    const auto request=configs->Resolve(task);
    if (!plan.held_station) throw std::invalid_argument("grasp requires a preceding verified navigation/hold task");
    if (!request.station_id.empty() && request.station_id!=*plan.held_station)
      throw std::invalid_argument("grasp fixture is installed at a different planned station");
    if (plan.payload_may_be_retained || Possible(state,agx_robot_task_runtime::effects::GripperHolding))
      throw std::invalid_argument("grasp requires an empty gripper; payload handling is not implemented");
    if (!ports.grasp || !ports.cancelled) throw std::invalid_argument("grasp execution ports are not bound");
    plan.payload_may_be_retained=true;
    state.planning.Set(agx_robot_task_runtime::kRobotSequencePlan,plan);
    agx_task_workflow::PreparedTask bound{task.id,task.type,"GRASP",[ports,request](const agx_task_workflow::PhaseSink&) {
      if (ports.cancelled()) return agx_task_workflow::StepResult{false,"cancelled"};
      return ports.grasp(request);
    }};
    BindLifecycle(bound,ports);
    if(ports.admit)bound.admission_check=[ports,request,id=task.id]{return ports.admit(id,request);};
    if(ports.begin)bound.begin=[ports,request]{ports.begin(request);};
    bound.continuation_check=ports.continuation_check;
    if (request.config) {
      const auto& config=*request.config;
      bound.required_interfaces=interfaces;
      for (auto& dependency:bound.required_interfaces) {
        if (dependency.key=="tube_detector") dependency.endpoint=config.robot.detector_service+"_configured";
        else if (dependency.key=="lift_control") dependency.endpoint=config.motion.lift_service_name;
        else if (dependency.key=="lift_state") dependency.endpoint=config.motion.lift_state_topic;
        else if (dependency.key=="arm_joint_state") dependency.endpoint=config.robot.joint_state_topic;
        else if (dependency.key=="gripper_control") dependency.endpoint=config.robot.gripper_command_topic;
        else if (dependency.key=="gripper_state") dependency.endpoint=config.robot.gripper_state_topic;
      }
    }
    return bound;
  };
  agx_task_workflow::RegisterTask(registry,std::move(grasp));
}
}  // namespace agx_tube_tasks
