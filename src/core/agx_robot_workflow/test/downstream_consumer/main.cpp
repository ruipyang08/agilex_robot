#include "agx_robot_task_runtime/robot_task_ports.hpp"
#include <agx_navigation_tasks/navigate_task.hpp>
#include <agx_tube_tasks/grasp_test_tube_task.hpp>
#include <agx_task_workflow/execution_context.hpp>
#include <agx_task_workflow/task_runtime_ports.hpp>
#include <agx_navigation_tasks/travel_profile.hpp>
#include <agx_xarm_pick/tube_live_observer.hpp>
#include <agx_xarm_pick/resolved_tube_config.hpp>
#include <stdexcept>
#include <memory>

static void Require(bool condition) {
  if (!condition) throw std::runtime_error("public workflow consumer contract failed");
}

int main() {
  agxcore::TubeObservationBuffer observations({1,1,1,1000,0,5});
  Require(!observations.Snapshot(1,1).gripper.known_empty);
  // Dependencies arrive through the selected task modules, not an application
  // umbrella library. This consumer never links to the ROS application host.
  const auto travel=agx_navigation_tasks::ParseTravelProfile(
      "schema_version: 1\nprofile_id: disabled\nrobot: null\nparameters: null\n"
      "simulation: {enabled: false, evidence_id: unqualified}\n"
      "hardware: {enabled: false, evidence_id: unqualified}\n");
  Require(!agx_navigation_tasks::TravelEnabled(travel,agx_navigation_tasks::TravelMode::Hardware));
  Require(travel.config_id.size()==71 && travel.config_id.substr(0,7)=="sha256:");
  double now=1.0;
  auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([&] {return now;},[&] {return now;});
  agx_robot_task_runtime::RobotTaskRuntimeBinding runtime(state);
  auto context=std::make_shared<agx_task_workflow::ExecutionContext>(2.0,[&] {return now;},[] {return false;});
  runtime.Activate("consumer-run","consumer-task","consumer-run/consumer-task",context);
  const auto runtime_ports=runtime.Bind("consumer-task");
  Require(runtime_ports.check()==agx_task_workflow::FailureKind::None);
  Require(runtime_ports.deadline_steady_s()==3.0);
  Require(runtime_ports.snapshot().revision==0);
  runtime.Expire();
  Require(runtime_ports.check()==agx_task_workflow::FailureKind::Fault);
  int dispatched=0, lifecycle_calls=0;
  agx_task_workflow::TaskRegistry registry;
  // Merely registering built-ins must not require navigation/grasp configuration.
  agx_navigation_tasks::RegisterNavigationTasks(registry,agx_navigation_tasks::NavigationInstallation{},{});
  agx_tube_tasks::RegisterTubeTasks(registry,agx_tube_tasks::TubeInstallation{},{});
  agx_task_workflow::TaskDefinition definition;
  definition.type="inert";
  definition.description="A third-party task with no hardware dependencies";
  definition.inputs={{"value",agx_task_workflow::InputType::String,true,"","Value to record"}};
  definition.effects={{agx_task_workflow::effects::ObservationOnly,agx_task_workflow::EffectCondition::TaskSucceeded,"value recorded"}};
  definition.prepare=[&](const agx_task_workflow::TaskSpec& task,agx_task_workflow::PreparationState&) {
    const auto value=agx_task_workflow::RequireString(task.parameters["value"],"value");
    agx_task_workflow::PreparedTask bound{task.id,task.type,"TASK_RUNNING",[&,value](const agx_task_workflow::PhaseSink& phase) {
      ++dispatched;phase("RECORD_VALUE");return agx_task_workflow::StepResult{true,value};
    }};
    bound.entry_check=[&] {++lifecycle_calls;return agx_task_workflow::StepResult{true,"no actuator prerequisites"};};
    bound.stop=[&](double timeout) {
      Require(timeout>0);++lifecycle_calls;agx_task_workflow::StepResult result{true,"no owned motion"};
      result.stop=agx_task_workflow::StopState::Confirmed;return result;
    };
    bound.cleanup=[&](double timeout) {Require(timeout>0);++lifecycle_calls;return agx_task_workflow::StepResult{true,"no pending work"};};
    return bound;
  };
  agx_task_workflow::RegisterTask(registry,definition);
  const auto described=agx_task_workflow::DescribeTask(registry,"inert");
  Require(described.metadata_complete && described.contract_version==1 && described.interfaces.empty());
  auto config=agx_task_workflow::ParseConfig("schema_version: 2\nworkflow_id: consumer\ntasks: [{id: one,type: inert,value: example}]\n");
  const auto recipe=agx_task_workflow::PrepareRecipe(config,registry);
  Require(dispatched==0 && lifecycle_calls==0 && recipe.tasks.size()==1);
  Require(recipe.required_interfaces.empty() && recipe.resources.empty());
  auto invalid=config;
  invalid.tasks.push_back({"later","missing",YAML::Node(YAML::NodeType::Map)});
  bool rejected=false;
  try {agx_task_workflow::PrepareRecipe(invalid,registry);}catch(const agx_task_workflow::PreparationError&){rejected=true;}
  Require(rejected && dispatched==0 && lifecycle_calls==0);
  const auto result=agx_task_workflow::Run(recipe.tasks,{[]{return false;},[]{},[](const std::string&){},[](const agx_task_workflow::TaskResult&){}});
  Require(result.success && dispatched==1 && result.tasks[0].detail=="example");
  return 0;
}
