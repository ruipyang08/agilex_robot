#include "agx_tube_tasks/grasp_test_tube_task.hpp"
#include "agx_robot_task_runtime/robot_sequence_plan.hpp"
#include <cassert>
#include <chrono>
#include <fstream>
using namespace agx_tube_tasks;
using namespace agx_task_workflow;
int main() {
  const auto source=std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/
      "agx_xarm_pick/test/fixtures/tube_task_config";
  const auto directory=std::filesystem::temp_directory_path()/
      ("tube-config-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::copy(source,directory,std::filesystem::copy_options::recursive);
  int executions=0,admissions=0;
  std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig> selected;
  TubeTaskPorts ports;ports.cancelled=[] {return false;};
  ports.grasp=[&](const GraspParameters& request) {
    ++executions;selected=request.config;return StepResult{true,request.fixture_profile};
  };
  ports.admit=[&](const auto&,const auto&){++admissions;return StepResult{true,"read-only admission"};};
  TaskRegistry registry;RegisterTubeTasks(registry,{},ports);
  const auto definition=DescribeTask(registry,"grasp_test_tube");
  auto spec=ParseConfig("schema_version: 2\nworkflow_id: tubes\ntasks: [{id: pick,type: grasp_test_tube,cap_color: green}]\n").tasks[0];
  spec.config_path=directory/"taskA.yaml";
  // A module consumer supplies the shared prospective arrival state. Physical
  // held-arrival evidence remains a separate execution prerequisite.
  auto snapshot=agx_xarm_pick::ResolveTubeTaskConfig(spec.config_path->string());
  PreparationState state;
  agx_robot_task_runtime::RobotSequencePlan plan;plan.held_station=snapshot->installation.station_id;
  state.planning.Set(agx_robot_task_runtime::kRobotSequencePlan,plan);
  auto prepared=definition.prepare(spec,state);
  assert(executions==0 && admissions==0);
  std::filesystem::remove_all(directory);
  assert(prepared.admission_check().ok && prepared.execute({}).ok);
  assert(selected && selected->config_id==snapshot->config_id);
  spec.config_path=source/"taskA.yaml";
  PreparationState absent;
  try {definition.prepare(spec,absent);assert(false);}catch(const std::invalid_argument&){}
  // A valid file at another station must not be relabeled as the same grasp.
  spec.config_path=source/"taskB.yaml";
  PreparationState wrong;wrong.planning.Set(agx_robot_task_runtime::kRobotSequencePlan,plan);
  try {definition.prepare(spec,wrong);assert(false);}catch(const std::invalid_argument&){}
}
