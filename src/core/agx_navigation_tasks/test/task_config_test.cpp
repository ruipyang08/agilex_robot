#include "agx_navigation_tasks/navigate_task.hpp"
#include <cassert>
#include <fstream>
#include <iterator>
#include <chrono>
#include <vector>
using namespace agx_navigation_tasks;
using namespace agx_task_workflow;
int main() {
  const auto directory=std::filesystem::temp_directory_path()/
      ("navigation-config-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(directory);
  const auto fixture=std::filesystem::path(__FILE__).parent_path()/"travel_profile_fixture.yaml";
  std::ifstream input(fixture);const std::string text{std::istreambuf_iterator<char>(input),{}};
  auto alternate=YAML::Load(text);alternate["profile_id"]="alternate";
  std::ofstream(directory/"default.yaml")<<text;
  std::ofstream(directory/"selected.yaml")<<alternate;
  std::ofstream(directory/"bad.yaml")<<"typo: true\n";
  NavigationInstallation installation;
  installation.workbenches={"destination","destination_b"};installation.default_travel_profile=directory/"default.yaml";
  installation.execution_mode=TravelMode::Simulation;
  int executions=0,admissions=0,begins=0;
  std::shared_ptr<const TravelProfile> selected;
  std::vector<std::string> selected_profiles;
  std::vector<std::string> selected_targets;
  NavigateTaskPorts ports;
  ports.cancelled=[] {return false;};
  ports.composed_navigation=[&](const NavigateParameters& request,const PhaseSink&) {
    ++executions;selected=request.travel_profile;
    selected_profiles.push_back(request.travel_profile->profile_id);
    selected_targets.push_back(request.target);
    return StepResult{true,request.target};
  };
  ports.admit=[&](const auto&,const auto&){++admissions;return StepResult{true,"read-only admission"};};
  ports.begin=[&](const auto&){++begins;};
  TaskRegistry registry;RegisterNavigationTasks(registry,installation,ports);
  auto config=ParseConfig("schema_version: 2\nworkflow_id: navigation\ntasks: [{id: first,type: navigate,target: destination}]\n");
  auto defaults=PrepareRecipe(config,registry);
  config.tasks[0].config_path=directory/"selected.yaml";
  const auto prepared=PrepareRecipe(config,registry);
  auto two_legs=ParseConfig("schema_version: 2\nworkflow_id: two_legs\ntasks:\n"
      "  - {id: approach_a, type: navigate, target: destination}\n"
      "  - {id: travel_b, type: navigate, target: destination_b}\n");
  two_legs.tasks[0].config_path=directory/"default.yaml";
  two_legs.tasks[1].config_path=directory/"selected.yaml";
  const auto independent=PrepareRecipe(two_legs,registry);
  assert(independent.tasks[0].id=="approach_a" && independent.tasks[1].id=="travel_b");
  assert(executions==0 && admissions==0 && begins==0);
  auto bad=config.tasks[0];bad.id="bad";bad.config_path=directory/"bad.yaml";config.tasks.push_back(bad);
  try {PrepareRecipe(config,registry);assert(false);}catch(const PreparationError& error){assert(error.task_id=="bad");}
  assert(executions==0 && admissions==0 && begins==0);
  // Files may change after preparation: execution retains the admitted snapshot.
  std::filesystem::remove_all(directory);
  assert(prepared.tasks[0].admission_check().ok);prepared.tasks[0].begin();
  assert(prepared.tasks[0].execute({}).ok && selected->profile_id=="alternate");
  assert(defaults.tasks[0].execute({}).ok && selected->profile_id!="alternate");
  assert(admissions==1 && begins==1);
  // Both prepared navigation invocations retain their own selected profile;
  // one invocation cannot inherit the other's choice.
  assert(independent.tasks[0].execute({}).ok);
  assert(independent.tasks[1].execute({}).ok);
  assert((selected_profiles==std::vector<std::string>{"alternate","synthetic_test_only",
      "synthetic_test_only","alternate"}));
  assert((selected_targets==std::vector<std::string>{"destination","destination",
      "destination","destination_b"}));
  // Explicit simulation/hardware qualification stays enforced by the module.
  installation.default_travel_profile=fixture;installation.execution_mode=TravelMode::Hardware;
  TaskRegistry hardware;RegisterNavigationTasks(hardware,installation,ports);
  config.tasks.resize(1);config.tasks[0].config_path.reset();
  try {PrepareRecipe(config,hardware);assert(false);}catch(const PreparationError&){}
}
