#pragma once
#include <map>
#include <stdexcept>
#include "agx_task_workflow/planning_state.hpp"
#include "agx_task_workflow/config.hpp"
#include "agx_task_workflow/runner.hpp"
#include "agx_task_workflow/task_contract.hpp"

namespace agx_task_workflow {
struct PreparedTask {
  std::string id, type;
  std::string entry_phase = "TASK_RUNNING";
  std::function<StepResult(const PhaseSink&)> execute;
  FailurePolicy failure_policy = FailurePolicy::Stop;
  std::function<StepResult()> entry_check;
  std::function<StepResult(double timeout_s)> stop, cleanup;
  double stop_timeout_s = 5.0;
  std::vector<InterfaceRequirement> required_interfaces;
  std::vector<TaskResource> resources;
  std::vector<StateRequirement> requirements;
  std::vector<PlannedEffect> expected_effects;
  bool metadata_complete = false;
  std::uint32_t contract_version = 0;
  // Optional read-only installation check, invoked by the host only after the
  // entire recipe prepares successfully. No command or worker dispatch here.
  std::function<StepResult()> admission_check;
  // Select this invocation's runtime adapters after host context activation.
  // Must return promptly and may not issue motion commands.
  std::function<void()> begin;
  // Explicit certification of this task's owned effects/work after bounded
  // stop and cleanup. The host also checks its persistent shared invariants.
  // Absence is not permission to continue after an ordinary failure.
  std::function<bool(const StepResult&)> continuation_check;
};
// Planned effects only. Adapters must separately verify physical prerequisites.
struct PreparationState {
  PlanningState planning;
  std::vector<PlannedEffect> conditional_effects;
};
using TaskRegistry = std::map<std::string, TaskDefinition>;
struct PreparedRecipe {
  std::vector<PreparedTask> tasks;
  std::vector<InterfaceRequirement> required_interfaces;
  std::vector<TaskResource> resources;
};
class PreparationError : public std::invalid_argument {
 public:
  PreparationError(const TaskSpec& task, const std::string& reason)
    : std::invalid_argument("task '"+task.id+"' ("+task.type+"): "+reason),
      task_id(task.id), task_type(task.type) {}
  const std::string task_id, task_type;
};
void RegisterTask(TaskRegistry&, const std::string& type, TaskFactory);
void RegisterTask(TaskRegistry&, TaskDefinition);
TaskDefinition DescribeTask(const TaskRegistry&, const std::string& type);
PreparedRecipe PrepareRecipe(const Config&, const TaskRegistry&);
std::vector<PreparedTask> PrepareTasks(const Config&, const TaskRegistry&);
}
