#pragma once
#include "agx_task_workflow/task_registry.hpp"
#include "agx_tube_tasks/task_config.hpp"

namespace agx_tube_tasks {
struct TubeTaskPorts {
  std::function<agx_task_workflow::StepResult(const GraspParameters&)> grasp;
  std::function<bool()> cancelled;
  std::function<agx_task_workflow::StepResult()> entry_check;
  std::function<agx_task_workflow::StepResult(double)> stop, cleanup;
  // Bound at preparation; called only after the entire recipe is prepared.
  std::function<agx_task_workflow::StepResult(const std::string&,const GraspParameters&)> admit;
  std::function<void(const GraspParameters&)> begin;
  std::function<bool(const agx_task_workflow::StepResult&)> continuation_check;
};
void RegisterTubeTasks(agx_task_workflow::TaskRegistry&,const TubeInstallation&,const TubeTaskPorts&);
}  // namespace agx_tube_tasks
