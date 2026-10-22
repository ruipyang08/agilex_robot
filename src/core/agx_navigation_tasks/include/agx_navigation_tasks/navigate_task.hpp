#pragma once
#include "agx_task_workflow/task_registry.hpp"
#include "agx_navigation_tasks/task_config.hpp"

namespace agx_navigation_tasks {
struct NavigateTaskPorts {
  std::function<agx_task_workflow::StepResult(const NavigateParameters&)> navigate;
  std::function<agx_task_workflow::StepResult()> verify_stop, hold_base;
  std::function<agx_task_workflow::StepResult(const NavigateParameters&,const agx_task_workflow::PhaseSink&)> composed_navigation;
  std::function<bool()> cancelled;
  std::function<agx_task_workflow::StepResult()> entry_check;
  std::function<agx_task_workflow::StepResult(double)> stop, cleanup;
  // Bound at preparation; called only after the entire recipe is prepared.
  std::function<agx_task_workflow::StepResult(const std::string&,const NavigateParameters&)> admit;
  std::function<void(const NavigateParameters&)> begin;
  std::function<bool(const agx_task_workflow::StepResult&)> continuation_check;
};
void RegisterNavigationTasks(agx_task_workflow::TaskRegistry&,const NavigationInstallation&,const NavigateTaskPorts&);
}  // namespace agx_navigation_tasks
