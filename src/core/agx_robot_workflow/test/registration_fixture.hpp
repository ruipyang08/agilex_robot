// Test-only compact bindings for synthetic installed selections.
#pragma once
#include "agx_task_workflow/task_registry.hpp"
#include <memory>

#include "agx_navigation_tasks/navigate_task.hpp"
#include "agx_tube_tasks/grasp_test_tube_task.hpp"

namespace robot_workflow_test {
using NavigateParameters=agx_navigation_tasks::NavigateParameters;
using GraspParameters=agx_tube_tasks::GraspParameters;
using TravelProfile=agx_navigation_tasks::TravelProfile;
struct InstalledSelections {
  std::set<std::string> workbenches;
  std::string fixture_profile, cap_color;
  std::map<std::string,GraspParameters> admitted_grasps;
  std::map<std::string,std::shared_ptr<const TravelProfile>> admitted_navigation;
};
struct BuiltinPorts {
  // Synthetic legacy bindings. Production supplies composed_navigation and
  // an admitted profile for every navigate invocation; missing admission is terminal.
  std::function<agx_task_workflow::StepResult(const NavigateParameters&)> navigate;
  std::function<agx_task_workflow::StepResult()> verify_stop, hold_base;
  std::function<agx_task_workflow::StepResult(const GraspParameters&)> grasp;
  std::function<bool()> cancelled;
  std::function<agx_task_workflow::StepResult(const std::string& task_type)> entry_check;
  std::function<agx_task_workflow::StepResult(double timeout_s)> stop, cleanup;
  std::function<agx_task_workflow::StepResult(const NavigateParameters&,const agx_task_workflow::PhaseSink&)> composed_navigation;
};
inline void RegisterBuiltinTasks(agx_task_workflow::TaskRegistry& registry,
                          const InstalledSelections& installed,const BuiltinPorts& ports) {
  agx_navigation_tasks::NavigateTaskPorts navigation;
  navigation.navigate=ports.navigate;navigation.verify_stop=ports.verify_stop;
  navigation.hold_base=ports.hold_base;navigation.composed_navigation=ports.composed_navigation;
  navigation.cancelled=ports.cancelled;navigation.stop=ports.stop;navigation.cleanup=ports.cleanup;
  if(ports.entry_check)navigation.entry_check=[ports]{return ports.entry_check("navigate");};
  agx_navigation_tasks::RegisterNavigationTasks(registry,
      {installed.workbenches,installed.admitted_navigation},navigation);
  agx_tube_tasks::TubeTaskPorts tubes;
  tubes.grasp=ports.grasp;tubes.cancelled=ports.cancelled;tubes.stop=ports.stop;tubes.cleanup=ports.cleanup;
  if(ports.entry_check)tubes.entry_check=[ports]{return ports.entry_check("grasp_test_tube");};
  agx_tube_tasks::RegisterTubeTasks(registry,
      {installed.fixture_profile,installed.cap_color,installed.admitted_grasps},tubes);
}

}  // namespace robot_workflow_test
