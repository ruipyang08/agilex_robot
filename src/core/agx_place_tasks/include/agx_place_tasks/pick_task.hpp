#pragma once

#include <functional>
#include <memory>
#include <string>

#include "agx_place_tasks/onboard_rack_profile.hpp"
#include "agx_place_tasks/place_task.hpp"  // shares PlaceInstallation
#include "agx_task_workflow/task_registry.hpp"

namespace agx_place_tasks {

struct PickParameters {
  std::size_t slot = 0;
  // Immutable snapshot taken at preparation; never re-read during execution.
  std::shared_ptr<const OnboardRackProfile> rack;
};

struct PickTaskPorts {
  std::function<agx_task_workflow::StepResult(const PickParameters&)> pick;
  std::function<bool()> cancelled;
  std::function<agx_task_workflow::StepResult()> entry_check;
  std::function<agx_task_workflow::StepResult(double)> stop, cleanup;
  // Bound at preparation; called only after the entire recipe is prepared.
  std::function<agx_task_workflow::StepResult(const std::string&, const PickParameters&)> admit;
  std::function<void(const PickParameters&)> begin;
  std::function<bool(const agx_task_workflow::StepResult&)> continuation_check;
};

void RegisterPickTasks(agx_task_workflow::TaskRegistry&, const PlaceInstallation&, const PickTaskPorts&);

}  // namespace agx_place_tasks
