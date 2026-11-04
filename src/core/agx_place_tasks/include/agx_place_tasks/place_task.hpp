#pragma once

#include <functional>
#include <memory>
#include <string>

#include "agx_place_tasks/onboard_rack_profile.hpp"
#include "agx_task_workflow/task_registry.hpp"

namespace agx_place_tasks {

struct PlaceParameters {
  std::size_t slot = 0;
  // Immutable snapshot taken at preparation; never re-read during execution.
  std::shared_ptr<const OnboardRackProfile> rack;
};

// The onboard rack is installed once. A missing or unqualified profile is a
// configuration error discovered at preparation, never a runtime surprise.
struct PlaceInstallation {
  std::shared_ptr<const OnboardRackProfile> rack;
};

struct PlaceTaskPorts {
  std::function<agx_task_workflow::StepResult(const PlaceParameters&)> place;
  std::function<bool()> cancelled;
  std::function<agx_task_workflow::StepResult()> entry_check;
  std::function<agx_task_workflow::StepResult(double)> stop, cleanup;
  // Bound at preparation; called only after the entire recipe is prepared.
  std::function<agx_task_workflow::StepResult(const std::string&, const PlaceParameters&)> admit;
  std::function<void(const PlaceParameters&)> begin;
  std::function<bool(const agx_task_workflow::StepResult&)> continuation_check;
};

void RegisterPlaceTasks(agx_task_workflow::TaskRegistry&, const PlaceInstallation&, const PlaceTaskPorts&);

}  // namespace agx_place_tasks
