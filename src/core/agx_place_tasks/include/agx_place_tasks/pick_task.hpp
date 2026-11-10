#pragma once

#include <array>
#include <functional>
#include <optional>
#include <memory>
#include <string>

#include "agx_place_tasks/onboard_rack_profile.hpp"
#include "agx_place_tasks/onboard_positions.hpp"
#include "agx_place_tasks/place_task.hpp"  // shares PlaceInstallation
#include "agx_task_workflow/task_registry.hpp"

namespace agx_place_tasks {

struct PickParameters {
  std::size_t slot = 0;
  // Immutable snapshot taken at preparation; never re-read during execution.
  std::shared_ptr<const OnboardRackProfile> rack;
  // Set when the recipe names a measured position instead of a taught slot.
  std::optional<PositionTarget> measured;
  std::string measured_frame;
  std::array<double, 3> measured_rpy_deg{{180.0, 0.0, 0.0}};
  double measured_yaw_offset_deg = 0.0;
  bool uses_measured_position = false;
};

struct PickTaskPorts {
  // Optional measured position table; required only for `position:` recipes.
  std::shared_ptr<const PositionTable> positions;
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
