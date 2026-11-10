#pragma once

#include <functional>
#include <memory>
#include <string>

#include "agx_place_tasks/pick_task.hpp"
#include "agx_place_tasks/place_execution.hpp"
#include "agx_robot_task_runtime/robot_state.hpp"

namespace agx_place_tasks {

// What the motion adapter measured. Only `grasped` may establish custody.
struct PickMeasurement {
  bool grasped = false;
  bool grasp_uncertain = false;
  bool worker_joined = false;
  bool measured_stop_confirmed = false;
  std::string detail;
  PlaceEvidence evidence;
};

struct PickExecutionPorts {
  std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state;
  std::string run_id, task_id;
  std::function<bool()> held_base;
  std::function<void()> refresh_custody;
  std::function<PickMeasurement(const PickParameters&)> run;
  std::function<agx_task_workflow::FailureKind()> runtime_failure;
  std::function<std::string()> stop_reason;
};

// Records custody only for a measured grasp; an empty slot produces no effect.
agx_task_workflow::StepResult ExecutePick(const PickParameters&, const PickExecutionPorts&);

}  // namespace agx_place_tasks
