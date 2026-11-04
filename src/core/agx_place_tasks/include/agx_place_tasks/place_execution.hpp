#pragma once

#include <functional>
#include <memory>
#include <string>

#include "agx_place_tasks/place_task.hpp"
#include "agx_robot_task_runtime/robot_state.hpp"

namespace agx_place_tasks {

// Source time and receipt time of the measurement that produced the fields
// below. Both are taken by the adapter from the measurement itself.
struct PlaceEvidence {
  std::string source;
  double source_stamp_s = 0.0;
  double receipt_steady_s = 0.0;
};

// Every field is a measurement taken by the motion adapter. Command echoes,
// planned goal tolerance and "execution finished" are not measurements.
struct PlaceMotionOutcome {
  bool released = false;           // measured open gripper after the descent
  bool release_uncertain = false;  // opening was attempted and cannot be excluded
  bool slot_occupancy_measured = false;
  bool slot_occupancy_confirmed = false;
  bool worker_joined = false;
  bool measured_stop_confirmed = false;
  std::string detail;
  PlaceEvidence evidence;
};

struct PlaceExecutionPorts {
  std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state;
  std::string run_id, task_id;
  std::function<bool()> held_base;
  std::function<void()> refresh_custody;
  std::function<PlaceMotionOutcome(const PlaceParameters&)> run;
  std::function<agx_task_workflow::FailureKind()> runtime_failure;
  std::function<std::string()> stop_reason;
};

// Applies custody effects only after measurement. A release without measured
// slot occupancy is recorded as an uncertain location instead of a placement.
agx_task_workflow::StepResult ExecutePlace(const PlaceParameters&, const PlaceExecutionPorts&);

}  // namespace agx_place_tasks
