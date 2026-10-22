#pragma once
#include "agx_tube_tasks/grasp_test_tube_task.hpp"
#include "agx_tube_tasks/grasp_task.hpp"
#include "agx_xarm_pick/tube_mission_runner.hpp"

namespace agx_tube_tasks {
// One invocation's execution bindings. The application owns the shared robot
// and ROS connections; this module owns custody admission, effect delivery and
// outcome reconciliation. Callbacks retained by a mission worker must retain
// their owner through an unjoined-worker shutdown.
struct GraspExecutionPorts {
  std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state;
  std::string run_id, task_id;
  std::function<bool()> held_base;
  std::function<void()> refresh_custody;
  std::function<agxcore::MonitoredTubeResult(const GraspParameters&,agxcore::TubeEffectSink)> run;
  std::function<agx_task_workflow::FailureKind()> runtime_failure;
  std::function<std::string()> stop_reason;
  std::function<void(const std::shared_ptr<GraspEffectAdapter>&)> effects_bound;
  // Mandatory: retain observer and terminal worker obligations before Finish.
  std::function<void(const agxcore::MonitoredTubeResult&)> mission_returned;
  std::function<void()> effects_finished;
};
agx_task_workflow::StepResult ExecuteGrasp(const GraspParameters&,const GraspExecutionPorts&);
}  // namespace agx_tube_tasks
