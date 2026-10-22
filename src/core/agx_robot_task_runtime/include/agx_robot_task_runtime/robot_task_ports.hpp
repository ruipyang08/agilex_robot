#pragma once
#include "agx_task_workflow/task_runtime_ports.hpp"
#include "agx_robot_task_runtime/robot_state.hpp"
#include "agx_robot_task_runtime/task_metadata.hpp"

namespace agx_robot_task_runtime {
struct RobotTaskPorts : agx_task_workflow::TaskRuntimePorts {
  std::function<RobotStateSnapshot()> snapshot;
  std::function<StateUpdate(StateRevision, const std::string&, const ObjectRef&,
                            std::optional<HolderRef>, std::optional<HolderRef>)> begin_transfer;
  std::function<StateUpdate(const PhysicalEffect&, const Evidence&,
                            const std::string&, StateRevision)> apply_effect;
  std::function<agx_task_workflow::StepResult(const std::string&)> held_arrival;
};
class RobotTaskRuntimeBinding {
 public:
  explicit RobotTaskRuntimeBinding(std::shared_ptr<RobotStateStore> state);
  void Activate(const std::string& run, const std::string& task, const std::string& operation,
      std::shared_ptr<agx_task_workflow::ExecutionContext> context,
      std::function<agx_task_workflow::StepResult(const std::string&)> held_arrival = {});
  RobotTaskPorts Bind(const std::string& task) const;
  void Expire();
 private:
  std::shared_ptr<RobotStateStore> state_;
  agx_task_workflow::TaskRuntimeBinding runtime_;
  mutable std::mutex mutex_;
  RobotTaskPorts ports_;
};
bool UsesSharedRobotActuators(const std::vector<agx_task_workflow::TaskResource>& resources);
}
