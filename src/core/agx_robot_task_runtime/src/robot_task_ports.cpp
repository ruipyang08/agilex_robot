#include "agx_robot_task_runtime/robot_task_ports.hpp"
#include <algorithm>

namespace agx_robot_task_runtime {
namespace task = agx_task_workflow;
namespace {
StateUpdate Rejected(const std::shared_ptr<RobotStateStore>& store, const std::string& reason) {
  const auto snapshot = store->Snapshot();
  return {UpdateStatus::Rejected, snapshot.revision, snapshot.halted, reason};
}
task::StepResult Interrupted(task::FailureKind failure, const std::string& detail) {
  task::StepResult result{false, detail}; result.failure = failure; return result;
}
}
RobotTaskRuntimeBinding::RobotTaskRuntimeBinding(std::shared_ptr<RobotStateStore> state)
    : state_(std::move(state)) {
  if (!state_) throw std::invalid_argument("robot task binding requires the host robot state store");
}
void RobotTaskRuntimeBinding::Activate(const std::string& run, const std::string& task_id,
    const std::string& operation, std::shared_ptr<task::ExecutionContext> context,
    std::function<task::StepResult(const std::string&)> held_arrival) {
  std::lock_guard<std::mutex> lock(mutex_);
  runtime_.Activate(run, task_id, operation, std::move(context));
  const auto token = runtime_.Acquire(task_id);
  RobotTaskPorts ports;
  static_cast<task::TaskRuntimePorts&>(ports) = token.Ports();
  ports.snapshot = [token, state = state_] {
    return token.WithActive([&](const task::InvocationIdentity&, const task::ExecutionContext&) {
      return state->Snapshot();
    });
  };
  ports.begin_transfer = [token, state = state_](StateRevision revision, const std::string& op,
      const ObjectRef& object, std::optional<HolderRef> source, std::optional<HolderRef> destination) {
    try {
      return token.WithActive([&](const task::InvocationIdentity& id, const task::ExecutionContext& context) {
        if (op != id.operation_id) return Rejected(state, "operation identity does not match the active task invocation");
        if (context.Check() != task::FailureKind::None)
          return Rejected(state, "task invocation was interrupted before a new transfer could begin");
        return state->BeginTransfer(revision, op, object, std::move(source), std::move(destination));
      });
    } catch (const task::InvocationExpired& error) { return Rejected(state, error.what()); }
  };
  ports.apply_effect = [token, state = state_](const PhysicalEffect& effect, const Evidence& evidence,
      const std::string& op, StateRevision revision) {
    try {
      return token.WithActive([&](const task::InvocationIdentity& id, const task::ExecutionContext&) {
        if (op != id.operation_id || evidence.run_id != id.run_id || evidence.task_id != id.task_id ||
            evidence.operation_id != id.operation_id)
          return Rejected(state, "effect identity does not match the active task invocation");
        // Cancellation stops new intent, not the recording of verified physical facts.
        return state->ApplyEffect(effect, evidence, op, revision);
      });
    } catch (const task::InvocationExpired& error) { return Rejected(state, error.what()); }
  };
  if (held_arrival) ports.held_arrival = [token, held = std::move(held_arrival)](const std::string& station) {
    try {
      return token.WithActive([&](const task::InvocationIdentity&, const task::ExecutionContext& context) {
        auto failure = context.Check();
        if (failure != task::FailureKind::None)
          return Interrupted(failure, "task invocation interrupted before held-arrival check");
        auto result = held(station);
        failure = context.Check();
        if (failure != task::FailureKind::None) {
          result.ok = false; result.failure = failure;
          if (!result.detail.empty()) result.detail += "; ";
          result.detail += "task invocation interrupted during held-arrival check";
        }
        return result;
      });
    } catch (const task::InvocationExpired& error) { return Interrupted(task::FailureKind::Fault, error.what()); }
  };
  ports_ = std::move(ports);
}
RobotTaskPorts RobotTaskRuntimeBinding::Bind(const std::string& task_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  (void)runtime_.Acquire(task_id);
  return ports_;
}
void RobotTaskRuntimeBinding::Expire() {
  std::lock_guard<std::mutex> lock(mutex_);
  runtime_.Expire();
  ports_ = {};
}
bool UsesSharedRobotActuators(const std::vector<task::TaskResource>& declared) {
  return std::any_of(declared.begin(), declared.end(), [](const auto& resource) {
    return resource == resources::Base || resource == resources::Arm ||
           resource == resources::Lift || resource == resources::Gripper;
  });
}
}
