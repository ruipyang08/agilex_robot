#include "agx_task_workflow/task_runtime_ports.hpp"

namespace agx_task_workflow {
TaskRuntimePorts InvocationToken::Ports() const {
  TaskRuntimePorts ports;
  static_cast<InvocationIdentity&>(ports) = WithActive(
      [](const InvocationIdentity& id, const ExecutionContext&) { return id; });
  ports.check = [token = *this] {
    try {
      return token.WithActive([](const InvocationIdentity&, const ExecutionContext& context) {
        return context.Check();
      });
    } catch (const InvocationExpired&) { return FailureKind::Fault; }
  };
  ports.deadline_steady_s = [token = *this] {
    return token.WithActive([](const InvocationIdentity&, const ExecutionContext& context) {
      return context.Deadline();
    });
  };
  return ports;
}
void TaskRuntimeBinding::Activate(const std::string& run, const std::string& task,
    const std::string& operation, std::shared_ptr<ExecutionContext> context) {
  if (run.empty() || task.empty() || operation.empty() || !context)
    throw std::invalid_argument("task runtime activation requires run, task, operation and context identity");
  auto next = std::make_shared<InvocationToken::Invocation>();
  next->identity = {run, task, operation};
  next->context = std::move(context);
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    std::lock_guard<std::mutex> invocation_lock(active_->mutex);
    active_->active = false;
  }
  active_ = std::move(next);
}
InvocationToken TaskRuntimeBinding::Acquire(const std::string& task) const {
  std::lock_guard<std::mutex> lock(mutex_);
  InvocationToken token(active_);
  token.WithActive([&](const InvocationIdentity& id, const ExecutionContext&) {
    if (id.task_id != task)
      throw std::runtime_error("task runtime binding belongs to another invocation");
  });
  return token;
}
TaskRuntimePorts TaskRuntimeBinding::Bind(const std::string& task) const { return Acquire(task).Ports(); }
void TaskRuntimeBinding::Expire() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    std::lock_guard<std::mutex> invocation_lock(active_->mutex);
    active_->active = false;
  }
  active_.reset();
}
}
