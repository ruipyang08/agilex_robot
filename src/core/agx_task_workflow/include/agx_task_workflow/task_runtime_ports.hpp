#pragma once
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include "agx_task_workflow/execution_context.hpp"

namespace agx_task_workflow {
struct InvocationIdentity { std::string run_id, task_id, operation_id; };
class InvocationExpired : public std::runtime_error {
 public:
  InvocationExpired() : std::runtime_error("task runtime binding expired") {}
};
struct TaskRuntimePorts : InvocationIdentity {
  std::function<FailureKind()> check;
  std::function<double()> deadline_steady_s;
};
using TaskRuntimePortsProvider = std::function<TaskRuntimePorts(const std::string& task_id)>;

// Optional domain ports use the same lock and lifetime as the generic invocation.
// Callbacks must be bounded and must not re-enter binding/expiry or this token.
class InvocationToken {
 public:
  template<class F> auto WithActive(F&& callback) const {
    auto current = invocation_.lock();
    if (!current) throw InvocationExpired();
    std::lock_guard<std::mutex> lock(current->mutex);
    if (!current->active) throw InvocationExpired();
    return std::forward<F>(callback)(
        static_cast<const InvocationIdentity&>(current->identity),
        static_cast<const ExecutionContext&>(*current->context));
  }
  TaskRuntimePorts Ports() const;
 private:
  struct Invocation {
    std::mutex mutex;
    bool active = true;
    InvocationIdentity identity;
    std::shared_ptr<ExecutionContext> context;
  };
  explicit InvocationToken(const std::shared_ptr<Invocation>& invocation) : invocation_(invocation) {}
  std::weak_ptr<Invocation> invocation_;
  friend class TaskRuntimeBinding;
};

class TaskRuntimeBinding {
 public:
  TaskRuntimeBinding() = default;
  ~TaskRuntimeBinding() { Expire(); }
  void Activate(const std::string& run_id, const std::string& task_id,
                const std::string& operation_id, std::shared_ptr<ExecutionContext> context);
  InvocationToken Acquire(const std::string& task_id) const;
  TaskRuntimePorts Bind(const std::string& task_id) const;
  void Expire();
 private:
  mutable std::mutex mutex_;
  std::shared_ptr<InvocationToken::Invocation> active_;
};
}
