#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace agx_task_workflow {
enum class FailureKind { None, Ordinary, Cancelled, Deadline, Fault, UnconfirmedStop };
enum class StopState { NotConfirmed, Confirmed };
enum class RunCompletion { Succeeded, CompletedWithFailures, Failed, Cancelled };
enum class FailurePolicy { Stop, Continue };
inline bool MayContinue(FailurePolicy policy, FailureKind kind, StopState stopped,
                        bool reconciled, bool terminal) {
  return policy==FailurePolicy::Continue && kind==FailureKind::Ordinary &&
         stopped==StopState::Confirmed && reconciled && !terminal;
}
struct StepResult {
  bool ok = false;
  std::string detail;
  std::vector<std::string> physical_effect_ids;
  std::uint64_t state_revision = 0;
  FailureKind failure = ok ? FailureKind::None : FailureKind::Fault;
  StopState stop = StopState::NotConfirmed;
};
using PhaseSink = std::function<void(const std::string&)>;
struct PreparedTask;
struct TaskResult {
  std::string id, type;
  std::string status = "not_run";
  bool dispatched = false;
  std::string detail, failed_phase;
  double elapsed_wall_s = 0.0;
  std::vector<std::string> physical_effect_ids;
  std::uint64_t state_revision = 0;
  FailureKind failure = FailureKind::Fault;
  StopState stop = StopState::NotConfirmed;
};
struct Result {
  bool success = false;
  std::string failed_task_id, failed_task_type, failed_phase, detail;
  std::vector<TaskResult> tasks;
  std::vector<std::string> physical_effect_ids;
  std::uint64_t state_revision = 0;
  FailureKind failure = FailureKind::Fault;
  StopState stop = StopState::NotConfirmed;
  RunCompletion completion = RunCompletion::Failed;
  std::vector<std::string> failed_task_ids;
};
struct Ports {
  std::function<bool()> cancelled;
  std::function<void()> stop;
  PhaseSink phase;
  std::function<void(const TaskResult&)> task_event;
  std::function<FailureKind()> runtime_guard;
  // Confirms reconciled effects and fresh, supported, nonpending measured state.
  std::function<bool(const PreparedTask&, const StepResult&)> continuation_allowed;
  // Optional monotonic clock; defaults to std::chrono::steady_clock.
  std::function<double()> steady_now;
  // Admission-only hook: establish task identity/deadline before its first guard or notification.
  std::function<void(const PreparedTask&)> task_begin;
};
Result Run(const std::vector<PreparedTask>& tasks, const Ports& ports);
}
