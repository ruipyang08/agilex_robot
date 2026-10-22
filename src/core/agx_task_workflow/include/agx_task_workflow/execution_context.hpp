#pragma once
#include <cmath>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <utility>
#include "agx_task_workflow/runner.hpp"

namespace agx_task_workflow {
// Wall/steady time only. ROS time may pause while cancellation must still work.
class ExecutionContext {
 public:
  using Clock = std::function<double()>;
  ExecutionContext(double timeout_s, Clock now, std::function<bool()> cancelled,
                   PhaseSink progress = {}, std::function<bool()> faulted = {})
      : now_(std::move(now)), cancelled_(std::move(cancelled)),
        progress_(std::move(progress)), faulted_(std::move(faulted)) {
    if (!now_ || !cancelled_ || !std::isfinite(timeout_s) || timeout_s <= 0)
      throw std::invalid_argument("execution requires a finite deadline, clock and cancellation source");
    last_now_ = now_();
    deadline_ = last_now_ + timeout_s;
    if (!std::isfinite(last_now_) || !std::isfinite(deadline_) || deadline_ <= last_now_)
      throw std::invalid_argument("execution requires a representable finite future deadline");
  }

  FailureKind Check() const {
    const bool cancelled = cancelled_();
    const bool faulted = faulted_ && faulted_();
    std::lock_guard<std::mutex> lock(mutex_);
    if (cause_ != FailureKind::None) return cause_;
    // Serialize sampling with committing last_now_: another caller may finish
    // its predicates first. The injected clock must not re-enter this context.
    const double now = now_();
    if (cancelled) cause_ = FailureKind::Cancelled;
    else if (faulted || !std::isfinite(now) || now < last_now_) cause_ = FailureKind::Fault;
    else if (now >= deadline_) cause_ = FailureKind::Deadline;
    if (std::isfinite(now)) last_now_ = now;
    return cause_;
  }

  FailureKind ReportProgress(const std::string& phase) const {
    auto cause = Check();
    if (cause != FailureKind::None) return cause;
    if (progress_) progress_(phase);
    return Check(); // A callback is also an interruption boundary.
  }

  double Deadline() const { return deadline_; }

 private:
  Clock now_;
  std::function<bool()> cancelled_;
  PhaseSink progress_;
  std::function<bool()> faulted_;
  double deadline_ = 0;
  mutable double last_now_ = 0;
  mutable FailureKind cause_ = FailureKind::None;
  mutable std::mutex mutex_;
};
}
