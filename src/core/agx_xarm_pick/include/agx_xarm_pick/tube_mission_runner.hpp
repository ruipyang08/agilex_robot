#pragma once
#include <algorithm>
#include <functional>
#include <future>
#include <thread>
#include <cmath>
#include <stdexcept>
#include "agx_xarm_pick/tube_grasp_mission.hpp"
#include "agx_xarm_pick/tube_grasp_config.hpp"

namespace agxcore {
struct TubeMonitorOptions {
  unsigned poll_ms = 50;
  double worker_stop_timeout_s = 15.0;
  double measured_stop_timeout_s = 1.0;
};
struct MonitoredTubeResult {
  TubeMissionStatus status = TubeMissionStatus::kInterrupted;
  // Caller must flush its report and _Exit: a blocked child cannot be allowed
  // to resume issuing commands after this process reports cancellation.
  bool exit_required = false;
  TubeMissionOutcome outcome;
  std::shared_ptr<TubeLiveObserver> observer;
};

inline MonitoredTubeResult RunTubeMissionMonitored(
    ros::NodeHandle node, const std::string& fixture, const std::string& color,
    const std::function<bool()>& keep_running,
    std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig> config = {},
    TubeEffectSink effect_sink = {}, TubeMonitorOptions options = {},
    std::function<bool()> held_base = {}) {
  if (!keep_running || options.poll_ms == 0 || options.poll_ms > 1000 ||
      !std::isfinite(options.worker_stop_timeout_s) || options.worker_stop_timeout_s <= 0 ||
      options.worker_stop_timeout_s > 60 || !std::isfinite(options.measured_stop_timeout_s) ||
      options.measured_stop_timeout_s <= 0 || options.measured_stop_timeout_s > 5)
    throw std::invalid_argument("invalid bounded tube monitor options");
  if (!ros::ok() || !keep_running()) return {TubeMissionStatus::kInterrupted,false};
  if (!config) config = SnapshotLegacyTubeTaskConfig(node);
  struct Shared {
    std::mutex mutex;
    std::shared_ptr<TubeGraspMission> mission;
    std::atomic<bool> stop{false};
    std::promise<TubeMissionStatus> result;
  };
  auto shared = std::make_shared<Shared>();
  auto result = shared->result.get_future();
  std::thread worker([shared, node, fixture, color, config, effect_sink, held_base]() mutable {
    try {
      auto mission = std::make_shared<TubeGraspMission>(&node, config, color, effect_sink, held_base);
      { std::lock_guard<std::mutex> lock(shared->mutex); shared->mission = mission; }
      if (shared->stop) mission->RequestStop();
      shared->result.set_value(mission->Run(fixture, color));
    } catch (...) { shared->result.set_exception(std::current_exception()); }
  });
  auto duration = [](double seconds) {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
  };
  auto stop_deadline = std::chrono::steady_clock::time_point::max();
  auto begin_stop = [&] {
    if (stop_deadline == std::chrono::steady_clock::time_point::max())
      stop_deadline = std::chrono::steady_clock::now() + duration(options.worker_stop_timeout_s);
  };
  auto remaining_stop_seconds = [&] {
    return std::max(0.0, std::chrono::duration<double>(
        stop_deadline - std::chrono::steady_clock::now()).count());
  };
  auto snapshot = [&](TubeMissionStatus status, bool joined, bool exit_required) {
    MonitoredTubeResult report;
    report.status = status; report.exit_required = exit_required;
    if (shared->mission) {
      report.outcome = shared->mission->Outcome(status);
      report.observer = shared->mission->Observer();
    } else report.outcome.status = status;
    report.outcome.worker_joined = joined;
    return report;
  };
  TubeMissionStatus stopped_status = TubeMissionStatus::kInterrupted;
  while (result.wait_for(std::chrono::milliseconds(options.poll_ms)) != std::future_status::ready) {
    bool mission_stopped = false;
    { std::lock_guard<std::mutex> lock(shared->mutex);
      if (shared->mission) {
        shared->mission->RefreshPayloadEvidence();
        mission_stopped = shared->mission->StopRequested();
      } }
    const bool external_stop = !ros::ok() || !keep_running();
    if (external_stop || mission_stopped) {
      if (!shared->stop.exchange(true)) {
        stopped_status = external_stop ? TubeMissionStatus::kInterrupted : TubeMissionStatus::kRetentionLost;
        begin_stop();
      }
    }
    if (shared->stop) {
      { std::lock_guard<std::mutex> lock(shared->mutex);
        if (shared->mission) shared->mission->RequestStop(); }
      if (std::chrono::steady_clock::now() >= stop_deadline) {
        worker.detach();
        std::lock_guard<std::mutex> lock(shared->mutex);
        // Seal stop-only dispatch as well: a blocked mission is terminal and
        // cannot leave a queued retry to issue after the report is flushed.
        if (shared->mission) {
          try { shared->mission->FinishStopRequests(0.0); } catch (...) {}
        }
        auto report = snapshot(stopped_status, false, true);
        report.outcome.measured_stop_confirmed = false;
        return report;
      }
    }
  }
  worker.join();
  auto status = TubeMissionStatus::kSetupUnavailable;
  try { status = result.get(); } catch (...) { /* Preserve observed effects on child exceptions. */ }
  bool exit_required = false, measured_stop = false, stop_finished = false;
  auto finish_stop = [&] {
    if (stop_finished) return;
    stop_finished = true;
    begin_stop();
    if (!shared->mission) return;
    // The mission worker may have accepted motion after its early stop RPC.
    // Join a final stop ordered behind that RPC, within the original budget.
    bool quiescent = false;
    try { quiescent = shared->mission->FinishStopRequests(remaining_stop_seconds()); }
    catch (...) { /* Transport/thread setup failure cannot authorize continuation. */ }
    if (!quiescent) { exit_required = true; return; }
    const TubeEvidence barrier{"tube_monitor_stop", ros::Time::now().toSec(),
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count()};
    const auto deadline = std::min(stop_deadline, std::chrono::steady_clock::now() +
        duration(options.measured_stop_timeout_s));
    while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
      shared->mission->RefreshPayloadEvidence();
      if (shared->mission->ObserveStopped(barrier)) { measured_stop = true; break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(options.poll_ms));
    }
  };
  // Effects already reached the shared state. Cancellation changes status,
  // never physical history, and every failure drains its owned stop transport.
  if (shared->stop || !ros::ok() || !keep_running())
    status = shared->stop ? stopped_status : TubeMissionStatus::kInterrupted;
  else if (shared->mission && shared->mission->StopRequested())
    status = TubeMissionStatus::kRetentionLost;
  if (status != TubeMissionStatus::kSuccess) finish_stop();
  auto report = snapshot(status, true, exit_required);
  if (report.status == TubeMissionStatus::kSuccess &&
      (!report.outcome.effects_accepted || report.outcome.payload != PayloadKnowledge::kHolding)) {
    status = TubeMissionStatus::kRetentionLost;
    finish_stop();
    report = snapshot(status, true, exit_required);
  }
  // Outcome/stop observers are callbacks too. This is the final success
  // linearization point: no observation callback follows an accepted guard.
  if (!ros::ok() || !keep_running()) {
    status = TubeMissionStatus::kInterrupted;
    finish_stop();
    // This failure-only resnapshot preserves effects produced during cleanup;
    // it cannot restore success or reuse a pre-cleanup exit_required value.
    report = snapshot(status, true, exit_required);
  }
  if (report.status != TubeMissionStatus::kSuccess)
    report.outcome.measured_stop_confirmed = measured_stop && !exit_required;
  return report;
}
}
