#pragma once
#include <optional>
#include "agx_robot_task_runtime/robot_state.hpp"
#include "agx_task_workflow/runner.hpp"
#include "agx_navigation_tasks/travel_profile.hpp"

namespace agx_navigation_tasks {
struct TravelArmObservation {
  agx_robot_task_runtime::Evidence evidence;
  bool valid = false;
  std::map<std::string, double> positions_rad, velocities_rad_s;
  bool stationary = false;
  double stationary_source_since_s = 0, stationary_receipt_since_s = 0;
};
struct TravelLiftObservation {
  agx_robot_task_runtime::Evidence evidence;
  bool valid = false;
  double position_m = 0, velocity_mps = 0;
  bool stationary = false;
  double stationary_source_since_s = 0, stationary_receipt_since_s = 0;
};
struct TravelProducerObservation {
  agx_robot_task_runtime::Evidence evidence;
  bool valid = false, arm_stopped = false, lift_stopped = false;
};
struct TravelStopJobs {
  bool lift_pending = false, hold_pending = false, sealed = false;
};
// Produced by the measured base guard and the command-worker supervisor.
struct TravelHoldObservation {
  agx_robot_task_runtime::Evidence evidence;
  std::string session_id;
  std::uint64_t hold_epoch = 0;
  bool held_stopped = false, command_guard_ok = false;
};
struct TravelSample {
  agx_robot_task_runtime::RobotStateSnapshot robot;
  TravelArmObservation arm;
  TravelLiftObservation lift;
  TravelProducerObservation producers;
  TravelHoldObservation hold;
  agxcore::TubeMotionWindow motion_window;
};
struct StartClearance {
  bool ok = false;
  std::string start_pose_id, path_id, detail;
};
struct TravelDecision {
  bool ready = false, arm_ready = false, lift_ready = false;
  bool producers_stopped = false, clearance_verified = false;
  agx_robot_task_runtime::StateRevision state_revision = 0;
  agx_task_workflow::FailureKind failure = agx_task_workflow::FailureKind::None;
  std::string detail;
};
StartClearance EvaluateStartClearance(const TravelProfile&, TravelMode,
    const TravelSample&, double source_now_s, double steady_now_s);
// One owner. No commands, state writes, permit acquisition or hold release.
class TravelPolicy {
 public:
  TravelPolicy(TravelProfile profile, TravelMode mode);
  StartClearance BeginPreparation(const TravelSample&, double source_now_s, double steady_now_s);
  TravelDecision CheckPreparation(const TravelSample&, double source_now_s, double steady_now_s);
  // Call after BOTH bounded commands return, including the already-default path.
  TravelDecision CommandsFinished(const agxcore::TubeMotionWindowToken&, double source_now_s, double steady_now_s);
  TravelDecision Observe(const TravelSample&, double source_now_s, double steady_now_s);
  TravelDecision CheckTravel(const TravelSample&, double source_now_s, double steady_now_s);
  // Caller invokes only after accepted hold release, before navigation dispatch.
  TravelDecision BeginTravel(const TravelSample&, double source_now_s, double steady_now_s);
  void Abort(agx_task_workflow::FailureKind cause);
  const TravelProfile& Profile() const { return profile_; }
 private:
  TravelDecision Fail(agx_task_workflow::FailureKind, const std::string&);
  bool Time(double source, double steady, bool preparation);
  bool SameCustody(const TravelSample&, double source, double steady) const;
  bool Ordered(const TravelSample&, bool clearance);
  TravelDecision Endpoint(const TravelSample&, double source, double steady) const;
  bool Retained(const TravelSample&, double source, double steady);
  TravelProfile profile_;
  TravelMode mode_;
  bool begun_ = false, commands_finished_ = false, ready_ = false, traveling_ = false;
  agxcore::TubeMotionWindowToken window_token_;
  agx_robot_task_runtime::TransportLoad load_ = agx_robot_task_runtime::TransportLoad::Unknown;
  std::optional<agx_robot_task_runtime::ObjectRef> object_;
  std::string session_id_;
  std::uint64_t hold_epoch_ = 0;
  double last_source_ = 0, last_steady_ = 0, deadline_ = 0;
  double barrier_source_ = 0, barrier_steady_ = 0;
  std::optional<TravelSample> previous_, settled_start_;
  std::map<std::string, std::pair<double,double>> settled_arm_range_;
  std::pair<double,double> settled_lift_range_;
  agx_robot_task_runtime::StateRevision ready_revision_ = 0;
  TravelDecision failure_;
};
}  // namespace agx_navigation_tasks
