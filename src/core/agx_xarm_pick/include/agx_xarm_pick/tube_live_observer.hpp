#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <mutex>
#include "agx_xarm_pick/tube_effects.hpp"

namespace ros { class NodeHandle; }
namespace agx_xarm_pick { struct ResolvedTubeTaskConfig; struct TubeTaskMotionConfig; }
namespace agxcore {
struct TubeObserverInterfaces {
  std::string joint_state_topic, gripper_state_topic, lift_state_topic;
};
void ValidateTubeObserverInterfaces(const TubeObserverInterfaces& interfaces);
struct TubeObserverConfig {
  double gripper_max_age_s = 0, arm_max_age_s = 0, lift_max_age_s = 0;
  double fully_open_command = 0, close_command = 0, command_tolerance = 0;
  // Grasp observers use these stop limits when the task supplies only the
  // freshness/command fields. Under fingertip contact, PhysX reports up to
  // 0.012 rad/s velocity noise despite <0.0002 rad positional variation. A
  // fresh one-second position window still rejects sustained slow motion.
  double stopped_velocity_rad_s = .02, stop_settle_s = 1.0;
  double stopped_arm_range_rad = .002, stopped_lift_range_m = .001;
};
struct TubeGripperObservation {
  TubeEvidence evidence;
  bool valid = false, fresh = false, initialized = false;
  int grip_state = 0;
  double position = 0, target_position = 0;
  bool known_empty = false, caught = false;
};
struct TubeMotionObservation {
  TubeEvidence evidence, previous_evidence;
  bool valid = false, fresh = false, stationary = false;
  double stationary_source_since_s = 0, stationary_receipt_since_s = 0;
};
struct TubeArmObservation : TubeMotionObservation {
  std::array<double, 6> positions{}, velocities{};
};
struct TubeLiftObservation : TubeMotionObservation {
  double position_m = 0;
  // Native measured speed in m/s (LiftMotorMsg.backSpeed is signed mm/s).
  // Missing speed is invalid; never derive it from quantized integer height.
  double velocity_mps = std::numeric_limits<double>::quiet_NaN();
};
struct TubeMotionWindowToken {
  std::uint64_t generation = 0;
  TubeEvidence barrier;
};
struct TubeMotionExtrema {
  double minimum = 0, maximum = 0, max_abs_velocity = 0;
  TubeEvidence minimum_evidence, maximum_evidence, peak_evidence;
};
// One independent, task-neutral window; all six arm axes are radians, lift metres.
struct TubeMotionWindow {
  TubeMotionWindowToken token;
  bool valid = false, fresh = false;
  std::array<TubeMotionExtrema, 6> arm;
  TubeMotionExtrema lift;
  TubeEvidence arm_evidence, lift_evidence, failure_evidence;
  std::string failure_detail;
};
struct TubeLiveSnapshot {
  TubeGripperObservation gripper;
  TubeArmObservation arm;
  TubeLiftObservation lift;
  double source_now_s = 0, steady_now_s = 0, stop_settle_s = 0;
  TubeMotionWindow motion_window;
  bool ArmLiftStoppedSince(const TubeEvidence& request) const;
};
// Pure typed observation port; no command endpoint or workflow dependency.
class TubeObservationBuffer {
 public:
  using Clock = std::function<double()>;
  explicit TubeObservationBuffer(TubeObserverConfig config);
  void Gripper(TubeGripperObservation observation);
  void Arm(TubeArmObservation observation);
  void Lift(TubeLiftObservation observation);
  TubeLiveSnapshot Snapshot(double source_now_s, double steady_now_s) const;
  // Capture clocks while updates are excluded. Clocks must not call this buffer.
  TubeLiveSnapshot Snapshot(const Clock& source_clock, const Clock& steady_clock) const;
  // ROS /clock and stamped state arrive on separate subscriptions. Wait briefly
  // for a small clock lead, then apply the unchanged strict freshness checks.
  // A stopped clock, large lead, or stale receipt still returns an invalid read.
  TubeLiveSnapshot SnapshotAfterClockCatchUp(const Clock& source_clock, const Clock& steady_clock,
      double max_future_source_s, double max_wait_s) const;
  // One attempt per buffer, including failed attempts. No reset or reanchoring.
  TubeMotionWindowToken ArmMotionWindow(const Clock& source_clock, const Clock& steady_clock);
 private:
  TubeLiveSnapshot SnapshotLocked(double source_now_s, double steady_now_s) const;
  TubeObserverConfig config_;
  mutable std::mutex mutex_;
  TubeLiveSnapshot state_;
  TubeLiftObservation last_lift_input_;
  void WindowArm(const TubeArmObservation&);
  void WindowLift(const TubeLiftObservation&);
  void WindowFailure(const std::string&, const TubeEvidence&);
  bool window_attempted_ = false, window_arm_valid_ = false, window_lift_valid_ = false;
  TubeArmObservation window_arm_input_;
  TubeLiftObservation window_lift_input_;
  double window_arm_receipt_ = 0, window_lift_receipt_ = 0;
  std::array<double, 6> arm_stationary_min_{}, arm_stationary_max_{};
  double lift_stationary_min_m_ = 0, lift_stationary_max_m_ = 0;
};
class TubeLiveObserver {
 public:
  virtual ~TubeLiveObserver() = default;
  virtual TubeLiveSnapshot Snapshot() const = 0;
  // Older observers cannot fabricate a retention certificate.
  virtual TubeMotionWindowToken ArmMotionWindow() { return {}; }
};
// Owns independent read-only subscriptions; retaining this handle retains live
// observations after the mission and its MoveIt interface have been destroyed.
std::shared_ptr<TubeLiveObserver> MakeTubeLiveObserver(ros::NodeHandle node,
    const agx_xarm_pick::ResolvedTubeTaskConfig& config,
    const agx_xarm_pick::TubeTaskMotionConfig& motion);
std::shared_ptr<TubeLiveObserver> MakeTubeLiveObserver(ros::NodeHandle node,
    const TubeObserverInterfaces& interfaces, const TubeObserverConfig& config);
}  // namespace agxcore
