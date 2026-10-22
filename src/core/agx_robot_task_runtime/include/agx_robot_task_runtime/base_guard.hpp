#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include "agx_robot_task_runtime/held_base_monitor.hpp"

namespace agx_robot_task_runtime {
struct BaseGuardConfig {
  double receipt_limit_s, header_limit_s, future_skew_s;
  double linear_limit_mps, angular_limit_radps, motion_dwell_s;
  double displacement_limit_m, yaw_limit_rad;
  double goal_status_limit_s, watchdog_limit_s;
  double emergency_displacement_limit_m = .002, emergency_yaw_limit_rad = .005;
};

// Adapters copy a coherent snapshot under their observation lock. Receipt
// times use the same steady clock supplied to the guard; source stamps use ROS.
struct BaseSample {
  bool authority_received = false, odometry_received = false, command_received = false;
  bool watchdog_received = false, goal_status_received = false;
  double authority_receipt_s = 0, odometry_receipt_s = 0, command_receipt_s = 0;
  double watchdog_receipt_s = 0, goal_status_receipt_s = 0, odometry_stamp_s = 0;
  bool authority_none = false, manipulation_hold = false, command_zero = false;
  bool owned_goal_terminal = false, any_goal_active = false;
  std::string owned_goal_id, session_id;
  std::uint64_t hold_epoch = 0;
  double x_m = 0, y_m = 0, yaw_rad = 0;
  double linear_x_mps = 0, linear_y_mps = 0, angular_z_radps = 0;
  double authority_stamp_s = 0, goal_status_stamp_s = 0;
  bool watchdog_ready = false, authority_fault = false;
};

struct BaseGuardResult { bool ok = false; std::string detail; };

// Action-status publishers and this consumer receive ROS clock ticks on separate
// connections. Apply the same configured source skew as the base guard; receipt
// clocks remain strictly local/monotonic and original stamps are never rewritten.
bool FreshStatusEvidence(double source_stamp, double receipt_steady,
    double ros_now, double steady_now, double source_limit, double receipt_limit,
    double allowed_source_skew);

// One task-thread owner; sensor callbacks only supply immutable BaseSample copies.
class BaseGuard {
 public:
  explicit BaseGuard(BaseGuardConfig config);
  BaseGuardResult CheckStopped(const BaseSample&, double steady_now, double ros_now,
                               bool require_hold, bool require_owned_goal) const;
  BaseGuardResult CaptureHold(const BaseSample&, double steady_now, double ros_now,
                             std::string station = {});
  BaseGuardResult CheckHeld(const BaseSample&, double steady_now, double ros_now);
  // Start before requesting stop. Only observations after this barrier may
  // confirm a stable inhibited base; this never captures normal motion authority.
  BaseGuardResult BeginEmergencyStop(double steady_now, double ros_now);
  BaseGuardResult ObserveEmergencyStop(const BaseSample&, double steady_now, double ros_now);
  void InvalidateArrival();
  bool ArrivalMatches(const std::string&, const BaseSample&, double steady_now, double ros_now);

 private:
  BaseGuardResult Common(const BaseSample&, double steady_now, double ros_now, bool held, bool emergency = false) const;
  bool BelowStoppedLimits(const BaseSample&) const;
  BaseGuardConfig config_;
  std::optional<BaseSample> held_;
  std::optional<std::string> station_;
  MotionDwell motion_;
  double last_check_s_ = 0;
  bool monitor_owned_goal_ = false;
  struct EmergencyWindow {
    BaseSample first, last;
    double first_now, last_now;
    double min_x, max_x, min_y, max_y, min_yaw = 0, max_yaw = 0;
  };
  std::optional<EmergencyWindow> emergency_window_;
  std::optional<double> emergency_steady_barrier_, emergency_ros_barrier_;
};
}
