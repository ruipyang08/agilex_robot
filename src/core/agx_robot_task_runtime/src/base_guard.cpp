#include "agx_robot_task_runtime/base_guard.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace agx_robot_task_runtime {
namespace {
bool Fresh(double stamp, double now, double limit) {
  return std::isfinite(stamp) && std::isfinite(now) && now >= stamp && now-stamp <= limit;
}
bool SourceFresh(double stamp,double now,double limit,double skew) {
  const double age=now-stamp;
  return std::isfinite(stamp) && std::isfinite(now) && std::isfinite(age) && stamp>0 && now>0 &&
      age>=-skew && age<=limit;
}
}

bool FreshStatusEvidence(double source, double receipt, double ros_now, double steady_now,
    double source_limit, double receipt_limit, double allowed_source_skew) {
  return std::isfinite(source_limit) && source_limit>0 &&
      std::isfinite(receipt_limit) && receipt_limit>0 &&
      std::isfinite(allowed_source_skew) && allowed_source_skew>=0 &&
      SourceFresh(source,ros_now,source_limit,allowed_source_skew) &&
      receipt>0 && Fresh(receipt,steady_now,receipt_limit);
}

BaseGuard::BaseGuard(BaseGuardConfig config) : config_(config) {
  for (double v : {config.receipt_limit_s, config.header_limit_s,
                   config.linear_limit_mps, config.angular_limit_radps,
                   config.motion_dwell_s, config.displacement_limit_m,
                   config.yaw_limit_rad, config.goal_status_limit_s, config.watchdog_limit_s,
                   config.emergency_displacement_limit_m, config.emergency_yaw_limit_rad})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("base guard limits must be finite and positive");
  if (!std::isfinite(config.future_skew_s) || config.future_skew_s < 0)
    throw std::invalid_argument("base guard future skew must be finite and nonnegative");
}

BaseGuardResult BaseGuard::Common(const BaseSample& s, double now, double ros_now, bool held, bool emergency) const {
  if (!s.authority_received || !s.odometry_received || !s.command_received ||
      !Fresh(s.authority_receipt_s, now, config_.receipt_limit_s) ||
      !Fresh(s.odometry_receipt_s, now, config_.receipt_limit_s) ||
      !Fresh(s.command_receipt_s, now, config_.receipt_limit_s))
    return {false, "base authority, odometry or output receipt is missing/stale"};
  if (!SourceFresh(s.odometry_stamp_s,ros_now,config_.header_limit_s,config_.future_skew_s))
    return {false, "odometry source timestamp is invalid/stale"};
  if (!SourceFresh(s.authority_stamp_s,ros_now,config_.header_limit_s,config_.future_skew_s))
    return {false, "authority source timestamp is invalid/stale"};
  if (!s.watchdog_received || !Fresh(s.watchdog_receipt_s, now, config_.watchdog_limit_s))
    return {false, "workflow watchdog is missing/stale"};
  if (!emergency && !s.watchdog_ready)
    return {false, "workflow watchdog is not ready"};
  const bool inhibited = emergency ? s.authority_none != s.authority_fault
                                  : s.authority_none && !s.authority_fault;
  if (!inhibited || s.manipulation_hold != held || !s.command_zero)
    return {false, "base authority, manipulation hold or zero output does not match"};
  for (double v : {s.x_m, s.y_m, s.yaw_rad, s.linear_x_mps, s.linear_y_mps, s.angular_z_radps})
    if (!std::isfinite(v)) return {false, "base pose or velocity is nonfinite"};
  return {true, "fresh base evidence"};
}

bool BaseGuard::BelowStoppedLimits(const BaseSample& s) const {
  return std::hypot(s.linear_x_mps, s.linear_y_mps) <= config_.linear_limit_mps &&
         std::abs(s.angular_z_radps) <= config_.angular_limit_radps;
}

BaseGuardResult BaseGuard::CheckStopped(const BaseSample& s, double now, double ros_now,
                                       bool held, bool goal) const {
  auto result = Common(s, now, ros_now, held);
  if (!result.ok) return result;
  if (!BelowStoppedLimits(s)) return {false, "measured base has not stopped"};
  if (!s.goal_status_received || s.any_goal_active ||
      !Fresh(s.goal_status_receipt_s, now, config_.goal_status_limit_s) ||
      !SourceFresh(s.goal_status_stamp_s,ros_now,config_.goal_status_limit_s,config_.future_skew_s))
    return {false, "fresh inactive navigation goal evidence is not confirmed"};
  if (goal && (s.owned_goal_id.empty() || !s.owned_goal_terminal))
    return {false, "owned navigation goal terminal state is not confirmed"};
  return {true, "fresh zero-output and measured stopped base"};
}

BaseGuardResult BaseGuard::CaptureHold(const BaseSample& s, double now, double ros_now,
                                     std::string station) {
  auto result = CheckStopped(s, now, ros_now, true, !station.empty());
  if (!result.ok) return result;
  if (s.session_id.empty()) return {false, "hold session identity is missing"};
  held_ = s;
  monitor_owned_goal_ = !station.empty();
  station_ = station.empty() ? std::optional<std::string>{} : std::move(station);
  motion_ = MotionDwell{};
  last_check_s_ = now;
  return {true, "measured hold captured"};
}

BaseGuardResult BaseGuard::CheckHeld(const BaseSample& s, double now, double ros_now) {
  auto result = Common(s, now, ros_now, true);
  if (!result.ok) return result;
  if (!held_ || s.session_id != held_->session_id || s.hold_epoch != held_->hold_epoch)
    return {false, "current hold differs from captured hold"};
  if (!s.goal_status_received || s.any_goal_active ||
      !Fresh(s.goal_status_receipt_s,now,config_.goal_status_limit_s) ||
      !SourceFresh(s.goal_status_stamp_s,ros_now,config_.goal_status_limit_s,config_.future_skew_s))
    return {false,"inactive navigation goal evidence was lost during hold"};
  if (monitor_owned_goal_ && (!s.owned_goal_terminal || s.owned_goal_id!=held_->owned_goal_id))
    return {false,"owned navigation goal changed during hold"};
  if (now < last_check_s_) return {false, "base guard steady clock regressed"};
  last_check_s_ = now;
  const double dyaw = s.yaw_rad-held_->yaw_rad;
  if (!std::isfinite(dyaw)) return {false, "measured yaw difference is nonfinite"};
  if (std::hypot(s.x_m-held_->x_m, s.y_m-held_->y_m) > config_.displacement_limit_m ||
      std::abs(std::atan2(std::sin(dyaw), std::cos(dyaw))) > config_.yaw_limit_rad)
    return {false, "measured base displacement exceeded arrival tolerance"};
  if (!motion_.Update(now, BelowStoppedLimits(s), config_.motion_dwell_s))
    return {false, "measured base motion persisted for stopped dwell"};
  return {true, "current measured hold preserved"};
}

BaseGuardResult BaseGuard::BeginEmergencyStop(double now, double ros_now) {
  held_.reset();
  station_.reset();
  emergency_window_.reset();
  emergency_steady_barrier_.reset();
  emergency_ros_barrier_.reset();
  if (!std::isfinite(now) || now < 0 || !std::isfinite(ros_now) || ros_now <= 0)
    return {false, "emergency stop barrier clocks are invalid"};
  emergency_steady_barrier_ = now;
  emergency_ros_barrier_ = ros_now;
  return {true, "emergency stop observation barrier started"};
}

BaseGuardResult BaseGuard::ObserveEmergencyStop(const BaseSample& s, double now, double ros_now) {
  const auto reject = [this](const std::string& detail) {
    emergency_window_.reset();
    return BaseGuardResult{false, detail};
  };
  if (!emergency_steady_barrier_ || !emergency_ros_barrier_)
    return reject("emergency stop observation barrier was not started");
  auto result = Common(s, now, ros_now, true, true);
  if (!result.ok) return reject(result.detail);
  if (s.session_id.empty() || !s.goal_status_received || s.any_goal_active ||
      (!s.owned_goal_id.empty() && !s.owned_goal_terminal) ||
      !Fresh(s.goal_status_receipt_s, now, config_.goal_status_limit_s) ||
      !SourceFresh(s.goal_status_stamp_s, ros_now, config_.goal_status_limit_s, config_.future_skew_s))
    return reject("emergency stop needs a session and fresh inactive goal evidence");
  for (double receipt : {s.authority_receipt_s, s.odometry_receipt_s, s.command_receipt_s,
                         s.watchdog_receipt_s, s.goal_status_receipt_s})
    if (receipt <= *emergency_steady_barrier_)
      return reject("emergency stop receipt predates the stop request");
  for (double stamp : {s.authority_stamp_s, s.odometry_stamp_s, s.goal_status_stamp_s})
    if (stamp <= *emergency_ros_barrier_)
      return reject("emergency stop source evidence predates the stop request");
  if (!BelowStoppedLimits(s)) return reject("emergency stop measured base is moving");

  if (emergency_window_) {
    const auto& w = *emergency_window_;
    const auto& last = w.last;
    if (now < w.last_now || s.odometry_stamp_s < last.odometry_stamp_s ||
        s.authority_stamp_s < last.authority_stamp_s || s.goal_status_stamp_s < last.goal_status_stamp_s ||
        s.odometry_receipt_s < last.odometry_receipt_s ||
        s.authority_receipt_s < last.authority_receipt_s ||
        s.command_receipt_s < last.command_receipt_s ||
        s.watchdog_receipt_s < last.watchdog_receipt_s ||
        s.goal_status_receipt_s < last.goal_status_receipt_s ||
        s.hold_epoch < last.hold_epoch)
      return reject("emergency stop evidence or observation clock regressed");
    if (s.session_id != last.session_id ||
        s.authority_none != last.authority_none || s.authority_fault != last.authority_fault ||
        now-w.last_now > std::min({config_.receipt_limit_s, config_.goal_status_limit_s,
                                  config_.watchdog_limit_s}))
      emergency_window_.reset();
  }
  if (!emergency_window_) {
    emergency_window_ = EmergencyWindow{s,s,now,now,s.x_m,s.x_m,s.y_m,s.y_m};
    return {false, "emergency stop measured dwell started"};
  }
  auto& w = *emergency_window_;
  const double yaw_difference = s.yaw_rad-w.first.yaw_rad;
  if (!std::isfinite(yaw_difference)) return reject("emergency stop yaw difference is nonfinite");
  const double yaw = std::atan2(std::sin(yaw_difference), std::cos(yaw_difference));
  w.min_x=std::min(w.min_x,s.x_m); w.max_x=std::max(w.max_x,s.x_m);
  w.min_y=std::min(w.min_y,s.y_m); w.max_y=std::max(w.max_y,s.y_m);
  w.min_yaw=std::min(w.min_yaw,yaw); w.max_yaw=std::max(w.max_yaw,yaw);
  if (std::hypot(w.max_x-w.min_x,w.max_y-w.min_y) >
          std::min(config_.displacement_limit_m,config_.emergency_displacement_limit_m) ||
      w.max_yaw-w.min_yaw > std::min(config_.yaw_limit_rad,config_.emergency_yaw_limit_rad))
    return reject("emergency stop displacement exceeded the stable dwell tolerance");
  w.last=s; w.last_now=now;
  // Advancing source and receipt clocks are both required. Re-reading cached
  // stationary data cannot turn a single observation into a measured dwell.
  if (now-w.first_now < config_.motion_dwell_s ||
      s.odometry_receipt_s-w.first.odometry_receipt_s < config_.motion_dwell_s ||
      s.odometry_stamp_s-w.first.odometry_stamp_s < config_.motion_dwell_s)
    return {false, "emergency stop measured dwell is incomplete"};
  return {true, "fresh inhibited base remained measured stopped for the dwell"};
}

void BaseGuard::InvalidateArrival() { station_.reset(); }

bool BaseGuard::ArrivalMatches(const std::string& station, const BaseSample& s,
                              double now, double ros_now) {
  return station_ && *station_ == station && CheckHeld(s, now, ros_now).ok;
}
}
