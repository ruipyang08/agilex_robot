#include "agx_xarm_pick/tube_live_observer.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
namespace agxcore {
void ValidateTubeObserverInterfaces(const TubeObserverInterfaces& interfaces) {
  for (const auto& name : {interfaces.joint_state_topic, interfaces.gripper_state_topic, interfaces.lift_state_topic}) {
    if (name.size() < 2 || name.front() != '/' || name.back() == '/' || name.find("//") != std::string::npos ||
        name.find_first_not_of("/abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos)
      throw std::invalid_argument("observer topics must be nonempty fully qualified ROS names");
  }
}
namespace {
void DiagnosticText(std::ostream& out,const std::string& text,std::size_t limit=96) {
  out.write(text.data(),std::min(text.size(),limit));
  if (text.size()>limit) out<<"...";
}
bool Valid(const TubeEvidence& e) {
  return !e.source.empty() && std::isfinite(e.source_stamp_s) && e.source_stamp_s > 0 &&
      std::isfinite(e.receipt_steady_s) && e.receipt_steady_s > 0;
}
bool Fresh(const TubeEvidence& e, double source, double receipt, double age) {
  return Valid(e) && std::isfinite(source) && std::isfinite(receipt) &&
      source >= e.source_stamp_s && source - e.source_stamp_s <= age &&
      receipt >= e.receipt_steady_s && receipt - e.receipt_steady_s <= age;
}
bool Ordered(const TubeEvidence& next, const TubeEvidence& previous) {
  return !Valid(previous) || (next.source_stamp_s >= previous.source_stamp_s &&
                            next.receipt_steady_s >= previous.receipt_steady_s);
}
void Motion(TubeMotionObservation* next, const TubeMotionObservation& previous, double age) {
  next->valid = next->valid && Valid(next->evidence) && Ordered(next->evidence, previous.evidence);
  next->previous_evidence = previous.evidence;
  const bool continuous = next->valid && next->stationary && previous.valid && previous.stationary &&
      Fresh(previous.evidence, next->evidence.source_stamp_s, next->evidence.receipt_steady_s, age);
  next->stationary_source_since_s = continuous ? previous.stationary_source_since_s : next->evidence.source_stamp_s;
  next->stationary_receipt_since_s = continuous ? previous.stationary_receipt_since_s : next->evidence.receipt_steady_s;
}
const char* WindowInput(const TubeEvidence& next, const TubeEvidence& original,
    double last_receipt, bool valid, bool same_payload, double age) {
  if (!valid || !Valid(next)) return "invalid measurement";
  if (!Valid(original)) return nullptr;
  if (next.source != original.source) return "measurement source changed";
  if (next.source_stamp_s < original.source_stamp_s || next.receipt_steady_s < last_receipt)
    return "measurement clock regressed";
  if (next.source_stamp_s-original.source_stamp_s > age ||
      next.receipt_steady_s-original.receipt_steady_s > age) return "original measurement gap";
  if (next.source_stamp_s == original.source_stamp_s && !same_payload)
    return "contradictory same-stamp measurement";
  return nullptr;
}
void Accumulate(TubeMotionExtrema* range, double position, double velocity, const TubeEvidence& e) {
  if (position < range->minimum) {range->minimum=position;range->minimum_evidence=e;}
  if (position > range->maximum) {range->maximum=position;range->maximum_evidence=e;}
  if (std::abs(velocity) > range->max_abs_velocity) {
    range->max_abs_velocity=std::abs(velocity);range->peak_evidence=e;
  }
}
}
TubeObservationBuffer::TubeObservationBuffer(TubeObserverConfig config) : config_(config) {
  for (double value : {config.gripper_max_age_s, config.arm_max_age_s, config.lift_max_age_s,
                       config.command_tolerance, config.stop_settle_s})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("observation limits must be finite and positive");
  if (!std::isfinite(config.fully_open_command) || !std::isfinite(config.close_command) ||
      !std::isfinite(config.stopped_velocity_rad_s) || config.stopped_velocity_rad_s < 0 ||
      !std::isfinite(config.stopped_arm_range_rad) || config.stopped_arm_range_rad < 0 ||
      !std::isfinite(config.stopped_lift_range_m) || config.stopped_lift_range_m < 0)
    throw std::invalid_argument("invalid gripper/stop observation limits");
}
void TubeObservationBuffer::Gripper(TubeGripperObservation observation) {
  std::lock_guard<std::mutex> lock(mutex_);
  observation.valid = observation.valid && Valid(observation.evidence) &&
      Ordered(observation.evidence, state_.gripper.evidence) && std::isfinite(observation.position) &&
      std::isfinite(observation.target_position);
  state_.gripper = std::move(observation);
}
void TubeObservationBuffer::Arm(TubeArmObservation observation) {
  std::lock_guard<std::mutex> lock(mutex_);
  WindowArm(observation);
  observation.stationary = true;
  for (std::size_t i = 0; i < observation.positions.size(); ++i) {
    observation.valid = observation.valid && std::isfinite(observation.positions[i]) && std::isfinite(observation.velocities[i]);
    observation.stationary = observation.stationary && std::abs(observation.velocities[i]) <= config_.stopped_velocity_rad_s;
  }
  const bool previous_current = state_.arm.valid && state_.arm.stationary &&
      Fresh(state_.arm.evidence, observation.evidence.source_stamp_s,
          observation.evidence.receipt_steady_s, config_.arm_max_age_s);
  for (std::size_t i = 0; i < observation.positions.size(); ++i) {
    if (!previous_current) arm_stationary_min_[i] = arm_stationary_max_[i] = observation.positions[i];
    else {
      arm_stationary_min_[i] = std::min(arm_stationary_min_[i], observation.positions[i]);
      arm_stationary_max_[i] = std::max(arm_stationary_max_[i], observation.positions[i]);
      observation.stationary = observation.stationary &&
          arm_stationary_max_[i] - arm_stationary_min_[i] <= config_.stopped_arm_range_rad;
    }
  }
  Motion(&observation, state_.arm, config_.arm_max_age_s);
  state_.arm = std::move(observation);
}
void TubeObservationBuffer::Lift(TubeLiftObservation observation) {
  std::lock_guard<std::mutex> lock(mutex_);
  WindowLift(observation);
  if (!Valid(observation.evidence) || (Valid(last_lift_input_.evidence) &&
      (observation.evidence.source != last_lift_input_.evidence.source ||
       !Ordered(observation.evidence, last_lift_input_.evidence)))) {
    state_.lift.valid = false; state_.lift.stationary = false;
    return;
  }
  if (Valid(last_lift_input_.evidence) &&
      observation.evidence.source_stamp_s == last_lift_input_.evidence.source_stamp_s) {
    const bool unchanged = observation.valid == last_lift_input_.valid &&
        observation.stationary == last_lift_input_.stationary &&
        observation.position_m == last_lift_input_.position_m &&
        observation.velocity_mps == last_lift_input_.velocity_mps;
    // Track receipt ordering, but never renew the measurement's receipt or dwell
    // with a repeated source sample. Contradictory same-stamp payloads fail closed.
    last_lift_input_.evidence.receipt_steady_s = observation.evidence.receipt_steady_s;
    if (!unchanged) { state_.lift.valid = false; state_.lift.stationary = false; }
    return;
  }
  last_lift_input_ = observation;
  observation.valid = observation.valid && std::isfinite(observation.position_m) &&
      std::isfinite(observation.velocity_mps);
  observation.stationary = observation.stationary && observation.velocity_mps == 0;
  const bool previous_current = state_.lift.valid && state_.lift.stationary &&
      Fresh(state_.lift.evidence, observation.evidence.source_stamp_s,
          observation.evidence.receipt_steady_s, config_.lift_max_age_s);
  if (!previous_current) lift_stationary_min_m_ = lift_stationary_max_m_ = observation.position_m;
  else {
    lift_stationary_min_m_ = std::min(lift_stationary_min_m_, observation.position_m);
    lift_stationary_max_m_ = std::max(lift_stationary_max_m_, observation.position_m);
    observation.stationary = observation.stationary &&
        lift_stationary_max_m_ - lift_stationary_min_m_ <= config_.stopped_lift_range_m;
  }
  Motion(&observation, state_.lift, config_.lift_max_age_s);
  state_.lift = std::move(observation);
}
void TubeObservationBuffer::WindowFailure(const std::string& detail, const TubeEvidence& evidence) {
  if (!window_attempted_) return;
  auto& w=state_.motion_window;
  if (w.failure_detail.empty()) {w.failure_detail=detail;w.failure_evidence=evidence;}
  w.valid=false;
}
void TubeObservationBuffer::WindowArm(const TubeArmObservation& next) {
  const auto& prior=window_arm_input_;
  bool finite=next.valid;
  for (std::size_t i=0;i<6;++i) finite=finite && std::isfinite(next.positions[i]) && std::isfinite(next.velocities[i]);
  const char* error=WindowInput(next.evidence,prior.evidence,window_arm_receipt_,finite,
      next.valid==prior.valid && next.positions==prior.positions && next.velocities==prior.velocities,
      window_attempted_ ? config_.arm_max_age_s : std::numeric_limits<double>::infinity());
  if (error) {window_arm_valid_=false;WindowFailure(std::string("arm: ")+error,next.evidence);return;}
  window_arm_receipt_=next.evidence.receipt_steady_s;
  if (Valid(prior.evidence) && next.evidence.source_stamp_s==prior.evidence.source_stamp_s) return;
  window_arm_input_=next;window_arm_valid_=true;
  if (!window_attempted_) return;
  auto& w=state_.motion_window;w.arm_evidence=next.evidence;
  for (std::size_t i=0;i<6;++i) Accumulate(&w.arm[i],next.positions[i],next.velocities[i],next.evidence);
}
void TubeObservationBuffer::WindowLift(const TubeLiftObservation& next) {
  const auto& prior=window_lift_input_;
  const char* error=WindowInput(next.evidence,prior.evidence,window_lift_receipt_,
      next.valid && std::isfinite(next.position_m) && std::isfinite(next.velocity_mps),
      next.valid==prior.valid && next.stationary==prior.stationary &&
      next.position_m==prior.position_m && next.velocity_mps==prior.velocity_mps,
      window_attempted_ ? config_.lift_max_age_s : std::numeric_limits<double>::infinity());
  if (error) {window_lift_valid_=false;WindowFailure(std::string("lift: ")+error,next.evidence);return;}
  window_lift_receipt_=next.evidence.receipt_steady_s;
  if (Valid(prior.evidence) && next.evidence.source_stamp_s==prior.evidence.source_stamp_s) return;
  window_lift_input_=next;window_lift_valid_=true;
  if (!window_attempted_) return;
  auto& w=state_.motion_window;w.lift_evidence=next.evidence;
  Accumulate(&w.lift,next.position_m,next.velocity_mps,next.evidence);
}
TubeMotionWindowToken TubeObservationBuffer::ArmMotionWindow(
    const Clock& source_clock, const Clock& steady_clock) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (window_attempted_) {WindowFailure("motion window cannot rearm",state_.motion_window.token.barrier);return {};}
  window_attempted_=true;
  auto& w=state_.motion_window;
  double source=0,receipt=0;
  try {
    if (!source_clock || !steady_clock) throw std::invalid_argument("missing clocks");
    source=source_clock();receipt=steady_clock();
  } catch (...) {WindowFailure("motion window clocks unavailable",{});return {};}
  const auto current=SnapshotLocked(source,receipt);
  if (!current.arm.fresh || !current.lift.fresh || !window_arm_valid_ || !window_lift_valid_ ||
      !Fresh(window_arm_input_.evidence,source,receipt,config_.arm_max_age_s) ||
      !Fresh(window_lift_input_.evidence,source,receipt,config_.lift_max_age_s)) {
    WindowFailure("motion window anchors are invalid or stale",{"motion_window",source,receipt});return {};
  }
  static std::atomic<std::uint64_t> next_generation{1};
  w.token={next_generation.fetch_add(1),{"motion_window",source,receipt}};
  if (!w.token.generation) {WindowFailure("motion window generation exhausted",w.token.barrier);return {};}
  w.valid=true;w.arm_evidence=window_arm_input_.evidence;w.lift_evidence=window_lift_input_.evidence;
  for (std::size_t i=0;i<6;++i) w.arm[i]={window_arm_input_.positions[i],window_arm_input_.positions[i],
      std::abs(window_arm_input_.velocities[i]),w.arm_evidence,w.arm_evidence,w.arm_evidence};
  w.lift={window_lift_input_.position_m,window_lift_input_.position_m,std::abs(window_lift_input_.velocity_mps),
      w.lift_evidence,w.lift_evidence,w.lift_evidence};
  return w.token;
}
TubeLiveSnapshot TubeObservationBuffer::Snapshot(double source, double receipt) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return SnapshotLocked(source, receipt);
}
TubeLiveSnapshot TubeObservationBuffer::Snapshot(const Clock& source_clock, const Clock& steady_clock) const {
  if (!source_clock || !steady_clock) throw std::invalid_argument("observation clocks are required");
  std::lock_guard<std::mutex> lock(mutex_);
  const double source = source_clock();
  const double receipt = steady_clock();
  return SnapshotLocked(source, receipt);
}
TubeLiveSnapshot TubeObservationBuffer::SnapshotAfterClockCatchUp(
    const Clock& source_clock, const Clock& steady_clock,
    double max_future_source_s, double max_wait_s) const {
  if (!std::isfinite(max_future_source_s) || max_future_source_s < 0 ||
      !std::isfinite(max_wait_s) || max_wait_s < 0)
    throw std::invalid_argument("clock catch-up bounds must be finite and nonnegative");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(max_wait_s);
  for (;;) {
    auto out = Snapshot(source_clock, steady_clock);
    const auto future = [&](const TubeEvidence& evidence) {
      return Valid(evidence) && std::isfinite(out.source_now_s) &&
          evidence.source_stamp_s > out.source_now_s;
    };
    double lead = 0;
    for (const TubeEvidence* evidence : {&out.gripper.evidence, &out.arm.evidence,
                                        &out.lift.evidence, &out.motion_window.arm_evidence,
                                        &out.motion_window.lift_evidence}) {
      if (future(*evidence))
        lead = std::max(lead, evidence->source_stamp_s - out.source_now_s);
    }
    if (lead == 0 || lead > max_future_source_s ||
        std::chrono::steady_clock::now() >= deadline) return out;
    // Never hold the observation mutex while /clock or state callbacks catch up.
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
TubeLiveSnapshot TubeObservationBuffer::SnapshotLocked(double source, double receipt) const {
  auto out = state_;
  out.source_now_s = source; out.steady_now_s = receipt; out.stop_settle_s = config_.stop_settle_s;
  auto& g = out.gripper;
  g.fresh = g.valid && Fresh(g.evidence, source, receipt, config_.gripper_max_age_s);
  g.known_empty = g.fresh && known_open_gripper_state(g.initialized, g.grip_state,
      g.position, g.target_position, config_.fully_open_command, config_.command_tolerance);
  g.caught = g.fresh && known_caught_gripper_state(g.initialized, g.grip_state,
      g.target_position, config_.close_command, config_.command_tolerance);
  out.arm.fresh = out.arm.valid && Fresh(out.arm.evidence, source, receipt, config_.arm_max_age_s);
  out.lift.fresh = out.lift.valid && Fresh(out.lift.evidence, source, receipt, config_.lift_max_age_s);
  auto& w=out.motion_window;
  w.fresh=w.valid && Fresh(w.arm_evidence,source,receipt,config_.arm_max_age_s) &&
      Fresh(w.lift_evidence,source,receipt,config_.lift_max_age_s);
  if (w.valid && !w.fresh && w.failure_detail.empty()) {
    // Annotate only this read; callback invalidation retains its own first cause.
    try {
      const bool arm_fresh=Fresh(w.arm_evidence,source,receipt,config_.arm_max_age_s);
      const bool lift_fresh=Fresh(w.lift_evidence,source,receipt,config_.lift_max_age_s);
      std::ostringstream detail;detail.exceptions(std::ios::badbit|std::ios::failbit);
      detail<<std::setprecision(17)<<"observer freshness: first="<<(arm_fresh?"lift":"arm")
          <<" arm_fresh="<<arm_fresh<<" lift_fresh="<<lift_fresh
          <<" source_now="<<source<<" steady_now="<<receipt
          <<" arm_max_age="<<config_.arm_max_age_s<<" lift_max_age="<<config_.lift_max_age_s;
      const auto evidence=[&](const char* name,const TubeEvidence& e) {
        detail<<" "<<name<<"_source=";DiagnosticText(detail,e.source);
        detail<<" "<<name<<"_stamp="<<e.source_stamp_s<<" "<<name<<"_receipt="<<e.receipt_steady_s;
      };
      evidence("arm",w.arm_evidence);evidence("lift",w.lift_evidence);
      auto message=detail.str();auto rejected=arm_fresh?w.lift_evidence:w.arm_evidence;
      // All potentially throwing work precedes these nonthrowing updates.
      w.failure_detail.swap(message);w.failure_evidence.source.swap(rejected.source);
      w.failure_evidence.source_stamp_s=rejected.source_stamp_s;
      w.failure_evidence.receipt_steady_s=rejected.receipt_steady_s;
    } catch (...) { /* Diagnostic failure cannot change the stale result. */ }
  }
  return out;
}
bool TubeLiveSnapshot::ArmLiftStoppedSince(const TubeEvidence& request) const {
  if (!Valid(request)) return false;
  const auto stopped = [&](const TubeMotionObservation& motion) {
    return motion.fresh && motion.stationary &&
        motion.previous_evidence.source_stamp_s > request.source_stamp_s &&
        motion.previous_evidence.receipt_steady_s > request.receipt_steady_s &&
        motion.evidence.source_stamp_s > motion.previous_evidence.source_stamp_s &&
        motion.evidence.receipt_steady_s > motion.previous_evidence.receipt_steady_s &&
        motion.evidence.source_stamp_s - std::max(request.source_stamp_s, motion.stationary_source_since_s) >= stop_settle_s &&
        motion.evidence.receipt_steady_s - std::max(request.receipt_steady_s, motion.stationary_receipt_since_s) >= stop_settle_s;
  };
  return stopped(arm) && stopped(lift);
}
}
