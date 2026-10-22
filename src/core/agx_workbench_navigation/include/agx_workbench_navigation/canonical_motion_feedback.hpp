#pragma once

#include <agx_workbench_navigation/command_source_gate.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>

namespace agx_workbench_navigation {

// Validated signed interval motion. Only CanonicalMotionFeedback turns authority
// telemetry into this input; a controller never estimates it from RF2O twist.
struct CanonicalMotionProgress {
  bool valid=false;
  double forward_mps=0, lateral_mps=0, yaw_radps=0;
  double linear_lead_mps=0, angular_lead_radps=0;
  // Original accepted observation identity, never the authority heartbeat time.
  std::uint64_t revision=0, source_stamp_ns=0;
  double source_stamp_s=0;
  // Bounded literal from the actual rejecting query; no logging on this path.
  const char* unavailable_reason="";
};

struct CanonicalMotionState {
  std::string session_id, grant_id, source_goal_id, frame_id;
  bool navigation_active=false, motion_valid=false;
  std::uint64_t revision=0, motion_stamp_ns=0;
  double authority_stamp_s=0, motion_stamp_s=0, receipt_age_s=0;
  double receipt_limit_s=0, header_limit_s=0, future_tolerance_s=0;
  double linear_lead_mps=0, angular_lead_radps=0;
  // vx, vy, vz, wx, wy, wz, in the canonical command frame.
  std::array<double,6> motion{};
};

// Caller serializes observe/current independently of the command binding gate.
// These snapshots provide evidence, never grant/relabel command authority.
class CanonicalMotionFeedback {
 public:
  void observe(const CanonicalMotionState& state,double receipt_steady_s) {
    latest_=state;latest_receipt_s_=receipt_steady_s;available_=false;
    if(state.session_id.empty() || !std::isfinite(receipt_steady_s) || receipt_steady_s<0) return;
    if(state.session_id!=active_session_) {
      if(retired_sessions_.count(state.session_id)) return;
      // Observing a new boot retires its predecessor even before navigation
      // becomes active. Late heartbeats cannot revive a still-bound old goal.
      if(!active_session_.empty()) retired_sessions_.insert(active_session_);
      active_session_=state.session_id;
    }
    auto& session=sessions_[state.session_id];
    const std::array<double,5> limits{{state.receipt_limit_s,state.header_limit_s,
        state.future_tolerance_s,state.linear_lead_mps,state.angular_lead_radps}};
    if(!session.initialized) {
      session.initialized=true;session.limits=limits;
    } else if(session.limits!=limits) session.invalid_limits=true;
    if(!validLimits(limits) || session.invalid_limits) return;
    // Keep the first local receipt and upstream age for a revision. Heartbeats
    // can only increase the conservative age, never renew the measurement.
    if(state.revision>session.sample.revision) {
      const bool advances=state.motion_stamp_ns>session.sample.motion_stamp_ns &&
          state.motion_stamp_s>session.sample.motion_stamp_s;
      session.sample=state;session.first_receipt_s=receipt_steady_s;
      session.original_age_s=state.receipt_age_s;session.invalid_sample=!advances;
    } else if(state.revision!=session.sample.revision) {
      return;
    } else {
      if(state.motion!=session.sample.motion || state.frame_id!=session.sample.frame_id ||
          state.motion_stamp_s!=session.sample.motion_stamp_s ||
          state.motion_stamp_ns!=session.sample.motion_stamp_ns) session.invalid_sample=true;
      const double elapsed=receipt_steady_s-session.first_receipt_s;
      if(elapsed<0 || !std::isfinite(elapsed)) session.invalid_sample=true;
      session.original_age_s=std::max(session.original_age_s,state.receipt_age_s-elapsed);
    }
    if(!validSample(state)) session.invalid_sample=true;
    available_=state.navigation_active && !session.invalid_sample;
  }

  CanonicalMotionProgress current(const CommandBinding& binding,double source_now_s,
                                  double steady_now_s) const {
    if(!available_) return unavailable("state_unavailable");
    if(binding.session_id.empty() || binding.grant_id.empty() ||
        binding.source_goal_id.empty() || latest_.session_id!=binding.session_id ||
        latest_.grant_id!=binding.grant_id || latest_.source_goal_id!=binding.source_goal_id)
      return unavailable("binding_mismatch");
    if(!std::isfinite(source_now_s) || !std::isfinite(steady_now_s))
      return unavailable("invalid_query_time");
    const auto found=sessions_.find(binding.session_id);
    if(found==sessions_.end()) return unavailable("missing_session");
    const auto& session=found->second;
    const double local_age=steady_now_s-session.first_receipt_s;
    const double receipt_age=session.original_age_s+local_age;
    const double heartbeat_age=steady_now_s-latest_receipt_s_;
    const double header_age=source_now_s-latest_.motion_stamp_s;
    const double authority_age=source_now_s-latest_.authority_stamp_s;
    if(session.invalid_limits || session.invalid_sample) return unavailable("invalid_session_sample");
    if(local_age<0 || heartbeat_age<0) return unavailable("query_precedes_receipt");
    if(!std::isfinite(receipt_age) || receipt_age>latest_.receipt_limit_s)
      return unavailable("measurement_receipt_expired");
    if(heartbeat_age>latest_.receipt_limit_s) return unavailable("heartbeat_receipt_expired");
    if(header_age < -latest_.future_tolerance_s) return unavailable("motion_header_future");
    if(header_age>latest_.header_limit_s) return unavailable("motion_header_expired");
    if(authority_age < -latest_.future_tolerance_s) return unavailable("authority_header_future");
    if(authority_age>latest_.header_limit_s) return unavailable("authority_header_expired");
    return {true,latest_.motion[0],latest_.motion[1],latest_.motion[5],
            latest_.linear_lead_mps,latest_.angular_lead_radps,
            latest_.revision,latest_.motion_stamp_ns,latest_.motion_stamp_s};
  }
  // Invoke only after acquiring the caller's evidence lock. Passing the clock
  // callable prevents an earlier decision time being joined to a newer callback.
  // The observation and its original receipt/age remain unchanged.
  template<typename SteadyClock>
  CanonicalMotionProgress currentAtRead(const CommandBinding& binding,double source_now_s,
                                        SteadyClock clock) const {
    return current(binding,source_now_s,clock());
  }
 private:
  static CanonicalMotionProgress unavailable(const char* reason) {
    CanonicalMotionProgress result;result.unavailable_reason=reason;return result;
  }
  struct Session {
    bool initialized=false, invalid_limits=false, invalid_sample=false;
    std::array<double,5> limits{};
    CanonicalMotionState sample;
    double first_receipt_s=0, original_age_s=0;
  };
  static bool validLimits(const std::array<double,5>& limits) {
    for(double value:limits) if(!std::isfinite(value) || value<0) return false;
    return limits[0]>0 && limits[1]>0 && limits[3]>0 && limits[4]>0;
  }
  static bool validSample(const CanonicalMotionState& state) {
    if(!state.motion_valid || state.revision==0 || state.motion_stamp_ns==0 || state.frame_id!="base_link" ||
        !std::isfinite(state.motion_stamp_s) || state.motion_stamp_s<=0 ||
        !std::isfinite(state.authority_stamp_s) || state.authority_stamp_s<=0 ||
        !std::isfinite(state.receipt_age_s) || state.receipt_age_s<0) return false;
    for(double value:state.motion) if(!std::isfinite(value)) return false;
    return state.motion[2]==0 && state.motion[3]==0 && state.motion[4]==0;
  }
  std::map<std::string,Session> sessions_;
  std::string active_session_;
  std::set<std::string> retired_sessions_;
  CanonicalMotionState latest_;
  double latest_receipt_s_=0;
  bool available_=false;
};
}  // namespace agx_workbench_navigation
