#include "agx_workbench_navigation/canonical_motion_feedback.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <cstring>
using namespace agx_workbench_navigation;
void retiredBootCannotReviveBoundOperation() {
  CommandBinding binding{"boot-a","grant-a","operation-a",
      {"map",10000000000ULL,{1,0,0},{0,0,0,1}}};
  CommandSourceGate gate;
  assert(gate.Bind(binding,[]{}));
  assert(gate.AcceptPlan(binding.endpoint,[]{}));
  CanonicalMotionState state;
  state.session_id=binding.session_id;state.grant_id=binding.grant_id;
  state.source_goal_id=binding.source_goal_id;
  state.navigation_active=true;state.motion_valid=true;state.revision=1;
  state.authority_stamp_s=10;state.motion_stamp_s=10;state.motion_stamp_ns=10000000000ULL;
  state.frame_id="base_link";state.receipt_age_s=.01;state.receipt_limit_s=.25;
  state.header_limit_s=.35;state.future_tolerance_s=.05;
  state.linear_lead_mps=.002;state.angular_lead_radps=.004;state.motion={{.1,0,0,0,0,0}};
  CanonicalMotionFeedback cache;
  cache.observe(state,100);
  assert(cache.current(binding,10,100).valid);
  auto new_boot=state;
  new_boot.session_id="boot-b";new_boot.grant_id="";new_boot.source_goal_id="";
  new_boot.navigation_active=false;
  cache.observe(new_boot,100.01);
  assert(!cache.current(binding,10.01,100.01).valid);
  // The authority callback does not rebind the gate. An old, still-fresh
  // heartbeat must never restore evidence to the captured original operation.
  cache.observe(state,100.02);
  assert(gate.Compute([&](const CommandBinding& captured,std::uint64_t,bool ready) {
    assert(ready);
    assert(!cache.current(captured,10.02,100.02).valid);
    return true;
  }));
  // Even a newer measurement revision cannot rehabilitate the retired boot.
  state.revision=2;state.motion_stamp_s=10.03;state.motion_stamp_ns=10030000000ULL;
  state.authority_stamp_s=10.03;
  cache.observe(state,100.03);
  assert(!cache.current(binding,10.03,100.03).valid);
  // The actual new session remains usable through a separately bound goal;
  // rejected replays must not mark that current boot as retired in turn.
  CommandBinding next{"boot-b","grant-b","operation-b",
      {"map",11000000000ULL,{1,0,0},{0,0,0,1}}};
  assert(gate.Bind(next,[]{}));
  assert(gate.AcceptPlan(next.endpoint,[]{}));
  new_boot.grant_id=next.grant_id;new_boot.source_goal_id=next.source_goal_id;
  new_boot.navigation_active=true;
  cache.observe(new_boot,100.04);
  assert(gate.Compute([&](const CommandBinding& captured,std::uint64_t,bool ready) {
    assert(ready && cache.current(captured,10.04,100.04).valid);
    return true;
  }));
}

int main() {
  retiredBootCannotReviveBoundOperation();
  CommandBinding binding{"session","grant","goal",{}};
  CanonicalMotionState state;
  const auto setStamp=[&](double stamp) {
    state.motion_stamp_s=stamp;
    state.motion_stamp_ns=static_cast<std::uint64_t>(stamp*1e9);
  };
  state.session_id="session";state.grant_id="grant";state.source_goal_id="goal";
  state.navigation_active=true;state.motion_valid=true;state.revision=1;
  state.authority_stamp_s=10;setStamp(10);state.frame_id="base_link";
  state.receipt_age_s=.1;state.receipt_limit_s=.25;state.header_limit_s=.35;
  state.future_tolerance_s=.01;state.linear_lead_mps=.002;state.angular_lead_radps=.004;
  state.motion={.1,-.02,0,0,0,-.05};
  CanonicalMotionFeedback cache;
  cache.observe(state,20);
  auto motion=cache.current(binding,10,20);
  assert(motion.valid && motion.forward_mps==.1 && motion.lateral_mps==-.02 && motion.yaw_radps==-.05);
  assert(motion.revision==1 && motion.source_stamp_ns==10000000000ULL && motion.source_stamp_s==10);
  // A query clock captured before a newer callback receipt must stay invalid,
  // but report that chronology failure separately from true stale evidence.
  auto rejected=cache.current(binding,10,19.999);
  assert(!rejected.valid && std::strcmp(rejected.unavailable_reason,"query_precedes_receipt")==0);
  rejected=cache.current(binding,10,20.3);
  assert(!rejected.valid && std::strcmp(rejected.unavailable_reason,"measurement_receipt_expired")==0);
  rejected=cache.current(binding,10.4,20);
  assert(!rejected.valid && std::strcmp(rejected.unavailable_reason,"motion_header_expired")==0);
  auto wrong_binding=binding;wrong_binding.grant_id="other";
  rejected=cache.current(wrong_binding,10,20);
  assert(!rejected.valid && std::strcmp(rejected.unavailable_reason,"binding_mismatch")==0);
  // Reproduce trial21's interleaving: the caller captured its clock, then
  // a callback installed a newer original measurement before the locked read.
  CanonicalMotionFeedback interleaved;
  auto newer=state;newer.revision=2;newer.motion_stamp_s=10.02;
  newer.motion_stamp_ns=10020000000ULL;newer.authority_stamp_s=10.02;
  interleaved.observe(state,20);
  const double early_query=20.001;
  interleaved.observe(newer,20.002);
  assert(!interleaved.current(binding,10.02,early_query).valid);
  int clock_reads=0;
  const auto read_clock=[&]{++clock_reads;return 20.003;};
  const auto joined=interleaved.currentAtRead(binding,10.02,read_clock);
  assert(joined.valid && joined.revision==2 && joined.source_stamp_ns==10020000000ULL);
  assert(clock_reads==1);
  // Sampling at read time must not renew the new measurement's original lease.
  assert(!interleaved.currentAtRead(binding,10.02,[]{return 20.3;}).valid);
  // Repeated heartbeats cannot give this measurement a new receipt lease.
  state.authority_stamp_s=10.1;state.receipt_age_s=.01;
  cache.observe(state,20.1);
  assert(cache.current(binding,10.1,20.1).valid);
  assert(cache.current(binding,10.1,20.1).source_stamp_ns==10000000000ULL);
  assert(cache.current(binding,10.1,20.1).revision==1);
  state.authority_stamp_s=10.2;cache.observe(state,20.2);
  assert(!cache.current(binding,10.2,20.2).valid);
  // New evidence, current state and binding are all independently required.
  state.revision=2;setStamp(10.2);state.receipt_age_s=0;
  cache.observe(state,20.2);
  assert(cache.current(binding,10.2,20.2).valid);
  auto other=binding;other.grant_id="other";
  assert(!cache.current(other,10.2,20.2).valid);
  assert(!cache.current(binding,10.6,20.2).valid);
  assert(!cache.current(binding,10.2,20.5).valid);
  assert(!cache.current(binding,10.1,20.2).valid);
  // Same revision may be rebound, but cannot mutate its sample or reset age.
  state.grant_id="other";cache.observe(state,20.21);
  assert(cache.current(other,10.21,20.21).valid);
  state.motion[0]=.2;cache.observe(state,20.22);
  assert(!cache.current(other,10.22,20.22).valid);
  state.motion[0]=.1;cache.observe(state,20.23);
  assert(!cache.current(other,10.23,20.23).valid);
  state.revision=3;setStamp(10.23);state.authority_stamp_s=10.23;
  cache.observe(state,20.23);assert(cache.current(other,10.23,20.23).valid);
  state.motion_valid=false;cache.observe(state,20.24);
  assert(!cache.current(other,10.24,20.24).valid);
  state.motion_valid=true;cache.observe(state,20.25);
  assert(!cache.current(other,10.25,20.25).valid);
  state.revision=4;setStamp(10.25);state.authority_stamp_s=10.25;
  cache.observe(state,20.25);assert(cache.current(other,10.25,20.25).valid);
  // Same-revision timestamp identity uses exact ROS nanoseconds, including
  // changes too small to survive conversion of epoch time into a double.
  CanonicalMotionFeedback exact_stamp;
  state.motion_stamp_ns=10250000000ULL;
  exact_stamp.observe(state,20.25);
  assert(exact_stamp.current(other,10.25,20.25).valid);
  ++state.motion_stamp_ns;
  exact_stamp.observe(state,20.25);
  assert(!exact_stamp.current(other,10.25,20.25).valid);
  // Limits are immutable within a session, including after restoration.
  state.linear_lead_mps=.05;cache.observe(state,20.26);
  assert(!cache.current(other,10.26,20.26).valid);
  state.linear_lead_mps=.002;state.revision=5;cache.observe(state,20.27);
  assert(!cache.current(other,10.27,20.27).valid);
  // An old session cannot satisfy the current command binding, and a new
  // operation reusing the revision still carries its original receipt age.
  CanonicalMotionFeedback rebound;
  state.session_id="new_session";state.revision=1;setStamp(10.3);
  state.authority_stamp_s=10.3;state.receipt_age_s=.2;
  rebound.observe(state,20.3);
  auto new_binding=other;new_binding.session_id="new_session";
  assert(rebound.current(new_binding,10.3,20.3).valid);
  assert(!rebound.current(other,10.3,20.3).valid);
  state.grant_id="third";state.source_goal_id="next_goal";state.receipt_age_s=0;
  state.authority_stamp_s=10.34;rebound.observe(state,20.34);
  new_binding.grant_id="third";new_binding.source_goal_id="next_goal";
  assert(rebound.current(new_binding,10.34,20.34).valid);
  assert(!rebound.current(new_binding,10.36,20.36).valid);
  // Malformed payloads and authority faults always deny fresh data.
  for(int kind=0;kind<5;++kind) {
    CanonicalMotionFeedback isolated;
    auto bad=state;bad.revision=1;
    if(kind==0) bad.frame_id="odom";
    if(kind==1) bad.motion[2]=.01;
    if(kind==2) bad.motion[0]=std::numeric_limits<double>::quiet_NaN();
    if(kind==3) bad.navigation_active=false;
    if(kind==4) bad.receipt_age_s=-1;
    isolated.observe(state,20.3);
    assert(isolated.current(new_binding,10.34,20.3).valid);
    isolated.observe(bad,20.3);assert(!isolated.current(new_binding,10.34,20.3).valid);
  }
}
