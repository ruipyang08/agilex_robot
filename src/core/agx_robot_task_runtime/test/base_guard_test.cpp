#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>
#include "agx_robot_task_runtime/base_guard.hpp"

using namespace agx_robot_task_runtime;

BaseSample Fresh(double now) {
  BaseSample sample;
  sample.authority_received = sample.odometry_received = sample.command_received = true;
  sample.watchdog_received = sample.goal_status_received = true;
  sample.watchdog_ready = true;
  sample.authority_receipt_s = sample.odometry_receipt_s = sample.command_receipt_s = now;
  sample.watchdog_receipt_s = sample.goal_status_receipt_s = now;
  sample.odometry_stamp_s = sample.authority_stamp_s = sample.goal_status_stamp_s = now;
  sample.authority_none = sample.manipulation_hold = sample.command_zero = true;
  sample.owned_goal_terminal = true;
  sample.owned_goal_id = "goal-1";
  sample.session_id = "arbiter-1";
  sample.hold_epoch = 1;
  return sample;
}

int main() {
  // A status producer may process the next /clock tick before its consumer.
  assert(FreshStatusEvidence(9.333333333,142.08,9.316666667,142.09,1.,1.,.05));
  assert(!FreshStatusEvidence(9.37,142.08,9.316666667,142.09,1.,1.,.05));
  assert(!FreshStatusEvidence(9.333333333,142.08,9.316666667,142.09,1.,1.,0.));
  assert(!FreshStatusEvidence(8.3,142.08,9.316666667,142.09,1.,1.,.05));
  assert(!FreshStatusEvidence(9.3,141.,9.316666667,142.09,1.,1.,.05));
  assert(!FreshStatusEvidence(9.3,142.1,9.316666667,142.09,1.,1.,.05));
  assert(!FreshStatusEvidence(0.,142.08,9.316666667,142.09,1.,1.,.05));
  assert(!FreshStatusEvidence(std::numeric_limits<double>::quiet_NaN(),142.08,9.316666667,142.09,1.,1.,.05));
  assert(!FreshStatusEvidence(9.3,142.08,9.316666667,142.09,1.,1.,-1.));
  BaseGuardConfig config{.25, .25, .02, .02, .01, .5, .015, .035, 1.0, .5};
  BaseGuard guard(config);
  auto sample = Fresh(10.0);
  assert(guard.CheckStopped(sample, 10.0, 10.0, true, true).ok);
  for (const auto& mutate : std::vector<std::function<void(BaseSample&)>>{
      [](auto& s) { s.authority_none=false; },
      [](auto& s) { s.authority_fault=true; },
      [](auto& s) { s.watchdog_ready=false; },
      [](auto& s) { s.manipulation_hold=false; },
      [](auto& s) { s.authority_received=false; },
      [](auto& s) { s.odometry_received=false; },
      [](auto& s) { s.command_received=false; },
      [](auto& s) { s.authority_receipt_s=9.0; },
      [](auto& s) { s.odometry_receipt_s=9.0; },
      [](auto& s) { s.command_receipt_s=9.0; },
      [](auto& s) { s.watchdog_receipt_s=9.0; },
      [](auto& s) { s.odometry_stamp_s=9.0; },
      [](auto& s) { s.authority_stamp_s=9.0; },
      [](auto& s) { s.authority_stamp_s=10.03; },
      [](auto& s) { s.authority_stamp_s=0.0; },
      [](auto& s) { s.authority_stamp_s=std::numeric_limits<double>::quiet_NaN(); },
      [](auto& s) { s.goal_status_stamp_s=8.0; },
      [](auto& s) { s.goal_status_stamp_s=10.03; },
      [](auto& s) { s.goal_status_stamp_s=0.0; },
      [](auto& s) { s.goal_status_stamp_s=std::numeric_limits<double>::infinity(); },
      [](auto& s) { s.odometry_stamp_s=std::numeric_limits<double>::quiet_NaN(); },
      [](auto& s) { s.authority_receipt_s=std::numeric_limits<double>::infinity(); },
      [](auto& s) { s.linear_y_mps=std::numeric_limits<double>::quiet_NaN(); },
      [](auto& s) { s.angular_z_radps=std::numeric_limits<double>::infinity(); },
      [](auto& s) { s.linear_x_mps=.016; s.linear_y_mps=.016; },
      [](auto& s) { s.angular_z_radps=.011; },
      [](auto& s) { s.goal_status_received=false; },
      [](auto& s) { s.owned_goal_id.clear(); },
      [](auto& s) { s.owned_goal_terminal=false; },
      [](auto& s) { s.goal_status_receipt_s=8.0; }}) {
    auto invalid=sample;
    mutate(invalid);
    assert(!guard.CheckStopped(invalid,10.0,10.0,true,true).ok);
  }
  assert(guard.CaptureHold(sample, 10.0, 10.0, "A").ok);
  assert(guard.ArrivalMatches("A", sample, 10.0, 10.0));
  guard.InvalidateArrival();
  assert(!guard.ArrivalMatches("A", sample, 10.0, 10.0));
  assert(guard.CheckHeld(sample, 10.0, 10.0).ok); // Hold survives departure admission.
  assert(guard.CaptureHold(sample,10.0,10.0,"B").ok);
  assert(guard.ArrivalMatches("B",sample,10.0,10.0));
  assert(!guard.ArrivalMatches("A",sample,10.0,10.0));
  assert(!guard.ArrivalMatches("B",sample,10.3,10.3)); // Cached identity cannot replace fresh evidence.

  for (const auto& mutate:std::vector<std::function<void(BaseSample&)>>{
      [](auto& s) {s.any_goal_active=true;},
      [](auto& s) {s.owned_goal_id="other-operation";},
      [](auto& s) {s.goal_status_receipt_s=8.0;},
      [](auto& s) {s.authority_stamp_s=9.0;},
      [](auto& s) {s.goal_status_stamp_s=10.03;}}) {
    auto active=sample;mutate(active);
    assert(!guard.CheckHeld(active,10.0,10.0).ok);
  }

  auto changed = sample;
  changed.command_zero = false;
  assert(!guard.CheckHeld(changed, 10.0, 10.0).ok);
  changed = sample; changed.hold_epoch = 2;
  assert(!guard.CheckHeld(changed, 10.0, 10.0).ok);
  changed = sample; changed.session_id = "restarted";
  assert(!guard.CheckHeld(changed, 10.0, 10.0).ok);
  changed = sample; changed.x_m = .016;
  assert(!guard.CheckHeld(changed, 10.0, 10.0).ok);
  changed = sample; changed.yaw_rad = .036;
  assert(!guard.CheckHeld(changed, 10.0, 10.0).ok);
  changed = sample; changed.x_m = std::numeric_limits<double>::quiet_NaN();
  assert(!guard.CheckHeld(changed, 10.0, 10.0).ok);
  assert(!guard.CheckStopped(sample, 10.26, 10.26, true, true).ok);
  changed = sample; changed.odometry_stamp_s = 10.03;
  assert(!guard.CheckStopped(changed, 10.0, 10.0, true, true).ok);
  changed = sample; changed.authority_receipt_s = 10.01;
  assert(!guard.CheckStopped(changed, 10.0, 10.0, true, true).ok);
  changed = sample; changed.any_goal_active = true;
  assert(!guard.CheckStopped(changed, 10.0, 10.0, true, true).ok);
  changed = sample; changed.watchdog_received = false;
  assert(!guard.CheckHeld(changed, 10.0, 10.0).ok);

  sample = Fresh(20.0); sample.yaw_rad = 3.141592653589793 - .005;
  assert(guard.CaptureHold(sample, 20.0, 20.0, "B").ok);
  sample.yaw_rad = -3.141592653589793 + .005;
  assert(guard.CheckHeld(sample, 20.0, 20.0).ok); // Wrapped angular displacement.
  sample.linear_x_mps = .03;
  assert(guard.CheckHeld(sample, 20.0, 20.0).ok); // Isolated spike.
  sample = Fresh(20.49); sample.yaw_rad = -3.141592653589793 + .005;
  sample.linear_x_mps = .03;
  assert(guard.CheckHeld(sample, 20.49, 20.49).ok);
  sample = Fresh(20.5); sample.yaw_rad = -3.141592653589793 + .005;
  sample.linear_x_mps = .03;
  assert(!guard.CheckHeld(sample, 20.5, 20.5).ok);

  sample=Fresh(30.0); sample.yaw_rad=-1e308;
  assert(guard.CaptureHold(sample,30.0,30.0).ok);
  sample.yaw_rad=1e308;
  assert(!guard.CheckHeld(sample,30.0,30.0).ok); // Finite subtraction can overflow.

  sample=Fresh(40.0);
  sample.owned_goal_id.clear();
  auto missing_status=sample; missing_status.goal_status_received=false;
  assert(!guard.CaptureHold(missing_status,40.0,40.0).ok);
  assert(guard.CaptureHold(sample,40.0,40.0).ok);
  for(const auto& mutate:std::vector<std::function<void(BaseSample&)>>{
      [](auto& s) {s.goal_status_received=false;},
      [](auto& s) {s.goal_status_stamp_s=38.0;},
      [](auto& s) {s.goal_status_receipt_s=38.0;},
      [](auto& s) {s.any_goal_active=true;}}) {
    auto invalid=sample; mutate(invalid);
    assert(!guard.CheckHeld(invalid,40.0,40.0).ok);
  }
  sample.linear_x_mps=.03;
  assert(guard.CheckHeld(sample,40.0,40.0).ok);
  sample=Fresh(40.4);
  assert(guard.CheckHeld(sample,40.4,40.4).ok); // Quiet observation resets moving dwell.
  sample=Fresh(40.5); sample.linear_x_mps=.03;
  assert(guard.CheckHeld(sample,40.5,40.5).ok);

  // A STOPPING watchdog still reports its heartbeat but withdraws readiness.
  // Emergency evidence can certify a stop without granting normal task authority.
  BaseGuard emergency(config);
  assert(emergency.CaptureHold(Fresh(50),50,50,"A").ok);
  assert(emergency.BeginEmergencyStop(50,50).ok);
  assert(!emergency.ArrivalMatches("A",Fresh(50),50,50));
  auto inhibited=[](double t) {
    auto s=Fresh(t); s.watchdog_ready=false; s.authority_none=false;
    s.authority_fault=true; s.owned_goal_id.clear(); return s;
  };
  assert(!emergency.ObserveEmergencyStop(inhibited(50),50,50).ok); // Pre-command evidence.
  assert(!emergency.ObserveEmergencyStop(inhibited(50.1),50.1,50.1).ok);
  assert(!emergency.ObserveEmergencyStop(inhibited(50.3),50.3,50.3).ok);
  assert(!emergency.ObserveEmergencyStop(inhibited(50.5),50.5,50.5).ok);
  assert(emergency.ObserveEmergencyStop(inhibited(50.7),50.7,50.7).ok);
  assert(!emergency.CheckStopped(inhibited(50.7),50.7,50.7,true,false).ok);
  assert(!emergency.CaptureHold(inhibited(50.7),50.7,50.7,"B").ok);
  assert(!emergency.CheckHeld(inhibited(50.7),50.7,50.7).ok);
  assert(!emergency.ArrivalMatches("A",inhibited(50.7),50.7,50.7));

  for (const auto& mutate:std::vector<std::function<void(BaseSample&)>>{
      [](auto& s) {s.watchdog_received=false;},
      [](auto& s) {s.watchdog_receipt_s-=1;},
      [](auto& s) {s.authority_fault=false;},
      [](auto& s) {s.authority_none=true;}, // Contradictory NONE+FAULT.
      [](auto& s) {s.manipulation_hold=false;},
      [](auto& s) {s.command_zero=false;},
      [](auto& s) {s.any_goal_active=true;},
      [](auto& s) {s.goal_status_received=false;},
      [](auto& s) {s.goal_status_stamp_s-=2;},
      [](auto& s) {s.goal_status_stamp_s+=.03;},
      [](auto& s) {s.odometry_stamp_s=0;},
      [](auto& s) {s.authority_stamp_s=0;},
      [](auto& s) {s.authority_stamp_s=50;}, // Fresh by age, but before barrier.
      [](auto& s) {s.command_receipt_s=50;},
      [](auto& s) {s.session_id.clear();},
      [](auto& s) {s.linear_x_mps=.03;},
      [](auto& s) {s.x_m=std::numeric_limits<double>::quiet_NaN();}}) {
    assert(emergency.BeginEmergencyStop(50,50).ok);
    for(double t:{50.1,50.3,50.5,50.7}) {
      auto bad=inhibited(t); mutate(bad);
      assert(!emergency.ObserveEmergencyStop(bad,t,t).ok);
    }
  }

  // Repeated reads and fresh receipts carrying one old source sample cannot mature a dwell.
  assert(emergency.BeginEmergencyStop(60,60).ok);
  for(double t:{60.1,60.3,60.5,60.7}) {
    auto replay=inhibited(t); replay.odometry_stamp_s=60.1;
    assert(!emergency.ObserveEmergencyStop(replay,t,t).ok);
  }
  // An unobserved interval, motion, session/epoch change or source regression restarts the dwell.
  for (const auto& mutate:std::vector<std::function<void(BaseSample&)>>{
      [](auto& s) {s.linear_x_mps=.03;},
      [](auto& s) {s.x_m=.0021;},
      [](auto& s) {s.yaw_rad=.0051;},
      [](auto& s) {s.session_id="restarted";},
      [](auto& s) {s.hold_epoch=2;},
      [](auto& s) {s.odometry_stamp_s=70.25;},
      [](auto& s) {s.authority_stamp_s=70.25;}}) {
    assert(emergency.BeginEmergencyStop(70,70).ok);
    assert(!emergency.ObserveEmergencyStop(inhibited(70.1),70.1,70.1).ok);
    assert(!emergency.ObserveEmergencyStop(inhibited(70.3),70.3,70.3).ok);
    auto disturbed=inhibited(70.5); mutate(disturbed);
    assert(!emergency.ObserveEmergencyStop(disturbed,70.5,70.5).ok);
    assert(!emergency.ObserveEmergencyStop(inhibited(70.7),70.7,70.7).ok);
  }
  assert(emergency.BeginEmergencyStop(80,80).ok);
  assert(!emergency.ObserveEmergencyStop(inhibited(80.1),80.1,80.1).ok);
  assert(!emergency.ObserveEmergencyStop(inhibited(80.7),80.7,80.7).ok);
  // Cumulative displacement, not adjacent-frame displacement, bounds drift.
  assert(emergency.BeginEmergencyStop(90,90).ok);
  for(int i=1;i<=4;++i) {
    const double t=90+i*.2; auto drifting=inhibited(t); drifting.x_m=i*.001;
    assert(!emergency.ObserveEmergencyStop(drifting,t,t).ok);
  }
  BaseGuard unstarted(config);
  assert(!unstarted.ObserveEmergencyStop(inhibited(100),100,100).ok);
  assert(!unstarted.BeginEmergencyStop(100,std::numeric_limits<double>::quiet_NaN()).ok);
  // Fresh NONE can also confirm an emergency stop, still without capture side effects.
  assert(emergency.BeginEmergencyStop(100,100).ok);
  for(double t:{100.1,100.3,100.5})
    assert(!emergency.ObserveEmergencyStop(Fresh(t),t,t).ok);
  assert(emergency.ObserveEmergencyStop(Fresh(100.7),100.7,100.7).ok);
  assert(!emergency.CheckHeld(Fresh(100.7),100.7,100.7).ok);

  // Reasserted emergency inhibition advances the epoch without physical motion.
  // This stop-only proof accepts it; normal captured holds still require exact epochs.
  assert(emergency.BeginEmergencyStop(110,110).ok);
  for(int i=1;i<=3;++i) {
    const double t=110+i*.2; auto reasserted=inhibited(t); reasserted.hold_epoch=i;
    assert(!emergency.ObserveEmergencyStop(reasserted,t,t).ok);
  }
  auto reasserted=inhibited(110.8); reasserted.hold_epoch=4;
  assert(emergency.ObserveEmergencyStop(reasserted,110.8,110.8).ok);
  reasserted=inhibited(110.9); reasserted.hold_epoch=3;
  assert(!emergency.ObserveEmergencyStop(reasserted,110.9,110.9).ok);

}
