#include <cassert>
#include <limits>
#include "travel_test_support.hpp"
using namespace travel_test;
int main() {
  auto p = Profile();
  auto wide = p; wide.starts[0].arm_tolerances_rad["joint2"] = .5;
  auto yaw_start = Sample(100,200,true); yaw_start.arm.positions_rad["joint2"] = -.1;
  assert(EvaluateStartClearance(wide,TravelMode::Simulation,yaw_start,100,200).ok);
  // Readiness is measured after both command completions, even when already default.
  TravelPolicy empty(p, TravelMode::Simulation);
  assert(empty.BeginPreparation(Sample(),100,200).ok);
  assert(!empty.Observe(Sample(),100,200).ready);
  empty.CommandsFinished(WindowToken(),100,200);
  assert(!empty.Observe(Sample(),100,200).ready);
  assert(!empty.Observe(Sample(100.1,200.1),100.1,200.1).ready);
  assert(!empty.Observe(Sample(100.1,200.1),100.3,200.3).ready); // Reads cannot extend dwell.
  assert(empty.Observe(Sample(100.4,200.4),100.4,200.4).ready);
  assert(empty.CheckTravel(Sample(100.5,200.5),100.5,200.5).ready);
  assert(!empty.CheckTravel(Sample(100.5,200.5),102,202).ready);
  assert(!empty.CheckTravel(Sample(102,202),102,202).ready); // Fault latches.

  TravelPolicy held(p, TravelMode::Simulation); Settle(held,true);
  auto lost = Sample(100.5,200.5,true); lost.robot.gripper.occupancy = agx_robot_task_runtime::GripperOccupancy::Unknown;
  assert(!held.CheckTravel(lost,100.5,200.5).ready);

  // A previous stale clearance may be replaced only by a named measured start.
  auto home = Sample(); home.arm.positions_rad = {{"joint1",0},{"joint2",0}}; home.lift.position_m = 0;
  home.robot.transport.clearance_evidence = E(90,190); home.robot.transport.ready = false;
  auto start = EvaluateStartClearance(p,TravelMode::Simulation,home,100,200);
  assert(start.ok && start.start_pose_id == "home" && start.path_id == "home-to-default");
  home.arm.positions_rad["joint1"] = .2;
  assert(!EvaluateStartClearance(p,TravelMode::Simulation,home,100,200).ok);
  home = Sample(100,200,true); home.arm.positions_rad = {{"joint1",0},{"joint2",0}}; home.lift.position_m = 0;
  assert(!EvaluateStartClearance(p,TravelMode::Simulation,home,100,200).ok);
  assert(!EvaluateStartClearance(p,TravelMode::Hardware,Sample(),100,200).ok);
  TravelPolicy active_producer(p,TravelMode::Simulation);
  auto active=Sample(); active.producers.arm_stopped=false;
  assert(!active_producer.BeginPreparation(active,100,200).ok);

  // Preparation consumes a start certificate while the same held custody is supervised.
  TravelPolicy prep(p,TravelMode::Simulation); assert(prep.BeginPreparation(Sample(),100,200).ok);
  auto during = Sample(100.1,200.1); during.arm.positions_rad["joint1"] = .2;
  during.robot.transport.eligibility = agx_robot_task_runtime::TravelEligibility::Unknown; during.robot.transport.ready = false;
  during.robot.transport.revision = 2;
  assert(prep.CheckPreparation(during,100.1,200.1).failure == agx_task_workflow::FailureKind::None);
  prep.CommandsFinished(WindowToken(100.2,200.2),100.2,200.2);
  auto finish = Sample(100.3,200.3); finish.motion_window.token=WindowToken(100.2,200.2); finish.robot.transport.revision = 3;
  assert(!prep.Observe(finish,100.3,200.3).ready);
  finish = Sample(100.6,200.6); finish.motion_window.token=WindowToken(100.2,200.2); finish.robot.transport.revision = 4;
  const auto ready = prep.Observe(finish,100.6,200.6);
  assert(ready.ready && ready.state_revision == 4);
  finish = Sample(100.7,200.7); finish.motion_window.token=WindowToken(100.2,200.2); finish.robot.transport.revision = 5;
  assert(!prep.CheckTravel(finish,100.7,200.7).ready);

  for (int fault = 0; fault < 11; ++fault) {
    TravelPolicy policy(p,TravelMode::Simulation); assert(policy.BeginPreparation(Sample(),100,200).ok);
    auto sample = Sample(100.1,200.1);
    if (fault == 0) sample.robot.transport.pending = true;
    if (fault == 1) sample.robot.transport.uncertain = true;
    if (fault == 2) sample.robot.transport.load = agx_robot_task_runtime::TransportLoad::Unsupported;
    if (fault == 3) sample.hold.hold_epoch++;
    if (fault == 4) sample.hold.held_stopped = false;
    if (fault == 5) sample.hold.command_guard_ok = false;
    if (fault == 6) sample.robot.gripper.evidence->source_stamp_s = 101;
    if (fault == 7) sample.robot.gripper.evidence->receipt_steady_s = 201;
    if (fault == 8) sample.robot.gripper.evidence->source_stamp_s = std::numeric_limits<double>::quiet_NaN();
    if (fault == 9) sample.robot.gripper.evidence->source_stamp_s = 99.99; // Out-of-order, still fresh.
    if (fault == 10) sample.robot.halted = true;
    assert(policy.CheckPreparation(sample,100.1,200.1).failure != agx_task_workflow::FailureKind::None);
  }
  TravelPolicy timeout(p,TravelMode::Simulation); timeout.BeginPreparation(Sample(),100,200);
  assert(timeout.CommandsFinished(WindowToken(),100,206).failure == agx_task_workflow::FailureKind::Deadline); // Paused ROS time.
  TravelPolicy cancelled(p,TravelMode::Simulation); cancelled.BeginPreparation(Sample(),100,200);
  cancelled.Abort(agx_task_workflow::FailureKind::Cancelled);
  assert(cancelled.Observe(Sample(100.1,200.1),100.1,200.1).failure == agx_task_workflow::FailureKind::Cancelled);
  TravelPolicy gap(p,TravelMode::Simulation); gap.BeginPreparation(Sample(),100,200); gap.CommandsFinished(WindowToken(),100,200);
  assert(!gap.Observe(Sample(100.1,200.1),100.1,200.1).ready);
  assert(!gap.Observe(Sample(102,202),102,202).ready); // A missed observation interval is not settled evidence.
  auto invalid_eligibility=Sample(); invalid_eligibility.robot.transport.eligibility=static_cast<agx_robot_task_runtime::TravelEligibility>(99);
  assert(!EvaluateStartClearance(p,TravelMode::Simulation,invalid_eligibility,100,200).ok);
  // Known held object identity cannot change during the qualified path.
  TravelPolicy switched(p,TravelMode::Simulation); switched.BeginPreparation(Sample(100,200,true),100,200);
  auto other=Sample(100.1,200.1,true);
  other.robot.gripper.object->id="other"; other.robot.transport.object=other.robot.gripper.object;
  auto location=other.robot.objects.at("tube"); other.robot.objects.clear(); location.object=*other.robot.gripper.object;
  other.robot.objects["other"]=location;
  assert(switched.CheckPreparation(other,100.1,200.1).failure == agx_task_workflow::FailureKind::Fault);
  TravelPolicy old_clearance(p,TravelMode::Simulation); old_clearance.BeginPreparation(Sample(),100,200);
  old_clearance.CommandsFinished(WindowToken(),100,200);
  assert(!old_clearance.Observe(Sample(100.2,200.2),100.2,200.2).ready);
  auto replay=Sample(100.5,200.5); replay.robot.transport.clearance_evidence=E(100.1,200.1);
  assert(old_clearance.Observe(replay,100.5,200.5).failure == agx_task_workflow::FailureKind::Fault);

  // Posture, stopped producers, clearance and both timestamp domains remain prerequisites.
  for (int fault = 0; fault < 10; ++fault) {
    TravelPolicy policy(p,TravelMode::Simulation); Settle(policy);
    auto sample = Sample(100.5,200.5);
    if (fault == 0) sample.arm.positions_rad["joint1"] += .1;
    if (fault == 1) sample.arm.velocities_rad_s["joint1"] = .1;
    if (fault == 2) sample.lift.position_m += .1;
    if (fault == 3) sample.lift.velocity_mps = .1;
    if (fault == 4) sample.producers.arm_stopped = false;
    if (fault == 5) sample.robot.transport.clearance_evidence = E(90,190);
    if (fault == 6) sample.arm.evidence = E(100.3,200.3); // Fresh but older than previous.
    if (fault == 7) sample.arm.positions_rad["joint1"] = std::numeric_limits<double>::infinity();
    if (fault == 8) sample.robot.transport.eligibility = agx_robot_task_runtime::TravelEligibility::CapturedInFixture;
    if (fault == 9) sample.robot.transport.clearance_evidence.reset();
    const auto rejected=policy.CheckTravel(sample,100.5,200.5);
    assert(!rejected.ready);
    if(fault==0 || fault==1) assert(rejected.detail.find("arm_ready=false")!=std::string::npos);
    if(fault==2 || fault==3) assert(rejected.detail.find("lift_ready=false")!=std::string::npos);
    if(fault==4) assert(rejected.detail.find("producers_stopped=false")!=std::string::npos);
    if(fault==9) assert(rejected.detail.find("clearance_verified=false")!=std::string::npos);
  }
}
