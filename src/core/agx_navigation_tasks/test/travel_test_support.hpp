#pragma once
#include "agx_navigation_tasks/travel_policy.hpp"
namespace travel_test {
using namespace agx_navigation_tasks;
// Synthetic contract-test values, not a qualified robot profile.
inline TravelProfile Profile() {
  TravelProfile p;
  p.profile_id = "synthetic"; p.config_id = "test-config";
  p.robot={"test_arm","/joints","/gripper","/lift_command","/lift_state",
      {1,1,1,1000,0,5,.001,.2,.002,.001}};
  p.simulation = {true, "unit-test-only"};
  p.arm = {{"joint1", .5, .01, .001}, {"joint2", -.5, .01, .001}};
  p.lift_position_m = .4; p.lift_tolerance_m = .001; p.lift_stopped_velocity_mps = .001;
  p.arm_motion_velocity_rad_s = .1; p.arm_motion_acceleration_rad_s2 = .1;
  p.lift_motion_velocity_mps = .01;
  p.container_profile_id = "test-tube";
  p.held_max_mass_kg = .1; p.held_max_radius_m = .01; p.held_max_length_m = .1;
  p.held_attachment_link = "test_gripper_base";
  p.held_center_xyz_m = {0,0,.1}; p.held_orientation_xyzw = {0,0,0,1};
  p.held_touch_links = {"test_left_finger","test_right_finger"};
  p.base = {"test-envelope", {{-.5,-.5},{.5,-.5},{.5,.5},{-.5,.5}}, .1,.1,.1,.1};
  p.source_max_age_s = 1; p.receipt_max_age_s = 1; p.settle_s = .2; p.preparation_timeout_s = 5;
  p.starts = {{"default", "measured-default", {{"joint1",.5},{"joint2",-.5}}, .4, true, true,
               {{"joint1",.01},{"joint2",.01}},.001},
              {"home", "home-to-default", {{"joint1",0},{"joint2",0}}, 0, true, false,
               {{"joint1",.01},{"joint2",.01}},.001}};
  return p;
}
inline agx_robot_task_runtime::Evidence E(double source, double steady, const std::string& name = "sensor") {
  return {name,source,steady,"run","task","operation"};
}
inline agxcore::TubeMotionWindowToken WindowToken(double source=100,double steady=200) {
  return {42,{"motion_window",source,steady}};
}
inline TravelSample Sample(double source = 100, double steady = 200, bool held = false) {
  TravelSample s;
  auto e = E(source,steady);
  s.robot.gripper = {held ? agx_robot_task_runtime::GripperOccupancy::Holding : agx_robot_task_runtime::GripperOccupancy::Empty, {}, e, 1, true};
  s.robot.transport = {held ? agx_robot_task_runtime::TransportLoad::SingleHeld : agx_robot_task_runtime::TransportLoad::Empty,
      agx_robot_task_runtime::TravelEligibility::SafeClear, {}, e,e,false,false,true,1};
  s.robot.revision = 1;
  if (held) {
    agx_robot_task_runtime::ObjectRef object{"tube",agx_robot_task_runtime::ObjectIdentity::RunLocal,"test-tube"};
    s.robot.gripper.object = object; s.robot.transport.object = object;
    agx_robot_task_runtime::ObjectLocation location;
    location.object = object; location.last_confirmed_holder = agx_robot_task_runtime::HolderRef{};
    location.last_confirmed_evidence = e; location.current_evidence = e;
    location.certainty = agx_robot_task_runtime::LocationCertainty::Confirmed; location.fresh = true;
    s.robot.objects[object.id] = location;
  }
  s.arm = {e,true,{{"joint1",.5},{"joint2",-.5}},{{"joint1",0},{"joint2",0}}};
  s.lift = {e,true,.4,0}; s.producers = {e,true,true,true};
  s.hold = {e,"session",1,true,true};
  s.arm.stationary=s.lift.stationary=true;
  s.arm.stationary_source_since_s=s.lift.stationary_source_since_s=100;
  s.arm.stationary_receipt_since_s=s.lift.stationary_receipt_since_s=200;
  auto& w=s.motion_window;w.token=WindowToken();w.valid=w.fresh=true;
  w.arm_evidence={"arm",source,steady};w.lift_evidence={"lift",source,steady};
  for(int i=0;i<6;++i) {double position=i==0?.5:i==1?-.5:0;
    w.arm[i]={position,position,0,w.arm_evidence,w.arm_evidence,w.arm_evidence};}
  w.lift={.4,.4,0,w.lift_evidence,w.lift_evidence,w.lift_evidence};
  return s;
}
inline void Settle(TravelPolicy& policy, bool held = false) {
  assert(policy.BeginPreparation(Sample(100,200,held),100,200).ok);
  assert(policy.CommandsFinished(WindowToken(),100,200).failure == agx_task_workflow::FailureKind::None);
  assert(!policy.Observe(Sample(100.1,200.1,held),100.1,200.1).ready);
  assert(policy.Observe(Sample(100.4,200.4,held),100.4,200.4).ready);
}
}
