#include <agx_workbench_navigation/navigation_progress_guard.hpp>
#include <agx_workbench_navigation/command_source_gate.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <string>
using namespace agx_workbench_navigation;
NavigationProgressPose Pose(double t,double x=0,double yaw=0,std::uint64_t revision=0) {
  return {revision?revision:static_cast<std::uint64_t>(t*1000),
      static_cast<std::uint64_t>(t*1e9),t,x,0,yaw};
}
int main(int argc,char** argv) {
  assert(argc==2);const std::string scenario=argv[1];
  NavigationProgressGuard guard(45,.2,.1);
  if(scenario=="unbound") {assert(!guard.Check(100,200));return 0;}
  if(scenario=="config") {
    for(double value:{0.,-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
      for(int field=0;field<3;++field) {bool rejected=false;
        try {NavigationProgressGuard bad(field==0?value:45,field==1?value:.2,field==2?value:.1);}
        catch(const std::invalid_argument&) {rejected=true;}assert(rejected);
      }
    }
    bool rejected=false;try {NavigationProgressGuard bad(45,.2,std::acos(-1.));}
    catch(const std::invalid_argument&) {rejected=true;}assert(rejected);return 0;
  }
  guard.Begin(100,200);
  assert(guard.Observe(Pose(100),100,200));
  if(scenario=="stall"||scenario=="late_progress") {
    assert(guard.Observe(Pose(144.999),144.999,244.999));
    if(scenario=="late_progress")assert(!guard.Observe(Pose(145,1),145,245));
    else assert(!guard.Check(145,245));
    assert(guard.faulted());const auto detail=guard.detail();
    assert(detail.find("no_progress")!=std::string::npos && detail.find("45")!=std::string::npos);
    assert(!guard.Observe(Pose(146,5,1),146,246));assert(guard.detail()==detail);return 0;
  }
  if(scenario=="rotation"||scenario=="mirrored_rotation"||scenario=="translation"||scenario=="wrap") {
    const double sign=scenario=="mirrored_rotation"?-1:1;
    if(scenario=="wrap") {guard.Begin(100,200);assert(guard.Observe(Pose(100,0,3.13),100,200));}
    for(int i=1;i<=120;++i) {
      const double yaw=scenario=="wrap"?std::atan2(std::sin(3.13+i*.05),std::cos(3.13+i*.05)):
          (scenario=="translation"?0:sign*i*.05);
      assert(guard.Observe(Pose(100+i,scenario=="translation"?i*.01:0,yaw),100+i,200+i));
    }
    assert(!guard.faulted());return 0;
  }
  if(scenario=="translation_boundary"||scenario=="rotation_boundary") {
    bool rotation=scenario=="rotation_boundary";
    assert(guard.Observe(Pose(140,rotation?0:.2,rotation?.1:0),140,240));
    assert(guard.Check(184.999,284.999));assert(!guard.Check(185,285));return 0;
  }
  if(scenario=="below_boundary") {
    assert(guard.Observe(Pose(140,std::nextafter(.2,0.),std::nextafter(.1,0.)),140,240));
    assert(!guard.Check(145,245));return 0;
  }
  if(scenario=="duplicate"||scenario=="jitter") {
    for(int i=1;i<45;++i) {
      auto pose=scenario=="duplicate"?Pose(100):Pose(100+i,(i%2?1:-1)*.01,(i%2?1:-1)*.01);
      assert(guard.Observe(pose,100+i,200+i));
    }
    assert(!guard.Check(145,245));return 0;
  }
  if(scenario=="source_regression"||scenario=="receipt_regression"||scenario=="bad_clock") {
    assert(guard.Check(110,210));
    assert(!guard.Check(scenario=="source_regression"?109:scenario=="bad_clock"?
        std::numeric_limits<double>::quiet_NaN():111,scenario=="receipt_regression"?209:211));
    assert(guard.faulted());assert(!guard.Check(112,212));return 0;
  }
  if(scenario=="paused_source") {assert(guard.Check(100,10000));assert(!guard.faulted());return 0;}
  if(scenario=="future_sample") {
    guard.Begin(100,200);
    assert(!guard.Observe(Pose(100.01,1),100,200)); // no first command from future pose
    assert(guard.Observe(Pose(100.01),100.01,200.01));
    for(int i=1;i<45;++i)assert(guard.Observe(Pose(200,10,1),100+i,200+i));
    assert(!guard.Check(145,245)); // no future-time credit or receipt renewal
    return 0;
  }
  if(scenario=="original_stamp") {
    assert(guard.Observe(Pose(110,.2),120,220));
    assert(guard.Check(154.999,254.999));assert(!guard.Check(155,255));return 0;
  }
  if(scenario=="bad_pose"||scenario=="regressed_sample"||scenario=="conflicting_repeat"||scenario=="nonadvancing_sample") {
    assert(guard.Observe(Pose(110),110,210));auto pose=Pose(111);
    if(scenario=="bad_pose")pose.x=std::numeric_limits<double>::infinity();
    if(scenario=="regressed_sample")pose=Pose(109);
    if(scenario=="conflicting_repeat")pose=Pose(110,1);
    if(scenario=="nonadvancing_sample")pose=Pose(110,0,0,111000);
    assert(!guard.Observe(pose,111,211));assert(guard.faulted());
    assert(!guard.Observe(Pose(112,1),112,212));return 0;
  }
  if(scenario=="gate_lifetime") {
    CommandSourceGate gate;CommandBinding a{"s","g","a",{"map",100000000000ULL,{0,0,0},{0,0,0,1}}};
    int resets=0;auto reset=[&]{++resets;guard.Begin(100,200);};
    assert(gate.Bind(a,reset));assert(guard.Observe(Pose(100),100,200));
    for(int i=1;i<45;++i) {
      assert(gate.Bind(a,reset));assert(gate.AcceptPlan(a.endpoint,[]{}));
      assert(guard.Observe(Pose(100+i),100+i,200+i));
    }
    assert(resets==1);assert(!guard.Check(145,245));
    assert(gate.Bind(a,reset));assert(gate.AcceptPlan(a.endpoint,[]{}));
    gate.InvalidatePlan([]{});assert(gate.AcceptPlan(a.endpoint,[]{}));
    assert(resets==1&&!guard.Check(146,246));
    auto b=a;b.grant_id="g2";b.source_goal_id="b";b.endpoint.stamp_ns=147000000000ULL;
    assert(gate.Bind(b,[&]{++resets;guard.Begin(147,247);}));
    assert(resets==2&&guard.Observe(Pose(147),147,247));return 0;
  }
  assert(false&&"unknown scenario");
}
