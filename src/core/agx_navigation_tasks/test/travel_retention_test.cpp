#include <cassert>
#include <cmath>
#include <cstdlib>
#include <new>
#include <iomanip>
#include <sstream>
#include "travel_test_support.hpp"
using namespace travel_test;
agxcore::TubeMotionWindowToken Token(){return {42,{"motion_window",100,200}};}
TravelSample Retained(double s=100,double r=200,double height=.4){
  auto value=Sample(s,r);value.arm.stationary=value.lift.stationary=true;
  value.lift.position_m=height;
  auto& w=value.motion_window;w.token=Token();w.valid=w.fresh=true;
  w.arm_evidence={"arm",s,r};w.lift_evidence={"lift",s,r};
  for(int i=0;i<6;++i){double position=i==0?.5:i==1?-.5:0;w.arm[i]={position,position,0,w.arm_evidence,w.arm_evidence,w.arm_evidence};}
  w.lift={height,height,0,w.lift_evidence,w.lift_evidence,w.lift_evidence};return value;
}
void Ready(TravelPolicy& policy){
  const double h=policy.Profile().lift_position_m;
  assert(policy.BeginPreparation(Retained(100,200,h),100,200).ok);
  assert(policy.CommandsFinished(Token(),100,200).failure==agx_task_workflow::FailureKind::None);
  assert(!policy.Observe(Retained(100.1,200.1,h),100.1,200.1).ready);
  assert(policy.Observe(Retained(100.4,200.4,h),100.4,200.4).ready);
}
void Depart(TravelPolicy& policy){Ready(policy);assert(policy.BeginTravel(Retained(100.41,200.41,policy.Profile().lift_position_m),100.41,200.41).ready);}
// Real buffer callbacks exercise strict continuity independently of policy polls.
struct BufferedPolicy {
  static TravelProfile Config(){auto p=Profile();p.robot.observer.stopped_velocity_rad_s=1e-6;
    p.settle_s=.25;return p;} // Binary-exact synthetic dwell boundary.
  TravelProfile profile=Config();agxcore::TubeObservationBuffer buffer{profile.robot.observer};
  TravelPolicy policy{profile,TravelMode::Simulation};agxcore::TubeMotionWindowToken token;
  void Put(double source,double receipt,int moving_axis=-1){
    agxcore::TubeArmObservation arm;arm.valid=true;arm.evidence={"arm",source,receipt};
    arm.positions[0]=.5;arm.positions[1]=-.5;if(moving_axis==0)arm.velocities[0]=.0005;
    agxcore::TubeLiftObservation lift;lift.valid=true;lift.evidence={"lift",source,receipt};
    lift.position_m=.4;lift.velocity_mps=moving_axis==1?.001:0;lift.stationary=moving_axis!=1;
    buffer.Arm(arm);buffer.Lift(lift);
  }
  TravelSample Read(double source,double receipt){
    auto s=Sample(source,receipt);const auto observed=buffer.Snapshot(source,receipt);
    const auto evidence=[](const agxcore::TubeEvidence&e){return E(e.source_stamp_s,e.receipt_steady_s,e.source);};
    s.arm.evidence=evidence(observed.arm.evidence);s.arm.valid=observed.arm.valid&&observed.arm.fresh;
    s.arm.stationary=observed.arm.stationary;
    s.arm.stationary_source_since_s=observed.arm.stationary_source_since_s;
    s.arm.stationary_receipt_since_s=observed.arm.stationary_receipt_since_s;
    for(const auto& joint:profile.arm){const auto index=joint.name[5]-'1';
      s.arm.positions_rad[joint.name]=observed.arm.positions[index];s.arm.velocities_rad_s[joint.name]=observed.arm.velocities[index];}
    s.lift.evidence=evidence(observed.lift.evidence);s.lift.valid=observed.lift.valid&&observed.lift.fresh;
    s.lift.position_m=observed.lift.position_m;s.lift.velocity_mps=observed.lift.velocity_mps;s.lift.stationary=observed.lift.stationary;
    s.lift.stationary_source_since_s=observed.lift.stationary_source_since_s;
    s.lift.stationary_receipt_since_s=observed.lift.stationary_receipt_since_s;
    s.motion_window=observed.motion_window;return s;
  }
  BufferedPolicy(){Put(100,200);assert(policy.BeginPreparation(Read(100,200),100,200).ok);
    token=buffer.ArmMotionWindow([]{return 100.;},[]{return 200.;});
    assert(policy.CommandsFinished(token,100,200).failure==agx_task_workflow::FailureKind::None);
    Put(100.125,200.125);assert(!policy.Observe(Read(100.125,200.125),100.125,200.125).ready);}
};
void HiddenStrictInterruption(int axis,int phase){
  BufferedPolicy fixture;auto& p=fixture.policy;
  if(phase==0){
    fixture.Put(100.25,200.25,axis);fixture.Put(100.3125,200.3125);fixture.Put(100.375,200.375);
    const auto s=fixture.Read(100.375,200.375);assert(s.arm.stationary&&s.lift.stationary);
    const auto d=p.Observe(s,100.375,200.375);assert(!d.ready&&d.failure==agx_task_workflow::FailureKind::None);
    fixture.Put(100.624,200.624);assert(!p.Observe(fixture.Read(100.624,200.624),100.624,200.624).ready);
    fixture.Put(100.625,200.625);assert(p.Observe(fixture.Read(100.625,200.625),100.625,200.625).ready);
  }else{
    fixture.Put(100.375,200.375);assert(p.Observe(fixture.Read(100.375,200.375),100.375,200.375).ready);
    if(phase==2){fixture.Put(100.40625,200.40625);assert(p.BeginTravel(fixture.Read(100.40625,200.40625),100.40625,200.40625).ready);}
    fixture.Put(100.4375,200.4375,axis);fixture.Put(100.5,200.5);fixture.Put(100.5625,200.5625);
    const auto s=fixture.Read(100.5625,200.5625);assert(s.arm.stationary&&s.lift.stationary);
    const auto d=phase==1?p.BeginTravel(s,100.5625,200.5625):p.CheckTravel(s,100.5625,200.5625);
    assert(d.ready==(phase==2));
    if(phase==1){assert(d.failure==agx_task_workflow::FailureKind::Fault);
      assert(d.detail.find(axis?"departure: lift":"departure: arm")!=std::string::npos);
      assert(d.detail.find("source_since=100.5 receipt_since=200.5")!=std::string::npos);
      assert(d.detail.find("settled_source=100.125 settled_receipt=200.125")!=std::string::npos);
      fixture.Put(100.625,200.625);
      assert(!p.CheckTravel(fixture.Read(100.625,200.625),100.625,200.625).ready);}
  }
  const auto window=fixture.buffer.Snapshot(100.625,200.625).motion_window;
  assert(window.valid&&window.token.generation==fixture.token.generation);
  assert((axis?window.lift.max_abs_velocity:window.arm[0].max_abs_velocity)==(axis?.001:.0005));
}
double& Since(TravelSample& s,int axis,int receipt){
  if(axis)return receipt?s.lift.stationary_receipt_since_s:s.lift.stationary_source_since_s;
  return receipt?s.arm.stationary_receipt_since_s:s.arm.stationary_source_since_s;
}
void StrictIntervalClocks(){
  const auto config=BufferedPolicy::Config();
  for(int axis=0;axis<2;++axis)for(int receipt=0;receipt<2;++receipt){
    const double initial=receipt?200:100;
    // Both original domains independently restart settling, even if the other
    // interval still predates the accepted first measurement.
    TravelPolicy settling(config,TravelMode::Simulation);
    assert(settling.BeginPreparation(Retained(),100,200).ok);
    assert(settling.CommandsFinished(Token(),100,200).failure==agx_task_workflow::FailureKind::None);
    assert(!settling.Observe(Retained(100.125,200.125),100.125,200.125).ready);
    auto s=Retained(100.375,200.375);Since(s,axis,receipt)=initial+.25;
    assert(!settling.Observe(s,100.375,200.375).ready);
    s=Retained(100.625,200.625);Since(s,axis,receipt)=initial+.25;
    assert(settling.Observe(s,100.625,200.625).ready);
    // Equality with the pinned settled evidence preserves the exact boundary;
    // the next representable later start invalidates it in either domain.
    TravelPolicy waiting(config,TravelMode::Simulation);
    assert(waiting.BeginPreparation(Retained(),100,200).ok);
    assert(waiting.CommandsFinished(Token(),100,200).failure==agx_task_workflow::FailureKind::None);
    assert(!waiting.Observe(Retained(100.125,200.125),100.125,200.125).ready);
    s=Retained(100.375,200.375);Since(s,axis,receipt)=initial+.125;
    assert(waiting.Observe(s,100.375,200.375).ready);
    s=Retained(100.5,200.5);Since(s,axis,receipt)=std::nextafter(initial+.125,INFINITY);
    assert(waiting.BeginTravel(s,100.5,200.5).failure==agx_task_workflow::FailureKind::Fault);
    for(double invalid:{0.,-1.,double(NAN),double(INFINITY),initial+1.}){
      auto bad=Retained();Since(bad,axis,receipt)=invalid;
      TravelPolicy initial_policy(config,TravelMode::Simulation);
      assert(!initial_policy.BeginPreparation(bad,100,200).ok);
      TravelPolicy ready(config,TravelMode::Simulation);Ready(ready);
      bad=Retained(100.5,200.5);Since(bad,axis,receipt)=invalid;
      assert(ready.BeginTravel(bad,100.5,200.5).failure==agx_task_workflow::FailureKind::Fault);
      TravelPolicy traveling(config,TravelMode::Simulation);Depart(traveling);
      assert(traveling.CheckTravel(bad,100.5,200.5).ready); // Strict interval no longer required.
    }
  }
  for(int axis=0;axis<2;++axis)for(bool ready:{false,true}){
    BufferedPolicy fixture;
    if(ready){fixture.Put(100.375,200.375);
      assert(fixture.policy.Observe(fixture.Read(100.375,200.375),100.375,200.375).ready);}
    const double source=ready?100.375:100.125;
    fixture.Put(source,200.4375,axis);fixture.Put(source,200.5); // Paused source, advancing receipts.
    const auto s=fixture.Read(source,200.5625);
    assert(!s.motion_window.valid); // Contradictory same-stamp input conservatively faults first.
    const auto d=ready?fixture.policy.BeginTravel(s,source,200.5625):fixture.policy.Observe(s,source,200.5625);
    assert(d.failure==agx_task_workflow::FailureKind::Fault&&!d.ready); // Strict lift invalidity may reject before the window.
  }
}
// Test-only, one-shot allocation failure: no production injection seam.
static bool fail_next_allocation=false;
void* operator new(std::size_t size){
  if(fail_next_allocation){fail_next_allocation=false;throw std::bad_alloc();}
  if(void* p=std::malloc(size?size:1))return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept {std::free(p);}
void operator delete(void* p,std::size_t) noexcept {std::free(p);}
void Contains(const std::string& text,const std::string& part){assert(text.find(part)!=std::string::npos);}
void SameEvidence(const agxcore::TubeEvidence& a,const agxcore::TubeEvidence& b){
  assert(a.source==b.source&&a.source_stamp_s==b.source_stamp_s&&a.receipt_steady_s==b.receipt_steady_s);
}
void ConsumptionFreshnessDiagnostic(int axis,int clock,bool future){
  TravelPolicy policy(Profile(),TravelMode::Simulation);Depart(policy);
  if(!future){
    const auto captured=Retained(100.5,200.5);
    assert(policy.CheckTravel(captured,100.5,200.5).ready);
    // An intermediate policy poll is within the supervision interval; the
    // exact same original window evidence is still fresh at this poll.
    auto interim=Retained(101,201);interim.motion_window=captured.motion_window;
    assert(policy.CheckTravel(interim,101,201).ready);
  }
  const double source=future?100.5:101.625,receipt=future?200.5:201.625;
  auto s=Retained(source,receipt);auto& w=s.motion_window;
  // Outer custody/current observations are fresh; captured window evidence
  // alone has aged since its earlier healthy consumption.
  for(int stream=0;stream<2;++stream)if(axis==stream||axis==2){
    auto& e=stream?w.lift_evidence:w.arm_evidence;
    (clock?e.receipt_steady_s:e.source_stamp_s)=clock?(future?200.75:200.5):(future?100.75:100.5);
  }
  const auto d=policy.CheckTravel(s,source,receipt);
  assert(!d.ready&&d.failure==agx_task_workflow::FailureKind::Fault);
  Contains(d.detail,"consumption valid=1 fresh=1");
  Contains(d.detail,"arm_fresh="+std::to_string(axis==1)+" lift_fresh="+std::to_string(axis==0));
  Contains(d.detail,future?"source_now=100.5 steady_now=200.5 source_max_age=1 receipt_max_age=1":
      "source_now=101.625 steady_now=201.625 source_max_age=1 receipt_max_age=1");
  for(int stream=0;stream<2;++stream){const auto& e=stream?w.lift_evidence:w.arm_evidence;
    const std::string name=stream?"lift":"arm";std::ostringstream fields;
    fields<<std::setprecision(17)<<name<<"_source="<<e.source<<" "<<name<<"_stamp="<<e.source_stamp_s
          <<" "<<name<<"_receipt="<<e.receipt_steady_s;Contains(d.detail,fields.str());}
  assert(d.detail.find("observer freshness")==std::string::npos);
  assert(policy.CheckTravel(Retained(source+.125,receipt+.125),source+.125,receipt+.125).detail==d.detail);
}
void TokenValidityDiagnostic(int bad){
  TravelPolicy policy(Profile(),TravelMode::Simulation);Depart(policy);
  auto s=Retained(100.5,200.5);auto& w=s.motion_window;
  if(bad==0)w.valid=false;if(bad==1)w.fresh=false;
  if(bad==2)w.token.generation=0;if(bad==3)w.token.generation=43;
  if(bad==4)w.token.barrier.source="different";
  if(bad==5)w.token.barrier.source_stamp_s=100.125;
  if(bad==6)w.token.barrier.receipt_steady_s=200.125;
  const auto d=policy.CheckTravel(s,100.5,200.5);assert(d.failure==agx_task_workflow::FailureKind::Fault&&!d.ready);
  Contains(d.detail,"consumption valid="+std::to_string(w.valid)+" fresh="+std::to_string(w.fresh));
  Contains(d.detail,"arm_fresh=1 lift_fresh=1");
  std::ostringstream actual;actual<<std::setprecision(17)<<"actual_generation="<<w.token.generation
    <<" actual_barrier_source="<<w.token.barrier.source<<" actual_barrier_stamp="<<w.token.barrier.source_stamp_s
    <<" actual_barrier_receipt="<<w.token.barrier.receipt_steady_s;Contains(d.detail,actual.str());
  Contains(d.detail,"expected_generation=42 expected_barrier_source=motion_window expected_barrier_stamp=100 expected_barrier_receipt=200");
}
void ObserverContextAtConsumption(){
  BufferedPolicy f;f.Put(100.375,200.375);assert(f.policy.Observe(f.Read(100.375,200.375),100.375,200.375).ready);
  f.Put(100.40625,200.40625);assert(f.policy.BeginTravel(f.Read(100.40625,200.40625),100.40625,200.40625).ready);
  // Lift stays current while arm's original source time becomes stale at read.
  f.buffer.Lift([]{agxcore::TubeLiftObservation l;l.valid=l.stationary=true;l.position_m=.4;l.velocity_mps=0;
    l.evidence={"lift",100.90625,200.90625};return l;}());
  f.buffer.Lift([]{agxcore::TubeLiftObservation l;l.valid=l.stationary=true;l.position_m=.4;l.velocity_mps=0;
    l.evidence={"lift",101.40625,201.40625};return l;}());
  const auto observed=f.buffer.Snapshot(101.5,201.5).motion_window;
  assert(observed.valid&&!observed.fresh);
  // Keep policy's supervision interval intact with an independent healthy
  // sample before consuming this captured observer window.
  auto interim=Retained(101.0,201.0);interim.motion_window.token=f.token;
  interim.arm.evidence.source="arm";interim.lift.evidence.source="lift";
  assert(f.policy.CheckTravel(interim,101.0,201.0).ready);
  auto s=Retained(101.625,201.625);s.motion_window=observed;
  const auto d=f.policy.CheckTravel(s,101.625,201.625);
  assert(d.failure==agx_task_workflow::FailureKind::Fault&&!d.ready);
  Contains(d.detail,"observer freshness: first=arm");
  Contains(d.detail,"source_now=101.5 steady_now=201.5");
  Contains(d.detail,"source_now=101.625 steady_now=201.625");
  Contains(d.detail,"failure_source=arm failure_stamp=100.40625 failure_receipt=200.40625");
  SameEvidence(s.motion_window.arm_evidence,observed.arm_evidence);SameEvidence(s.motion_window.lift_evidence,observed.lift_evidence);
}
void LatchedAndBoundedDiagnostic(){
  for(bool large:{false,true}){TravelPolicy policy(Profile(),TravelMode::Simulation);Depart(policy);
    auto s=Retained(100.5,200.5);auto& w=s.motion_window;w.valid=w.fresh=false;
    w.failure_detail=large?std::string(10000,'d'):"original callback continuity failure";
    w.failure_evidence={large?std::string(10000,'f'):"first_arm",100.25,200.25};
    if(large){w.arm_evidence.source=std::string(10000,'a');w.lift_evidence.source=std::string(10000,'l');
      w.token.barrier.source=std::string(10000,'b');}
    const auto original=w;const auto d=policy.CheckTravel(s,100.5,200.5);
    assert(d.failure==agx_task_workflow::FailureKind::Fault&&!d.ready);assert(d.detail.size()<4000);
    if(large)Contains(d.detail,"...");else{
      Contains(d.detail,"original callback continuity failure");
      Contains(d.detail,"failure_source=first_arm failure_stamp=100.25 failure_receipt=200.25");}
    assert(w.failure_detail==original.failure_detail);SameEvidence(w.failure_evidence,original.failure_evidence);
    assert(policy.CheckTravel(Retained(100.625,200.625),100.625,200.625).detail==d.detail);
  }
}
void ConsumptionDiagnosticAllocationFailure(){
  TravelPolicy policy(Profile(),TravelMode::Simulation);Depart(policy);
  auto s=Retained(100.5,200.5);s.motion_window.fresh=false;
  fail_next_allocation=true;const auto d=policy.CheckTravel(s,100.5,200.5);
  const bool diagnostic_attempted=!fail_next_allocation;fail_next_allocation=false;
  assert(diagnostic_attempted&&d.failure==agx_task_workflow::FailureKind::Fault&&!d.ready&&d.detail=="window fault");
  assert(policy.CheckTravel(Retained(100.625,200.625),100.625,200.625).detail==d.detail);
}
void RetentionDiagnostics(){
  for(int axis=0;axis<3;++axis)for(int clock=0;clock<2;++clock)for(bool future:{false,true})
    ConsumptionFreshnessDiagnostic(axis,clock,future);
  for(int bad=0;bad<7;++bad)TokenValidityDiagnostic(bad);
  ObserverContextAtConsumption();LatchedAndBoundedDiagnostic();ConsumptionDiagnosticAllocationFailure();
}
int main(int argc,char** argv){
  if(argc==5){ConsumptionFreshnessDiagnostic(std::stoi(argv[1]),std::stoi(argv[2]),std::stoi(argv[3])!=0);return 0;}
  if(argc==4){TokenValidityDiagnostic(std::stoi(argv[1]));return 0;}
  if(argc==2){const std::string selected=argv[1];
    if(selected=="observer")ObserverContextAtConsumption();
    else if(selected=="bounded")LatchedAndBoundedDiagnostic();
    else if(selected=="allocation")ConsumptionDiagnosticAllocationFailure();
    else assert(false);
    return 0;}
  RetentionDiagnostics();
  if(argc==3){HiddenStrictInterruption(std::stoi(argv[1]),std::stoi(argv[2]));return 0;}
  auto p=Profile();
  for(int axis=0;axis<2;++axis){
    TravelPolicy policy(p,TravelMode::Simulation);auto s=Retained();
    (axis?s.lift.stationary:s.arm.stationary)=false;
    assert(!policy.BeginPreparation(s,100,200).ok);
    TravelPolicy prepare(p,TravelMode::Simulation);assert(prepare.BeginPreparation(Retained(),100,200).ok);
    prepare.CommandsFinished(Token(),100,200);
    s=Retained(100.1,200.1);(axis?s.lift.stationary:s.arm.stationary)=false;
    assert(!prepare.Observe(s,100.1,200.1).ready);
    s=Retained(100.4,200.4);(axis?s.lift.stationary:s.arm.stationary)=false;
    assert(!prepare.Observe(s,100.4,200.4).ready);
    TravelPolicy waiting(p,TravelMode::Simulation);Ready(waiting);
    s=Retained(100.5,200.5);(axis?s.lift.stationary:s.arm.stationary)=false;
    assert(!waiting.CheckTravel(s,100.5,200.5).ready); // Permit/release waits remain strict.
  }
  for(int sign:{-1,1})for(double magnitude:{.0005,.001,.00100001})for(int axis=0;axis<2;++axis){
    TravelPolicy policy(p,TravelMode::Simulation);Depart(policy);auto s=Retained(100.5,200.5);
    if(axis){s.lift.velocity_mps=sign*magnitude;s.lift.stationary=false;s.motion_window.lift.max_abs_velocity=magnitude;}
    else{s.arm.velocities_rad_s["joint1"]=sign*magnitude;s.arm.stationary=false;s.motion_window.arm[0].max_abs_velocity=magnitude;}
    const auto d=policy.CheckTravel(s,100.5,200.5);assert(d.ready==(magnitude<=.001));
  }
  // A peak between polls is independently rejected even when current state is zero.
  for(int axis=0;axis<2;++axis){TravelPolicy policy(p,TravelMode::Simulation);Depart(policy);
    auto s=Retained(100.5,200.5);auto& range=axis?s.motion_window.lift:s.motion_window.arm[0];
    range.max_abs_velocity=.0011;range.peak_evidence={axis?"lift":"arm",100.45,200.45};
    auto d=policy.CheckTravel(s,100.5,200.5);assert(!d.ready&&d.detail.find("peak")!=std::string::npos);
    assert(!policy.CheckTravel(Retained(100.6,200.6),100.6,200.6).ready);
  }
  // Position range is the existing observer limit, not the wider target tolerance.
  auto wide=p;wide.lift_tolerance_m=.01;
  for(int sign:{-1,1})for(int axis=0;axis<2;++axis)for(bool over:{false,true}){
    TravelPolicy policy(wide,TravelMode::Simulation);Depart(policy);auto s=Retained(100.5,200.5);
    auto& range=axis?s.motion_window.lift:s.motion_window.arm[0];
    const double limit=axis?.001:.002;
    if(sign>0)range.maximum=range.minimum+limit;
    else range.minimum=range.maximum-limit;
    if(over)range.maximum=std::nextafter(range.minimum+limit,INFINITY);
    auto d=policy.CheckTravel(s,100.5,200.5);assert(d.ready==!over);
  }
  // Quantized height at a realistic absolute position: exactly one tick passes,
  // two ticks fail. No epsilon physical margin is introduced.
  for(double height:{.010,.400,.599})for(int ticks:{1,2}){
    auto shifted=wide;shifted.lift_position_m=height;shifted.starts[0].lift_position_m=height;
    TravelPolicy policy(shifted,TravelMode::Simulation);Depart(policy);auto s=Retained(100.5,200.5,height);
    s.motion_window.lift.minimum=height;s.motion_window.lift.maximum=height+ticks*.001;
    assert(policy.CheckTravel(s,100.5,200.5).ready==(ticks==1));
  }
  for(int bad=0;bad<8;++bad){TravelPolicy policy(p,TravelMode::Simulation);Depart(policy);auto s=Retained(100.5,200.5);
    if(bad==0)s.motion_window.token.generation++;
    if(bad==1)s.motion_window.token.barrier.source_stamp_s+=.01;
    if(bad==2)s.motion_window.token.barrier.receipt_steady_s+=.01;
    if(bad==3)s.motion_window.valid=false;
    if(bad==4)s.motion_window.fresh=false;
    if(bad==5)s.motion_window.arm_evidence.source_stamp_s=98;
    if(bad==6)s.motion_window.lift.max_abs_velocity=NAN;
    if(bad==7)s.motion_window.arm[0].minimum=-1;
    assert(!policy.CheckTravel(s,100.5,200.5).ready);
  }
  TravelPolicy early(p,TravelMode::Simulation);assert(!early.BeginTravel(Retained(),100,200).ready);
  TravelPolicy twice(p,TravelMode::Simulation);Depart(twice);assert(!twice.BeginTravel(Retained(100.5,200.5),100.5,200.5).ready);
  TravelPolicy not_stopped(p,TravelMode::Simulation);Ready(not_stopped);auto s=Retained(100.5,200.5);s.lift.stationary=false;
  assert(!not_stopped.BeginTravel(s,100.5,200.5).ready);
  TravelPolicy settling(p,TravelMode::Simulation);settling.BeginPreparation(Retained(),100,200);settling.CommandsFinished(Token(),100,200);
  s=Retained(100.1,200.1);s.motion_window.lift.maximum=.402;
  assert(settling.Observe(s,100.1,200.1).failure==agx_task_workflow::FailureKind::Fault);
  assert(!settling.Observe(Retained(100.4,200.4),100.4,200.4).ready);
  for(int invalid=0;invalid<6;++invalid){
    TravelPolicy policy(p,TravelMode::Simulation);policy.BeginPreparation(Retained(),100,200);auto t=Token();
    if(invalid==0)t.generation=0;
    if(invalid==1)t.barrier.source_stamp_s=NAN;
    if(invalid==2)t.barrier.source_stamp_s=100.1;
    if(invalid==3)t.barrier.receipt_steady_s=199.9;
    if(invalid==4)t.barrier.source="other";
    if(invalid==5)t.barrier.receipt_steady_s=0;
    assert(policy.CommandsFinished(t,100.05,200.05).failure==agx_task_workflow::FailureKind::Fault);
  }
  // Binding later cannot move the original checkpoint or lose intervening input.
  TravelPolicy original(p,TravelMode::Simulation);original.BeginPreparation(Retained(),100,200);
  assert(original.CommandsFinished(Token(),100.05,200.05).failure==agx_task_workflow::FailureKind::None);
  assert(!original.Observe(Retained(100.02,200.02),100.1,200.1).ready);
  assert(original.Observe(Retained(100.3,200.3),100.3,200.3).ready);
  TravelPolicy queued(p,TravelMode::Simulation);queued.BeginPreparation(Retained(),100,200);
  auto t=Token();t.barrier.source_stamp_s=100.05;t.barrier.receipt_steady_s=200.05;
  assert(queued.CommandsFinished(t,100.05,200.05).failure==agx_task_workflow::FailureKind::None);
  s=Retained(100.1,200.1);s.motion_window.token=t;s.motion_window.arm_evidence={"arm",100.02,200.06};
  assert(!queued.Observe(s,100.1,200.1).ready); // Queued source evidence is pre-barrier.
  for(int axis=0;axis<2;++axis)for(int phase=0;phase<3;++phase)HiddenStrictInterruption(axis,phase);
  StrictIntervalClocks();
}
