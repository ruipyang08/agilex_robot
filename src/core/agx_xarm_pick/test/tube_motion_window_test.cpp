#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <new>
#include <iomanip>
#include <sstream>
#include "agx_xarm_pick/tube_live_observer.hpp"
using namespace agxcore;
TubeObserverConfig Config(){return {.5,.5,.5,1000,0,5,1e-6,.2,.002,.001};}
TubeArmObservation Arm(double s=10,double r=20){TubeArmObservation a;a.valid=true;a.evidence={"arm",s,r};return a;}
TubeLiftObservation Lift(double s=10,double r=20){TubeLiftObservation l;l.valid=true;l.stationary=true;l.position_m=.4;l.velocity_mps=0;l.evidence={"lift",s,r};return l;}
void Seed(TubeObservationBuffer& b){b.Arm(Arm());b.Lift(Lift());}
TubeMotionWindowToken Start(TubeObservationBuffer& b,double s=10.05,double r=20.05){return b.ArmMotionWindow([=]{return s;},[=]{return r;});}
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
void SnapshotFreshnessDiagnostic(int axis,int clock,bool future){
  TubeObservationBuffer b(Config());Seed(b);const auto token=Start(b);
  // axis 2 rejects both; other cases leave exactly one independent stream fresh.
  if((axis==0||axis==2)==future)b.Arm(Arm(10.25,20.25));
  if((axis==1||axis==2)==future)b.Lift(Lift(10.25,20.25));
  const auto before=b.Snapshot(10.375,20.375).motion_window;
  const double source=clock==0?(future?10.125:10.625):10.375;
  const double receipt=clock==1?(future?20.125:20.625):20.375;
  const auto out=b.Snapshot(source,receipt);const auto& w=out.motion_window;
  assert(w.valid&&!w.fresh&&w.token.generation==token.generation);
  SameEvidence(w.token.barrier,token.barrier);
  SameEvidence(w.arm_evidence,before.arm_evidence);SameEvidence(w.lift_evidence,before.lift_evidence);
  assert(w.arm[0].maximum==before.arm[0].maximum&&w.lift.maximum==before.lift.maximum);
  Contains(w.failure_detail,"observer freshness: first="+std::string(axis==1?"lift":"arm"));
  Contains(w.failure_detail,"arm_fresh="+std::to_string(axis==1));
  Contains(w.failure_detail,"lift_fresh="+std::to_string(axis==0));
  std::ostringstream clocks;clocks<<std::setprecision(17)<<"source_now="<<source<<" steady_now="<<receipt;
  Contains(w.failure_detail,clocks.str());Contains(w.failure_detail,"arm_max_age=0.5 lift_max_age=0.5");
  for(int stream=0;stream<2;++stream){const auto& e=stream?w.lift_evidence:w.arm_evidence;
    const std::string name=stream?"lift":"arm";std::ostringstream fields;
    fields<<std::setprecision(17)<<name<<"_source="<<e.source<<" "<<name<<"_stamp="<<e.source_stamp_s
          <<" "<<name<<"_receipt="<<e.receipt_steady_s;
    Contains(w.failure_detail,fields.str());}
  SameEvidence(w.failure_evidence,axis==1?w.lift_evidence:w.arm_evidence);
  // Both monotonic callback streams recover within their original allowed gap.
  b.Arm(Arm(10.5,20.5));b.Lift(Lift(10.5,20.5));
  const auto recovered=b.Snapshot(10.625,20.625).motion_window;
  assert(recovered.valid&&recovered.fresh&&recovered.failure_detail.empty());
  SameEvidence(recovered.failure_evidence,{});assert(recovered.token.generation==token.generation);
}
void SnapshotFailurePreservation(){
  TubeObservationBuffer b(Config());Seed(b);Start(b);
  const auto healthy=b.Snapshot(10.125,20.125).motion_window;
  assert(healthy.valid&&healthy.fresh&&healthy.failure_detail.empty());SameEvidence(healthy.failure_evidence,{});
  auto bad=Arm(10.25,20.25);bad.evidence.source="original_bad_arm";b.Arm(bad);
  const auto first=b.Snapshot(10.25,20.25).motion_window;
  assert(!first.valid&&!first.failure_detail.empty());SameEvidence(first.failure_evidence,bad.evidence);
  auto second=Lift(10.375,20.375);second.evidence.source="later_bad_lift";b.Lift(second);
  b.Arm(Arm(10.5,20.5));b.Lift(Lift(10.5,20.5));
  for(double clock:{10.5,11.5}){const auto w=b.Snapshot(clock,clock+10).motion_window;
    assert(!w.valid&&!w.fresh&&w.failure_detail==first.failure_detail);
    SameEvidence(w.failure_evidence,first.failure_evidence);}
}
void SnapshotBoundedDiagnostic(){
  TubeObservationBuffer b(Config());auto a=Arm();auto l=Lift();
  a.evidence.source=std::string(10000,'a');l.evidence.source=std::string(10000,'l');b.Arm(a);b.Lift(l);Start(b);
  const auto w=b.Snapshot(10.625,20.625).motion_window;
  assert(w.valid&&!w.fresh);Contains(w.failure_detail,"observer freshness");
  assert(w.failure_detail.size()<1200);Contains(w.failure_detail,"...");
  SameEvidence(w.arm_evidence,a.evidence);SameEvidence(w.lift_evidence,l.evidence);
  SameEvidence(w.failure_evidence,a.evidence);
}
void SnapshotDiagnosticAllocationFailure(){
  TubeObservationBuffer b(Config());Seed(b);Start(b);
  fail_next_allocation=true;
  const auto healthy=b.Snapshot(10.125,20.125).motion_window;
  const bool healthy_allocated=!fail_next_allocation;fail_next_allocation=false;
  assert(!healthy_allocated&&healthy.fresh&&healthy.failure_detail.empty());
  fail_next_allocation=true;
  const auto stale=b.Snapshot(10.625,20.625).motion_window;
  const bool diagnostic_attempted=!fail_next_allocation;fail_next_allocation=false;
  assert(diagnostic_attempted&&stale.valid&&!stale.fresh&&stale.failure_detail.empty());
  SameEvidence(stale.failure_evidence,{});
  assert(b.Snapshot(10.125,20.125).motion_window.failure_detail.empty());
}
void SnapshotDiagnostics(){
  for(int axis=0;axis<3;++axis)for(int clock=0;clock<2;++clock)for(bool future:{false,true})
    SnapshotFreshnessDiagnostic(axis,clock,future);
  SnapshotFailurePreservation();SnapshotBoundedDiagnostic();SnapshotDiagnosticAllocationFailure();
}
void SnapshotClockCatchUp(){
  TubeObservationBuffer b(Config());Seed(b);Start(b);
  b.Arm(Arm(10.1,20.1));b.Lift(Lift(10.1,20.1));
  int clock_reads=0;
  const auto caught=b.SnapshotAfterClockCatchUp([&] {
    return ++clock_reads<3?10.083333333:10.1;
  },[]{return 20.12;},.05,.05);
  assert(clock_reads>=3);
  assert(caught.source_now_s==10.1&&caught.arm.fresh&&caught.lift.fresh&&caught.motion_window.fresh);
  assert(caught.motion_window.failure_detail.empty());

  // Waiting for /clock never makes an old receipt or a stopped clock fresh.
  const auto stale=b.SnapshotAfterClockCatchUp([]{return 10.1;},[]{return 20.7;},.05,.05);
  assert(!stale.arm.fresh&&!stale.lift.fresh&&!stale.motion_window.fresh);
  const auto began=std::chrono::steady_clock::now();
  const auto stopped=b.SnapshotAfterClockCatchUp([]{return 10.083333333;},[]{return 20.12;},.05,.02);
  const auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
  assert(elapsed<.5&&!stopped.motion_window.fresh);
  int far_reads=0;
  const auto far=b.SnapshotAfterClockCatchUp([&]{++far_reads;return 9.9;},[]{return 20.12;},.05,.05);
  assert(far_reads==1&&!far.motion_window.fresh);
}
void GraspStopWithMeasuredJointNoise(){
  // The grasp observer supplies the first six fields and uses the explicit
  // default stop limits. Under finger contact PhysX reports 0.012 rad/s
  // peaks while the measured arm position varies by less than 0.0002 rad.
  TubeObserverConfig grasp{.25,.25,.25,1000,0,20};
  TubeObservationBuffer b(grasp);Seed(b);
  const TubeEvidence request{"workflow_stop",10.05,20.05};
  for(double t:{10.1,10.25,10.4,10.55,10.7,10.85,11.0,11.15,11.3}){
    auto a=Arm(t,t+10);a.velocities[1]=.012;a.positions[1]=.0001;
    b.Arm(a);b.Lift(Lift(t,t+10));
  }
  assert(b.Snapshot(11.3,21.3).ArmLiftStoppedSince(request));
  assert(!b.Snapshot(11.7,21.7).ArmLiftStoppedSince(request));

  TubeObservationBuffer moving(grasp);Seed(moving);
  for(double t:{10.1,10.25,10.4,10.55,10.7,10.85,11.0,11.15,11.3}){
    auto a=Arm(t,t+10);a.velocities[1]=.003;a.positions[1]=.003*(t-10);
    moving.Arm(a);moving.Lift(Lift(t,t+10));
  }
  assert(!moving.Snapshot(11.3,21.3).ArmLiftStoppedSince(request));
}
int main(int argc,char** argv){
  if(argc==5){SnapshotFreshnessDiagnostic(std::stoi(argv[1]),std::stoi(argv[2]),std::stoi(argv[3])!=0);return 0;}
  if(argc==2){const std::string selected=argv[1];
    if(selected=="preservation")SnapshotFailurePreservation();
    else if(selected=="bounded")SnapshotBoundedDiagnostic();
    else if(selected=="allocation")SnapshotDiagnosticAllocationFailure();
    else assert(false);
    return 0;}
  SnapshotDiagnostics();SnapshotClockCatchUp();GraspStopWithMeasuredJointNoise();
  {TubeObservationBuffer recovering(Config());Seed(recovering);
   recovering.Arm(Arm(10.6,20.6));recovering.Lift(Lift(10.6,20.6));
   assert(Start(recovering,10.65,20.65).generation);}
  TubeObservationBuffer b(Config());Seed(b);const auto token=Start(b);
  assert(token.generation&&token.barrier.source_stamp_s==10.05&&token.barrier.receipt_steady_s==20.05);
  // Queued pre-barrier measurements still contribute extrema before policy binding.
  auto a=Arm(10.02,20.06);a.positions[2]=.001;a.velocities[2]=-.0007;b.Arm(a);
  auto l=Lift(10.02,20.06);l.position_m=.401;l.velocity_mps=-.001;l.stationary=false;b.Lift(l);
  auto w=b.Snapshot(10.06,20.07).motion_window;
  assert(w.valid&&w.fresh&&w.arm[2].maximum==.001&&w.lift.maximum==.401);
  assert(w.arm_evidence.source_stamp_s<token.barrier.source_stamp_s);
  assert(w.lift.max_abs_velocity==.001&&w.lift.peak_evidence.source_stamp_s==10.02);
  a=Arm(10.1,20.1);l=Lift(10.1,20.1);b.Arm(a);b.Lift(l);
  w=b.Snapshot(10.1,20.1).motion_window;
  assert(w.arm[2].maximum==.001&&w.lift.maximum==.401&&w.lift.max_abs_velocity==.001);
  assert(!b.Snapshot(10.1,20.1).ArmLiftStoppedSince(token.barrier));
  // Every arm joint and both signs of rate/position survive out-and-back callbacks.
  for(int axis=0;axis<7;++axis)for(int sign:{-1,1}){
    TubeObservationBuffer x(Config());Seed(x);Start(x);
    auto aa=Arm(10.1,20.1);auto ll=Lift(10.1,20.1);
    if(axis<6){aa.positions[axis]=sign*.003;aa.velocities[axis]=sign*.02;}
    else{ll.position_m+=sign*.002;ll.velocity_mps=sign*.003;ll.stationary=false;}
    x.Arm(aa);x.Lift(ll);x.Arm(Arm(10.2,20.2));x.Lift(Lift(10.2,20.2));
    const auto z=x.Snapshot(10.2,20.2).motion_window;
    assert(z.valid);const auto& range=axis<6?z.arm[axis]:z.lift;
    assert(range.max_abs_velocity==(axis<6?.02:.003));
    assert(range.minimum<range.maximum);
    assert(range.peak_evidence.source_stamp_s==10.1);
  }
  // Poison survives all later good updates, including strict stationary recovery.
  for(int axis=0;axis<2;++axis)for(int fault=0;fault<11;++fault){
    TubeObservationBuffer x(Config());Seed(x);Start(x);
    auto aa=Arm(10.1,20.1);auto ll=Lift(10.1,20.1);
    auto& e=axis?ll.evidence:aa.evidence;
    if(fault==0)e.source="replacement";
    if(fault==1)e.source_stamp_s=9.9;
    if(fault==2)e.receipt_steady_s=19.9;
    if(fault==3)e.source_stamp_s=10.6;
    if(fault==4)e.receipt_steady_s=20.6;
    if(fault==5)e.source_stamp_s=NAN;
    if(fault==6){if(axis)ll.position_m=NAN;else aa.positions[3]=NAN;}
    if(fault==7){if(axis)ll.velocity_mps=INFINITY;else aa.velocities[4]=INFINITY;}
    if(fault==8){if(axis)ll.valid=false;else aa.valid=false;}
    if(fault==9){e.source_stamp_s=10;if(axis)ll.velocity_mps=.001;else aa.velocities[0]=.001;}
    if(fault==10){e.source_stamp_s=10;if(axis)ll.position_m=.401;else aa.positions[0]=.001;}
    if(axis)x.Lift(ll);else x.Arm(aa);
    assert(!x.Snapshot(10.7,20.7).motion_window.valid);
    x.Arm(Arm(10.2,20.2));x.Lift(Lift(10.2,20.2));
    assert(!x.Snapshot(10.2,20.2).motion_window.valid);
    assert(!Start(x,10.2,20.2).generation);
    assert(!x.Snapshot(10.2,20.2).motion_window.valid);
  }
  // Identical repeats never renew original receipts, including before arming.
  for(int axis=0;axis<2;++axis){
    TubeObservationBuffer x(Config());Seed(x);Start(x);
    for(double receipt:{20.2,20.4}){if(axis)x.Lift(Lift(10,receipt));else x.Arm(Arm(10,receipt));}
    auto z=x.Snapshot(10.4,20.4).motion_window;
    assert(z.valid&&(axis?z.lift_evidence:z.arm_evidence).receipt_steady_s==20);
    assert(!x.Snapshot(10.4,20.6).motion_window.fresh);
    if(axis)x.Lift(Lift(10.1,20.6));else x.Arm(Arm(10.1,20.6));
    assert(!x.Snapshot(10.4,20.6).motion_window.valid);
    TubeObservationBuffer prior(Config());Seed(prior);
    prior.Arm(Arm(10,20.4));prior.Lift(Lift(10,20.4));
    assert(!Start(prior,10.4,20.6).generation);
  }
  for(int invalid=0;invalid<5;++invalid){
    TubeObservationBuffer x(Config());if(invalid!=0)Seed(x);
    double s=10.05,r=20.05;
    if(invalid==1)s=NAN;if(invalid==2)r=INFINITY;if(invalid==3)s=10.6;if(invalid==4)r=19;
    assert(!Start(x,s,r).generation);assert(!Start(x).generation);
  }
  assert(!Start(b,10.2,20.2).generation);assert(!b.Snapshot(10.2,20.2).motion_window.valid);
  TubeObservationBuffer other(Config());Seed(other);assert(Start(other).generation!=token.generation);
}
