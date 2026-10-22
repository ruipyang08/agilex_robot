#include "agx_navigation_tasks/travel_policy.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
namespace agx_navigation_tasks {
namespace {
void DiagnosticText(std::ostream& out,const std::string& text,std::size_t limit=96) {
  out.write(text.data(),std::min(text.size(),limit));
  if (text.size()>limit) out<<"...";
}
template<class Observation>
bool StationaryThrough(const Observation& observation,const agx_robot_task_runtime::Evidence& start) {
  const double source=observation.stationary_source_since_s,receipt=observation.stationary_receipt_since_s;
  return observation.stationary && std::isfinite(source) && std::isfinite(receipt) && source>0 && receipt>0 &&
      source<=observation.evidence.source_stamp_s && receipt<=observation.evidence.receipt_steady_s &&
      source<=start.source_stamp_s && receipt<=start.receipt_steady_s;
}
template<class EvidenceType>
bool Fresh(const TravelProfile& p,const EvidenceType& e,double source,double steady) {
  return !e.source.empty() && std::isfinite(source) && std::isfinite(steady) &&
      std::isfinite(e.source_stamp_s) && std::isfinite(e.receipt_steady_s) &&
      e.source_stamp_s > 0 && e.receipt_steady_s > 0 && source >= e.source_stamp_s &&
      steady >= e.receipt_steady_s && source-e.source_stamp_s <= p.source_max_age_s &&
      steady-e.receipt_steady_s <= p.receipt_max_age_s;
}
bool Same(const std::optional<agx_robot_task_runtime::ObjectRef>& a,const std::optional<agx_robot_task_runtime::ObjectRef>& b) {
  return a.has_value() == b.has_value() && (!a || (a->id == b->id &&
      a->identity == b->identity && a->container_profile == b->container_profile));
}
bool Custody(const TravelProfile& p,const TravelSample& s,double source,double steady) {
  const auto& r=s.robot; const auto& t=r.transport; const auto& g=r.gripper;
  if (r.halted || t.pending || t.uncertain || !g.fresh || !g.evidence ||
      !Fresh(p,*g.evidence,source,steady) || !t.custody_evidence ||
      !Fresh(p,*t.custody_evidence,source,steady) || !Same(g.object,t.object)) return false;
  for (const auto& item : r.objects)
    if (item.second.pending || item.second.certainty != agx_robot_task_runtime::LocationCertainty::Confirmed) return false;
  if (t.eligibility != agx_robot_task_runtime::TravelEligibility::Unknown && t.eligibility != agx_robot_task_runtime::TravelEligibility::SafeClear) return false;
  if (t.load == agx_robot_task_runtime::TransportLoad::Empty)
    return g.occupancy == agx_robot_task_runtime::GripperOccupancy::Empty && !g.object;
  if (t.load != agx_robot_task_runtime::TransportLoad::SingleHeld || g.occupancy != agx_robot_task_runtime::GripperOccupancy::Holding || !g.object ||
      g.object->id.empty() || g.object->container_profile != p.container_profile_id ||
      (g.object->identity != agx_robot_task_runtime::ObjectIdentity::Registered && g.object->identity != agx_robot_task_runtime::ObjectIdentity::RunLocal)) return false;
  const auto location=r.objects.find(g.object->id);
  return location != r.objects.end() && Same(location->second.object,g.object) &&
      location->second.fresh && location->second.current_evidence &&
      Fresh(p,*location->second.current_evidence,source,steady) &&
      location->second.last_confirmed_holder && location->second.last_confirmed_holder->kind == agx_robot_task_runtime::HolderKind::Gripper;
}
bool ArmFresh(const TravelProfile& p,const TravelSample& s,double source,double steady) {
  if (!s.arm.valid || !Fresh(p,s.arm.evidence,source,steady) ||
      s.arm.positions_rad.size() != p.arm.size() || s.arm.velocities_rad_s.size() != p.arm.size()) return false;
  for (const auto& joint : p.arm)
    if (!s.arm.positions_rad.count(joint.name) || !s.arm.velocities_rad_s.count(joint.name) ||
        !std::isfinite(s.arm.positions_rad.at(joint.name)) || !std::isfinite(s.arm.velocities_rad_s.at(joint.name))) return false;
  return true;
}
bool LiftFresh(const TravelProfile& p,const TravelSample& s,double source,double steady) {
  return s.lift.valid && Fresh(p,s.lift.evidence,source,steady) &&
      std::isfinite(s.lift.position_m) && std::isfinite(s.lift.velocity_mps);
}
bool ArmAt(const TravelProfile& p,const TravelSample& s,const std::map<std::string,double>& positions,
    const std::map<std::string,double>& tolerances = {}) {
  for (const auto& j : p.arm)
    if (std::abs(s.arm.positions_rad.at(j.name)-positions.at(j.name)) >
        (tolerances.empty() ? j.tolerance_rad : tolerances.at(j.name)) ||
        std::abs(s.arm.velocities_rad_s.at(j.name)) > j.stopped_velocity_rad_s) return false;
  return true;
}
bool Hold(const TravelProfile& p,const TravelSample& s,double source,double steady) {
  return s.hold.held_stopped && s.hold.command_guard_ok && !s.hold.session_id.empty() &&
      s.hold.hold_epoch > 0 && Fresh(p,s.hold.evidence,source,steady);
}
bool After(const agx_robot_task_runtime::Evidence& e,double source,double steady) {
  return e.source_stamp_s > source && e.receipt_steady_s > steady;
}
bool Newer(const agx_robot_task_runtime::Evidence& e,const agx_robot_task_runtime::Evidence& previous) {
  return e.source == previous.source && e.source_stamp_s >= previous.source_stamp_s &&
      e.receipt_steady_s >= previous.receipt_steady_s;
}
agx_robot_task_runtime::Evidence WindowEvidence(const agxcore::TubeEvidence& e) {return {e.source,e.source_stamp_s,e.receipt_steady_s,{},{},{}};}
}
StartClearance EvaluateStartClearance(const TravelProfile& p,TravelMode mode,
    const TravelSample& s,double source,double steady) {
  ValidateTravelProfile(p);
  if (!TravelEnabled(p,mode)) return {false,{},{},"travel mode is not qualified"};
  if (!Custody(p,s,source,steady)) return {false,{},{},"load is not fresh, known and supported"};
  if (!ArmFresh(p,s,source,steady) || !LiftFresh(p,s,source,steady))
    return {false,{},{},"starting arm/lift evidence is invalid or stale"};
  for (const auto& start : p.starts) {
    const bool supported=s.robot.transport.load == agx_robot_task_runtime::TransportLoad::Empty ? start.empty_allowed : start.single_held_allowed;
    if (supported && ArmAt(p,s,start.arm_positions_rad,start.arm_tolerances_rad) &&
        std::abs(s.lift.position_m-start.lift_position_m) <= start.lift_tolerance_m &&
        std::abs(s.lift.velocity_mps) <= p.lift_stopped_velocity_mps)
      return {true,start.id,start.path_id,"measured qualified starting posture"};
  }
  return {false,{},{},"no qualified starting path matches measured posture and load"};
}
TravelPolicy::TravelPolicy(TravelProfile p,TravelMode mode): profile_(std::move(p)),mode_(mode) {
  ValidateTravelProfile(profile_); (void)TravelEnabled(profile_,mode_);
}
TravelDecision TravelPolicy::Fail(agx_task_workflow::FailureKind kind,const std::string& detail) {
  if (failure_.failure == agx_task_workflow::FailureKind::None) { failure_.failure=kind; failure_.detail=detail; }
  return failure_;
}
bool TravelPolicy::Time(double source,double steady,bool preparation) {
  if (failure_.failure != agx_task_workflow::FailureKind::None) return false;
  if (!std::isfinite(source) || !std::isfinite(steady) || source < last_source_ || steady < last_steady_) {
    Fail(agx_task_workflow::FailureKind::Fault,"travel clock is invalid or regressed"); return false;
  }
  if (preparation && steady >= deadline_) { Fail(agx_task_workflow::FailureKind::Deadline,"travel preparation deadline expired"); return false; }
  if (steady-last_steady_ > profile_.receipt_max_age_s || source-last_source_ > profile_.source_max_age_s) {
    Fail(agx_task_workflow::FailureKind::Fault,"travel supervision missed its observation interval"); return false;
  }
  last_source_=source; last_steady_=steady;
  return true;
}
bool TravelPolicy::SameCustody(const TravelSample& s,double source,double steady) const {
  return Custody(profile_,s,source,steady) && s.robot.transport.load == load_ && Same(s.robot.transport.object,object_);
}
bool TravelPolicy::Ordered(const TravelSample& s,bool clearance) {
  bool ordered=true;
  if (previous_) {
    const auto& p=*previous_;
    ordered=Newer(s.arm.evidence,p.arm.evidence) && Newer(s.lift.evidence,p.lift.evidence) &&
        Newer(*s.robot.gripper.evidence,*p.robot.gripper.evidence);
    if (s.producers.valid && p.producers.valid) ordered=ordered && Newer(s.producers.evidence,p.producers.evidence);
    if (clearance && s.robot.transport.clearance_evidence && p.robot.transport.clearance_evidence)
      ordered=ordered && Newer(*s.robot.transport.clearance_evidence,*p.robot.transport.clearance_evidence);
  }
  if (ordered) {
    // An invalid intermediate producer report must not reset its high-water mark.
    const auto producer=(previous_ && !s.producers.valid) ? previous_->producers : s.producers;
    const auto clearance_evidence=(previous_ && !s.robot.transport.clearance_evidence) ?
        previous_->robot.transport.clearance_evidence : s.robot.transport.clearance_evidence;
    previous_=s; previous_->producers=producer; previous_->robot.transport.clearance_evidence=clearance_evidence;
  }
  return ordered;
}
StartClearance TravelPolicy::BeginPreparation(const TravelSample& s,double source,double steady) {
  if (begun_ || failure_.failure != agx_task_workflow::FailureKind::None) {
    Fail(agx_task_workflow::FailureKind::Fault,"travel preparation cannot restart"); return {false,{},{},failure_.detail};
  }
  auto start=EvaluateStartClearance(profile_,mode_,s,source,steady);
  if (!start.ok || !StationaryThrough(s.arm,s.arm.evidence) || !StationaryThrough(s.lift,s.lift.evidence) ||
      !Hold(profile_,s,source,steady) || !s.producers.valid ||
      !s.producers.arm_stopped || !s.producers.lift_stopped || !Fresh(profile_,s.producers.evidence,source,steady)) {
    Fail(agx_task_workflow::FailureKind::Fault,start.ok ? "fresh stopped hold and command guard are required" : start.detail);
    return {false,{},{},failure_.detail};
  }
  begun_=true; load_=s.robot.transport.load; object_=s.robot.transport.object;
  session_id_=s.hold.session_id; hold_epoch_=s.hold.hold_epoch;
  last_source_=source; last_steady_=steady; deadline_=steady+profile_.preparation_timeout_s;
  if (!std::isfinite(deadline_)) { Fail(agx_task_workflow::FailureKind::Fault,"invalid preparation deadline"); return {false,{},{},failure_.detail}; }
  previous_=s;
  return start;
}
TravelDecision TravelPolicy::CheckPreparation(const TravelSample& s,double source,double steady) {
  if (!begun_) return Fail(agx_task_workflow::FailureKind::Fault,"preparation has not begun");
  if (!Time(source,steady,true)) return failure_;
  if (!SameCustody(s,source,steady)) return Fail(agx_task_workflow::FailureKind::Fault,"preparation custody changed or became uncertain");
  if (!Hold(profile_,s,source,steady) || s.hold.session_id != session_id_ || s.hold.hold_epoch != hold_epoch_)
    return Fail(agx_task_workflow::FailureKind::Fault,"preparation lost its measured hold or command guard");
  if (!ArmFresh(profile_,s,source,steady))
    return Fail(agx_task_workflow::FailureKind::Fault,"preparation arm observation is invalid or stale");
  if (!LiftFresh(profile_,s,source,steady))
    return Fail(agx_task_workflow::FailureKind::Fault,"preparation lift observation is invalid or stale");
  if (commands_finished_ && !Retained(s,source,steady)) return failure_;
  if (!Ordered(s,commands_finished_))
    return Fail(agx_task_workflow::FailureKind::Fault,"preparation observations are out of order");
  TravelDecision out; out.detail="qualified preparation remains guarded"; return out;
}
TravelDecision TravelPolicy::CommandsFinished(const agxcore::TubeMotionWindowToken& token,double source,double steady) {
  if (!begun_ || commands_finished_) return Fail(agx_task_workflow::FailureKind::Fault,"invalid command completion boundary");
  const double prior_source=last_source_,prior_steady=last_steady_;
  if (!Time(source,steady,true)) return failure_;
  const auto& barrier=token.barrier;
  if (!token.generation || barrier.source!="motion_window" ||
      !Fresh(profile_,WindowEvidence(barrier),source,steady) ||
      barrier.source_stamp_s<prior_source || barrier.receipt_steady_s<prior_steady)
    return Fail(agx_task_workflow::FailureKind::Fault,"invalid original motion-window completion barrier");
  commands_finished_=true;window_token_=token;
  barrier_source_=barrier.source_stamp_s;barrier_steady_=barrier.receipt_steady_s;
  TravelDecision out; out.detail="awaiting post-command measured endpoint and dwell"; return out;
}
TravelDecision TravelPolicy::Endpoint(const TravelSample& s,double source,double steady) const {
  TravelDecision out;
  std::map<std::string,double> targets;
  for (const auto& joint : profile_.arm) targets[joint.name]=joint.position_rad;
  out.arm_ready=ArmFresh(profile_,s,source,steady) && ArmAt(profile_,s,targets) &&
      (traveling_ || StationaryThrough(s.arm,s.arm.evidence));
  out.lift_ready=LiftFresh(profile_,s,source,steady) &&
      std::abs(s.lift.position_m-profile_.lift_position_m) <= profile_.lift_tolerance_m &&
      std::abs(s.lift.velocity_mps) <= profile_.lift_stopped_velocity_mps &&
      (traveling_ || StationaryThrough(s.lift,s.lift.evidence));
  out.producers_stopped=s.producers.valid && s.producers.arm_stopped && s.producers.lift_stopped &&
      Fresh(profile_,s.producers.evidence,source,steady);
  out.clearance_verified=s.robot.transport.ready && s.robot.transport.eligibility == agx_robot_task_runtime::TravelEligibility::SafeClear &&
      s.robot.transport.clearance_evidence && Fresh(profile_,*s.robot.transport.clearance_evidence,source,steady);
  out.ready=out.arm_ready && out.lift_ready && out.producers_stopped && out.clearance_verified;
  out.state_revision=s.robot.transport.revision;
  return out;
}
bool TravelPolicy::Retained(const TravelSample& s,double source,double steady) {
  const auto& w=s.motion_window;const auto& token=w.token;
  auto reject=[&](const std::string& detail){Fail(agx_task_workflow::FailureKind::Fault,detail);return false;};
  if (!w.valid || !w.fresh || !token.generation || token.generation!=window_token_.generation ||
      token.barrier.source!=window_token_.barrier.source ||
      token.barrier.source_stamp_s!=barrier_source_ || token.barrier.receipt_steady_s!=barrier_steady_ ||
      !Fresh(profile_,WindowEvidence(w.arm_evidence),source,steady) ||
      !Fresh(profile_,WindowEvidence(w.lift_evidence),source,steady)) {
    std::string message;
    try {
      std::ostringstream detail;detail.exceptions(std::ios::badbit|std::ios::failbit);
      detail<<std::setprecision(17)<<"motion window invalid, stale or changed: consumption valid="<<w.valid
          <<" fresh="<<w.fresh<<" arm_fresh="<<Fresh(profile_,w.arm_evidence,source,steady)
          <<" lift_fresh="<<Fresh(profile_,w.lift_evidence,source,steady)
          <<" source_now="<<source<<" steady_now="<<steady
          <<" source_max_age="<<profile_.source_max_age_s<<" receipt_max_age="<<profile_.receipt_max_age_s;
      const auto evidence=[&](const char* name,const agxcore::TubeEvidence& e) {
        detail<<" "<<name<<"_source=";DiagnosticText(detail,e.source);
        detail<<" "<<name<<"_stamp="<<e.source_stamp_s<<" "<<name<<"_receipt="<<e.receipt_steady_s;
      };
      evidence("arm",w.arm_evidence);evidence("lift",w.lift_evidence);
      detail<<" actual_generation="<<token.generation;evidence("actual_barrier",token.barrier);
      detail<<" expected_generation="<<window_token_.generation;
      evidence("expected_barrier",window_token_.barrier);
      evidence("failure",w.failure_evidence);
      detail<<" prior_detail=";DiagnosticText(detail,w.failure_detail,1536);
      message=detail.str();
    } catch (...) { return reject("window fault"); }
    return reject(message);
  }
  if (previous_ && previous_->motion_window.token.generation==token.generation &&
      (!Newer(WindowEvidence(w.arm_evidence),WindowEvidence(previous_->motion_window.arm_evidence)) ||
       !Newer(WindowEvidence(w.lift_evidence),WindowEvidence(previous_->motion_window.lift_evidence))))
    return reject("motion window original evidence regressed");
  const auto bounded=[&](const std::string& name,const agxcore::TubeMotionExtrema& range,
      double target,double tolerance,double span,double speed) {
    const char* reason=nullptr;double bound=0;agxcore::TubeEvidence evidence;
    if (!std::isfinite(range.minimum) || !std::isfinite(range.maximum) ||
        !std::isfinite(range.max_abs_velocity) || range.minimum>range.maximum || range.max_abs_velocity<0)
      reason="nonfinite or invalid extrema";
    else if (range.minimum<target-tolerance) {reason="minimum posture";bound=target-tolerance;evidence=range.minimum_evidence;}
    else if (range.maximum>target+tolerance) {reason="maximum posture";bound=target+tolerance;evidence=range.maximum_evidence;}
    // Compare representable interval endpoints, avoiding cancellation in a
    // one-mm quantized height subtraction. No physical epsilon is added.
    else if (range.maximum>range.minimum+span) {reason="position range";bound=span;evidence=range.maximum_evidence;}
    else if (range.max_abs_velocity>speed) {reason="peak reported velocity";bound=speed;evidence=range.peak_evidence;}
    if (!reason) return true;
    std::ostringstream detail;detail<<"motion window "<<name<<": "<<reason<<std::setprecision(17)
        <<" min="<<range.minimum<<" max="<<range.maximum<<" peak="<<range.max_abs_velocity<<" bound="<<bound
        <<" source="<<evidence.source<<" stamp="<<evidence.source_stamp_s<<" receipt="<<evidence.receipt_steady_s
        <<" min_stamp="<<range.minimum_evidence.source_stamp_s<<" min_receipt="<<range.minimum_evidence.receipt_steady_s;
    return reject(detail.str());
  };
  for (const auto& joint:profile_.arm) {
    if (joint.name.size()!=6 || joint.name.compare(0,5,"joint")!=0 || joint.name[5]<'1' || joint.name[5]>'6')
      return reject("motion window cannot identify configured arm joint");
    if (!bounded(joint.name,w.arm[joint.name[5]-'1'],joint.position_rad,joint.tolerance_rad,
        profile_.robot.observer.stopped_arm_range_rad,joint.stopped_velocity_rad_s)) return false;
  }
  return bounded("lift",w.lift,profile_.lift_position_m,profile_.lift_tolerance_m,
      profile_.robot.observer.stopped_lift_range_m,profile_.lift_stopped_velocity_mps);
}
TravelDecision TravelPolicy::Observe(const TravelSample& s,double source,double steady) {
  if (ready_) return CheckTravel(s,source,steady);
  auto check=CheckPreparation(s,source,steady);
  if (check.failure != agx_task_workflow::FailureKind::None) return check;
  if (!commands_finished_) { check.detail="commands have not finished"; return check; }
  auto out=Endpoint(s,source,steady);
  if (!out.ready || !After(s.arm.evidence,barrier_source_,barrier_steady_) ||
      !After(s.lift.evidence,barrier_source_,barrier_steady_) ||
      !After(WindowEvidence(s.motion_window.arm_evidence),barrier_source_,barrier_steady_) ||
      !After(WindowEvidence(s.motion_window.lift_evidence),barrier_source_,barrier_steady_) ||
      !After(s.producers.evidence,barrier_source_,barrier_steady_) ||
      !After(*s.robot.gripper.evidence,barrier_source_,barrier_steady_) ||
      !After(*s.robot.transport.clearance_evidence,barrier_source_,barrier_steady_)) {
    settled_start_.reset(); out.ready=false; out.detail="awaiting fresh post-command endpoint"; return out;
  }
  bool stable=settled_start_ && StationaryThrough(s.arm,settled_start_->arm.evidence) &&
      StationaryThrough(s.lift,settled_start_->lift.evidence);
  if (stable) {
    for (const auto& joint : profile_.arm) {
      auto& range=settled_arm_range_[joint.name]; const double value=s.arm.positions_rad.at(joint.name);
      range.first=std::min(range.first,value); range.second=std::max(range.second,value);
      stable=stable && range.second-range.first <= joint.tolerance_rad;
    }
    settled_lift_range_.first=std::min(settled_lift_range_.first,s.lift.position_m);
    settled_lift_range_.second=std::max(settled_lift_range_.second,s.lift.position_m);
    stable=stable && settled_lift_range_.second-settled_lift_range_.first <= profile_.lift_tolerance_m;
  }
  if (!stable) {
    settled_start_=s;
    for (const auto& item : s.arm.positions_rad) settled_arm_range_[item.first]={item.second,item.second};
    settled_lift_range_={s.lift.position_m,s.lift.position_m};
  }
  const auto& start=*settled_start_;
  const auto dwell=[&](const agx_robot_task_runtime::Evidence& now,const agx_robot_task_runtime::Evidence& first) {
    return now.source_stamp_s-first.source_stamp_s >= profile_.settle_s &&
        now.receipt_steady_s-first.receipt_steady_s >= profile_.settle_s;
  };
  out.ready=dwell(s.arm.evidence,start.arm.evidence) && dwell(s.lift.evidence,start.lift.evidence) &&
      dwell(s.producers.evidence,start.producers.evidence);
  if (out.ready) { ready_=true; ready_revision_=s.robot.transport.revision; }
  out.detail=out.ready ? "measured default posture and transport readiness" : "awaiting measured settled dwell";
  return out;
}
TravelDecision TravelPolicy::CheckTravel(const TravelSample& s,double source,double steady) {
  if (!ready_) return Fail(agx_task_workflow::FailureKind::Fault,"travel was not qualified");
  if (!Time(source,steady,false)) return failure_;
  if (!SameCustody(s,source,steady) || s.robot.transport.revision != ready_revision_)
    return Fail(agx_task_workflow::FailureKind::Fault,"travel custody or semantic state revision changed");
  if (!Retained(s,source,steady)) return failure_;
  if (!ArmFresh(profile_,s,source,steady) || !LiftFresh(profile_,s,source,steady) || !Ordered(s,true))
    return Fail(agx_task_workflow::FailureKind::Fault,"travel observations are invalid, stale or out of order");
  if (!traveling_) {
    if (!settled_start_) return Fail(agx_task_workflow::FailureKind::Fault,"certified strict stationary interval is missing");
    const auto& start=*settled_start_;
    const bool arm=StationaryThrough(s.arm,start.arm.evidence),lift=StationaryThrough(s.lift,start.lift.evidence);
    if (!arm || !lift) {
      const auto& evidence=arm?s.lift.evidence:s.arm.evidence;
      const auto& pinned=arm?start.lift.evidence:start.arm.evidence;
      std::ostringstream detail;detail<<"strict stationary interval interrupted before departure: "<<(arm?"lift":"arm")
          <<std::setprecision(17)<<" stationary="<<(arm?s.lift.stationary:s.arm.stationary)
          <<" source_since="<<(arm?s.lift.stationary_source_since_s:s.arm.stationary_source_since_s)
          <<" receipt_since="<<(arm?s.lift.stationary_receipt_since_s:s.arm.stationary_receipt_since_s)
          <<" settled_source="<<pinned.source_stamp_s<<" settled_receipt="<<pinned.receipt_steady_s
          <<" source="<<evidence.source<<" stamp="<<evidence.source_stamp_s<<" receipt="<<evidence.receipt_steady_s;
      return Fail(agx_task_workflow::FailureKind::Fault,detail.str());
    }
  }
  auto out=Endpoint(s,source,steady);
  if (!out.ready) {
    const auto flag=[](bool value) {return value ? "true" : "false";};
    return Fail(agx_task_workflow::FailureKind::Fault,std::string("live travel readiness lost: arm_ready=")+flag(out.arm_ready)+
        ", lift_ready="+flag(out.lift_ready)+", producers_stopped="+flag(out.producers_stopped)+
        ", clearance_verified="+flag(out.clearance_verified));
  }
  out.detail="live measured travel readiness"; return out;
}
TravelDecision TravelPolicy::BeginTravel(const TravelSample& s,double source,double steady) {
  if (traveling_) return Fail(agx_task_workflow::FailureKind::Fault,"travel departure cannot repeat");
  auto out=CheckTravel(s,source,steady); // Still requires strict physical stop.
  if (out.ready) {traveling_=true;out.detail="released hold with strict stopped endpoint; bounded travel retention active";}
  return out;
}
void TravelPolicy::Abort(agx_task_workflow::FailureKind cause) {
  if (cause != agx_task_workflow::FailureKind::Cancelled && cause != agx_task_workflow::FailureKind::Deadline && cause != agx_task_workflow::FailureKind::Fault)
    cause=agx_task_workflow::FailureKind::Fault;
  Fail(cause,"travel preparation or supervision interrupted");
}
}
