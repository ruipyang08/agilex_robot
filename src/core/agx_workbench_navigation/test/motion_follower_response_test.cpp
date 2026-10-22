#include <agx_workbench_navigation/motion_route_follower.hpp>
#include <agx_workbench_navigation/motion_speed_profile.hpp>
#include <agx_workbench_navigation/ranger_control.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>

namespace n=agx_workbench_navigation;

// Exercise the real follower -> producer limiter boundary. This response model
// represents the recorded bend failure (linear response ~0.1 s, curvature
// response ~0.5 s); it is a regression model, not hardware qualification.
void delayedBend(double direction) {
  n::MotionLimits limits;
  n::MotionRoute route;route.route_id="response";route.invocation_id="invocation";
  route.model_digest="model";route.map_digest="map";route.registry_digest="registry";
  const auto append=[&](n::MotionSegment segment) {
    assert(!segment.samples.empty());route.estimated_time_s+=segment.estimated_time_s;
    route.segments.push_back(std::move(segment));
  };
  append(n::MakeForwardPrimitive({},0,0,.8,0,.5,.5,limits));
  for(int i=0;i<2;++i)
    append(n::MakeForwardPrimitive(route.segments.back().samples.back().pose,0,0,.8,.5,.5,.5,limits));
  append(n::MakeForwardPrimitive(route.segments.back().samples.back().pose,0,0,.4,.5,.1,.5,limits));
  const double curvature=direction/limits.min_turn_radius_m;
  append(n::MakeForwardPrimitive(route.segments.back().samples.back().pose,0,curvature,.2,.1,.1,.1,limits));
  append(n::MakeForwardPrimitive(route.segments.back().samples.back().pose,curvature,0,.2,.1,.1,.1,limits));
  append(n::MakeForwardPrimitive(route.segments.back().samples.back().pose,0,0,.8,.1,0,.5,limits));
  route.goal=route.segments.back().samples.back().pose;
  assert(n::ValidateMotionRoute(route,limits).empty());
  n::MotionRouteFollower follower(route,limits,.02);
  n::RangerPlannerConfig config{};
  config.max_linear_speed_mps=limits.max_speed_mps;config.max_spin_speed_radps=limits.max_yaw_rate_radps;
  config.min_turn_radius_m=limits.min_turn_radius_m;config.max_linear_accel_mps2=limits.acceleration_mps2;
  config.max_linear_decel_mps2=limits.deceleration_mps2;config.max_lateral_accel_mps2=limits.lateral_acceleration_mps2;
  config.max_angular_accel_radps2=limits.angular_acceleration_radps2;
  config.max_curvature_rate_per_m_s=limits.curvature_rate_per_m_s;config.rate_limit_max_dt_s=.2;
  n::RangerCommandLimiter limiter(config);
  n::FollowerObservation actual;actual.source_stamp_s=1;actual.revision=1;
  std::deque<n::FollowerObservation> history(6,actual);
  n::RangerControlDecision issued{n::PlannerState::FOLLOW_ARC,0,0,0,0,"initial"};
  n::FollowerCommand intent;double speed=0,steering=0;
  bool passed_bend=false,entered_bend=false;double entry_speed=0;
  for(int step=0;step<20000 && !intent.complete;++step) {
    const double now=1+step*.01;
    if(step%10==0) {
      const auto& measured=history.front();
      intent=follower.Tick(measured,now);
      if(!intent.ok)std::fprintf(stderr,"delayed bend rejected: %s at x=%.6f y=%.6f yaw=%.6f segment=%zu\n",
          intent.reason.c_str(),actual.pose.x_m,actual.pose.y_m,actual.pose.yaw_rad,intent.segment);
      assert(intent.ok);
      n::CanonicalMotionProgress motion{true,measured.forward_mps,0,measured.yaw_radps,.03,.06};
      issued=limiter.limit({intent.complete?n::PlannerState::GOAL_REACHED:n::PlannerState::FOLLOW_ARC,
          intent.vx_mps,0,intent.wz_radps,0,intent.reason},now,motion);
      assert(issued.state!=n::PlannerState::BLOCKED);
      follower.AcceptLimitedCommand(issued.linear_x_mps,issued.angular_z_radps);
      passed_bend=passed_bend || intent.segment>=6;
    }
    speed+=(issued.linear_x_mps-speed)*(1-std::exp(-.01/.1));
    const double target=issued.linear_x_mps>1e-9?issued.angular_z_radps/issued.linear_x_mps:steering;
    steering+=(target-steering)*(1-std::exp(-.01/.5));
    const double yaw_rate=speed*steering,mid_yaw=actual.pose.yaw_rad+yaw_rate*.005;
    actual.pose.x_m+=speed*std::cos(mid_yaw)*.01;actual.pose.y_m+=speed*std::sin(mid_yaw)*.01;
    actual.pose.yaw_rad+=yaw_rate*.01;actual.forward_mps=speed;actual.yaw_radps=yaw_rate;
    if(!entered_bend && actual.pose.x_m>=2.8) {entered_bend=true;entry_speed=speed;}
    actual.source_stamp_s=now+.01;++actual.revision;
    history.push_back(actual);history.pop_front();
  }
  if(!intent.complete)std::fprintf(stderr,"delayed route unfinished: x=%g y=%g yaw=%g speed=%g segment=%zu state=%d reason=%s\n",
      actual.pose.x_m,actual.pose.y_m,actual.pose.yaw_rad,speed,intent.segment,int(intent.state),intent.reason.c_str());
  assert(passed_bend && intent.complete);
  std::printf("Delayed %s bend: entered at %.3f m/s, reached verified endpoint\n",direction>0?"left":"right",entry_speed);
  if(entry_speed>.13)std::fprintf(stderr,"bend entry speed=%g m/s\n",entry_speed);
  assert(entered_bend && entry_speed<=.1+.03); // Profile plus measured-progress lead.
  assert(std::hypot(actual.pose.x_m-route.goal.x_m,actual.pose.y_m-route.goal.y_m)<=.005);
  assert(std::abs(n::MotionAngleDifference(actual.pose.yaw_rad,route.goal.yaw_rad))<=.015);
  assert(speed<=.002);
}
// Initial spin -> forward route with delayed pose and physical angular response.
// The test uses production following and limiting, not a replacement controller.
void delayedInitialSpin(double direction,double feedback_delay,double response_delay,double start_yaw) {
  n::MotionLimits limits;n::MotionRoute route;route.route_id="initial-spin";route.invocation_id="spin";
  route.model_digest="model";route.map_digest="map";route.registry_digest="registry";
  route.measured_start=route.coarse_start={0,0,start_yaw};
  auto spin=n::MakeSpinPrimitive(route.coarse_start,direction*std::acos(-1.)/2,limits);
  auto forward=n::MakeForwardPrimitive(spin.samples.back().pose,0,0,1,0,0,.3,limits);
  route.segments={spin,forward};route.goal=forward.samples.back().pose;
  route.estimated_time_s=spin.estimated_time_s+forward.estimated_time_s;
  n::MotionRouteFollower follower(route,limits,.02);
  n::RangerPlannerConfig config{};
  config.max_linear_speed_mps=limits.max_speed_mps;config.max_spin_speed_radps=limits.max_yaw_rate_radps;
  config.min_turn_radius_m=limits.min_turn_radius_m;config.max_linear_accel_mps2=limits.acceleration_mps2;
  config.max_linear_decel_mps2=limits.deceleration_mps2;config.max_lateral_accel_mps2=limits.lateral_acceleration_mps2;
  config.max_angular_accel_radps2=limits.angular_acceleration_radps2;
  config.max_curvature_rate_per_m_s=limits.curvature_rate_per_m_s;config.rate_limit_max_dt_s=.2;
  n::RangerCommandLimiter limiter(config);
  n::FollowerObservation actual;actual.pose=route.measured_start;actual.source_stamp_s=1;actual.revision=1;
  std::deque<n::FollowerObservation> history(static_cast<std::size_t>(feedback_delay/.01)+1,actual);
  n::RangerControlDecision issued{n::PlannerState::SPIN_ALIGN,0,0,0,0,"initial"};
  n::FollowerCommand intent;double v=0,w=0,settled_error=100;bool passed_spin=false;
  for(int step=0;step<20000 && !intent.complete;++step) {
    const double now=1+step*.01;
    if(step%10==0) {
      const auto& measured=history.front();intent=follower.Tick(measured,now);
      if(!intent.ok)std::fprintf(stderr,"initial spin rejected: %s feedback=%g response=%g error_deg=%g\n",
          intent.reason.c_str(),feedback_delay,response_delay,
          n::MotionAngleDifference(spin.samples.back().pose.yaw_rad,actual.pose.yaw_rad)*180/std::acos(-1.));
      assert(intent.ok);
      n::CanonicalMotionProgress motion{true,measured.forward_mps,0,measured.yaw_radps,.03,.06};
      issued=limiter.limit({intent.complete?n::PlannerState::GOAL_REACHED:
          intent.vx_mps>0?n::PlannerState::FOLLOW_ARC:n::PlannerState::SPIN_ALIGN,
          intent.vx_mps,0,intent.wz_radps,0,intent.reason},now,motion);
      assert(issued.state!=n::PlannerState::BLOCKED);
      follower.AcceptLimitedCommand(issued.linear_x_mps,issued.angular_z_radps);
      if(!passed_spin && intent.segment>0) {
        passed_spin=true;settled_error=n::MotionAngleDifference(spin.samples.back().pose.yaw_rad,actual.pose.yaw_rad);
        assert(std::abs(w)<=.005 && std::abs(settled_error)<=.015);
      }
    }
    v+=(issued.linear_x_mps-v)*(1-std::exp(-.01/.1));
    w+=(issued.angular_z_radps-w)*(1-std::exp(-.01/response_delay));
    const double mid=actual.pose.yaw_rad+w*.005;
    actual.pose.x_m+=v*std::cos(mid)*.01;actual.pose.y_m+=v*std::sin(mid)*.01;
    actual.pose.yaw_rad=std::atan2(std::sin(actual.pose.yaw_rad+w*.01),std::cos(actual.pose.yaw_rad+w*.01));
    actual.forward_mps=v;actual.yaw_radps=w;actual.source_stamp_s=now+.01;++actual.revision;
    history.push_back(actual);history.pop_front();
  }
  assert(passed_spin && intent.complete);
  std::printf("Initial spin sign=%g feedback=%g response=%g settled_error_deg=%.4f: route completed\n",
      direction,feedback_delay,response_delay,settled_error*180/std::acos(-1.));
}

// Inject a small physical settling error only after the planned spin has
// requested braking. The same follower and limiter must recover or fail boundedly.
void stoppedSpinCorrection(double residual,const std::string& fault="",double start=0) {
  n::MotionLimits limits;n::MotionRoute route;route.route_id="correction";route.invocation_id="owned";
  route.model_digest="model";route.map_digest="map";route.registry_digest="registry";
  route.measured_start=route.coarse_start={0,0,start};
  auto spin=n::MakeSpinPrimitive(route.coarse_start,1.5707963267948966,limits);
  auto forward=n::MakeForwardPrimitive(spin.samples.back().pose,0,0,.3,0,0,.1,limits);
  route.segments={spin,forward};route.goal=forward.samples.back().pose;
  route.estimated_time_s=spin.estimated_time_s+forward.estimated_time_s;
  n::MotionRouteFollower follower(route,limits,.02);
  n::RangerPlannerConfig config{};config.max_linear_speed_mps=limits.max_speed_mps;
  config.max_spin_speed_radps=limits.max_yaw_rate_radps;config.min_turn_radius_m=limits.min_turn_radius_m;
  config.max_linear_accel_mps2=limits.acceleration_mps2;config.max_linear_decel_mps2=limits.deceleration_mps2;
  config.max_lateral_accel_mps2=limits.lateral_acceleration_mps2;
  config.max_angular_accel_radps2=limits.angular_acceleration_radps2;
  config.max_curvature_rate_per_m_s=limits.curvature_rate_per_m_s;config.rate_limit_max_dt_s=.2;
  n::RangerCommandLimiter limiter(config);
  n::FollowerObservation o;o.pose=route.coarse_start;o.source_stamp_s=1;o.revision=1;
  auto r=follower.Tick(o,1);double now=1,entry=-1,v=0,w=0;
  bool turned=false,injected=false,corrected=false;int reinjections=0;
  for(int tick=0;tick<5000 && r.ok && r.segment==0;++tick) {
    now+=.1;v+=(r.vx_mps-v)*(1-std::exp(-.1/.1));w+=(r.wz_radps-w)*(1-std::exp(-.1/.2));
    o.pose.yaw_rad+=w*.1;o.forward_mps=v;o.yaw_radps=w;
    turned=turned || w!=0;
    if(turned && r.state==n::FollowerState::BrakeToStop && !injected) {
      o.pose.yaw_rad=spin.samples.back().pose.yaw_rad-residual;w=o.yaw_radps=0;injected=true;
    }
    o.pose.yaw_rad=std::atan2(std::sin(o.pose.yaw_rad),std::cos(o.pose.yaw_rad));
    o.source_stamp_s=now;++o.revision;
    if(entry>=0) {
      if(fault=="stale")o.source_stamp_s=entry;
      if(fault=="drift")o.pose.x_m=.0201;
      if(fault=="stalled") {o.pose.yaw_rad=spin.samples.back().pose.yaw_rad-residual;w=o.yaw_radps=0;}
      if(fault=="cancel")follower.Block("cancelled");
      if(fault=="attempts" && r.reason=="waiting_for_corrected_stop" && reinjections<2) {
        o.pose.yaw_rad=spin.samples.back().pose.yaw_rad-residual;w=o.yaw_radps=0;++reinjections;
      }
    }
    r=follower.Tick(o,now);
    if(r.reason=="correcting_stopped_spin") {
      corrected=true;if(entry<0)entry=now;
      assert(r.vx_mps==0 && std::abs(r.wz_radps)<=.05);
    }
    if(!r.ok)break;
    n::CanonicalMotionProgress motion{true,o.forward_mps,0,o.yaw_radps,.03,.06};
    auto limited=limiter.limit({r.vx_mps>0?n::PlannerState::FOLLOW_ARC:n::PlannerState::SPIN_ALIGN,
        r.vx_mps,0,r.wz_radps,0,r.reason},now,motion);
    assert(limited.state!=n::PlannerState::BLOCKED);
    follower.AcceptLimitedCommand(limited.linear_x_mps,limited.angular_z_radps);
    r.vx_mps=limited.linear_x_mps;r.wz_radps=limited.angular_z_radps;
  }
  if(!injected)std::fprintf(stderr,"no injection: %s residual=%g start=%g\n",r.reason.c_str(),residual,start);
  assert(injected);
  if(fault.empty() && std::abs(residual)<=.05235987755982989) {
    if(!r.ok)std::fprintf(stderr,"correction rejected: %s residual=%g\n",r.reason.c_str(),residual);
    assert(corrected && r.ok && r.segment==1);
    assert(std::abs(n::MotionAngleDifference(spin.samples.back().pose.yaw_rad,o.pose.yaw_rad))<=.015);
    assert(std::abs(o.yaw_radps)<=.005);
  } else {
    if(r.ok)std::fprintf(stderr,"fault escaped: %s reason=%s segment=%zu entry=%g now=%g reinjected=%d\n",fault.c_str(),r.reason.c_str(),r.segment,entry,now,reinjections);
    assert(!r.ok && r.vx_mps==0 && r.wz_radps==0 && r.segment==0);
    if(fault=="stalled")assert(now-entry<=10.2);
    if(fault=="cancel")assert(r.reason=="cancelled");
    if(fault=="stale")assert(r.reason=="spin_correction_evidence_stale");
    if(fault=="attempts")assert(r.reason.find("attempts_exhausted")!=std::string::npos);
    if(fault=="drift")assert(r.reason=="spin_position_tracking_escape");
    assert(!follower.Tick(o,now+.1).ok); // Terminal fault cannot restart.
  }
}
int main() {
  for(double error:{-.026,.026})for(double start:{0.,3.12})stoppedSpinCorrection(error,"",start);
  for(const std::string fault:{"stale","drift","stalled","cancel","attempts"})stoppedSpinCorrection(.026,fault);
  stoppedSpinCorrection(.06);
  delayedBend(1);delayedBend(-1);
  for(double direction:{-1.,1.})for(double feedback:{0.,.1})for(double response:{.1,.2})
    for(double start:{std::acos(-1.)/2,3.12})delayedInitialSpin(direction,feedback,response,start);
}
