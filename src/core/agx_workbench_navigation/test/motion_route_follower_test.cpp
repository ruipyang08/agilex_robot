#include <agx_workbench_navigation/motion_route_follower.hpp>
#include <agx_workbench_navigation/motion_speed_profile.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <utility>
namespace n=agx_workbench_navigation;
int main() {
  n::MotionLimits limits;
  n::MotionRoute route;route.route_id="r";route.invocation_id="i";
  route.model_digest="model";route.map_digest="map";route.registry_digest="registry";
  auto forward=n::MakeForwardPrimitive({},0,0,1,0,0,.3,limits);
  assert(!forward.samples.empty());
  auto spin=n::MakeSpinPrimitive(forward.samples.back().pose,1.5707963267948966,limits);
  route.segments={forward,spin};route.goal=spin.samples.back().pose;
  route.estimated_time_s=forward.estimated_time_s+spin.estimated_time_s;
  n::MotionRouteFollower follower(route,limits,.02);
  n::FollowerObservation observation;observation.source_stamp_s=1;observation.revision=1;
  auto result=follower.Tick(observation,1);
  assert(result.ok && result.vx_mps==0 && result.wz_radps==0);
  // A repeated stopped sample cannot establish measured stop dwell.
  result=follower.Tick(observation,1.5);assert(result.vx_mps==0);
  observation.source_stamp_s=1.6;observation.revision=2;
  result=follower.Tick(observation,1.6);assert(result.ok && result.vx_mps>0);
  bool saw_spin=false,saw_stopped_boundary=false;
  double now=1.6;
  for(int i=0;i<5000 && !result.complete;++i) {
    constexpr double dt=.02;now+=dt;
    observation.pose.x_m+=result.vx_mps*std::cos(observation.pose.yaw_rad)*dt;
    observation.pose.y_m+=result.vx_mps*std::sin(observation.pose.yaw_rad)*dt;
    observation.pose.yaw_rad+=result.wz_radps*dt;
    observation.forward_mps=result.vx_mps;observation.yaw_radps=result.wz_radps;
    observation.source_stamp_s=now;++observation.revision;
    result=follower.Tick(observation,now);
    assert(result.ok);
    if(result.wz_radps!=0) {saw_spin=true;assert(result.vx_mps==0);assert(std::abs(observation.pose.x_m-1)<.01);}
    if(observation.pose.x_m>.98 && result.vx_mps==0 && result.wz_radps==0)saw_stopped_boundary=true;
  }
  assert(result.complete && saw_spin && saw_stopped_boundary);
  assert(std::hypot(observation.pose.x_m-route.goal.x_m,observation.pose.y_m-route.goal.y_m)<.01);
  assert(std::abs(n::MotionAngleDifference(route.goal.yaw_rad,observation.pose.yaw_rad))<.02);
  n::MotionRouteFollower escaped(route,limits,.02);
  observation={};observation.source_stamp_s=1;observation.revision=1;observation.pose.y_m=.08;
  assert(!escaped.Tick(observation,1).ok);
  observation.pose.y_m=0;observation.source_stamp_s=2;++observation.revision;
  assert(!escaped.Tick(observation,2).ok); // Terminal tracking escape cannot restart.
  n::MotionRouteFollower stalled(route,limits,.02);
  observation={};observation.source_stamp_s=1;observation.revision=1;stalled.Tick(observation,1);
  observation.source_stamp_s=1.2;++observation.revision;
  assert(stalled.Tick(observation,1.2).vx_mps>0);
  observation.source_stamp_s=4;++observation.revision;
  assert(!stalled.Tick(observation,4).ok);
  // Prepared segments have a duplicate stopped pose before moving. Signed
  // rotation must come from the first nonzero turn, not that duplicate.
  for(double sign: {-1.,1.}) {
    auto turn=n::MakeSpinPrimitive({},sign*1.5707963267948966,limits);
    auto turning=route;turning.segments={turn};turning.goal=turn.samples.back().pose;
    turning.estimated_time_s=turn.estimated_time_s;
    n::MotionRouteFollower follow(turning,limits,.02);
    n::FollowerObservation o;o.source_stamp_s=1;o.revision=1;
    auto r=follow.Tick(o,1);double t=1;
    for(int i=0;i<3000 && !r.complete;++i) {
      t+=.02;o.pose.yaw_rad+=r.wz_radps*.02;o.yaw_radps=r.wz_radps;
      o.source_stamp_s=t;++o.revision;r=follow.Tick(o,t);
      assert(r.ok);assert(sign*r.wz_radps>=0);
    }
    assert(r.complete);
  }
  // Trial22 reached staging but drifted 5.37 mm during the final rotation.
  // The installed coarse XY tolerance may be used within the tracking bound;
  // an explicitly stricter follower and an escape outside that bound still fail.
  auto arrival=route;
  const auto final_turn=n::MakeSpinPrimitive({},1.5707963267948966,limits);
  arrival.segments={final_turn};arrival.goal=final_turn.samples.back().pose;
  arrival.estimated_time_s=final_turn.estimated_time_s;
  for(double tolerance:{.005,.02}) {
    n::MotionRouteFollower configured(arrival,limits,.02,.25,tolerance);
    n::FollowerObservation measured;measured.pose={-.0051999995975685,.0013507097360424,0};
    measured.source_stamp_s=1;measured.revision=1;
    auto demand=configured.Tick(measured,1);double clock=1;
    for(int i=0;i<3000 && demand.ok && !demand.complete;++i) {
      clock+=.02;measured.pose.yaw_rad+=demand.wz_radps*.02;
      measured.yaw_radps=demand.wz_radps;measured.source_stamp_s=clock;++measured.revision;
      demand=configured.Tick(measured,clock);
    }
    if(tolerance==.005)assert(!demand.ok && demand.reason=="segment_endpoint_position_outside_tolerance");
    else assert(demand.ok && demand.complete);
  }
  n::MotionRouteFollower bounded(arrival,limits,.02,.25,.02);
  n::FollowerObservation outside;outside.pose.x_m=.0201;outside.source_stamp_s=1;outside.revision=1;
  assert(!bounded.Tick(outside,1).ok);
  n::MotionRouteFollower widened(arrival,limits,.02,.25,.03);
  outside.pose.x_m=0;assert(!widened.Tick(outside,1).ok);
  // Trial23 stopped within coarse XY accuracy, then its settled heading
  // residual was 0.02692 rad. Configure terminal coarse yaw independently of
  // strict internal-turn alignment; never widen the normal heading bound.
  for(double tolerance:{.015,.15}) {
    n::MotionRouteFollower configured(arrival,limits,.02,.25,.02,tolerance);
    n::FollowerObservation measured;measured.source_stamp_s=1;measured.revision=1;
    auto demand=configured.Tick(measured,1);double clock=1;bool turned=false,injected=false;
    for(int i=0;i<3000 && demand.ok && !demand.complete;++i) {
      clock+=.02;measured.pose.yaw_rad+=demand.wz_radps*.02;
      measured.yaw_radps=demand.wz_radps;turned=turned || demand.wz_radps!=0;
      if(turned && demand.state==n::FollowerState::BrakeToStop && !injected) {
        measured.pose.yaw_rad=arrival.goal.yaw_rad-.02691905132321981;
        measured.pose.x_m=.010424442959770899;measured.yaw_radps=0;injected=true;
      }
      measured.source_stamp_s=clock;++measured.revision;demand=configured.Tick(measured,clock);
    }
    assert(injected);
    // Strict accuracy now recovers through a bounded stopped correction; it
    // still must satisfy its original tolerance before completing.
    assert(demand.ok && demand.complete);
    assert(std::abs(n::MotionAngleDifference(arrival.goal.yaw_rad,measured.pose.yaw_rad))<=tolerance);
  }
  n::MotionRouteFollower invalid_yaw(arrival,limits,.02,.25,.02,.151);
  outside.pose.x_m=0;assert(!invalid_yaw.Tick(outside,1).ok);
  // A planned prepared quarter arc must stay near its geometric path when
  // measured motion follows commands. It must not substitute an in-place spin.
  auto curved=n::MakePreparedForwardPrimitive({},0,1/limits.min_turn_radius_m,
      limits.min_turn_radius_m*1.5707963267948966,0,.15,limits,.005);
  auto turning=route;turning.segments={curved};turning.goal=curved.samples.back().pose;
  turning.estimated_time_s=curved.estimated_time_s;
  n::MotionRouteFollower follow(turning,limits,.02);
  n::FollowerObservation o;o.source_stamp_s=1;o.revision=1;
  auto r=follow.Tick(o,1);double t=1;
  for(int i=0;i<5000 && !r.complete;++i) {
    t+=.02;const double heading=o.pose.yaw_rad+r.wz_radps*.01;
    o.pose.x_m+=r.vx_mps*std::cos(heading)*.02;o.pose.y_m+=r.vx_mps*std::sin(heading)*.02;
    o.pose.yaw_rad+=r.wz_radps*.02;o.forward_mps=r.vx_mps;o.yaw_radps=r.wz_radps;
    o.source_stamp_s=t;++o.revision;r=follow.Tick(o,t);
    if(!r.ok)std::fprintf(stderr,"arc rejected: %s at x=%g y=%g yaw=%g, goal=%g %g %g\n",r.reason.c_str(),o.pose.x_m,o.pose.y_m,o.pose.yaw_rad,turning.goal.x_m,turning.goal.y_m,turning.goal.yaw_rad);
    assert(r.ok);assert(r.vx_mps!=0 || r.wz_radps==0);
  }
  if(!r.complete)std::fprintf(stderr,"ideal arc unfinished: x=%g y=%g yaw=%g v=%g w=%g segment=%zu sample=%zu reason=%s\n",
      o.pose.x_m,o.pose.y_m,o.pose.yaw_rad,r.vx_mps,r.wz_radps,r.segment,r.sample,r.reason.c_str());
  assert(r.complete);
  // Real control updates can pass a non-stopping segment boundary between
  // samples. Continue the admitted route without demanding a 5 mm hit.
  auto first=n::MakeForwardPrimitive({},0,0,.8,0,.5,.5,limits);
  auto next=n::MakeForwardPrimitive(first.samples.back().pose,0,0,.8,.5,0,.5,limits);
  auto continuous=route;continuous.segments={first,next};continuous.goal=next.samples.back().pose;
  continuous.estimated_time_s=first.estimated_time_s+next.estimated_time_s;
  assert(!first.stop_after && next.stop_after);
  n::MotionRouteFollower joined(continuous,limits,.02);
  n::FollowerObservation moving;moving.source_stamp_s=1;moving.revision=1;
  auto command=joined.Tick(moving,1);double source=1;
  for(int i=0;i<100 && moving.pose.x_m<.72;++i) {
    source+=.1;moving.pose.x_m+=command.vx_mps*.1;moving.forward_mps=command.vx_mps;
    moving.source_stamp_s=source;++moving.revision;command=joined.Tick(moving,source);
    if(!command.ok)std::fprintf(stderr,"connected boundary rejected: %s at x=%g\n",command.reason.c_str(),moving.pose.x_m);
    assert(command.ok);
  }
  moving.pose.x_m=.79;moving.forward_mps=.45;source+=.1;
  moving.source_stamp_s=source;++moving.revision;command=joined.Tick(moving,source);
  assert(command.ok && command.segment==0);
  // A continuous edge join must not demand final stopped yaw accuracy either.
  moving.pose.x_m=.798;moving.pose.yaw_rad=.02;source+=.1;
  moving.source_stamp_s=source;++moving.revision;command=joined.Tick(moving,source);
  assert(command.ok && command.segment==1);
  moving.pose.yaw_rad=0;
  moving.pose.x_m=.835;source+=.1;moving.source_stamp_s=source;++moving.revision;
  command=joined.Tick(moving,source);
  assert(command.ok && command.segment==1 && command.vx_mps>0);

  // Trial16 stopped about 7 mm short of a required endpoint. A short startup
  // response delay then made braking preview alternate positive/zero demands,
  // repeatedly restarting the final owner's stopped dwell. Keep a continuous
  // slow approach outside the 5 mm acceptance disk, without relaxing that disk.
  for(double lateral: {-0.0015,0.0015}) {
    auto endpoint_route=route;endpoint_route.segments={forward};
    endpoint_route.goal=forward.samples.back().pose;
    endpoint_route.estimated_time_s=forward.estimated_time_s;
    n::MotionRouteFollower endpoint(endpoint_route,limits,.02);
    n::FollowerObservation e;e.source_stamp_s=1;e.revision=1;
    endpoint.Tick(e,1);e.source_stamp_s=1.2;++e.revision;
    auto c=endpoint.Tick(e,1.2);
    double clock=1.3;
    for(int tick=0;tick<1000 && e.pose.x_m<.95;++tick) {
      e.pose.x_m+=c.vx_mps*.1;e.forward_mps=c.vx_mps;
      e.source_stamp_s=clock;++e.revision;c=endpoint.Tick(e,clock);clock+=.1;
      assert(c.ok);
    }
    assert(e.pose.x_m>=.95);
    e.pose.x_m=.9932;e.pose.y_m=lateral;e.forward_mps=0;
    // Retain a stationary physical sample for two control periods while the
    // actuator starts. The initial braking command itself may be zero.
    for(int tick=0;tick<3;++tick) {
      e.source_stamp_s=clock;++e.revision;c=endpoint.Tick(e,clock);clock+=.1;
      assert(c.ok);
      if(tick>0)assert(c.vx_mps>0);
    }
    for(int tick=0;tick<100 && !c.complete;++tick) {
      e.pose.x_m+=c.vx_mps*std::cos(e.pose.yaw_rad)*.1;
      e.pose.y_m+=c.vx_mps*std::sin(e.pose.yaw_rad)*.1;
      e.pose.yaw_rad+=c.wz_radps*.1;e.forward_mps=c.vx_mps;e.yaw_radps=c.wz_radps;
      e.source_stamp_s=clock;++e.revision;c=endpoint.Tick(e,clock);clock+=.1;
      assert(c.ok);
    }
    assert(c.complete);
    assert(std::hypot(e.pose.x_m-1,e.pose.y_m)<=.005);
  }

  // An internal stopped join retains the route's 20 mm tracking bound. Its
  // longitudinal arrival remains within 5 mm, and final arrival stays strict.
  auto internal=route;
  auto onward=n::MakeForwardPrimitive(spin.samples.back().pose,0,0,1,0,0,.3,limits);
  internal.segments.push_back(onward);internal.goal=onward.samples.back().pose;
  internal.estimated_time_s+=onward.estimated_time_s;
  n::MotionRouteFollower strict_internal_yaw(internal,limits,.02,.25,.02,.15);
  n::FollowerObservation middle;middle.source_stamp_s=1;middle.revision=1;
  auto mid_command=strict_internal_yaw.Tick(middle,1);bool spun=false,injected_yaw=false;
  for(int tick=0;tick<5000 && mid_command.ok && !mid_command.complete;++tick) {
    const double heading=middle.pose.yaw_rad+mid_command.wz_radps*.01;
    middle.pose.x_m+=mid_command.vx_mps*std::cos(heading)*.02;
    middle.pose.y_m+=mid_command.vx_mps*std::sin(heading)*.02;
    middle.pose.yaw_rad+=mid_command.wz_radps*.02;
    middle.forward_mps=mid_command.vx_mps;middle.yaw_radps=mid_command.wz_radps;
    spun=spun || (mid_command.segment==1 && mid_command.wz_radps!=0);
    if(spun && mid_command.segment==1 && mid_command.state==n::FollowerState::BrakeToStop && !injected_yaw) {
      middle.pose=spin.samples.back().pose;middle.pose.yaw_rad-=.02691905132321981;
      middle.forward_mps=middle.yaw_radps=0;injected_yaw=true;
    }
    middle.source_stamp_s+=.02;++middle.revision;
    mid_command=strict_internal_yaw.Tick(middle,middle.source_stamp_s);
  }
  assert(injected_yaw && mid_command.ok && mid_command.complete);
  const auto nearEnd=[&](n::MotionRouteFollower& f,double lateral) {
    n::FollowerObservation e;e.source_stamp_s=1;e.revision=1;f.Tick(e,1);
    e.source_stamp_s=1.2;++e.revision;auto c=f.Tick(e,1.2);double clock=1.3;
    for(int tick=0;tick<1000 && e.pose.x_m<.95;++tick) {
      e.pose.x_m+=c.vx_mps*.1;e.forward_mps=c.vx_mps;
      e.source_stamp_s=clock;++e.revision;c=f.Tick(e,clock);clock+=.1;assert(c.ok);
    }
    e.pose.x_m=.9973;e.pose.y_m=lateral;e.forward_mps=0;
    e.source_stamp_s=clock;++e.revision;c=f.Tick(e,clock);
    return std::make_pair(c,e);
  };
  n::MotionRouteFollower internal_join(internal,limits,.02);
  auto internal_command=nearEnd(internal_join,.013);
  assert(internal_command.first.ok && internal_command.first.state==n::FollowerState::BrakeToStop);
  auto continued=internal_command.first;auto physical=internal_command.second;
  for(int tick=0;tick<5000 && !continued.complete;++tick) {
    const double heading=physical.pose.yaw_rad+continued.wz_radps*.05;
    physical.pose.x_m+=continued.vx_mps*std::cos(heading)*.1;
    physical.pose.y_m+=continued.vx_mps*std::sin(heading)*.1;
    physical.pose.yaw_rad+=continued.wz_radps*.1;
    physical.forward_mps=continued.vx_mps;physical.yaw_radps=continued.wz_radps;
    physical.source_stamp_s+=.1;++physical.revision;
    continued=internal_join.Tick(physical,physical.source_stamp_s);assert(continued.ok);
  }
  assert(continued.complete);
  assert(std::hypot(physical.pose.x_m-internal.goal.x_m,physical.pose.y_m-internal.goal.y_m)<=.005);
  n::MotionRouteFollower final_position(route,limits,.02);
  auto final_command=nearEnd(final_position,.013);
  assert(final_command.first.ok && !final_command.first.complete && final_command.first.state!=n::FollowerState::BrakeToStop);
  n::MotionRouteFollower outside_join(internal,limits,.02);
  assert(!nearEnd(outside_join,.021).first.ok);

}
