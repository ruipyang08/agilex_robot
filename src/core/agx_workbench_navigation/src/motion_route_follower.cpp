#include <agx_workbench_navigation/motion_route_follower.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
namespace agx_workbench_navigation {
namespace {
struct ForwardProjection {double distance=1e100,u=0;std::size_t sample=0;};
ForwardProjection ProjectForward(const MotionSegment& segment,const MotionPose& pose,std::size_t begin) {
  ForwardProjection result;result.sample=begin;
  const auto& samples=segment.samples;
  for(std::size_t i=begin;i+1<samples.size() && i<begin+80;++i) {
    const auto& a=samples[i].pose;const auto& b=samples[i+1].pose;
    const double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m,length2=dx*dx+dy*dy;
    if(length2<1e-15)continue;
    const double u=std::max(0.,std::min(1.,((pose.x_m-a.x_m)*dx+(pose.y_m-a.y_m)*dy)/length2));
    const double error=std::hypot(pose.x_m-a.x_m-u*dx,pose.y_m-a.y_m-u*dy);
    if(error<result.distance)result={error,u,i};
  }
  return result;
}
}
MotionRouteFollower::MotionRouteFollower(MotionRoute route,MotionLimits limits,double tracking_bound,double slowdown,
    double goal_position_tolerance,double goal_yaw_tolerance,std::size_t first_segment)
    :route_(std::move(route)),limits_(limits),tracking_bound_(tracking_bound),curvature_tracking_slowdown_(slowdown),
     goal_position_tolerance_(goal_position_tolerance),goal_yaw_tolerance_(goal_yaw_tolerance) {
  if(!ValidateMotionRoute(route_,limits_).empty() || !std::isfinite(tracking_bound_) ||
     tracking_bound_<=0 || tracking_bound_>.25 || !std::isfinite(slowdown) || slowdown<=0 ||
     !std::isfinite(goal_position_tolerance_) || goal_position_tolerance_<=0 ||
     goal_position_tolerance_>tracking_bound_ || !std::isfinite(goal_yaw_tolerance_) ||
     goal_yaw_tolerance_<=0 || goal_yaw_tolerance_>.15)
    Block("invalid_route_or_tracking_bound");
  if(first_segment && (first_segment!=1 || route_.segments.size()<2 ||
      route_.segments.front().kind!=MotionKind::DockDeparture))Block("invalid_initial_segment");
  segment_=first_segment;
  for(std::size_t i=segment_;i<route_.segments.size();++i)
    if(route_.segments[i].kind==MotionKind::DockDeparture)Block("departure_requires_its_registered_owner");
}
void MotionRouteFollower::Block(const std::string& reason) {
  if(state_!=FollowerState::Blocked)reason_=reason;
  state_=FollowerState::Blocked;
}
void MotionRouteFollower::AcceptLimitedCommand(double v,double w) {
  if(!std::isfinite(v) || !std::isfinite(w) || v<0 || v>limits_.max_speed_mps ||
      (v>0 && std::abs(w)>v/limits_.min_turn_radius_m+1e-12) ||
      (v==0 && std::abs(w)>limits_.max_yaw_rate_radps)) {
    Block("invalid_accepted_command");return;
  }
  previous_v_=v;previous_w_=w;previous_curvature_=v>0?w/v:0;
}
FollowerCommand MotionRouteFollower::Tick(const FollowerObservation& observation,double now) {
  const auto output=[&](double v=0,double w=0,bool complete=false) {
    FollowerCommand result{state_!=FollowerState::Blocked,complete,v,w,state_,segment_,sample_,reason_};
    result.correcting=correction_began_s_>=0 && segment_<route_.segments.size();
    if(result.correcting)result.correction_target_yaw_rad=route_.segments[segment_].samples.back().pose.yaw_rad;
    result.correction_attempts=correction_attempts_;return result;
  };
  const auto fail=[&](const char* reason) {Block(reason);return output();};
  if(state_==FollowerState::Blocked)return output();
  if(!ValidMotionPose(observation.pose) || !std::isfinite(now) || now<=0 || now<last_now_ ||
     !std::isfinite(observation.source_stamp_s) || observation.source_stamp_s<=0 ||
     observation.revision==0 || !std::isfinite(observation.forward_mps) ||
     !std::isfinite(observation.lateral_mps) || !std::isfinite(observation.yaw_radps))return fail("invalid_motion_evidence");
  if(observation.source_stamp_s>now) {reason_="waiting_for_original_sample_time";return output();}
  if(correction_began_s_>=0 && now-observation.source_stamp_s>.25)
    return fail("spin_correction_evidence_stale");
  if(correction_began_s_>=0 && observation.source_stamp_s-correction_began_s_>10)
    return fail("spin_correction_timeout");
  if(last_observation_.revision) {
    if(observation.revision<last_observation_.revision || observation.source_stamp_s<last_observation_.source_stamp_s)
      return fail("motion_evidence_regressed");
    if(observation.revision==last_observation_.revision &&
       (observation.source_stamp_s!=last_observation_.source_stamp_s ||
        !SameMotionPose(observation.pose,last_observation_.pose,0) ||
        observation.forward_mps!=last_observation_.forward_mps ||
        observation.lateral_mps!=last_observation_.lateral_mps ||
        observation.yaw_radps!=last_observation_.yaw_radps))return fail("contradictory_motion_repeat");
  }
  const double dt=last_now_<0?.05:std::min(.1,now-last_now_);last_now_=now;last_observation_=observation;
  const bool stopped=std::hypot(observation.forward_mps,observation.lateral_mps)<=.002 &&
      std::abs(observation.yaw_radps)<=.005;
  if(stopped) {
    if(stopped_since_<0)stopped_since_=observation.source_stamp_s;
  } else stopped_since_=-1;
  const bool settled=stopped_since_>=0 && observation.source_stamp_s-stopped_since_>=limits_.stop_dwell_s;
  constexpr double position_tolerance=.005,yaw_tolerance=.015;
  if(segment_>=route_.segments.size()) {
    state_=FollowerState::VerifyEndpoint;
    if(std::hypot(observation.pose.x_m-route_.goal.x_m,observation.pose.y_m-route_.goal.y_m)>goal_position_tolerance_)
      return fail("terminal_endpoint_position_outside_tolerance");
    if(std::abs(MotionAngleDifference(route_.goal.yaw_rad,observation.pose.yaw_rad))>goal_yaw_tolerance_)
      return fail("terminal_endpoint_yaw_outside_tolerance");
    reason_="verified_route_endpoint";return output(0,0,settled);
  }
  // A connected forward edge is a discretization boundary, not a stopping
  // target. A measured update can cross it without ever landing within 5 mm.
  // Only advance from its final projection window onto the admitted successor;
  // stopped boundaries and actual lateral escapes retain their original checks.
  while(state_==FollowerState::FollowForward && segment_+1<route_.segments.size()) {
    const auto& current=route_.segments[segment_];const auto& next=route_.segments[segment_+1];
    if(current.kind!=MotionKind::Forward || current.stop_after || next.kind!=MotionKind::Forward ||
       sample_+80<current.samples.size())break;
    const auto& end=current.samples.back().pose;
    const double ahead=(observation.pose.x_m-end.x_m)*std::cos(end.yaw_rad)+
        (observation.pose.y_m-end.y_m)*std::sin(end.yaw_rad);
    if(ahead<0 || ProjectForward(next,observation.pose,0).distance>tracking_bound_)break;
    ++segment_;sample_=0;
  }
  const auto& segment=route_.segments[segment_];const auto& samples=segment.samples;
  const auto& start=samples.front().pose;const auto& end=samples.back().pose;
  // Internal stopped joins inherit the checked tracking corridor. A forward
  // arrival still reaches its longitudinal endpoint; sideways tracking residual
  // cannot be removed by stopped steering. The final coarse destination uses its
  // configured accuracy, capped by the tracking bound; docking owns parking.
  const bool internal_stop=segment.stop_after &&
      std::hypot(end.x_m-route_.goal.x_m,end.y_m-route_.goal.y_m)>1e-9;
  const double endpoint_position_tolerance=internal_stop?tracking_bound_:goal_position_tolerance_;
  const double endpoint_yaw_tolerance=SameMotionPose(end,route_.goal,1e-9)?goal_yaw_tolerance_:yaw_tolerance;
  if(segment.kind==MotionKind::Spin || segment.kind==MotionKind::VerifyStop) {
    if(std::hypot(observation.pose.x_m-start.x_m,observation.pose.y_m-start.y_m)>tracking_bound_)
      return fail("spin_position_tracking_escape");
  }
  if(state_==FollowerState::BrakeToStop) {
    if(!finishing_segment_ && std::hypot(observation.pose.x_m-start.x_m,observation.pose.y_m-start.y_m)>tracking_bound_)
      return fail("segment_start_tracking_escape");
    previous_v_=previous_w_=0;
    if(!settled) {reason_="waiting_for_measured_stop";return output();}
    if(finishing_segment_) {
      if(std::hypot(observation.pose.x_m-end.x_m,observation.pose.y_m-end.y_m)>endpoint_position_tolerance)
        return fail("segment_endpoint_position_outside_tolerance");
      const double error=MotionAngleDifference(end.yaw_rad,observation.pose.yaw_rad);
      if(std::abs(error)>endpoint_yaw_tolerance) {
        const auto headingFailure=[&](const char* cause) {
          std::ostringstream detail;detail<<cause<<" target_rad="<<end.yaw_rad<<" error_rad="<<error
              <<" tolerance_rad="<<endpoint_yaw_tolerance<<" attempts="<<correction_attempts_;
          Block(detail.str());return output();
        };
        if(segment.kind!=MotionKind::Spin || std::abs(error)>.05235987755982989)
          return headingFailure("segment_endpoint_yaw_outside_tolerance");
        if(now-observation.source_stamp_s>.25)return fail("spin_correction_evidence_stale");
        if(correction_attempts_>=2)return headingFailure("spin_correction_attempts_exhausted");
        if(correction_began_s_<0)correction_began_s_=observation.source_stamp_s;
        ++correction_attempts_;state_=FollowerState::CorrectSpin;stopped_since_=-1;
        reason_="correcting_stopped_spin";return output();
      }
      ++segment_;sample_=0;finishing_segment_=false;previous_curvature_=0;
      correction_attempts_=0;correction_began_s_=-1;
      return output();
    }
    state_=FollowerState::SettleMode;settle_since_=now;
  }
  if(state_==FollowerState::CorrectSpin) {
    const double error=MotionAngleDifference(end.yaw_rad,observation.pose.yaw_rad);
    if(std::abs(error)>.05235987755982989)return fail("spin_correction_heading_escape");
    if(std::abs(error)<=endpoint_yaw_tolerance/2) {
      state_=FollowerState::BrakeToStop;previous_w_=0;reason_="waiting_for_corrected_stop";return output();
    }
    const double response_s=std::max(0.,now-observation.source_stamp_s)+dt+.1;
    const double reserve=std::max(std::abs(previous_w_),std::abs(observation.yaw_radps))*response_s;
    const double approach=std::max(0.,std::abs(error)-reserve-endpoint_yaw_tolerance/4);
    const double requested=std::min(.05,1.5*approach);
    const double w=std::copysign(std::min(requested,std::abs(previous_w_)+limits_.angular_acceleration_radps2*dt),error);
    reason_="correcting_stopped_spin";return output(0,w);
  }
  if(segment.kind==MotionKind::VerifyStop) {
    finishing_segment_=true;state_=FollowerState::BrakeToStop;return output();
  }
  if(state_==FollowerState::SettleMode) {
    if(std::hypot(observation.forward_mps,observation.lateral_mps)>.0001 ||
        std::abs(observation.yaw_radps)>.0001)
      state_=segment.kind==MotionKind::Spin?FollowerState::FollowSpin:FollowerState::FollowForward;
    else if(now-settle_since_>limits_.steering_settle_s)return fail("steering_or_propulsion_settle_timeout");
  }
  double v=0,w=0;
  if(segment.kind==MotionKind::Spin) {
    double planned_angle=0;
    for(std::size_t i=1;i<samples.size();++i)
      planned_angle+=MotionAngleDifference(samples[i].pose.yaw_rad,samples[i-1].pose.yaw_rad);
    const double direction=planned_angle>=0?1.:-1.;
    double closest=1e100;std::size_t best=sample_;
    for(std::size_t i=sample_;i<samples.size() && i<sample_+80;++i) {
      const double error=std::abs(MotionAngleDifference(samples[i].pose.yaw_rad,observation.pose.yaw_rad));
      if(error<closest) {closest=error;best=i;}
    }
    sample_=best;
    double remaining=MotionAngleDifference(samples[best].pose.yaw_rad,observation.pose.yaw_rad);
    for(std::size_t i=best+1;i<samples.size();++i)
      remaining+=MotionAngleDifference(samples[i].pose.yaw_rad,samples[i-1].pose.yaw_rad);
    remaining*=direction;
    if(remaining< -.05235987755982989)return fail("spin_overshoot");
    // Small moving overshoot first brakes. Reverse motion may begin only after
    // the fresh measured stop, through the checked correction state above.
    if(remaining<=endpoint_yaw_tolerance) {
      finishing_segment_=true;state_=FollowerState::BrakeToStop;previous_w_=0;return output();
    }
    // Original feedback age, a control interval and the same 0.1 s response
    // allowance used for forward braking consume angle before speed responds.
    // Remember the accepted command; the requested ramp is not physical motion.
    const double response_s=std::max(0.,now-observation.source_stamp_s)+dt+.1;
    const double reaction_angle=std::max(std::abs(previous_w_),std::abs(observation.yaw_radps))*response_s;
    const double approach=std::max(0.,remaining-reaction_angle-endpoint_yaw_tolerance/2);
    const double requested=std::min(limits_.max_yaw_rate_radps,
        std::min(std::sqrt(2*limits_.angular_acceleration_radps2*approach),1.5*approach));
    w=direction*std::min(requested,std::abs(previous_w_)+limits_.angular_acceleration_radps2*dt);
    if(state_==FollowerState::SettleMode)w=direction*std::min(std::abs(w),.1);
    reason_="following_planned_spin";
  } else {
    const auto projection=ProjectForward(segment,observation.pose,sample_);
    const double u_best=projection.u;const auto best=projection.sample;
    if(projection.distance>tracking_bound_)return fail("forward_position_tracking_escape");
    sample_=best;
    const auto& a=samples[best];const auto& b=samples[best+1];
    const double reference_yaw=a.pose.yaw_rad+u_best*MotionAngleDifference(b.pose.yaw_rad,a.pose.yaw_rad);
    const double heading_error=MotionAngleDifference(reference_yaw,observation.pose.yaw_rad);
    if(std::abs(heading_error)>.15)return fail("forward_heading_tracking_escape");
    const double endpoint_along=(observation.pose.x_m-end.x_m)*std::cos(end.yaw_rad)+
        (observation.pose.y_m-end.y_m)*std::sin(end.yaw_rad);
    if(std::hypot(observation.pose.x_m-end.x_m,observation.pose.y_m-end.y_m)<=endpoint_position_tolerance &&
       (!internal_stop || std::abs(endpoint_along)<=position_tolerance)) {
      if(segment.stop_after && std::abs(MotionAngleDifference(end.yaw_rad,observation.pose.yaw_rad))>yaw_tolerance)
        return fail("forward_endpoint_heading_escape");
      if(segment.stop_after) {finishing_segment_=true;state_=FollowerState::BrakeToStop;previous_v_=previous_w_=0;}
      else {++segment_;sample_=0;return output(previous_v_,previous_w_);}
      return output();
    }
    double remaining=0;for(std::size_t i=best+1;i<samples.size();++i)
      remaining+=(i==best+1?1-u_best:1)*std::hypot(
          samples[i].pose.x_m-samples[i-1].pose.x_m,samples[i].pose.y_m-samples[i-1].pose.y_m);
    double ceiling=std::min(limits_.max_speed_mps,
        std::max(samples[best].speed_ceiling_mps,samples[best+1].speed_ceiling_mps));
    // Preview only the connected forward route. Source age, one control period,
    // and the stopping sweep's 0.1 s response allowance consume braking distance;
    // the profile at the current pose alone assumes instantaneous actuation.
    const double response_s=std::max(0.,now-observation.source_stamp_s)+dt+.1;
    const double reaction_distance=std::max(previous_v_,std::max(0.,observation.forward_mps))*response_s;
    const double horizon=reaction_distance+limits_.max_speed_mps*limits_.max_speed_mps/(2*limits_.deceleration_mps2);
    double distance=0;bool preview_done=false;
    for(std::size_t j=segment_;j<route_.segments.size() && !preview_done;++j) {
      const auto& ahead=route_.segments[j];
      if(ahead.kind!=MotionKind::Forward || (j>segment_ && ahead.stop_before))break;
      for(std::size_t k=j==segment_?best+1:1;k<ahead.samples.size();++k) {
        double ds=std::hypot(ahead.samples[k].pose.x_m-ahead.samples[k-1].pose.x_m,
            ahead.samples[k].pose.y_m-ahead.samples[k-1].pose.y_m);
        if(j==segment_ && k==best+1)ds*=1-u_best;
        if(ds<=1e-12)continue; // Preparation and stopped dwell duplicate poses.
        distance+=ds;
        // Aim inside the acceptance disk, leaving room for lateral residual.
        const double tolerance=ahead.stop_after && ahead.samples[k].speed_ceiling_mps==0?position_tolerance/2:0;
        const double braking_distance=std::max(0.,distance-reaction_distance-tolerance);
        ceiling=std::min(ceiling,std::sqrt(ahead.samples[k].speed_ceiling_mps*ahead.samples[k].speed_ceiling_mps+
            2*limits_.deceleration_mps2*braking_distance));
        if(distance>horizon) {preview_done=true;break;}
      }
      if(ahead.stop_after)break;
    }
    v=std::min(ceiling,previous_v_+limits_.acceleration_mps2*dt);
    if(segment.stop_after) {
      const double approach=std::max(0.,remaining-position_tolerance/2);
      v=std::min(v,std::min(std::sqrt(2*limits_.deceleration_mps2*approach),1.5*approach));
      // Match the spin endpoint's proportional approach. A square-root-only
      // tail can request another startup pulse after braking preview reaches
      // zero, then immediately request zero again before the actuator responds.
    }
    if(state_==FollowerState::SettleMode)v=std::min(v,.04);
    const double rx=a.pose.x_m+u_best*(b.pose.x_m-a.pose.x_m)-observation.pose.x_m;
    const double ry=a.pose.y_m+u_best*(b.pose.y_m-a.pose.y_m)-observation.pose.y_m;
    const double lateral=-std::sin(observation.pose.yaw_rad)*rx+std::cos(observation.pose.yaw_rad)*ry;
    double curvature=a.curvature_per_m+u_best*(b.curvature_per_m-a.curvature_per_m)+
        std::max(-.5,std::min(.5,8*lateral+4*heading_error));
    curvature=std::max(-1/limits_.min_turn_radius_m,std::min(1/limits_.min_turn_radius_m,curvature));
    const double measured_curvature=observation.forward_mps>.002?
        observation.yaw_radps/observation.forward_mps:previous_curvature_;
    const double lag=std::max(std::abs(curvature-previous_curvature_),std::abs(curvature-measured_curvature));
    // Leave rate headroom for steering response and tracking correction instead
    // of spending the whole profile ceiling on translation during catch-up.
    const double tracking_scale=curvature_tracking_slowdown_/(curvature_tracking_slowdown_+lag);
    v=std::min(v,ceiling*tracking_scale*tracking_scale);
    // This limit is d(curvature)/dt, matching the final producer limiter.
    const double change=limits_.curvature_rate_per_m_s*dt;
    curvature=std::max(previous_curvature_-change,std::min(previous_curvature_+change,curvature));
    if(std::abs(curvature)>0)v=std::min(v,std::sqrt(limits_.lateral_acceleration_mps2/std::abs(curvature)));
    w=v*curvature;
    const double angular_ceiling=std::abs(previous_w_)+limits_.angular_acceleration_radps2*dt;
    if(std::abs(w)>angular_ceiling && std::abs(curvature)>1e-9) {
      v=std::min(v,angular_ceiling/std::abs(curvature));w=v*curvature;
    }
    previous_curvature_=curvature;reason_="following_planned_forward";
  }
  previous_v_=v;previous_w_=w;return output(v,w);
}
}
