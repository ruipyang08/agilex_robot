#include <agx_workbench_navigation/motion_route.hpp>
#include <cmath>

namespace agx_workbench_navigation {
namespace {
constexpr double kEpsilon=1e-6;
bool Nonnegative(double value) {return std::isfinite(value) && value>=0;}
bool Stopped(const MotionSample& s) {return s.speed_ceiling_mps==0 && s.yaw_ceiling_radps==0;}
}
double MotionAngleDifference(double to,double from) {return std::atan2(std::sin(to-from),std::cos(to-from));}
bool ValidMotionPose(const MotionPose& p) {
  return std::isfinite(p.x_m) && std::isfinite(p.y_m) && std::isfinite(p.yaw_rad) &&
      std::abs(p.x_m)<=1e6 && std::abs(p.y_m)<=1e6 && std::abs(p.yaw_rad)<=std::acos(-1.)+1e-12;
}
bool SameMotionPose(const MotionPose& a,const MotionPose& b,double eps) {
  return ValidMotionPose(a) && ValidMotionPose(b) && std::isfinite(eps) && eps>=0 &&
      std::hypot(a.x_m-b.x_m,a.y_m-b.y_m)<=eps && std::abs(MotionAngleDifference(a.yaw_rad,b.yaw_rad))<=eps;
}
std::string ValidateMotionLimits(const MotionLimits& l) {
  for(double v:{l.max_speed_mps,l.acceleration_mps2,l.deceleration_mps2,l.max_yaw_rate_radps,
      l.angular_acceleration_radps2,l.curvature_rate_per_m_s,l.lateral_acceleration_mps2,
      l.min_turn_radius_m,l.steering_settle_s,l.stop_dwell_s})
    if(!std::isfinite(v) || v<=0)return "motion limits must be finite and positive";
  if(l.max_speed_mps>.5 || l.acceleration_mps2>.25 || l.deceleration_mps2>.3 ||
     l.max_yaw_rate_radps>kMotionYawRateCeilingRadps || l.angular_acceleration_radps2>.4 ||
     l.curvature_rate_per_m_s>.5 || l.lateral_acceleration_mps2>.2 ||
     l.min_turn_radius_m<.4764 || l.steering_settle_s>2 || l.stop_dwell_s<.12)
    return "motion limits exceed reviewed ceilings";
  return {};
}
std::string ValidateMotionRoute(const MotionRoute& r,const MotionLimits& l) {
  const auto invalid_limits=ValidateMotionLimits(l);if(!invalid_limits.empty())return invalid_limits;
  for(const auto* id:{&r.route_id,&r.invocation_id,&r.model_digest,&r.map_digest,&r.registry_digest})
    if(id->empty() || id->size()>256)return "missing or oversized route identity";
  if(!ValidMotionPose(r.measured_start) || !ValidMotionPose(r.coarse_start) || !ValidMotionPose(r.goal) ||
     !Nonnegative(r.estimated_time_s) || r.segments.empty() || r.segments.size()>20000)return "invalid route header";
  std::size_t count=0;double total=0;
  MotionPose previous=r.measured_start;
  bool previous_stop=true;MotionKind previous_kind=r.segments.front().kind;
  bool departed=false;
  for(std::size_t i=0;i<r.segments.size();++i) {
    const auto& seg=r.segments[i];count+=seg.samples.size();
    if(count>20000 || seg.samples.empty() || !Nonnegative(seg.estimated_time_s) ||
       !Nonnegative(seg.geometry_error_m) || seg.geometry_error_m>.05 ||
       !Nonnegative(seg.curvature_bound_per_m) || seg.curvature_bound_per_m>1/l.min_turn_radius_m+kEpsilon)
      return "invalid segment size/time/error";
    if(!SameMotionPose(previous,seg.samples.front().pose))return "discontinuous segment boundary";
    if(i==0 && !seg.stop_before)return "route must start stopped";
    if(i>0 && (previous_stop!=seg.stop_before ||
       (previous_kind!=seg.kind && (!previous_stop || !seg.stop_before))))return "missing transition stop";
    if(i>0 && !previous_stop && std::abs(r.segments[i-1].samples.back().curvature_per_m-
                                        seg.samples.front().curvature_per_m)>kEpsilon)
      return "curvature discontinuity while moving";
    if(seg.stop_before && !Stopped(seg.samples.front()))return "nonzero entry at required stop";
    if(seg.stop_after && !Stopped(seg.samples.back()))return "nonzero exit at required stop";
    if(seg.samples.front().time_s!=0 || std::abs(seg.samples.back().time_s-seg.estimated_time_s)>kEpsilon)
      return "segment sample clock mismatch";
    if(seg.kind==MotionKind::DockDeparture) {
      if(i!=0 || departed)return "departure must be a single prefix";
      departed=true;
    } else if(seg.kind!=MotionKind::Forward && seg.kind!=MotionKind::Spin && seg.kind!=MotionKind::VerifyStop)
      return "unsupported motion kind";
    if(seg.kind==MotionKind::VerifyStop) {
      if(i+1!=r.segments.size() || seg.samples.size()!=1 || seg.estimated_time_s!=0 ||
          !seg.stop_before || !seg.stop_after || !Stopped(seg.samples.front()))return "invalid VerifyStop";
    } else if(seg.samples.size()<2 || seg.estimated_time_s<=0)return "empty movement segment";
    double last_time=-1,spin_direction=0;MotionPose last=seg.samples.front().pose;
    for(const auto& s:seg.samples) {
      if(!ValidMotionPose(s.pose) || !std::isfinite(s.curvature_per_m) ||
         !Nonnegative(s.speed_ceiling_mps) || !Nonnegative(s.yaw_ceiling_radps) ||
         !Nonnegative(s.time_s) || s.time_s<=last_time ||
         s.speed_ceiling_mps>l.max_speed_mps+kEpsilon || s.yaw_ceiling_radps>l.max_yaw_rate_radps+kEpsilon)
        return "invalid motion sample";
      if(seg.kind==MotionKind::Spin || seg.kind==MotionKind::VerifyStop) {
        if(s.curvature_per_m!=0 || s.speed_ceiling_mps!=0 ||
           std::hypot(s.pose.x_m-last.x_m,s.pose.y_m-last.y_m)>kEpsilon)return "translation in stationary segment";
      } else {
        if(std::abs(s.curvature_per_m)>1/l.min_turn_radius_m+kEpsilon ||
           s.speed_ceiling_mps*s.speed_ceiling_mps*std::abs(s.curvature_per_m)>l.lateral_acceleration_mps2+kEpsilon)
          return "curvature or lateral acceleration exceeded";
      }
      if(last_time>=0) {
        const double dt=s.time_s-last_time;
        const double dyaw=MotionAngleDifference(s.pose.yaw_rad,last.yaw_rad);
        const double midyaw=last.yaw_rad+dyaw/2;
        const double longitudinal=(s.pose.x_m-last.x_m)*std::cos(midyaw)+
                                  (s.pose.y_m-last.y_m)*std::sin(midyaw);
        if(seg.kind==MotionKind::Forward && longitudinal < -kEpsilon)return "reverse in forward segment";
        if(seg.kind==MotionKind::DockDeparture && (longitudinal>kEpsilon ||
           std::abs(dyaw)>kEpsilon || s.curvature_per_m!=0))return "invalid reverse departure";
        if(seg.kind==MotionKind::Spin && std::abs(dyaw)>kEpsilon) {
          if(spin_direction*dyaw<0)return "spin direction reversed inside segment";
          spin_direction=dyaw;
        }
        if(std::hypot(s.pose.x_m-last.x_m,s.pose.y_m-last.y_m)>l.max_speed_mps*dt+kEpsilon ||
           std::abs(MotionAngleDifference(s.pose.yaw_rad,last.yaw_rad))>l.max_yaw_rate_radps*dt+kEpsilon)
          return "sample displacement exceeds motion limits";
      }
      last_time=s.time_s;last=s.pose;
    }
    if(i==0 && !SameMotionPose(seg.kind==MotionKind::DockDeparture?seg.samples.back().pose:r.measured_start,r.coarse_start))
      return "coarse start does not match departure endpoint";
    previous=seg.samples.back().pose;previous_stop=seg.stop_after;previous_kind=seg.kind;total+=seg.estimated_time_s;
  }
  if(!previous_stop || !SameMotionPose(previous,r.goal) || std::abs(total-r.estimated_time_s)>kEpsilon)
    return "invalid terminal pose/stop/time";
  return {};
}
}  // namespace agx_workbench_navigation
