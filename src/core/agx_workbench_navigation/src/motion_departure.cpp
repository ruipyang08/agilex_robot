#include <agx_workbench_navigation/motion_departure.hpp>
#include <agx_workbench_navigation/motion_speed_profile.hpp>
#include <algorithm>
#include <cmath>

namespace agx_workbench_navigation {
bool ObservedDepartureStop(const std::vector<MotionDeparturePose>& samples,double latest) {
  if(samples.size()<2 || samples.size()>64 || !std::isfinite(latest) || latest<=0 ||
     samples.back().source_stamp_s!=latest)return false;
  std::size_t begin=samples.size()-1;
  while(begin>0 && latest-samples[begin].source_stamp_s<.5)--begin;
  if(latest-samples[begin].source_stamp_s<.5)return false;
  for(std::size_t i=begin;i<samples.size();++i) {
    const auto& sample=samples[i];
    if(!ValidMotionPose(sample.pose) || !std::isfinite(sample.source_stamp_s) || sample.source_stamp_s<=0)return false;
    if(i==begin)continue;
    const auto& previous=samples[i-1];const double dt=sample.source_stamp_s-previous.source_stamp_s;
    if(dt<=0 || dt>.25 || std::hypot(sample.pose.x_m-previous.pose.x_m,sample.pose.y_m-previous.pose.y_m)/dt>.02 ||
       std::abs(MotionAngleDifference(sample.pose.yaw_rad,previous.pose.yaw_rad))/dt>.01)return false;
  }
  return true;
}
MotionDepartureResult AdmitRegisteredDeparture(const MotionGrid& grid,
    const MotionDeparturePolicy& policy, const std::vector<RegisteredMotionDeparture>& stations,
    const std::string& source_id, const MotionPose& start, const MotionPose& endpoint) {
  auto fail=[](RouteStatus status,const char* detail) {
    return MotionDepartureResult{status,{},detail};
  };
  if(!policy.enabled)return fail(RouteStatus::EvidenceUnavailable,"registered departure phase is not installed");
  if(!ValidMotionPose(start) || !ValidMotionPose(endpoint) || source_id.empty() || stations.size()>64 ||
      !ValidMotionGrid(grid) || !ValidMotionPolygon(policy.geometry.footprint) ||
      !std::isfinite(policy.max_distance_m) || policy.max_distance_m<=0 || policy.max_distance_m>2 ||
      !std::isfinite(policy.max_speed_mps) || policy.max_speed_mps<=0 || policy.max_speed_mps>.05 ||
      !std::isfinite(policy.acceleration_mps2) || policy.acceleration_mps2<=0 || policy.acceleration_mps2>.1 ||
      !std::isfinite(policy.max_cross_track_m) || policy.max_cross_track_m<=0 || policy.max_cross_track_m>.025 ||
      !std::isfinite(policy.max_yaw_drift_rad) || policy.max_yaw_drift_rad<=0 || policy.max_yaw_drift_rad>.035 ||
      !std::isfinite(policy.stop_dwell_s) || policy.stop_dwell_s<.5)
    return fail(RouteStatus::EvidenceUnavailable,"invalid installed departure phase");
  const RegisteredMotionDeparture* source=nullptr;std::size_t nearby=0;
  for(const auto& station:stations) {
    if(!ValidMotionPose(station.marker) || !ValidMotionPose(station.staging))
      return fail(RouteStatus::EvidenceUnavailable,"invalid installed departure station");
    if(std::hypot(start.x_m-station.marker.x_m,start.y_m-station.marker.y_m)<=.5) {
      ++nearby;if(station.station_id==source_id)source=&station;
    }
  }
  if(nearby!=1 || !source || std::abs(MotionAngleDifference(start.yaw_rad,source->marker.yaw_rad))>.2)
    return fail(RouteStatus::InvalidRequest,"source is not the unique measured registered dock");
  const double c=std::cos(start.yaw_rad),s=std::sin(start.yaw_rad);
  const double dx=source->staging.x_m-start.x_m,dy=source->staging.y_m-start.y_m;
  const double distance=-(c*dx+s*dy),cross=-s*dx+c*dy;
  if(distance<.5 || distance>policy.max_distance_m || std::abs(cross)>.15 ||
      !SameMotionPose(endpoint,{start.x_m-c*distance,start.y_m-s*distance,start.yaw_rad}))
    return fail(RouteStatus::InvalidRequest,"departure endpoint does not match the registered reverse axis");
  MotionLimits limits;limits.max_speed_mps=policy.max_speed_mps;
  limits.acceleration_mps2=policy.acceleration_mps2;limits.deceleration_mps2=policy.acceleration_mps2;
  limits.stop_dwell_s=policy.stop_dwell_s;
  auto segment=MakeForwardPrimitive(start,0,0,distance,0,0,limits.max_speed_mps,limits,
                                    policy.geometry.translation_step_m);
  if(segment.samples.empty())return fail(RouteStatus::EvidenceUnavailable,"departure profile exceeds supported bounds");
  segment.kind=MotionKind::DockDeparture;
  std::vector<MotionPose> poses;
  for(auto& sample:segment.samples) {
    sample.pose.x_m=2*start.x_m-sample.pose.x_m;
    sample.pose.y_m=2*start.y_m-sample.pose.y_m;
    poses.push_back(sample.pose);
  }
  auto geometry=policy.geometry;
  double radius=0;for(const auto& xy:geometry.footprint)radius=std::max(radius,std::hypot(xy[0],xy[1]));
  // The installed controller may deviate within these existing bounds. Reserve
  // their swept displacement as well as the original rear stopping clearance.
  geometry.hard_padding_m+=policy.max_cross_track_m+2*radius*std::sin(policy.max_yaw_drift_rad/2);
  if(!CheckMotionSweep(grid,geometry,poses).clear())
    return fail(RouteStatus::NoRoute,"registered reverse corridor is blocked or unknown");
  return {RouteStatus::Ready,std::move(segment),"registered directional reverse corridor checked"};
}
}  // namespace agx_workbench_navigation
