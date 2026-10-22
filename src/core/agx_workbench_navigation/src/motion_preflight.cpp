#include <agx_workbench_navigation/motion_preflight.hpp>
#include <cmath>
#include <random>
#include <sstream>
#include <stdexcept>

namespace agx_workbench_navigation {
namespace {
bool Identifier(const std::string& value) {return !value.empty() && value.size()<=128;}
bool StartClear(const MotionPlanningEvidence& evidence,const MotionGeometryPolicy& geometry,
                const MotionPose& nominal) {
  // Cover the admitted tracking-offset region as well as the nominal start.
  // This is a collision check, not an executable lateral/reverse connector.
  std::vector<MotionPose> poses{evidence.measured_pose};
  if(!SameMotionPose(evidence.measured_pose,nominal))poses.push_back(nominal);
  return CheckMotionSweep(evidence.grid,geometry,poses).clear();
}
bool Clear(const MotionPlanningEvidence& evidence,const MotionGeometryPolicy& geometry,
           const MotionRoute& route,bool include_departure=true) {
  const auto& grid=evidence.grid;
  const MotionSweepChecker checker(grid);
  for(const auto& segment:route.segments) {
    if(segment.kind==MotionKind::DockDeparture) {
      if(include_departure && AdmitRegisteredDeparture(grid,evidence.departure_policy,
          evidence.registered_departures,route.departure_source_id,route.measured_start,
          route.coarse_start).status!=RouteStatus::Ready)return false;
      continue;
    }
    auto policy=geometry;policy.integration_error_m+=segment.geometry_error_m;
    std::vector<MotionPose> poses;poses.reserve(segment.samples.size());
    for(const auto& sample:segment.samples)poses.push_back(sample.pose);
    if(!checker.Check(policy,poses).clear())return false;
  }
  return true;
}
}
MotionPreflight::MotionPreflight(MotionPreflightConfig config,MotionRouteStore& store,
    std::function<double()> clock):config_(std::move(config)),store_(store),clock_(std::move(clock)) {
  if(!clock_ || !Identifier(config_.model_digest) || !Identifier(config_.frame_id) ||
     !ValidateMotionLimits(config_.limits).empty() || !ValidMotionPolygon(config_.geometry.footprint) ||
     !std::isfinite(config_.start_position_tolerance_m) || config_.start_position_tolerance_m<0 ||
     !std::isfinite(config_.start_yaw_tolerance_rad) || config_.start_yaw_tolerance_rad<0)
    throw std::invalid_argument("invalid preflight configuration");
  std::random_device random;std::ostringstream id;id<<std::hex;
  for(int i=0;i<4;++i)id<<random()<<'-';session_=id.str();
}
bool MotionPreflight::Fresh(const MotionPlanningEvidence& e,double now) const {
  return std::isfinite(now) && now>=0 && std::isfinite(e.observed_at_s) && e.observed_at_s>=0 &&
      e.observed_at_s<=now && std::isfinite(e.pose_deadline_s) && e.pose_deadline_s>now &&
      std::isfinite(e.obstacle_deadline_s) && e.obstacle_deadline_s>now &&
      e.frame_id==config_.frame_id && Identifier(e.static_map_digest) && Identifier(e.registry_digest) &&
      ValidMotionPose(e.measured_pose) && ValidMotionGrid(e.grid) && e.grid.cells.size()<=1000000;
}
bool MotionPreflight::StartMatches(const MotionPose& a,const MotionPose& b) const {
  return std::hypot(a.x_m-b.x_m,a.y_m-b.y_m)<=config_.start_position_tolerance_m &&
      std::abs(MotionAngleDifference(a.yaw_rad,b.yaw_rad))<=config_.start_yaw_tolerance_rad;
}
RouteResult MotionPreflight::Plan(const MotionPreflightRequest& request,const EvidenceSource& source,
    const std::function<bool()>& cancelled) {
  std::unique_lock<std::mutex> work(work_mutex_,std::try_to_lock);
  if(!work.owns_lock())return {RouteStatus::EvidenceUnavailable,{},"route worker is busy"};
  std::string candidate_id;
  auto fail=[&](RouteStatus status,const std::string& detail) {
    if(!candidate_id.empty())store_.release(candidate_id,request.invocation_id);
    if(!request.existing_route_id.empty())store_.release(request.existing_route_id,request.invocation_id);
    return RouteResult{status,{},detail};
  };
  try {
    auto interrupted=[&] {return cancelled && cancelled();};
    if(interrupted())return fail(RouteStatus::RouteInvalidated,"planning cancelled");
    if(!source || !Identifier(request.invocation_id) || request.existing_route_id.size()>128 ||
       request.model_digest!=config_.model_digest || !Identifier(request.registry_digest) ||
       request.frame_id!=config_.frame_id || !ValidMotionPose(request.measured_start) ||
       !ValidMotionPose(request.coarse_start) || !ValidMotionPose(request.goal))
      return fail(RouteStatus::InvalidRequest,"invalid request or installed model/frame mismatch");
    const bool departing=!request.departure_source_id.empty();
    if(!request.departure_prefix.empty())
      return fail(RouteStatus::InvalidRequest,"departure samples must come from the installed phase owner");
    if(!departing && !SameMotionPose(request.measured_start,request.coarse_start))
      return fail(RouteStatus::EvidenceUnavailable,"registered departure source is missing");
    const auto first=source();const double began=clock_();
    if(!Fresh(first,began))return fail(RouteStatus::EvidenceUnavailable,
        "map, pose or obstacle evidence is unavailable or stale: "+first.unavailable_detail.substr(0,256));
    if(first.registry_digest!=request.registry_digest)
      return fail(RouteStatus::RouteInvalidated,"registered installation changed");
    if(!StartMatches(first.measured_pose,request.measured_start) ||
       (!departing && !StartClear(first,config_.geometry,request.measured_start)))
      return fail(RouteStatus::StartMismatch,"measured start or its bounded offset is unavailable");
    MotionDepartureResult departure;
    if(departing) {
      departure=AdmitRegisteredDeparture(first.grid,first.departure_policy,
          first.registered_departures,request.departure_source_id,request.measured_start,request.coarse_start);
      if(departure.status!=RouteStatus::Ready)return fail(departure.status,departure.detail);
    }
    // Every coarse join must fit the ordinary full travel outline.
    if(departing) {
      auto join=first;join.measured_pose=request.coarse_start;
      if(!StartClear(join,config_.geometry,request.coarse_start))
        return fail(RouteStatus::NoRoute,"registered departure join is blocked");
    }
    MotionRoute route;
    if(!request.existing_route_id.empty()) {
      auto retained=store_.get(request.existing_route_id,request.invocation_id);
      if(!retained)return fail(RouteStatus::RouteInvalidated,"route token is missing, expired or belongs to another invocation");
      route=*retained;
      if(route.model_digest!=config_.model_digest || route.map_digest!=first.static_map_digest ||
         route.registry_digest!=first.registry_digest || !SameMotionPose(route.goal,request.goal) ||
         !SameMotionPose(route.measured_start,request.measured_start) ||
         !SameMotionPose(route.coarse_start,request.coarse_start) ||
         route.departure_source_id!=request.departure_source_id)
        return fail(RouteStatus::RouteInvalidated,"stored route identities or endpoints changed");
      if(!Clear(first,config_.geometry,route))
        return fail(RouteStatus::RouteInvalidated,"stored route is blocked by current occupancy");
    } else {
      route.route_id=session_+std::to_string(++next_route_);route.invocation_id=request.invocation_id;
      route.model_digest=config_.model_digest;route.map_digest=first.static_map_digest;
      route.registry_digest=first.registry_digest;route.measured_start=request.measured_start;
      route.coarse_start=request.coarse_start;route.goal=request.goal;
      auto search=config_.search;search.cancelled=interrupted;
      auto coarse=route;coarse.measured_start=request.coarse_start;
      auto found=SearchMotionRoute(first.grid,config_.geometry,config_.limits,coarse,search);
      if(found.result.status!=RouteStatus::Ready)return fail(found.result.status,found.result.detail);
      route=std::move(found.result.route);route.measured_start=request.measured_start;
      route.departure_source_id=request.departure_source_id;
      if(departing) {
        route.segments.insert(route.segments.begin(),std::move(departure.segment));
        route.estimated_time_s+=route.segments.front().estimated_time_s;
      }
      if(!ValidateMotionRoute(route,config_.limits).empty())
        return fail(RouteStatus::EvidenceUnavailable,"invalid departure-to-travel join");
    }
    if(interrupted())return fail(RouteStatus::RouteInvalidated,"planning cancelled");
    const auto current=source();const double finished=clock_();
    if(interrupted())return fail(RouteStatus::RouteInvalidated,"planning cancelled");
    if(finished<began || !Fresh(current,finished))
      return fail(RouteStatus::EvidenceUnavailable,
          "evidence expired or clock regressed during planning: "+current.unavailable_detail.substr(0,256));
    if(current.static_map_digest!=first.static_map_digest || current.registry_digest!=first.registry_digest)
      return fail(RouteStatus::RouteInvalidated,"map or registration changed during planning");
    if(!StartMatches(current.measured_pose,route.measured_start) ||
       (!departing && !StartClear(current,config_.geometry,route.measured_start)))
      return fail(RouteStatus::StartMismatch,"rover moved during planning; request a new measured start");
    if(!Clear(current,config_.geometry,route))
      return fail(RouteStatus::RouteInvalidated,"route became blocked during planning");
    // Recheck age after the potentially expensive final geometry pass.
    const double validated=clock_();
    if(validated<finished || !Fresh(current,validated))
      return fail(RouteStatus::EvidenceUnavailable,"evidence expired during final validation");
    if(interrupted())return fail(RouteStatus::RouteInvalidated,"planning cancelled");
    if(request.existing_route_id.empty()) {
      candidate_id=route.route_id;
      if(!store_.put(route,config_.limits))return fail(RouteStatus::EvidenceUnavailable,"route store is full or unavailable");
    } else if(!store_.get(route.route_id,route.invocation_id)) {
      return fail(RouteStatus::RouteInvalidated,"route lease expired during validation");
    }
    const double committed=clock_();
    if(committed<validated || !Fresh(current,committed))
      return fail(RouteStatus::EvidenceUnavailable,"evidence expired while retaining the route");
    if(interrupted())return fail(RouteStatus::RouteInvalidated,"planning cancelled");
    return {RouteStatus::Ready,std::move(route),"complete travel route checked; no motion authority granted"};
  } catch(const std::exception&) {
    return fail(RouteStatus::EvidenceUnavailable,"route evidence or planning raised an exception");
  }
}
RouteResult MotionPreflight::CompleteDeparture(const std::string& id,const std::string& invocation,
    const EvidenceSource& source) {
  std::unique_lock<std::mutex> work(work_mutex_,std::try_to_lock);
  if(!work.owns_lock())return {RouteStatus::EvidenceUnavailable,{},"route worker is busy"};
  const auto route=store_.get(id,invocation);
  if(!route || route->departure_source_id.empty())
    return {RouteStatus::RouteInvalidated,{},"owned departure route is missing or expired"};
  try {
    const auto evidence=source();const double now=clock_();
    if(!Fresh(evidence,now))return {RouteStatus::EvidenceUnavailable,{},"departure endpoint evidence unavailable or stale"};
    if(route->model_digest!=config_.model_digest || route->registry_digest!=evidence.registry_digest ||
       route->map_digest!=evidence.static_map_digest)
      return {RouteStatus::RouteInvalidated,{},"departure installation changed"};
    if(!evidence.departure_stop_observed ||
       std::hypot(evidence.measured_pose.x_m-route->coarse_start.x_m,
                  evidence.measured_pose.y_m-route->coarse_start.y_m)>.02 ||
       std::abs(MotionAngleDifference(evidence.measured_pose.yaw_rad,route->coarse_start.yaw_rad))>.015)
      return {RouteStatus::StartMismatch,{},"departure endpoint has no bounded measured stop"};
    if(!StartClear(evidence,config_.geometry,route->coarse_start) ||
       !Clear(evidence,config_.geometry,*route,false))
      return {RouteStatus::RouteInvalidated,{},"onward route or departure connector is blocked"};
    if(!Fresh(evidence,clock_()) || !store_.completeDeparture(id,invocation))
      return {RouteStatus::RouteInvalidated,{},"departure evidence expired or route is not pinned to its owner"};
    return {RouteStatus::Ready,*route,"measured stopped departure endpoint and remaining route verified"};
  } catch(const std::exception&) {
    return {RouteStatus::EvidenceUnavailable,{},"departure endpoint evidence unavailable"};
  }
}
}  // namespace agx_workbench_navigation
