#include <agx_workbench_navigation/motion_plan_service.hpp>
#include <cmath>

namespace agx_workbench_navigation {
namespace {
bool Pose(const geometry_msgs::PoseStamped& source,const std::string& frame,MotionPose& pose) {
  const auto& p=source.pose.position;const auto& q=source.pose.orientation;
  if(source.header.frame_id!=frame || source.header.stamp.isZero() ||
     !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
     !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
     std::abs(q.x)>1e-6 || std::abs(q.y)>1e-6 ||
     std::abs(q.z*q.z+q.w*q.w-1)>1e-6)return false;
  pose={p.x,p.y,std::atan2(2*q.z*q.w,1-2*q.z*q.z)};
  return ValidMotionPose(pose);
}
geometry_msgs::Pose2D Pose(const MotionPose& source) {
  geometry_msgs::Pose2D pose;pose.x=source.x_m;pose.y=source.y_m;pose.theta=source.yaw_rad;return pose;
}
NavigationRoute Message(const MotionRoute& route,const std::string& frame) {
  NavigationRoute message;message.header.frame_id=frame;message.header.stamp=ros::Time::now();
  message.route_id=route.route_id;message.invocation_id=route.invocation_id;
  message.model_digest=route.model_digest;message.map_digest=route.map_digest;message.registry_digest=route.registry_digest;
  message.measured_start=Pose(route.measured_start);message.coarse_start=Pose(route.coarse_start);message.goal=Pose(route.goal);
  message.estimated_time_s=route.estimated_time_s;
  message.departure_source_id=route.departure_source_id;
  for(const auto& segment:route.segments) {
    PlannedMotionSegment s;s.kind=static_cast<std::uint8_t>(segment.kind);
    s.stop_before=segment.stop_before;s.stop_after=segment.stop_after;
    s.estimated_time_s=segment.estimated_time_s;s.geometry_error_m=segment.geometry_error_m;
    s.curvature_bound_per_m=segment.curvature_bound_per_m;
    for(const auto& sample:segment.samples) {
      PlannedMotionSample p;p.pose=Pose(sample.pose);p.curvature_per_m=sample.curvature_per_m;
      p.speed_ceiling_mps=sample.speed_ceiling_mps;p.yaw_ceiling_radps=sample.yaw_ceiling_radps;p.time_s=sample.time_s;
      s.samples.push_back(p);
    }
    message.segments.push_back(std::move(s));
  }
  return message;
}
}
MotionPlanService::MotionPlanService(ros::NodeHandle node,MotionPreflightConfig config,
    MotionRouteStore& store,MotionPreflight::EvidenceSource evidence,std::function<double()> clock)
    :node_(std::move(node)),planner_(config,store,std::move(clock)),store_(store),evidence_(std::move(evidence)),
     frame_(config.frame_id),spinner_(2,&queue_) {
  node_.setCallbackQueue(&queue_);
  service_=node_.advertiseService("plan_navigation_route",&MotionPlanService::Handle,this);
  management_=node_.advertiseService("manage_navigation_route",&MotionPlanService::Manage,this);
  spinner_.start();
}
MotionPlanService::~MotionPlanService() {
  stopping_=true;service_.shutdown();management_.shutdown();queue_.disable();spinner_.stop();queue_.clear();
}
bool MotionPlanService::Manage(ManageNavigationRoute::Request& request,ManageNavigationRoute::Response& response) {
  response.accepted=false;
  if(stopping_ || request.invocation_id.empty() || request.route_id.empty()) {
    response.reason="invalid or stopped route owner";return true;
  }
  if(request.release && request.complete_departure) {
    response.reason="conflicting route management operations";return true;
  }
  if(request.release) {
    store_.release(request.route_id,request.invocation_id);
    response.accepted=true;response.reason="owned route released";
  } else if(request.complete_departure) {
    const auto reply=planner_.CompleteDeparture(request.route_id,request.invocation_id,evidence_);
    response.accepted=reply.status==RouteStatus::Ready;response.reason=reply.detail;
  } else {
    response.accepted=store_.pin(request.route_id,request.invocation_id,request.task_deadline_s);
    response.reason=response.accepted?"route pinned to original task deadline; no motion authority":
        "route missing, expired, foreign or already pinned to another deadline";
  }
  return true;
}
bool MotionPlanService::Handle(PlanNavigationRoute::Request& req,PlanNavigationRoute::Response& res) {
  res=PlanNavigationRoute::Response();
  res.status=static_cast<std::uint8_t>(RouteStatus::InvalidRequest);
  res.detail="invalid pose, frame or unsupported departure request";
  // Source age is checked using owner evidence. Request stamps do not fabricate
  // freshness, and a request cannot provide its own map, geometry or clear flag.
  MotionPreflightRequest request;
  request.invocation_id=req.invocation_id;request.existing_route_id=req.existing_route_id;
  request.model_digest=req.model_digest;request.registry_digest=req.registry_digest;request.frame_id=frame_;
  request.departure_source_id=req.departure_source_id;
  if(!Pose(req.measured_start,frame_,request.measured_start) ||
     !Pose(req.coarse_start,frame_,request.coarse_start) || !Pose(req.goal,frame_,request.goal))return true;
  if(!req.departure_prefix.empty()) {
    res.detail="departure samples must come from the installed phase owner";return true;
  }
  const auto result=planner_.Plan(request,evidence_,[&] {return stopping_.load() || !ros::ok();});
  res.status=static_cast<std::uint8_t>(result.status);res.detail=result.detail;
  if(result.status==RouteStatus::Ready)res.route=Message(result.route,frame_);
  return true;
}
}  // namespace agx_workbench_navigation
