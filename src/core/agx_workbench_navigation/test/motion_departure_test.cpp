#include <agx_workbench_navigation/motion_departure.hpp>
#include <cassert>
#include <agx_workbench_navigation/motion_preflight.hpp>
#include <agx_workbench_navigation/motion_route_follower.hpp>
namespace nav=agx_workbench_navigation;
void CompleteRouteLifecycle() {
  double now=10.;nav::MotionRouteStore store([&]{return now;});
  nav::MotionPreflightConfig config;config.model_digest="model";config.frame_id="map";
  config.geometry.hard_padding_m=.02;config.search.max_expansions=1;
  nav::MotionPreflight preflight(config,store,[&]{return now;});
  nav::MotionPlanningEvidence evidence;
  evidence.grid.width=160;evidence.grid.height=100;evidence.grid.resolution_m=.05;
  evidence.grid.origin_x_m=-5;evidence.grid.origin_y_m=-2.5;evidence.grid.cells.assign(16000,0);
  evidence.frame_id="map";evidence.static_map_digest="map";evidence.registry_digest="registry";
  evidence.observed_at_s=now;evidence.pose_deadline_s=evidence.obstacle_deadline_s=now+1;
  evidence.departure_policy.enabled=true;
  evidence.departure_policy.geometry.footprint={{{-.5,-.33}},{{0,-.33}},{{0,.33}},{{-.5,.33}}};
  evidence.departure_policy.geometry.hard_padding_m=.25;
  evidence.registered_departures={{"a",{.36,0,0},{-1,0,0}}};
  nav::MotionPreflightRequest request;request.invocation_id="two";
  request.model_digest="model";request.registry_digest="registry";request.frame_id="map";
  request.departure_source_id="a";request.coarse_start={-1,0,0};request.goal={0,0,0};
  auto source=[&]{return evidence;};
  auto result=preflight.Plan(request,source);assert(result.status==nav::RouteStatus::Ready);
  assert(result.route.segments.front().kind==nav::MotionKind::DockDeparture);
  assert(nav::ValidateMotionRoute(result.route,config.limits).empty());
  const auto id=result.route.route_id;request.existing_route_id=id;
  assert(store.pin(id,"two",100));
  nav::CommandBinding binding{"session","grant","goal",{"map",1,{0,0,0},{0,0,0,1}}};
  binding.route_id=id;binding.invocation_id="two";binding.task_deadline_s=100;
  assert(!store.bind(binding)); // Planning/pinning cannot prove the prefix completed.
  evidence.measured_pose=request.coarse_start;
  assert(preflight.CompleteDeparture(id,"two",source).status==nav::RouteStatus::StartMismatch);
  evidence.departure_stop_observed=true;
  evidence.pose_deadline_s=now;
  assert(preflight.CompleteDeparture(id,"two",source).status==nav::RouteStatus::EvidenceUnavailable);
  evidence.pose_deadline_s=now+1;
  assert(preflight.CompleteDeparture(id,"foreign",source).status==nav::RouteStatus::RouteInvalidated);
  assert(preflight.CompleteDeparture(id,"two",source).status==nav::RouteStatus::Ready);
  assert(store.bind(binding));
  nav::FollowerObservation observation;observation.pose=request.coarse_start;
  observation.source_stamp_s=1;observation.revision=1;
  nav::MotionRouteFollower uncompleted(result.route,config.limits,.02);
  assert(!uncompleted.Tick(observation,1).ok);
  nav::MotionRouteFollower completed(result.route,config.limits,.02,.25,.005,.015,1);
  const auto command=completed.Tick(observation,1);
  assert(command.ok && command.segment==1 && command.vx_mps==0);
  std::size_t segment=0;assert(store.boundGoal(binding.endpoint,&segment));assert(segment==1);
  store.release(id,"two");
  assert(preflight.CompleteDeparture(id,"two",source).status==nav::RouteStatus::RouteInvalidated);
  // No onward route means the dock start retains no executable prefix/token.
  request.existing_route_id="";evidence.measured_pose={};evidence.departure_stop_observed=false;
  evidence.grid.cells[50*160+110]=254;
  result=preflight.Plan(request,source);
  assert(result.status!=nav::RouteStatus::Ready && result.route.segments.empty());
  evidence.grid.cells[50*160+110]=0;
  result=preflight.Plan(request,source,[]{return true;});
  assert(result.status==nav::RouteStatus::RouteInvalidated && result.route.segments.empty());
  result=preflight.Plan(request,source);assert(result.status==nav::RouteStatus::Ready);
  const auto expiring=result.route.route_id;assert(store.pin(expiring,"two",now+.1));
  now+=.2;evidence.observed_at_s=now;evidence.pose_deadline_s=evidence.obstacle_deadline_s=now+1;
  evidence.measured_pose=request.coarse_start;evidence.departure_stop_observed=true;
  assert(preflight.CompleteDeparture(expiring,"two",source).status==nav::RouteStatus::RouteInvalidated);
}
int main() {
  CompleteRouteLifecycle();
  std::vector<nav::MotionDeparturePose> stopped;
  for(int i=0;i<=10;++i)stopped.push_back({{},10+i*.05});
  assert(nav::ObservedDepartureStop(stopped,10.5));
  auto moving=stopped;for(std::size_t i=0;i<moving.size();++i)moving[i].pose.x_m=.1*i;
  assert(!nav::ObservedDepartureStop(moving,10.5));
  auto repeated=stopped;repeated.back().source_stamp_s=repeated[9].source_stamp_s;
  assert(!nav::ObservedDepartureStop(repeated,repeated.back().source_stamp_s));
  auto rotating=stopped;for(std::size_t i=0;i<rotating.size();++i)rotating[i].pose.yaw_rad=.01*i;
  assert(!nav::ObservedDepartureStop(rotating,10.5));
  assert(!nav::ObservedDepartureStop(stopped,11));
  nav::MotionGrid grid;grid.width=160;grid.height=100;grid.resolution_m=.05;
  grid.origin_x_m=-5;grid.origin_y_m=-2.5;grid.cells.assign(16000,0);
  nav::RegisteredMotionDeparture station{"a",{.36,0,0},{-1,0,0}};
  nav::MotionDeparturePolicy policy;
  policy.enabled=true;
  policy.geometry.footprint={{{-.5,-.33}},{{0,-.33}},{{0,.33}},{{-.5,.33}}};
  policy.geometry.hard_padding_m=.25;
  auto result=nav::AdmitRegisteredDeparture(grid,policy,{station},"a",{0,0,0},{-1,0,0});
  assert(result.status==nav::RouteStatus::Ready);
  assert(result.segment.kind==nav::MotionKind::DockDeparture);
  assert(nav::SameMotionPose(result.segment.samples.front().pose,{0,0,0}));
  assert(nav::SameMotionPose(result.segment.samples.back().pose,{-1,0,0}));
  grid.cells[50*160+84]=254; // Rear corridor obstacle at (-.8,0).
  assert(nav::AdmitRegisteredDeparture(grid,policy,{station},"a",{0,0,0},{-1,0,0}).status==nav::RouteStatus::NoRoute);
  grid.cells[50*160+84]=0;
  grid.cells[50*160+84]=255; // Unknown rear corridor cannot authorize reverse.
  assert(nav::AdmitRegisteredDeparture(grid,policy,{station},"a",{0,0,0},{-1,0,0}).status==nav::RouteStatus::NoRoute);
  grid.cells[50*160+84]=0;
  assert(nav::AdmitRegisteredDeparture(grid,policy,{station},"a",{0,0,0},{-1,.01,0}).status==nav::RouteStatus::InvalidRequest);
  assert(nav::AdmitRegisteredDeparture(grid,policy,{station},"foreign",{0,0,0},{-1,0,0}).status!=nav::RouteStatus::Ready);
  policy.enabled=false;
  assert(nav::AdmitRegisteredDeparture(grid,policy,{station},"a",{0,0,0},{-1,0,0}).status==nav::RouteStatus::EvidenceUnavailable);
}
