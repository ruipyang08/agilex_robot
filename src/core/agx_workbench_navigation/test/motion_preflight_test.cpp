#include <agx_workbench_navigation/motion_preflight.hpp>
#include <cassert>
#include <cmath>
#include <stdexcept>
namespace nav=agx_workbench_navigation;
int main() {
  double now=10.;nav::MotionRouteStore store([&] {return now;});
  nav::MotionPreflightConfig config;config.model_digest="model";config.frame_id="map";
  config.geometry.hard_padding_m=.02;config.geometry.soft_clearance_m=0;
  config.search.max_expansions=1;
  nav::MotionPlanningEvidence evidence;
  evidence.grid.width=100;evidence.grid.height=100;evidence.grid.resolution_m=.1;
  evidence.grid.origin_x_m=-5;evidence.grid.origin_y_m=-5;evidence.grid.cells.assign(10000,0);
  evidence.frame_id="map";evidence.static_map_digest="map-a";
  evidence.registry_digest="registry-a";evidence.pose_deadline_s=20;
  evidence.obstacle_deadline_s=20;evidence.observed_at_s=10;
  nav::MotionPreflightRequest request;request.invocation_id="invocation-a";
  request.model_digest="model";request.registry_digest="registry-a";request.frame_id="map";
  request.goal={1,0,0};
  nav::MotionPreflight preflight(config,store,[&] {return now;});
  int reads=0;auto source=[&] {++reads;return evidence;};
  auto result=preflight.Plan(request,source);
  assert(result.status==nav::RouteStatus::Ready && reads>=2);
  const auto id=result.route.route_id;
  assert(store.get(id,"invocation-a"));
  assert(!store.get(id,"invocation-b"));
  request.existing_route_id=id;
  auto repeated=preflight.Plan(request,source);
  assert(repeated.status==nav::RouteStatus::Ready && repeated.route.route_id==id);
  // A dynamic change outside the route does not invalidate it.
  evidence.grid.cells[0]=254;
  assert(preflight.Plan(request,source).status==nav::RouteStatus::Ready);
  // But an obstacle in the retained route invalidates the token.
  evidence.grid.cells[50*100+56]=254;
  result=preflight.Plan(request,source);
  assert(result.status==nav::RouteStatus::RouteInvalidated && result.route.segments.empty());
  assert(!store.get(id,"invocation-a"));
  evidence.grid.cells[50*100+56]=0;request.existing_route_id="";
  request.invocation_id="invocation-b";
  result=preflight.Plan(request,source);assert(result.status==nav::RouteStatus::Ready);
  const auto second_id=result.route.route_id;assert(second_id!=id);
  request.existing_route_id=second_id;evidence.static_map_digest="map-b";
  assert(preflight.Plan(request,source).status==nav::RouteStatus::RouteInvalidated);
  assert(!store.get(second_id,"invocation-b"));
  request.existing_route_id="";evidence.static_map_digest="map-a";
  evidence.pose_deadline_s=now;
  assert(preflight.Plan(request,source).status==nav::RouteStatus::EvidenceUnavailable);
  evidence.pose_deadline_s=20;evidence.observed_at_s=now+1;
  assert(preflight.Plan(request,source).status==nav::RouteStatus::EvidenceUnavailable);
  evidence.observed_at_s=now;evidence.measured_pose={.2,0,0};
  assert(preflight.Plan(request,source).status==nav::RouteStatus::StartMismatch);
  evidence.measured_pose={};
  request.model_digest="other";reads=0;
  assert(preflight.Plan(request,source).status==nav::RouteStatus::InvalidRequest && reads==0);
  request.model_digest="model";
  // Evidence can change while searching, without yielding an admitted token.
  reads=0;auto changing=[&] {auto e=evidence;if(++reads>1)e.grid.cells[50*100+56]=254;return e;};
  result=preflight.Plan(request,changing);
  assert(result.status==nav::RouteStatus::RouteInvalidated && result.route.segments.empty());
  reads=0;auto changed_registry=[&] {auto e=evidence;if(++reads>1)e.registry_digest="new";return e;};
  assert(preflight.Plan(request,changed_registry).status==nav::RouteStatus::RouteInvalidated);
  bool cancelled=false;reads=0;
  auto cancel_after_read=[&] {++reads;if(reads>1)cancelled=true;return evidence;};
  result=preflight.Plan(request,cancel_after_read,[&] {return cancelled;});
  assert(result.status==nav::RouteStatus::RouteInvalidated && result.route.segments.empty());
  request.coarse_start={1,0,0};
  assert(preflight.Plan(request,source).status==nav::RouteStatus::EvidenceUnavailable);
  request.coarse_start={};
  auto expired=[&] {auto e=evidence;e.obstacle_deadline_s=now;return e;};
  assert(preflight.Plan(request,expired).status==nav::RouteStatus::EvidenceUnavailable);
  // A source failure cannot leak a candidate or partial route.
  auto broken=[]() -> nav::MotionPlanningEvidence {throw std::runtime_error("sensor disconnected");};
  assert(preflight.Plan(request,broken).status==nav::RouteStatus::EvidenceUnavailable);
  // Evidence expiring specifically while the store reads its clock is caught
  // after retention; the failed candidate must be removed again.
  double store_now=10;bool retaining=false;
  nav::MotionRouteStore changing_clock_store([&] {if(retaining)store_now=20;return store_now;});
  nav::MotionPreflight second(config,changing_clock_store,[&] {return store_now;});
  auto expires_while_retaining=[&] {auto e=evidence;retaining=true;return e;};
  result=second.Plan(request,expires_while_retaining);
  assert(result.status==nav::RouteStatus::EvidenceUnavailable && result.route.segments.empty());
  // A pose within the allowed start tolerance can still collide. Checking
  // only the nominal route would admit a rover already overlapping a cell.
  auto near_wall=[&] {
    auto e=evidence;e.grid.origin_x_m=-5.025;e.grid.cells.assign(10000,0);
    e.grid.cells[50*100+44]=254;e.measured_pose={-.009,0,0};return e;
  };
  result=preflight.Plan(request,near_wall);
  assert(result.status==nav::RouteStatus::StartMismatch && result.route.segments.empty());
  // Expired leases are not recreated under the same existing-route request.
  result=preflight.Plan(request,source);assert(result.status==nav::RouteStatus::Ready);
  request.existing_route_id=result.route.route_id;now=71;
  evidence.observed_at_s=now;evidence.pose_deadline_s=now+1;evidence.obstacle_deadline_s=now+1;
  assert(preflight.Plan(request,source).status==nav::RouteStatus::RouteInvalidated);
}
