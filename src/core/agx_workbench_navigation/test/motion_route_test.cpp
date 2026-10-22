#include <agx_workbench_navigation/motion_route.hpp>
#include <cassert>
#include <cmath>
namespace nav=agx_workbench_navigation;
nav::MotionRoute route() {
  nav::MotionRoute r;
  r.route_id="route";r.invocation_id="invocation";r.model_digest="model";
  r.map_digest="map";r.registry_digest="registry";
  r.goal={1,0,0};r.estimated_time_s=6;
  nav::MotionSegment s;s.estimated_time_s=6;
  s.samples={{{0,0,0},0,0,0,0},{{.5,0,0},0,.25,0,3},{{1,0,0},0,0,0,6}};
  r.segments.push_back(s);return r;
}
int main() {
  nav::MotionLimits limits;
  assert(nav::ValidateMotionRoute(route(),limits).empty());
  auto r=route();r.segments.clear();assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.route_id.clear();assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.goal.y_m=.1;assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.segments[0].samples[1].speed_ceiling_mps=-.1;assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.segments[0].samples[1].curvature_per_m=10;assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.segments[0].samples[1].time_s=7;assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.segments[0].samples[0].speed_ceiling_mps=.1;assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.segments[0].kind=nav::MotionKind::Spin;assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.segments.push_back(r.segments[0]);assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.coarse_start.x_m=.1;assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.segments[0].kind=static_cast<nav::MotionKind>(99);assert(!nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.goal.x_m=-1;r.segments[0].samples[1].pose.x_m=-.5;r.segments[0].samples[2].pose.x_m=-1;
  assert(!nav::ValidateMotionRoute(r,limits).empty()); // Reverse is not Forward.
  r.segments[0].kind=nav::MotionKind::DockDeparture;r.coarse_start=r.goal;
  assert(nav::ValidateMotionRoute(r,limits).empty());
  r=route();r.segments[0].kind=nav::MotionKind::Spin;r.goal={0,0,.1};
  r.segments[0].samples={{{0,0,0},0,0,0,0},{{0,0,.2},0,0,.2,3},{{0,0,.1},0,0,0,6}};
  assert(!nav::ValidateMotionRoute(r,limits).empty()); // No hidden spin reversal.
  r=route();r.goal={};r.estimated_time_s=0;
  r.segments[0].kind=nav::MotionKind::VerifyStop;
  r.segments[0].samples.resize(1);r.segments[0].estimated_time_s=0;
  assert(nav::ValidateMotionRoute(r,limits).empty());
  r.segments[0].stop_after=false;assert(!nav::ValidateMotionRoute(r,limits).empty());
  limits.min_turn_radius_m=0;assert(!nav::ValidateMotionLimits(limits).empty());
}
