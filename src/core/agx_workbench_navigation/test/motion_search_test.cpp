#include <agx_workbench_navigation/motion_search.hpp>
#include <cassert>
#include <cmath>
namespace nav=agx_workbench_navigation;
int main() {
  nav::MotionGrid grid;grid.width=160;grid.height=160;grid.resolution_m=.05;
  grid.origin_x_m=-4;grid.origin_y_m=-4;grid.cells.assign(25600,0);
  nav::MotionGeometryPolicy geometry;geometry.hard_padding_m=.05;
  nav::MotionRoute request;request.route_id="r";request.invocation_id="i";
  request.model_digest="m";request.map_digest="map";request.registry_digest="reg";
  request.goal={2,1,0};nav::MotionLimits limits;nav::MotionSearchOptions options;
  options.max_expansions=30;options.timeout_s=1;
  auto result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::Ready);
  assert(nav::ValidateMotionRoute(result.result.route,limits).empty());
  bool curved=false;
  for(const auto& seg:result.result.route.segments) {
    assert(seg.kind!=nav::MotionKind::Spin);
    for(const auto& s:seg.samples)if(std::abs(s.curvature_per_m)>.01)curved=true;
  }
  assert(curved);
  // Reflections and translated bins must preserve feasible curved solutions.
  request.goal={2,-1,0};
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::Ready);
  request.measured_start={.026,.026,0};request.coarse_start=request.measured_start;request.goal={2.026,1.026,0};
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::Ready);
  request.measured_start={};request.coarse_start={};
  request.goal={0,0,std::acos(-1.)/2};
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::Ready);
  assert(result.result.route.segments.size()==1 && result.result.route.segments[0].kind==nav::MotionKind::Spin);
  request.goal={};result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::Ready);
  assert(result.result.route.segments[0].kind==nav::MotionKind::VerifyStop);
  request.goal={2,1,0};grid.cells[100*160+120]=254; // Inside goal footprint.
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::NoRoute && result.result.route.segments.empty());
  assert(result.collision_reported && result.result.detail=="goal footprint is blocked");
  assert(result.collision.cell_x==120 && result.collision.cell_y==100);
  grid.cells[100*160+120]=0;
  // Separating wall: a tiny search budget must not return a partial route.
  for(std::size_t y=0;y<grid.height;++y)grid.cells[y*grid.width+100]=254;
  options.max_expansions=1;
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::SearchLimit && result.result.route.segments.empty());
  grid.cells.assign(grid.cells.size(),0);
  options.cancelled=[] {return true;};
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status!=nav::RouteStatus::Ready && result.result.route.segments.empty());
  options.cancelled={};options.max_expansions=0;
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::InvalidRequest);
  // A checked minimum-radius quarter arc fits the .86 m L passage; an
  // in-place 90-degree rectangle turn does not. Search must recover the arc.
  grid.width=300;grid.height=400;grid.resolution_m=.01;
  grid.origin_x_m=-2;grid.origin_y_m=-4;grid.cells.assign(grid.width*grid.height,254);
  for(std::size_t y=0;y<grid.height;++y)for(std::size_t x=0;x<grid.width;++x) {
    const double px=grid.origin_x_m+(x+.5)*.01,py=grid.origin_y_m+(y+.5)*.01;
    if((px<.43 && py> -3.43 && py< -2.57) ||
       (px> -.43 && px<.43 && py> -3.43))grid.cells[y*grid.width+x]=0;
  }
  geometry.hard_padding_m=0;geometry.soft_clearance_m=0;
  geometry.translation_step_m=.005;geometry.rotation_step_rad=.004;
  const double pi=std::acos(-1.),radius=.4764;
  request.measured_start={-.52743602,-2.94896398,0};request.coarse_start=request.measured_start;
  request.goal={request.measured_start.x_m+radius,request.measured_start.y_m+radius,pi/2};
  std::vector<nav::MotionPose> witness;
  for(int i=0;i<=180;++i) {
    const double yaw=i*pi/360;
    witness.push_back({request.measured_start.x_m+radius*std::sin(yaw),
                       request.measured_start.y_m+radius*(1-std::cos(yaw)),yaw});
  }
  assert(nav::CheckMotionSweep(grid,geometry,witness).clear());
  assert(!nav::CheckMotionSweep(grid,geometry,{{0,-3,0},{0,-3,pi/2}}).clear());
  options={};options.max_expansions=1;options.timeout_s=5;
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::Ready);
  assert(nav::ValidateMotionRoute(result.result.route,limits).empty());
  assert(result.result.route.segments.size()==1);
  const auto& arc=result.result.route.segments.front();
  assert(arc.kind==nav::MotionKind::Forward && arc.stop_before && arc.stop_after);
  assert(arc.samples.front().curvature_per_m==0);
  assert(arc.samples[1].curvature_per_m>2.0);
  assert(nav::SameMotionPose(arc.samples[0].pose,arc.samples[1].pose));
  assert(arc.samples[0].speed_ceiling_mps==0 && arc.samples[1].speed_ceiling_mps==0);
  assert(arc.samples[1].time_s>=2.0/limits.curvature_rate_per_m_s);
  // No gap in swept collision checking merely because an exact connector exists.
  const auto midpoint=witness[90];
  const auto cx=static_cast<std::size_t>((midpoint.x_m-grid.origin_x_m)/.01);
  const auto cy=static_cast<std::size_t>((midpoint.y_m-grid.origin_y_m)/.01);
  grid.cells[cy*grid.width+cx]=254;
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status!=nav::RouteStatus::Ready && result.result.route.segments.empty());

  grid.cells[cy*grid.width+cx]=0;
  request.measured_start={-1.32743602,-2.94896398,0};request.coarse_start=request.measured_start;
  request.goal={-.05103602,-1,pi/2};
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::Ready);
  assert(result.result.route.segments.size()==3);
  for(const auto& segment:result.result.route.segments)assert(segment.kind==nav::MotionKind::Forward);

  // Same corner map, separately declared diagnostic outline, explicit finite
  // rotation at its checked offset. Both start/end remain stationary.
  geometry.footprint={{{-0.44000044405612004,-0.23000000173912524}},{{-0.33000007649072044,-0.30600024604658294}},{{-0.1300000735104882,-0.30600024604658294}},{{0.34999991308560874,-0.2900003180894404}},{{0.3499999695686572,-0.2900000948846131}},{{0.36609705862885905,-0.06250013041938066}},{{0.36609705862885905,0.027499878971559675}},{{0.35000006477181683,0.29000006754510554}},{{-0.14999997082679203,0.29000011194348047}},{{-0.3499999738070243,0.29000011194348047}},{{-0.44000044405612004,0.23000000876031373}},{{-0.4829544494624158,0.10687112175218619}},{{-0.4829544494624158,-0.04207562583936106}}};
  geometry.hard_padding_m=.02;
  request.measured_start={-.08530891517377209,-2.8967565394743113,0};
  request.coarse_start=request.measured_start;
  request.goal={request.measured_start.x_m,request.measured_start.y_m,pi/2};
  assert(nav::CheckMotionSweep(grid,geometry,{request.measured_start,request.goal}).clear());
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::Ready);
  assert(result.result.route.segments.size()==1 && result.result.route.segments[0].kind==nav::MotionKind::Spin);
  request.measured_start={-1.3,-2.8967565394743113,0};request.coarse_start=request.measured_start;
  request.goal={-.08530891517377209,-1,pi/2};
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::Ready);
  bool contains_spin=false;
  for(const auto& segment:result.result.route.segments)contains_spin |= segment.kind==nav::MotionKind::Spin;
  assert(contains_spin);
  options.max_labels_per_bin=0;
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::InvalidRequest);

  options={};options.heuristic_weight=INFINITY;
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::InvalidRequest);
  options.heuristic_weight=.99;
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::InvalidRequest);
  options={};
  grid.width=1001;grid.height=1000;grid.resolution_m=.01;
  grid.cells.assign(grid.width*grid.height,0);
  result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  assert(result.result.status==nav::RouteStatus::EvidenceUnavailable);
  assert(result.result.detail=="map snapshot exceeds bounded search size");

}
