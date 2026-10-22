#include <agx_workbench_navigation/motion_geometry.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <random>
namespace nav=agx_workbench_navigation;
int main() {
  auto check=[](const nav::MotionGrid& grid,const nav::MotionGeometryPolicy& policy,
                const std::vector<nav::MotionPose>& poses) {
    const auto reference=nav::CheckMotionSweep(grid,policy,poses);
    const auto indexed=nav::MotionSweepChecker(grid).Check(policy,poses);
    assert(reference.status==indexed.status && reference.samples==indexed.samples);
    assert(reference.cell_x==indexed.cell_x && reference.cell_y==indexed.cell_y);
    assert(reference.interpolation_padding_m==indexed.interpolation_padding_m);
    return reference;
  };
  nav::MotionGrid grid; grid.width=200;grid.height=200;grid.resolution_m=.05;
  grid.origin_x_m=-5;grid.origin_y_m=-5;grid.cells.assign(40000,0);
  nav::MotionGeometryPolicy policy;policy.hard_padding_m=0;
  assert(check(grid,policy,{{0,0,0}}).clear());
  // Filled interior, not just polygon boundary.
  grid.cells[100*200+100]=254;
  assert(check(grid,policy,{{0,0,0}}).status==nav::MotionCollision::Occupied);
  grid.cells[100*200+100]=255;
  assert(check(grid,policy,{{0,0,0}}).status==nav::MotionCollision::Unknown);
  grid.cells[100*200+100]=253;
  assert(check(grid,policy,{{0,0,0}}).clear());
  // Thin obstacle strictly between clear endpoint footprints.
  grid.cells[100*200+100]=254;
  assert(check(grid,policy,{{-1,0,0}}).clear());
  assert(check(grid,policy,{{1,0,0}}).clear());
  assert(!check(grid,policy,{{-1,0,0},{1,0,0}}).clear());
  grid.cells[100*200+100]=0;
  // Analytic finite-spin example from Python fixture, high-resolution cell.
  grid.width=400;grid.height=400;grid.resolution_m=.005;
  grid.origin_x_m=-1;grid.origin_y_m=-1;grid.cells.assign(160000,0);
  grid.cells[122*400+278]=254; // x=[.390,.395], y=[-.390,-.385]
  const double pi=std::acos(-1.);
  policy.translation_step_m=.002;policy.rotation_step_rad=.002;
  assert(check(grid,policy,{{0,0,0},{0,0,pi/2}}).clear());
  assert(!check(grid,policy,{{0,0,0},{0,0,-pi/2}}).clear());
  const auto before=grid.cells;
  assert(!check(grid,policy,{{0,0,0},{0,0,pi/2},{0,0,pi},{0,0,-pi/2},{0,0,0}}).clear());
  assert(grid.cells==before);
  grid.cells.assign(160000,0);
  assert(check(grid,policy,{{0,0,pi-.01},{0,0,-pi+.01}}).clear());
  assert(check(grid,policy,{{.6,0,0}}).status==nav::MotionCollision::MapBoundary);
  assert(!check(grid,policy,{}).clear());
  assert(!check(grid,policy,{{NAN,0,0}}).clear());
  policy.translation_step_m=0;
  assert(!check(grid,policy,{{0,0,0}}).clear());
  policy.translation_step_m=.01;policy.max_samples=1;
  assert(check(grid,policy,{{0,0,0},{.1,0,0}}).status==nav::MotionCollision::SampleLimit);
  nav::MotionPolygon star;
  for(int i=0;i<5;++i)star.push_back({{std::cos(i*4*pi/5),std::sin(i*4*pi/5)}});
  assert(!nav::ValidMotionPolygon(star));
  assert(nav::MotionPolygonContains(policy.footprint,policy.footprint));
  nav::MotionPolygon small{{{-.4,-.3}},{{.4,-.3}},{{.4,.3}},{{-.4,.3}}};
  assert(nav::MotionPolygonContains(policy.footprint,small));
  assert(!nav::MotionPolygonContains(small,policy.footprint));
  // Indexed checking must preserve the first collision and interpolation
  // semantics, including unknown cells, soft costs, turns and map boundaries.
  grid.width=100;grid.height=100;grid.resolution_m=.05;
  grid.origin_x_m=-2.5;grid.origin_y_m=-2.5;grid.cells.assign(10000,0);
  std::mt19937 random(73);
  for(int i=0;i<250;++i)grid.cells[random()%10000]=253+random()%3;
  const nav::MotionSweepChecker checker(grid);
  policy={};policy.translation_step_m=.03;policy.rotation_step_rad=.02;
  for(int i=0;i<300;++i) {
    auto coordinate=[&]() {return (static_cast<int>(random()%500)-250)/100.;};
    const nav::MotionPose a{coordinate(),coordinate(),coordinate()};
    const nav::MotionPose b{a.x_m+coordinate()/3,a.y_m+coordinate()/3,coordinate()};
    policy.hard_padding_m=(random()%20)/100.;
    policy.max_samples=i%3==0?2:10000;
    const std::vector<nav::MotionPose> poses{i%2==0?a:b,b};
    const auto reference=nav::CheckMotionSweep(grid,policy,poses);
    const auto indexed=checker.Check(policy,poses);
    assert(reference.status==indexed.status && reference.samples==indexed.samples);
    assert(reference.cell_x==indexed.cell_x && reference.cell_y==indexed.cell_y);
    assert(nav::SameMotionPose(reference.first_pose,indexed.first_pose));
    assert(reference.interpolation_padding_m==indexed.interpolation_padding_m);
  }
  // The index owns a coherent snapshot; editing the source cannot stale it.
  grid.cells.assign(10000,0);policy={};policy.hard_padding_m=0;
  const nav::MotionSweepChecker empty_snapshot(grid);
  grid.cells[50*100+50]=255;
  assert(empty_snapshot.Check(policy,{{0,0,0}}).clear());
  assert(!nav::CheckMotionSweep(grid,policy,{{0,0,0}}).clear());
  assert(!nav::MotionSweepChecker(nav::MotionGrid{}).Check(policy,{{0,0,0}}).clear());

}
