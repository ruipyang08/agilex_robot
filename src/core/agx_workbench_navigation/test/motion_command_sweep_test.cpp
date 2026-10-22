#include <agx_workbench_navigation/motion_command_sweep.hpp>
#include <cassert>
#include <cmath>
#include <limits>
namespace n=agx_workbench_navigation;
int main() {
  n::MotionLimits limits;
  auto paths=n::CommandStoppingSweeps({},.3,0,0,.2,0,limits,.25);
  assert(paths.size()==3);
  // Response travel plus v^2/(2a), independently of candidate reduction.
  assert(std::abs(paths[0].back().x_m-(.3*.25+.3*.3/(2*.3)))<.001);
  assert(paths[0].back().x_m>paths[1].back().x_m);
  paths=n::CommandStoppingSweeps({},0,0,.4,0,.1,limits,.25);
  assert(paths.size()==3);
  assert(std::abs(paths[0].back().yaw_rad-(.4*.25+.4*.4/(2*.4)))<.001);
  assert(paths[0].back().x_m==0 && paths[0].back().y_m==0);
  paths=n::CommandStoppingSweeps({},.3,.05,-.2,.2,.1,limits,.25);
  assert(paths.size()==3 && paths[0].back().y_m!=0);
  // Trial20 rejected an unobstructed final rotation because its predicted
  // stopping angle crossed +pi. Every generated pose must use the same canonical
  // angular range as planner/sensor poses, including the mirrored -pi crossing.
  n::MotionGrid free;free.width=free.height=200;free.resolution_m=.05;
  free.origin_x_m=free.origin_y_m=-5;free.cells.assign(40000,0);
  n::MotionGeometryPolicy geometry;
  for(double sign:{-1.,1.}) {
    const n::MotionPose start{1.2545859813690186,3.9511468410491943,sign*2.896703274683063};
    paths=n::CommandStoppingSweeps(start,.000697363277842355,
        -.00007639158078807752,sign*.39975875484524426,0,sign*.4,limits,.116666667);
    assert(paths.size()==3);
    for(const auto& path:paths) {
      double travel=0;
      for(std::size_t i=0;i<path.size();++i) {
        assert(n::ValidMotionPose(path[i]));
        if(i)travel+=n::MotionAngleDifference(path[i].yaw_rad,path[i-1].yaw_rad);
      }
      assert(sign*travel>.245 && sign*travel<.248);
      assert(n::CheckMotionSweep(free,geometry,path).clear());
    }
    // Canonicalization never removes a real occupied-cell intersection.
    auto occupied=free;
    const auto x=static_cast<std::size_t>((start.x_m-free.origin_x_m)/free.resolution_m);
    const auto y=static_cast<std::size_t>((start.y_m-free.origin_y_m)/free.resolution_m);
    occupied.cells[y*occupied.width+x]=254;
    assert(n::CheckMotionSweep(occupied,geometry,paths.front()).status==n::MotionCollision::Occupied);
  }
  assert(n::CommandStoppingSweeps({},0,0,0,-.1,0,limits,.25).empty());
  assert(n::CommandStoppingSweeps({},0,0,0,.6,0,limits,.25).empty());
  assert(n::CommandStoppingSweeps({},0,0,0,0,0,limits,3).empty());
  assert(n::CommandStoppingSweeps({},std::numeric_limits<double>::quiet_NaN(),0,0,0,0,limits,.25).empty());
}
