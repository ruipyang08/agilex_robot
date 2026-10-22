#include <agx_workbench_navigation/motion_sensor_evidence.hpp>
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
namespace n=agx_workbench_navigation;
bool Reject(const std::function<void()>& call) {try {call();return false;}catch(const std::invalid_argument&) {return true;}}
int main() {
  n::MotionGrid grid;grid.width=100;grid.height=100;grid.resolution_m=.1;
  grid.origin_x_m=-5;grid.origin_y_m=-5;grid.cells.assign(10000,0);
  n::CapturedMotionCloud cloud;cloud.source_stamp_s=10;cloud.transform_stamp_s=10;
  cloud.receipt_steady_s=100;cloud.source_frame="rslidar";cloud.target_frame="base_link";
  cloud.pose_frame="odom";cloud.model_digest="model";cloud.filter_digest="filter";
  cloud.capture_pose.position={1,0,0};cloud.points={{{1,0,.5}}};
  n::MotionCaptureTransform map_from_odom;map_from_odom.position={0,2,0};
  auto joined=n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter");
  assert(joined.grid.cells[70*100+70]==254); // original map point (2,2)
  assert(joined.grid.cells[50*100+60]==0);  // not reinterpreted at current robot
  assert(std::abs(joined.deadline_s-100.35)<1e-8);
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.36,100.2,"model","filter");}));
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,101.01,"model","filter");}));
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,9.9,100.1,"model","filter");}));
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,99.9,"model","filter");}));
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"changed","filter");}));
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","changed");}));
  cloud.capture_pose.orientation={0,0,std::sqrt(.5),std::sqrt(.5)};
  joined=n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter");
  assert(joined.grid.cells[80*100+60]==254); // odom point (1,1), map (1,3)
  cloud.capture_pose.orientation={std::sqrt(.5),0,0,std::sqrt(.5)};
  joined=n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter");
  assert(joined.grid.cells[65*100+70]==254); // z affects y under capture roll
  cloud.transform_stamp_s=0;cloud.transform_static=true;
  assert(n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter").deadline_s>100.1);
  cloud.transform_static=false;
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter");}));
  cloud.transform_static=true;cloud.state=3;
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter");}));
  cloud.state=1; // directional BLOCKED is still usable geometry
  assert(n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter").deadline_s>100.1);
  cloud.capture_pose.orientation={0,0,0,2};
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter");}));
  cloud.capture_pose.orientation={0,0,0,1};cloud.points[0][0]=std::numeric_limits<double>::quiet_NaN();
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter");}));
  cloud.points.assign(250001,{{1,0,.5}});
  assert(Reject([&]{n::OverlayCapturedObstacles(grid,cloud,map_from_odom,10.1,100.1,"model","filter");}));
  assert(Reject([&]{n::MotionEvidenceDeadline(100.1,10.1,10,100,.0,.75);}));
}
