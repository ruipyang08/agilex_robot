#include <agx_workbench_navigation/motion_command_sweep.hpp>
#include <algorithm>
#include <cmath>
namespace agx_workbench_navigation {
std::vector<std::vector<MotionPose>> CommandStoppingSweeps(const MotionPose& pose,
    double vx,double vy,double w,double cx,double cw,const MotionLimits& limits,double latency) {
  if(!ValidMotionPose(pose) || !ValidateMotionLimits(limits).empty() || !std::isfinite(latency) ||
     latency<0 || latency>2 || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(w) ||
     !std::isfinite(cx) || !std::isfinite(cw) || cx<0 || cx>limits.max_speed_mps+1e-6 ||
     std::abs(cw)>limits.max_yaw_rate_radps+1e-6)return {};
  std::vector<std::vector<MotionPose>> sweeps;
  for(int branch=0;branch<3;++branch) {
    const double x=branch==0?vx:branch==1?cx:std::max(std::abs(vx),cx);
    const double y=branch==1?0:vy;
    const double z=branch==0?w:branch==1?cw:(std::abs(w)>std::abs(cw)?w:cw);
    const double linear=std::hypot(x,y),angular=std::abs(z);
    if(linear>2*limits.max_speed_mps || angular>2*limits.max_yaw_rate_radps)return {};
    const double duration=latency+std::max(linear/limits.deceleration_mps2,
                                         angular/limits.angular_acceleration_radps2);
    const double dt=std::min(.02,.004/std::max(.001,linear+angular));
    if(duration/dt>5000)return {};
    std::vector<MotionPose> points{pose};MotionPose current=pose;
    for(double t=0;t<duration;) {
      const double step=std::min(dt,duration-t),middle=t+step/2;
      const double braking=std::max(0.,middle-latency);
      const double scale=linear>0?std::max(0.,1-limits.deceleration_mps2*braking/linear):0;
      const double yaw=std::copysign(std::max(0.,angular-limits.angular_acceleration_radps2*braking),z);
      const double heading=current.yaw_rad+yaw*step/2;
      current.x_m+=(x*std::cos(heading)-y*std::sin(heading))*scale*step;
      current.y_m+=(x*std::sin(heading)+y*std::cos(heading))*scale*step;
      // MotionPose uses [-pi, pi], including when a valid stopping rotation
      // crosses that representation seam. The physical sweep is continuous.
      current.yaw_rad=MotionAngleDifference(current.yaw_rad+yaw*step,0);
      t+=step;points.push_back(current);
    }
    sweeps.push_back(std::move(points));
  }
  return sweeps;
}
}
