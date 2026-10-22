#include <agx_workbench_navigation/motion_speed_profile.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
namespace nav=agx_workbench_navigation;
int main() {
  nav::MotionLimits limits;
  auto straight=nav::MakeForwardPrimitive({},0,0,2,0,0,.5,limits);
  assert(straight.samples.size()>10 && straight.stop_before && straight.stop_after);
  assert(straight.samples.front().speed_ceiling_mps==0 && straight.samples.back().speed_ceiling_mps==0);
  assert(std::abs(straight.samples.back().pose.x_m-2)<1e-6);
  assert(straight.estimated_time_s>4);
  for(std::size_t i=1;i<straight.samples.size();++i) {
    const auto& a=straight.samples[i-1];const auto& b=straight.samples[i];
    const double ds=std::hypot(b.pose.x_m-a.pose.x_m,b.pose.y_m-a.pose.y_m);
    assert(b.speed_ceiling_mps*b.speed_ceiling_mps<=a.speed_ceiling_mps*a.speed_ceiling_mps+2*.25*ds+1e-8);
    assert(a.speed_ceiling_mps*a.speed_ceiling_mps<=b.speed_ceiling_mps*b.speed_ceiling_mps+2*.3*ds+1e-8);
    assert(b.time_s>a.time_s);
  }
  auto curve=nav::MakeForwardPrimitive({},0,1,1,0,.1,.25,limits);
  assert(!curve.samples.empty() && !curve.stop_after);
  assert(curve.samples.back().pose.y_m>0 && curve.samples.back().curvature_per_m==1);
  for(std::size_t i=1;i<curve.samples.size();++i) {
    const auto& a=curve.samples[i-1];const auto& b=curve.samples[i];
    const double dt=b.time_s-a.time_s;
    assert(std::abs(b.curvature_per_m-a.curvature_per_m)<=limits.curvature_rate_per_m_s*dt+1e-7);
    const double wa=a.speed_ceiling_mps*a.curvature_per_m,wb=b.speed_ceiling_mps*b.curvature_per_m;
    assert(std::abs(wb-wa)<=limits.angular_acceleration_radps2*dt+1e-7);
  }
  auto continuing=nav::MakeForwardPrimitive(curve.samples.back().pose,1,0,.5,.1,0,.25,limits);
  assert(!continuing.samples.empty() && !continuing.stop_before);
  assert(nav::SameMotionPose(curve.samples.back().pose,continuing.samples.front().pose));
  assert(continuing.samples.front().curvature_per_m==1);
  auto too_short=nav::MakeForwardPrimitive({},0,0,.01,.5,0,.5,limits);
  assert(too_short.samples.empty());
  auto invalid=nav::MakeForwardPrimitive({},0,10,1,0,0,.5,limits);
  assert(invalid.samples.empty());
  auto spin=nav::MakeSpinPrimitive({},std::acos(-1.)/2,limits);
  assert(!spin.samples.empty());
  assert(std::abs(spin.samples.back().pose.yaw_rad-std::acos(-1.)/2)<1e-6);
  assert(spin.estimated_time_s>std::acos(-1.)/.8);
  for(const auto& s:spin.samples)assert(s.pose.x_m==0 && s.pose.y_m==0 && s.speed_ceiling_mps==0);
  nav::MotionRoute route;
  route.route_id="r";route.invocation_id="i";route.model_digest="m";route.map_digest="map";route.registry_digest="reg";
  route.segments={straight};route.goal=straight.samples.back().pose;route.estimated_time_s=straight.estimated_time_s;
  assert(nav::ValidateMotionRoute(route,limits).empty());
  route.segments={spin};route.goal=spin.samples.back().pose;route.estimated_time_s=spin.estimated_time_s;
  assert(nav::ValidateMotionRoute(route,limits).empty());
  // Steering changes happen at a fixed pose with zero propulsion, followed by
  // settling allowance. Circular integration reaches the exact endpoint.
  const double k=1/limits.min_turn_radius_m,angle=std::acos(-1.)/2;
  for(double sign:{-1.,1.}) {
    auto prepared=nav::MakePreparedForwardPrimitive({},0,sign*k,angle/k,0,.5,limits);
    assert(!prepared.samples.empty());
    assert(nav::SameMotionPose(prepared.samples.front().pose,prepared.samples[1].pose));
    assert(prepared.samples[1].time_s>=k/limits.curvature_rate_per_m_s);
    assert(prepared.samples[2].time_s-prepared.samples[1].time_s>=limits.steering_settle_s-1e-9);
    assert(std::abs(prepared.samples.back().pose.x_m-limits.min_turn_radius_m)<1e-9);
    assert(std::abs(prepared.samples.back().pose.y_m-sign*limits.min_turn_radius_m)<1e-9);
    for(std::size_t i=1;i<prepared.samples.size();++i) {
      const auto& a=prepared.samples[i-1];const auto& b=prepared.samples[i];
      const double dt=b.time_s-a.time_s;
      assert(std::abs(b.curvature_per_m-a.curvature_per_m)<=limits.curvature_rate_per_m_s*dt+1e-8);
      if(i<=2)assert(a.speed_ceiling_mps==0 && b.speed_ceiling_mps==0);
      assert(std::abs(b.speed_ceiling_mps*b.curvature_per_m-a.speed_ceiling_mps*a.curvature_per_m)
             <=limits.angular_acceleration_radps2*dt+1e-8);
    }
    route.segments={prepared};route.goal=prepared.samples.back().pose;route.estimated_time_s=prepared.estimated_time_s;
    assert(nav::ValidateMotionRoute(route,limits).empty());
  }
  assert(nav::MakePreparedForwardPrimitive({},NAN,1,1,0,.5,limits).samples.empty());

  // Requested tight-turn speed is about half the 0.5 m/s forward cap.
  auto tuned=limits;tuned.max_yaw_rate_radps=.55;
  assert(nav::ValidateMotionLimits(tuned).empty());
  const auto faster=nav::MakePreparedForwardPrimitive({},0,k,angle/k,0,.5,tuned);
  assert(!faster.samples.empty());double peak=0;
  for(const auto& sample:faster.samples) {
    peak=std::max(peak,sample.speed_ceiling_mps);
    assert(sample.speed_ceiling_mps*sample.speed_ceiling_mps*std::abs(sample.curvature_per_m)
           <=tuned.lateral_acceleration_mps2+1e-8);
    assert(std::abs(sample.speed_ceiling_mps*sample.curvature_per_m)<=.55+1e-8);
  }
  assert(std::abs(peak-.55*tuned.min_turn_radius_m)<1e-6);
  route.segments={faster};route.goal=faster.samples.back().pose;route.estimated_time_s=faster.estimated_time_s;
  assert(nav::ValidateMotionRoute(route,tuned).empty());
  tuned.max_yaw_rate_radps=.550001;
  assert(!nav::ValidateMotionLimits(tuned).empty());

}
