#include <agx_workbench_navigation/motion_speed_profile.hpp>
#include <algorithm>
#include <cmath>

namespace agx_workbench_navigation {
namespace {
bool Nonnegative(double v) {return std::isfinite(v) && v>=0;}
double Wrap(double yaw) {return std::atan2(std::sin(yaw),std::cos(yaw));}
void AddStoppedDwell(MotionSegment& s,const MotionLimits& l) {
  if(s.stop_before) {
    auto first=s.samples.front();
    for(auto& sample:s.samples)sample.time_s+=l.steering_settle_s;
    first.time_s=0;s.samples.insert(s.samples.begin(),first);
  }
  if(s.stop_after) {
    auto last=s.samples.back();last.time_s+=l.stop_dwell_s;s.samples.push_back(last);
  }
  s.estimated_time_s=s.samples.back().time_s;
}
}
bool ProfileForwardMotion(MotionSegment& output,const MotionLimits& l,double v0,double v1,std::string* detail) {
  auto fail=[&](const char* why) {if(detail)*detail=why;return false;};
  if(!ValidateMotionLimits(l).empty() || !Nonnegative(v0) || !Nonnegative(v1) ||
      v0>l.max_speed_mps || v1>l.max_speed_mps || output.samples.size()<3 || output.samples.size()>19996 ||
      output.kind!=MotionKind::Forward)return fail("invalid forward profile request");
  MotionSegment s=output;
  const std::size_t n=s.samples.size();
  std::vector<double> ds(n,0),cap(n,l.max_speed_mps),kprime(n,0);
  if(!Nonnegative(s.curvature_bound_per_m) || s.curvature_bound_per_m>1/l.min_turn_radius_m+1e-9)
    return fail("invalid curvature bound");
  double maxk=s.curvature_bound_per_m;
  for(std::size_t i=0;i<n;++i) {
    const auto& sample=s.samples[i];
    if(!ValidMotionPose(sample.pose) || !std::isfinite(sample.curvature_per_m) ||
       !Nonnegative(sample.speed_ceiling_mps))return fail("invalid forward geometry");
    const double k=std::abs(sample.curvature_per_m);maxk=std::max(maxk,k);
    if(k>1/l.min_turn_radius_m+1e-9)return fail("curvature exceeds model");
    if(sample.speed_ceiling_mps>0)cap[i]=std::min(cap[i],sample.speed_ceiling_mps);
    if(k>1e-12)cap[i]=std::min(cap[i],std::min(std::sqrt(l.lateral_acceleration_mps2/k),l.max_yaw_rate_radps/k));
    if(i) {
      ds[i]=std::hypot(sample.pose.x_m-s.samples[i-1].pose.x_m,sample.pose.y_m-s.samples[i-1].pose.y_m);
      if(ds[i]<=1e-9)return fail("duplicate forward geometry point");
      const double dk=std::abs(sample.curvature_per_m-s.samples[i-1].curvature_per_m)/ds[i];
      kprime[i]=std::max(kprime[i],dk);kprime[i-1]=std::max(kprime[i-1],dk);
    }
  }
  // Reserve half the angular-acceleration budget for a*k and half for v²*dk/ds.
  const double accel=std::min(l.acceleration_mps2,maxk>1e-12?l.angular_acceleration_radps2/(2*maxk):l.acceleration_mps2);
  const double decel=std::min(l.deceleration_mps2,maxk>1e-12?l.angular_acceleration_radps2/(2*maxk):l.deceleration_mps2);
  for(std::size_t i=0;i<n;++i)if(kprime[i]>1e-12)
    cap[i]=std::min(cap[i],std::min(l.curvature_rate_per_m_s/kprime[i],std::sqrt(l.angular_acceleration_radps2/(2*kprime[i]))));
  if(cap.front()+1e-9<v0 || cap.back()+1e-9<v1)return fail("boundary speed exceeds curvature ceiling");
  cap.front()=v0;cap.back()=v1;
  for(std::size_t i=1;i<n;++i)cap[i]=std::min(cap[i],std::sqrt(cap[i-1]*cap[i-1]+2*accel*ds[i]));
  if(cap.back()+1e-9<v1)return fail("insufficient acceleration distance");
  for(std::size_t i=n-1;i>0;--i)cap[i-1]=std::min(cap[i-1],std::sqrt(cap[i]*cap[i]+2*decel*ds[i]));
  if(cap.front()+1e-9<v0)return fail("insufficient braking distance");
  cap.front()=v0;cap.back()=v1;
  double time=0;
  for(std::size_t i=0;i<n;++i) {
    if(i) {
      if(cap[i]+cap[i-1]<=1e-9)return fail("zero speed over positive distance");
      time+=2*ds[i]/(cap[i]+cap[i-1]);
    }
    s.samples[i].time_s=time;s.samples[i].speed_ceiling_mps=cap[i];
    s.samples[i].yaw_ceiling_radps=cap[i]*std::abs(s.samples[i].curvature_per_m);
  }
  s.stop_before=v0==0;s.stop_after=v1==0;AddStoppedDwell(s,l);
  if(!std::isfinite(s.estimated_time_s))return fail("nonfinite profile duration");
  output=std::move(s);if(detail)detail->clear();return true;
}
MotionSegment MakeForwardPrimitive(const MotionPose& start,double k0,double k1,double hold,
    double v0,double v1,double ceiling,const MotionLimits& l,double step) {
  MotionSegment s;
  if(!ValidMotionPose(start) || !ValidateMotionLimits(l).empty() ||
      !std::isfinite(k0) || !std::isfinite(k1) || std::abs(k0)>1/l.min_turn_radius_m ||
      std::abs(k1)>1/l.min_turn_radius_m || !std::isfinite(hold) || hold<=0 ||
      !std::isfinite(step) || step<=0 || !std::isfinite(ceiling) || ceiling<=0 || ceiling>l.max_speed_mps)return s;
  const double ramp=std::abs(k1-k0)*ceiling/l.curvature_rate_per_m_s;
  const double length=ramp+hold;
  if(!std::isfinite(length) || length/step>19990)return s;
  const std::size_t count=std::max<std::size_t>(2,static_cast<std::size_t>(std::ceil(length/step)));
  const double ds=length/count;
  const double maxk=std::max(std::abs(k0),std::abs(k1));
  s.curvature_bound_per_m=maxk;
  const double dkds=ramp>0?std::abs(k1-k0)/ramp:0;
  // Midpoint integration and the one interval crossing the ramp/hold join.
  // A one-metre body radius conservatively covers the configured rectangle.
  s.geometry_error_m=length*ds*ds*(maxk*maxk+dkds)/8+dkds*ds*ds/8;
  if(k0==k1)s.geometry_error_m=0; // Analytic straight/circular poses.
  if(s.geometry_error_m>.05)return {};
  auto curvature=[&](double distance) {return ramp>0?k0+(k1-k0)*std::min(1.,distance/ramp):k1;};
  MotionPose pose=start;s.samples.push_back({pose,k0,ceiling,0,0});
  for(std::size_t i=1;i<=count;++i) {
    const double k=curvature((i-.5)*ds),yaw=pose.yaw_rad+k*ds/2;
    if(k0==k1 && std::abs(k)>1e-12) {
      // Exact circular integration also preserves checked connector endpoints.
      const double next_yaw=pose.yaw_rad+k*ds;
      pose.x_m+=(std::sin(next_yaw)-std::sin(pose.yaw_rad))/k;
      pose.y_m+=(std::cos(pose.yaw_rad)-std::cos(next_yaw))/k;
    } else {pose.x_m+=ds*std::cos(yaw);pose.y_m+=ds*std::sin(yaw);}
    pose.yaw_rad=Wrap(pose.yaw_rad+k*ds);
    s.samples.push_back({pose,curvature(i*ds),ceiling,0,0});
  }
  s.samples.back().curvature_per_m=k1;
  if(!ProfileForwardMotion(s,l,v0,v1))return {};
  return s;
}
MotionSegment MakePreparedForwardPrimitive(const MotionPose& start,double k0,double k1,
    double length,double v1,double ceiling,const MotionLimits& l,double step) {
  if(!std::isfinite(k0) || std::abs(k0)>1/l.min_turn_radius_m)return {};
  auto s=MakeForwardPrimitive(start,k1,k1,length,0,v1,ceiling,l,step);
  if(s.samples.empty())return s;
  const double preparation=std::abs(k1-k0)/l.curvature_rate_per_m_s;
  if(preparation>0) {
    auto first=s.samples.front();first.curvature_per_m=k0;first.time_s=0;
    for(auto& sample:s.samples)sample.time_s+=preparation;
    s.samples.insert(s.samples.begin(),first);
    s.estimated_time_s=s.samples.back().time_s;
    s.curvature_bound_per_m=std::max(std::abs(k0),std::abs(k1));
  }
  return s;
}
MotionSegment MakeSpinPrimitive(const MotionPose& start,double angle,const MotionLimits& l,double step) {
  MotionSegment s;s.kind=MotionKind::Spin;
  if(!ValidMotionPose(start) || !ValidateMotionLimits(l).empty() || !std::isfinite(angle) ||
     std::abs(angle)<1e-9 || std::abs(angle)>2*std::acos(-1.) || !std::isfinite(step) || step<=0 ||
     std::abs(angle)/step>19990)return s;
  const double length=std::abs(angle);
  const auto count=std::max<std::size_t>(2,static_cast<std::size_t>(std::ceil(length/step)));
  const double da=length/count;
  std::vector<double> speeds(count+1,l.max_yaw_rate_radps);speeds.front()=0;speeds.back()=0;
  for(std::size_t i=1;i<=count;++i)speeds[i]=std::min(speeds[i],std::sqrt(speeds[i-1]*speeds[i-1]+2*l.angular_acceleration_radps2*da));
  for(std::size_t i=count;i>0;--i)speeds[i-1]=std::min(speeds[i-1],std::sqrt(speeds[i]*speeds[i]+2*l.angular_acceleration_radps2*da));
  double time=0;
  for(std::size_t i=0;i<=count;++i) {
    if(i)time+=2*da/(speeds[i-1]+speeds[i]);
    MotionPose pose=start;pose.yaw_rad=Wrap(start.yaw_rad+angle*i/count);
    s.samples.push_back({pose,0,0,speeds[i],time});
  }
  AddStoppedDwell(s,l);return s;
}
}  // namespace agx_workbench_navigation
