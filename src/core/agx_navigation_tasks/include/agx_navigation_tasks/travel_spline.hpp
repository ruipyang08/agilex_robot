#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace agx_navigation_tasks {
namespace travel_spline_detail {
template<std::size_t N>
double Sample(std::array<double,N> control,double u) {
  for(std::size_t remaining=N-1;remaining>0;--remaining)
    for(std::size_t i=0;i<remaining;++i)control[i]=(1-u)*control[i]+u*control[i+1];
  return control[0];
}
template<std::size_t N>
std::pair<double,double> Bounds(const std::array<double,N>& control,unsigned depth=4) {
  // Subdivision tightens the Bernstein convex-hull bound. Each resulting hull
  // still encloses every interior extremum; sampling is not the bound.
  if(!depth) {
    const auto limits=std::minmax_element(control.begin(),control.end());
    return {*limits.first,*limits.second};
  }
  auto work=control;std::array<double,N> left{},right{};
  left[0]=work[0];right[N-1]=work[N-1];
  for(std::size_t level=1;level<N;++level) {
    for(std::size_t i=0;i<N-level;++i)work[i]=.5*work[i]+.5*work[i+1];
    left[level]=work[0];right[N-level-1]=work[N-level-1];
  }
  const auto a=Bounds(left,depth-1),b=Bounds(right,depth-1);
  return {std::min(a.first,b.first),std::max(a.second,b.second)};
}
inline double AbsoluteUpper(const std::pair<double,double>& bounds) {
  const double magnitude=std::max(std::abs(bounds.first),std::abs(bounds.second));
  // Outward numerical margin; time scaling adds a further relative margin.
  return std::nextafter(magnitude,std::numeric_limits<double>::infinity());
}
}  // namespace travel_spline_detail

struct TravelQuinticCurve {
  std::array<double,6> position;
  std::array<double,5> velocity;
  std::array<double,4> acceleration;
  double duration_s;
  std::array<double,3> Sample(double fraction) const {
    using travel_spline_detail::Sample;
    return {{Sample(position,fraction),Sample(velocity,fraction),Sample(acceleration,fraction)}};
  }
  bool Inside(double lower,double upper) const {
    const auto bounds=travel_spline_detail::Bounds(position);
    return std::isfinite(lower) && std::isfinite(upper) && lower<=upper &&
        bounds.first>=lower && bounds.second<=upper;
  }
  double VelocityBound() const {
    return travel_spline_detail::AbsoluteUpper(travel_spline_detail::Bounds(velocity));
  }
  double AccelerationBound() const {
    return travel_spline_detail::AbsoluteUpper(travel_spline_detail::Bounds(acceleration));
  }
  double TimeScale(double max_velocity,double max_acceleration) const {
    if(!std::isfinite(max_velocity) || !std::isfinite(max_acceleration) ||
        max_velocity<=0 || max_acceleration<=0)return std::numeric_limits<double>::infinity();
    return std::max({1.0,VelocityBound()/max_velocity,
        std::sqrt(AccelerationBound()/max_acceleration)})*(1.0+1e-6);
  }
};

inline std::optional<TravelQuinticCurve> TravelQuintic(double p0,double v0,double a0,
    double p1,double v1,double a1,double duration) {
  for(double value:{p0,v0,a0,p1,v1,a1,duration})if(!std::isfinite(value))return {};
  if(duration<=0)return {};
  TravelQuinticCurve result{};result.duration_s=duration;
  // Equivalent boundary-value quintic to Noetic QuinticSplineSegment. All
  // waypoint dimensions are present, so its cubic/linear fallbacks are unused.
  result.position={{p0,p0+v0*duration/5,
      p0+2*v0*duration/5+a0*duration*duration/20,
      p1-2*v1*duration/5+a1*duration*duration/20,p1-v1*duration/5,p1}};
  for(double value:result.position)if(!std::isfinite(value))return {};
  for(std::size_t i=0;i<5;++i) {
    result.velocity[i]=5*(result.position[i+1]-result.position[i])/duration;
    if(!std::isfinite(result.velocity[i]))return {};
  }
  for(std::size_t i=0;i<4;++i) {
    result.acceleration[i]=4*(result.velocity[i+1]-result.velocity[i])/duration;
    if(!std::isfinite(result.acceleration[i]))return {};
  }
  if(!std::isfinite(result.VelocityBound()) || !std::isfinite(result.AccelerationBound()))return {};
  return result;
}
}  // namespace agx_navigation_tasks
