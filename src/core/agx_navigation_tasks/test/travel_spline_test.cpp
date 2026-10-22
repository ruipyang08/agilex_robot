#include "agx_navigation_tasks/travel_spline.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using namespace agx_navigation_tasks;
int main() {
  // Endpoints have zero derivatives, but the quintic interior does not.
  const auto moving=TravelQuintic(0,0,0,1,0,0,1);
  assert(moving);
  assert(moving->VelocityBound()>=1.875);
  assert(moving->AccelerationBound()>=5.77);
  const double scale=moving->TimeScale(.1,.2);
  assert(scale>=18.75);
  const auto slowed=TravelQuintic(0,0,0,1,0,0,scale);
  assert(slowed && slowed->VelocityBound()<=.1+1e-12);
  assert(slowed->AccelerationBound()<=.2+1e-12);
  for(int i=0;i<=1000;++i) {
    const auto sample=moving->Sample(i/1000.0);
    assert(std::isfinite(sample[0]));
    assert(std::abs(sample[1])<=moving->VelocityBound()+1e-12);
    assert(std::abs(sample[2])<=moving->AccelerationBound()+1e-12);
    assert(std::abs(sample[0]-slowed->Sample(i/1000.0)[0])<1e-12);
  }
  assert(moving->Inside(0,1));
  // Same endpoints can still leave the named corridor due to derivatives.
  const auto overshoot=TravelQuintic(0,4,0,0,-4,0,1);
  assert(overshoot && !overshoot->Inside(-.1,.1));
  assert(overshoot->Sample(.5)[0]>.1);
  const auto curved=TravelQuintic(.2,.3,-.1,.8,.2,.4,2);
  assert(curved);
  const auto start=curved->Sample(0),end=curved->Sample(1);
  assert(std::abs(start[0]-.2)<1e-12 && std::abs(start[1]-.3)<1e-12 && std::abs(start[2]+.1)<1e-12);
  assert(std::abs(end[0]-.8)<1e-12 && std::abs(end[1]-.2)<1e-12 && std::abs(end[2]-.4)<1e-12);
  assert(!TravelQuintic(0,0,0,1,0,0,0));
  assert(!TravelQuintic(0,0,0,1,0,0,std::numeric_limits<double>::infinity()));
  assert(!TravelQuintic(0,1e308,1e308,1,1e308,1e308,1e308));
}
