#include <agx_workbench_navigation/motion_route_store.hpp>
#include <cassert>
#include <agx_workbench_navigation/motion_speed_profile.hpp>
namespace nav=agx_workbench_navigation;
nav::MotionRoute make(const std::string& id) {
  nav::MotionRoute r;r.route_id=id;r.invocation_id="i";r.model_digest="m";
  r.map_digest="map";r.registry_digest="registry";
  nav::MotionSegment s;s.kind=nav::MotionKind::VerifyStop;s.samples.push_back({});r.segments={s};return r;
}
int main() {
  {
    double time=1;nav::MotionRouteStore owned([&]{return time;});
    auto route=make("owned");assert(owned.put(route));
    nav::CommandBinding binding{"session","grant","source",{"map",1,{0,0,0},{0,0,0,1}}};
    binding.route_id="owned";binding.invocation_id="i";binding.task_deadline_s=100;
    assert(!owned.bind(binding)); // An unpinned preview carries no task lifetime.
    assert(owned.pin("owned","i",100));
    auto wrong=binding;wrong.task_deadline_s=101;assert(!owned.bind(wrong));
    wrong=binding;wrong.invocation_id="foreign";assert(!owned.bind(wrong));
    wrong=binding;wrong.endpoint.position[0]=.1;assert(!owned.bind(wrong));
    assert(owned.bind(binding));assert(owned.bound(binding));
    assert(owned.boundGoal(binding.endpoint));
    assert(owned.progress(binding,0,0));
    assert(!owned.progress(binding,0,1));
    assert(owned.progress(binding,1,0));
    assert(!owned.progress(binding,0,0));
    std::size_t segment=0,sample=1;
    assert(owned.boundGoal(binding.endpoint,&segment,&sample));
    assert(segment==1 && sample==0);
    wrong=binding;wrong.grant_id="another";assert(!owned.bind(wrong));
    assert(!owned.bound(wrong));
    auto endpoint=binding.endpoint;endpoint.stamp_ns=2;assert(!owned.boundGoal(endpoint));
    assert(owned.bind(binding)); // Exact replay cannot renew the task deadline.
    owned.block(binding.route_id,binding.invocation_id,"new_obstacle");
    assert(!owned.bound(binding));assert(!owned.boundGoal(binding.endpoint));
    assert(!owned.bind(binding));assert(!owned.progress(binding,1,0));
    assert(owned.blocked(binding)=="new_obstacle");
    owned.block(binding.route_id,binding.invocation_id,"later_failure");
    assert(owned.blocked(binding)=="new_obstacle");
    time=100;assert(!owned.bound(binding));assert(!owned.boundGoal(binding.endpoint));
    assert(!owned.bind(binding));
  }
  double now=0;nav::MotionRouteStore store([&] {return now;});
  assert(store.put(make("a")));assert(!store.put(make("a")));
  now=1;assert(!store.get("a","other"));assert(store.get("a","i"));
  now=2;assert(store.pin("a","i",100));assert(!store.pin("a","i",200));
  now=70;assert(store.get("a","i"));
  now=100;assert(!store.get("a","i"));
  assert(store.put(make("b")));now=160;assert(!store.get("b","i"));
  now=161;for(int i=0;i<8;++i)assert(store.put(make(std::to_string(i))));
  assert(!store.put(make("ninth")));
  now=162;assert(store.pin("0","i",300));assert(!store.pin("1","i",300));
  store.release("0","wrong");assert(store.get("0","i"));
  now=222;assert(store.put(make("ninth"))); // Other pending leases expired; active preserved.
  assert(store.get("0","i"));
  store.release("0","i");assert(!store.get("0","i"));
  now=220;assert(!store.get("ninth","i")); // Regressed clock invalidates all records.
  now=223;assert(!store.get("ninth","i"));
  // A stricter installed profile must reject a route allowed by defaults.
  auto tuned=make("tuned");nav::MotionLimits limits;limits.max_speed_mps=.1;
  auto segment=nav::MakeForwardPrimitive({},0,0,3,0,0,.5,nav::MotionLimits{});
  assert(!segment.samples.empty());
  tuned.segments={segment};tuned.goal=segment.samples.back().pose;
  tuned.estimated_time_s=segment.estimated_time_s;
  assert(!store.put(tuned,limits));
  segment=nav::MakeForwardPrimitive({},0,0,3,0,0,.1,limits);
  assert(!segment.samples.empty());
  tuned.segments={segment};tuned.goal=segment.samples.back().pose;
  tuned.estimated_time_s=segment.estimated_time_s;
  assert(store.put(tuned,limits));
}
