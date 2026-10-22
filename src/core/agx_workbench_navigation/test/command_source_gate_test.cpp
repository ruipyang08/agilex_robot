#include "agx_workbench_navigation/command_source_gate.hpp"
#include <cassert>
#include <future>
#include <limits>

using namespace agx_workbench_navigation;

void DeliveryDispositionSeparatesAcknowledgmentFromInstallation() {
  CommandSourceGate gate;
  CommandBinding a{"session","grant-a","goal-a",{"map",100,{1,2,0},{0,0,0,1}}};
  auto b=a;b.grant_id="grant-b";b.source_goal_id="goal-b";b.endpoint.stamp_ns=200;
  b.endpoint.position[0]=3;
  int installed=0;
  const auto delivers=[&](const CommandGoalEndpoint& endpoint,PlanDisposition expected) {
    const int before=installed;
    assert(gate.DeliverPlan(endpoint,[&]{++installed;})==expected);
    assert(installed==before+(expected==PlanDisposition::Installed ? 1 : 0));
  };
  delivers(a.endpoint,PlanDisposition::Rejected);
  assert(gate.Bind(a,[]{}));delivers(a.endpoint,PlanDisposition::Installed);
  assert(gate.Bind(b,[]{}));delivers(a.endpoint,PlanDisposition::SupersededDelivery);
  assert(installed==1 && !gate.Inspect([]{return true;}));
  assert(!gate.AcceptPlan(a.endpoint,[&]{++installed;}));
  assert(gate.Compute([&](const auto& current,auto sequence,bool ready) {
    assert(current.source_goal_id==b.source_goal_id && sequence==1 && !ready);return true;
  }));
  delivers(b.endpoint,PlanDisposition::Installed);
  delivers(a.endpoint,PlanDisposition::SupersededDelivery);
  auto unknown=a.endpoint;unknown.stamp_ns=150;unknown.position[1]=99;
  delivers(unknown,PlanDisposition::SupersededDelivery);
  assert(installed==2 && gate.Inspect([]{return true;}));
  assert(gate.Compute([&](const auto& current,auto sequence,bool ready) {
    assert(current.source_goal_id==b.source_goal_id && sequence==2 && ready);return true;
  }));
  const auto rejects=[&](const CommandGoalEndpoint& endpoint) {
    delivers(endpoint,PlanDisposition::Rejected);
    assert(installed==2 && gate.Inspect([]{return true;}));
  };
  auto wrong=a.endpoint;wrong.frame="odom";rejects(wrong);
  wrong=a.endpoint;wrong.stamp_ns=0;rejects(wrong);
  wrong=a.endpoint;wrong.position[0]=std::numeric_limits<double>::quiet_NaN();rejects(wrong);
  wrong=a.endpoint;wrong.position[2]=std::numeric_limits<double>::infinity();rejects(wrong);
  wrong=a.endpoint;wrong.orientation[3]=.5;rejects(wrong);
  wrong=a.endpoint;wrong.orientation[0]=std::numeric_limits<double>::infinity();rejects(wrong);
  wrong=b.endpoint;wrong.stamp_ns++;rejects(wrong);
  wrong=b.endpoint;wrong.position[0]+=.1;rejects(wrong);
  wrong=b.endpoint;wrong.orientation={0,0,1,0};rejects(wrong);
  auto sign=b.endpoint;for(auto& value:sign.orientation)value=-value;
  delivers(sign,PlanDisposition::Installed);
}

void SupersededDeliveryCannotInstallOrChangeReadiness() {
  CommandSourceGate gate;
  CommandBinding a{"session","grant-a","goal-a",{"map",100,{1,2,0},{0,0,0,1}}};
  auto b=a;b.grant_id="grant-b";b.source_goal_id="goal-b";b.endpoint.stamp_ns=200;
  b.endpoint.position[0]=3;
  int installed=0;
  assert(!gate.AcceptPlan(a.endpoint,[&]{++installed;})); // No binding.
  assert(gate.Bind(a,[]{}));
  assert(gate.AcceptPlan(a.endpoint,[&]{++installed;}));
  assert(gate.Bind(b,[]{}));
  assert(!gate.AcceptPlan(a.endpoint,[&]{++installed;}));
  assert(installed==1 && !gate.Inspect([]{return true;}));
  assert(gate.Compute([&](const auto& current,auto sequence,bool ready) {
    assert(current.source_goal_id==b.source_goal_id && sequence==1 && !ready);return true;
  }));
  assert(gate.AcceptPlan(b.endpoint,[&]{++installed;}));
  auto unknown=a.endpoint;unknown.stamp_ns=150;unknown.position[1]=99;
  assert(!gate.AcceptPlan(unknown,[&]{++installed;}));
  assert(!gate.AcceptPlan(a.endpoint,[&]{++installed;}));
  assert(installed==2 && gate.Inspect([]{return true;}));
  assert(gate.Compute([&](const auto& current,auto sequence,bool ready) {
    assert(current.source_goal_id==b.source_goal_id && sequence==2 && ready);return true;
  }));
  const auto rejects=[&](const CommandGoalEndpoint& endpoint) {
    assert(!gate.AcceptPlan(endpoint,[&]{++installed;}));
    assert(installed==2 && gate.Inspect([]{return true;}));
  };
  auto wrong=a.endpoint;wrong.frame="odom";rejects(wrong);
  wrong=a.endpoint;wrong.stamp_ns=0;rejects(wrong);
  wrong=a.endpoint;wrong.position[0]=std::numeric_limits<double>::quiet_NaN();rejects(wrong);
  wrong=a.endpoint;wrong.position[2]=std::numeric_limits<double>::infinity();rejects(wrong);
  wrong=a.endpoint;wrong.orientation[3]=.5;rejects(wrong);
  wrong=a.endpoint;wrong.orientation[0]=std::numeric_limits<double>::infinity();rejects(wrong);
  wrong=b.endpoint;wrong.stamp_ns++;rejects(wrong);
  wrong=b.endpoint;wrong.position[0]+=.1;rejects(wrong);
  wrong=b.endpoint;wrong.orientation={0,0,1,0};rejects(wrong);
  auto sign=b.endpoint;for(auto& value:sign.orientation)value=-value;
  assert(gate.AcceptPlan(sign,[&]{++installed;})); // Same quaternion rotation.
  assert(installed==3);
}

void RebindWaitsForAnAlreadyInstallingPlan() {
  CommandSourceGate gate;
  CommandBinding a{"session","grant-a","goal-a",{"map",100,{1,0,0},{0,0,0,1}}};
  auto b=a;b.grant_id="grant-b";b.source_goal_id="goal-b";b.endpoint.stamp_ns=200;
  assert(gate.Bind(a,[]{}));
  std::promise<void> installing,release;
  auto released=release.get_future().share();
  int active_plan=0;
  auto old=std::async(std::launch::async,[&] {
    return gate.DeliverPlan(a.endpoint,[&]{installing.set_value();released.wait();active_plan=1;});
  });
  installing.get_future().wait();
  auto next=std::async(std::launch::async,[&]{return gate.Bind(b,[&]{active_plan=0;});});
  assert(next.wait_for(std::chrono::milliseconds(20))==std::future_status::timeout);
  release.set_value();assert(old.get()==PlanDisposition::Installed);assert(next.get());
  assert(active_plan==0 && !gate.Inspect([]{return true;}));
  assert(!gate.AcceptPlan(a.endpoint,[&]{active_plan=1;}));
  assert(active_plan==0);
}

int main() {
  DeliveryDispositionSeparatesAcknowledgmentFromInstallation();
  SupersededDeliveryCannotInstallOrChangeReadiness();
  RebindWaitsForAnAlreadyInstallingPlan();
  CommandSourceGate gate;
  CommandBinding a{"session","grant-a","goal-a",{"map",100,{1,2,0},{0,0,0,1}}};
  int resets=0,installed=0;
  assert(gate.Bind(a,[&]{++resets;}));
  assert(resets==1);
  gate.Compute([&](const auto& bound,auto sequence,bool ready) {
    assert(bound.source_goal_id=="goal-a" && sequence==1 && !ready);
    return true;
  });
  assert(gate.AcceptPlan(a.endpoint,[&]{++installed;}));
  assert(gate.Bind(a,[&]{++resets;}));
  assert(resets==1 && installed==1); // Retry cannot reset a live producer.
  auto changed=a;changed.endpoint.stamp_ns=103;changed.endpoint.position[0]+=1;
  assert(!gate.Bind(changed,[&]{++resets;}));
  changed.grant_id="another-grant";
  assert(!gate.Bind(changed,[&]{++resets;})); // A source operation is immutable too.
  changed=a;changed.source_goal_id="another-goal";changed.endpoint.stamp_ns=103;
  assert(!gate.Bind(changed,[&]{++resets;}));
  std::promise<void> computing,release;
  auto released=release.get_future().share();
  auto worker=std::async(std::launch::async,[&] {
    return gate.Compute([&](const auto& bound,auto sequence,bool ready) {
      assert(bound.grant_id=="grant-a" && sequence==2 && ready);
      computing.set_value();released.wait();return true;
    });
  });
  computing.get_future().wait();
  auto b=a;b.grant_id="grant-b";b.source_goal_id="goal-b";b.endpoint.stamp_ns=101;
  auto binding=std::async(std::launch::async,[&]{return gate.Bind(b,[&]{++resets;});});
  assert(binding.wait_for(std::chrono::milliseconds(20))==std::future_status::timeout);
  release.set_value();assert(worker.get());assert(binding.get());
  assert(!gate.AcceptPlan(a.endpoint,[&]{++installed;}));
  gate.Compute([&](const auto& bound,auto sequence,bool ready) {
    assert(bound.grant_id=="grant-b" && sequence==1 && !ready);return true;
  });
  auto wrong=b.endpoint;wrong.position[0]+=0.1;
  assert(!gate.AcceptPlan(wrong,[&]{++installed;}));
  assert(gate.AcceptPlan(b.endpoint,[&]{++installed;}));
  gate.Compute([&](const auto& bound,auto sequence,bool ready) {
    assert(bound.grant_id=="grant-b" && sequence==2 && ready);return true;
  });
  assert(!gate.Bind(a,[]{}));
  changed=a;changed.endpoint.stamp_ns=105;
  assert(!gate.Bind(changed,[]{})); // Superseding a binding does not free its identity.
  auto invalid=b;invalid.grant_id="reuse-stamp";
  assert(!gate.Bind(invalid,[]{}));
  invalid.endpoint.stamp_ns=102;invalid.endpoint.frame="odom";
  assert(!gate.Bind(invalid,[]{}));
  invalid.endpoint.frame="map";invalid.endpoint.position[0]=std::numeric_limits<double>::quiet_NaN();
  assert(!gate.Bind(invalid,[]{}));
  agx_workbench_navigation::CommandSourceGate route_gate;
  auto routed=a;routed.invocation_id="invocation";routed.route_id="route";routed.task_deadline_s=100;
  assert(route_gate.Bind(routed,[]{}));
  auto changed_route=routed;changed_route.route_id="foreign";
  assert(!route_gate.Bind(changed_route,[]{}));
  changed_route=routed;changed_route.task_deadline_s=101;
  assert(!route_gate.Bind(changed_route,[]{}));
}
