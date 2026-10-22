#include "agx_navigation_tasks/navigation_task.hpp"
#include <cassert>
#include <vector>
#include <string>
using namespace agx_navigation_tasks;
using namespace agx_task_workflow;
struct Fixture {
  std::vector<std::string> called, phases;
  std::string fail, cancel_at;
  bool cancelled=false;
  NavigationTaskPorts ports;
  Fixture() {
    ports.runtime_guard=[this] {return cancelled?FailureKind::Cancelled:FailureKind::None;};
    ports.invalidate_arrival=[this] {called.push_back("invalidate_arrival");};
    ports.plan_route=Step("plan_route");
    ports.establish_departure_hold=Step("establish_departure_hold");
    ports.prepare_travel=Step("prepare_travel");
    ports.verify_travel_ready=Step("verify_travel_ready");
    ports.revalidate_route=Step("revalidate_route");
    ports.release_hold=Step("release_hold");ports.travel=Step("travel");
    ports.verify_stop=Step("verify_stop");ports.acquire_arrival_hold=Step("acquire_arrival_hold");
    ports.commit_arrival=Step("commit_arrival");
  }
  std::function<StepResult()> Step(std::string name) {
    return [this,name] {
      called.push_back(name);
      StepResult result{name!=fail,name};
      if(!result.ok)result.failure=FailureKind::Ordinary;
      if(name==cancel_at) {
        cancelled=true;result.physical_effect_ids={"observed-effect"};result.state_revision=17;
      }
      return result;
    };
  }
  StepResult Run() {return RunNavigationTask(ports,[this](const std::string& p){phases.push_back(p);});}
};
int main() {
  const std::vector<std::string> success{"plan_route","invalidate_arrival","establish_departure_hold",
    "prepare_travel","verify_travel_ready","revalidate_route","release_hold","travel",
    "verify_stop","acquire_arrival_hold","commit_arrival"};
  Fixture ready;assert(ready.Run().ok);assert(ready.called==success);
  // Every failing port terminates before all later preparation/motion/effects.
  for(std::size_t i=0;i<success.size();++i) {
    if(success[i]=="invalidate_arrival")continue;
    Fixture f;f.fail=success[i];auto result=f.Run();
    assert(!result.ok && result.failure==FailureKind::Ordinary);
    assert(f.called==std::vector<std::string>(success.begin(),success.begin()+i+1));
    if(f.fail=="plan_route")assert(f.phases.back()=="PLAN_ROUTE");
    if(f.fail=="revalidate_route")assert(f.phases.back()=="REVALIDATE_ROUTE");
  }
  // A late successful planning RPC cannot allow physical dispatch after cancel.
  for(std::size_t i=0;i<success.size();++i) {
    if(success[i]=="invalidate_arrival")continue;
    Fixture f;f.cancel_at=success[i];auto result=f.Run();
    assert(!result.ok && result.failure==FailureKind::Cancelled);
    assert(f.called==std::vector<std::string>(success.begin(),success.begin()+i+1));
    assert(result.state_revision==17 && result.physical_effect_ids==std::vector<std::string>{"observed-effect"});
  }
  Fixture callback;auto denied=RunNavigationTask(callback.ports,[&](const std::string& p) {
    if(p=="RELEASE_BASE")callback.cancelled=true;
  });
  assert(!denied.ok && callback.called==std::vector<std::string>(success.begin(),success.begin()+6));
  Fixture absent;absent.ports.revalidate_route={};assert(!absent.Run().ok && absent.called.empty());
  Fixture missing;missing.ports.plan_route={};assert(!missing.Run().ok && missing.called.empty());
}
