#include "agx_navigation_tasks/bounded_motion.hpp"
#include <cassert>
#include <future>
#include <mutex>
#include <limits>
using namespace agx_navigation_tasks;
int main() {
  int stops=0;std::atomic<bool> cancel{false};
  auto guard=[&]{return cancel?agx_task_workflow::FailureKind::Cancelled:agx_task_workflow::FailureKind::None;};
  auto stop=[&]{++stops;};
  for(double invalid:{1e300,std::numeric_limits<double>::denorm_min()}) {
    bool rejected=false;
    try {RunBoundedMotion([](const MotionStopToken&){return agx_task_workflow::StepResult{true,"unexpected"};},guard,stop,invalid,.1,1);}
    catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  auto ok=RunBoundedMotion([](const MotionStopToken&) {return agx_task_workflow::StepResult{true,"done"};},guard,stop,1,.1,1);
  assert(ok.step.ok && !ok.exit_required && stops==0);
  auto late=RunBoundedMotion([&](const MotionStopToken&){cancel=true;return agx_task_workflow::StepResult{true,"captured",{"capture"},4};},guard,stop,1,.1,1);
  assert(!late.step.ok && late.step.failure==agx_task_workflow::FailureKind::Cancelled && late.step.physical_effect_ids.size()==1 && stops==1);
  cancel=false;stops=0;
  auto unblock=std::make_shared<std::promise<void>>();auto wait=unblock->get_future().share();
  auto ended=std::make_shared<std::promise<bool>>();auto done=ended->get_future();
  auto blocked=RunBoundedMotion([wait,ended](const MotionStopToken& token) {
    wait.wait(); ended->set_value(token->load());return agx_task_workflow::StepResult{true,"late response"};
  },guard,stop,.01,.01,1);
  assert(blocked.exit_required && !blocked.step.ok && blocked.step.failure==agx_task_workflow::FailureKind::UnconfirmedStop && stops>=2);
  unblock->set_value();assert(done.get());
  stops=0;
  auto failed=RunBoundedMotion([](const MotionStopToken&)->agx_task_workflow::StepResult {throw std::runtime_error("device error");},guard,stop,1,.1,1);
  assert(!failed.step.ok && !failed.exit_required && stops==1);
  // A vendor dispatch already past its token read may race the first stop.
  // It must still receive a stop after its worker has returned and joined.
  std::promise<void> first_stop;
  auto stopped=first_stop.get_future().share();
  std::mutex trace_mutex;std::vector<std::string> trace;
  stops=0;cancel=false;
  auto raced=RunBoundedMotion([&](const MotionStopToken& token) {
    if(token->load())return agx_task_workflow::StepResult{false,"interrupted"};
    cancel=true;stopped.wait();
    {std::lock_guard<std::mutex> lock(trace_mutex);trace.push_back("motion");}
    return agx_task_workflow::StepResult{true,"late RPC returned"};
  },guard,[&] {
    {std::lock_guard<std::mutex> lock(trace_mutex);trace.push_back("stop");}
    if(stops++==0)first_stop.set_value();
  },1,.1,1);
  assert(!raced.step.ok && !raced.exit_required && raced.step.failure==agx_task_workflow::FailureKind::Cancelled);
  assert(trace.front()=="stop" && trace.back()=="stop" && stops>=2);
}
