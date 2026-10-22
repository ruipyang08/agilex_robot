#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <thread>
#include "agx_task_workflow/runner.hpp"

namespace agx_navigation_tasks {
constexpr double kMaximumMotionTimeoutSeconds=3600.0;
using MotionStopToken=std::shared_ptr<const std::atomic<bool>>;
struct BoundedMotionResult { agx_task_workflow::StepResult step; bool exit_required=false; };
// Work owns everything it captures. It must test the token at each command
// boundary. A blocked producer cannot be killed in C++: the caller must flush
// evidence and _Exit when exit_required is true, retaining watchdog stopping.
// request_stop is an independent bounded, stop-only path; joining is NOT a
// measured hardware stop acknowledgement.
inline BoundedMotionResult RunBoundedMotion(
    std::function<agx_task_workflow::StepResult(const MotionStopToken&)> work,
    std::function<agx_task_workflow::FailureKind()> guard,std::function<void()> request_stop,
    double timeout_s,double join_timeout_s=15.0,unsigned poll_ms=25) {
  using Clock=std::chrono::steady_clock;
  const double minimum_s=std::chrono::duration<double>(Clock::duration(1)).count();
  if (!work || !guard || !request_stop || !std::isfinite(timeout_s) || timeout_s<minimum_s ||
      timeout_s>kMaximumMotionTimeoutSeconds || !std::isfinite(join_timeout_s) ||
      join_timeout_s<minimum_s || join_timeout_s>60 || poll_ms==0 || poll_ms>1000)
    throw std::invalid_argument("invalid bounded motion ports or deadlines");
  auto duration=[](double v) {return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(v));};
  auto check=[&] {try{return guard();}catch(...){return agx_task_workflow::FailureKind::Fault;}};
  const auto initial=check();
  if(initial!=agx_task_workflow::FailureKind::None) {agx_task_workflow::StepResult r{false,"motion entry interrupted"};r.failure=initial;return {r};}
  auto token=std::make_shared<std::atomic<bool>>(false);
  auto promise=std::make_shared<std::promise<agx_task_workflow::StepResult>>();auto future=promise->get_future();
  const auto deadline=Clock::now()+duration(timeout_s);
  std::thread worker([work=std::move(work),token,promise] {
    try {promise->set_value(work(token));}
    catch(const std::exception& e) {promise->set_value({false,e.what()});}
    catch(...) {promise->set_value({false,"motion worker threw"});}
  });
  auto stop=[&] {token->store(true);try{request_stop();}catch(...){}};
  agx_task_workflow::FailureKind failure=agx_task_workflow::FailureKind::None;
  auto join_deadline=Clock::time_point::max();
  while(future.wait_for(std::chrono::milliseconds(poll_ms))!=std::future_status::ready) {
    const auto current=check();
    if(failure==agx_task_workflow::FailureKind::None && (current!=agx_task_workflow::FailureKind::None || Clock::now()>=deadline)) {
      failure=current==agx_task_workflow::FailureKind::None?agx_task_workflow::FailureKind::Deadline:current;
      join_deadline=Clock::now()+duration(join_timeout_s);
    }
    // A vendor RPC may already have passed its token check when interruption
    // arrives. Repeat the independent stop while draining that worker.
    if(failure!=agx_task_workflow::FailureKind::None)stop();
    if(Clock::now()>=join_deadline) {
      worker.detach();agx_task_workflow::StepResult r{false,"motion producer did not join within stop bound"};
      r.failure=agx_task_workflow::FailureKind::UnconfirmedStop;return {r,true};
    }
  }
  worker.join();auto result=future.get();
  const auto final=check();
  if(failure==agx_task_workflow::FailureKind::None && final!=agx_task_workflow::FailureKind::None)failure=final;
  if(failure==agx_task_workflow::FailureKind::None && Clock::now()>=deadline)failure=agx_task_workflow::FailureKind::Deadline;
  if(failure!=agx_task_workflow::FailureKind::None) {result.ok=false;result.failure=failure;result.detail="motion interrupted";}
  // The final stop is after the producer has joined, so a late accepted vendor
  // command cannot become the last request. Physical stop still needs evidence.
  if(!result.ok)stop();
  return {result,false};
}
}  // namespace agx_navigation_tasks
