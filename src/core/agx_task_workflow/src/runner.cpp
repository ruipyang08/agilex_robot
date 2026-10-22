#include "agx_task_workflow/task_registry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

namespace agx_task_workflow {
namespace {
using Clock = std::chrono::steady_clock;
struct Interrupted { FailureKind kind; std::string detail; };

FailureKind Failure(FailureKind kind) {
  switch(kind) {
    case FailureKind::Ordinary: case FailureKind::Cancelled: case FailureKind::Deadline:
    case FailureKind::Fault: case FailureKind::UnconfirmedStop: return kind;
    default: return FailureKind::Fault;
  }
}
void Append(std::string& detail,const std::string& extra) {
  if(extra.empty() || detail==extra)return;
  if(!detail.empty())detail+="; ";
  detail+=extra;
}
class SequentialRun {
 public:
  SequentialRun(const std::vector<PreparedTask>& tasks,const Ports& ports):tasks_(tasks),ports_(ports) {
    for(const auto& task:tasks_) {TaskResult result;result.id=task.id;result.type=task.type;result_.tasks.push_back(result);}
  }
  Result Execute() {
    if(tasks_.empty()) {result_.failed_phase="VALIDATE";result_.detail="no prepared tasks";return result_;}
    try {
      for(index_=0;index_<tasks_.size();++index_) {
        stop_attempted_=false;cleanup_attempted_=false;
        const auto& task=tasks_[index_];auto& record=result_.tasks[index_];
        current_=task.entry_phase;started_=ReadTime();
        // The previous task context has ended. Establish this task's context
        // before evaluating its runtime guard or attributing progress to it.
        if(ports_.cancelled())throw Interrupted{FailureKind::Cancelled,"cancelled"};
        if(ports_.task_begin)ports_.task_begin(task);
        Check();
        if(task.begin)task.begin();
        Check();Phase(current_);Check();
        record.status="running";ports_.task_event(record);Check();
        if(task.metadata_complete && (!task.entry_check || !task.stop || !task.cleanup ||
            !std::isfinite(task.stop_timeout_s) || task.stop_timeout_s<=0.0))
          throw std::invalid_argument("complete task requires bounded entry/stop/cleanup bindings");
        if(task.entry_check) {
          const auto entry=task.entry_check();Absorb(entry);record.stop=entry.stop;
          try {Check();}
          catch(...) {Append(record.detail,entry.detail);throw;}
          if(!entry.ok || (task.metadata_complete && entry.failure!=FailureKind::None))
            return Terminal(Failure(entry.failure),"entry check: "+entry.detail);
        }
        record.dispatched=true;
        auto outcome=task.execute([this](const std::string& phase){Phase(phase);});
        // Preserve effects before checking cancellation triggered while the adapter returned.
        Absorb(outcome);record.stop=outcome.stop;
        // A concurrent guard failure changes control flow, not the physical
        // outcome already returned by the task. Preserve its explanation too.
        try {Check();}
        catch(...) {Append(record.detail,outcome.detail);throw;}
        if(outcome.ok && task.metadata_complete && outcome.failure!=FailureKind::None) {
          outcome.ok=false;outcome.failure=FailureKind::Fault;
          Append(outcome.detail,"success result has a failure classification");
        }
        if(!outcome.ok) {
          SetFailure(Failure(outcome.failure),outcome.detail);
          const bool candidate=task.metadata_complete && ports_.runtime_guard && task.failure_policy==FailurePolicy::Continue &&
                               record.failure==FailureKind::Ordinary;
          if(!candidate)SignalStopping();
          Cleanup();
          bool reconciled=false;
          if(candidate && record.failure==FailureKind::Ordinary && record.stop==StopState::Confirmed &&
              task.continuation_check && ports_.continuation_allowed) {
            Check();
            outcome.failure=record.failure;outcome.stop=record.stop;
            outcome.physical_effect_ids=record.physical_effect_ids;outcome.state_revision=record.state_revision;
            reconciled=task.continuation_check(outcome);
            Check();
            if(reconciled)reconciled=ports_.continuation_allowed(task,outcome);
            Check();
          }
          if(MayContinue(task.failure_policy,record.failure,record.stop,reconciled,!candidate)) {
            record.elapsed_wall_s=Elapsed();
            ports_.task_event(record);Check();
            ports_.phase("TASK_FAILED");Check();
            continue;
          }
          return Terminal(record.failure,record.detail);
        }
        record.detail=outcome.detail;record.failure=FailureKind::None;
        record.elapsed_wall_s=Elapsed();record.status="succeeded";
        ports_.task_event(record);Check();
      }
      index_=tasks_.size()-1;Check();
      result_.success=result_.failed_task_ids.empty();
      result_.completion=result_.success?RunCompletion::Succeeded:RunCompletion::CompletedWithFailures;
      result_.failure=result_.success?FailureKind::None:FailureKind::Ordinary;
      result_.stop=result_.tasks.back().stop;
      // Commit before the terminal notification. A notification exception still fails closed.
      ports_.phase(result_.success?"SUCCEEDED":"COMPLETED_WITH_FAILURES");
      return result_;
    } catch(const Interrupted& interrupted) {return Terminal(interrupted.kind,interrupted.detail);}
    catch(const std::exception& error) {return Terminal(FailureKind::Fault,error.what());}
    catch(...) {return Terminal(FailureKind::Fault,"unknown task exception");}
  }
 private:
  double RawTime() const {
    return ports_.steady_now?ports_.steady_now():std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
  }
  double ReadTime() {
    const double now=RawTime();
    if(!std::isfinite(now) || (std::isfinite(last_now_) && now<last_now_))
      throw Interrupted{FailureKind::Fault,"workflow monotonic clock is invalid"};
    last_now_=now;return now;
  }
  double Elapsed() const {return std::isfinite(last_now_)?std::max(0.0,last_now_-started_):0.0;}
  void Check() {
    ReadTime();
    if(ports_.cancelled())throw Interrupted{FailureKind::Cancelled,"cancelled"};
    if(ports_.runtime_guard) {
      const auto kind=ports_.runtime_guard();
      if(kind!=FailureKind::None)
        throw Interrupted{kind==FailureKind::Ordinary?FailureKind::Fault:Failure(kind),"runtime guard rejected execution"};
    }
    if(ports_.cancelled())throw Interrupted{FailureKind::Cancelled,"cancelled"};
  }
  void Phase(const std::string& value) {
    if(value.empty() || value=="IDLE" || value=="VALIDATE" || value=="SUCCEEDED" || value=="FAILED" ||
        value=="STOPPING" || value=="TASK_FAILED" || value=="COMPLETED_WITH_FAILURES")
      throw std::invalid_argument("task cannot publish reserved workflow phase: "+value);
    Check();current_=value;ports_.phase(value);Check();
  }
  void Absorb(const StepResult& outcome) {
    auto& record=result_.tasks[index_];
    for(const auto& effect:outcome.physical_effect_ids) {
      if(std::find(record.physical_effect_ids.begin(),record.physical_effect_ids.end(),effect)==record.physical_effect_ids.end())
        record.physical_effect_ids.push_back(effect);
      if(std::find(result_.physical_effect_ids.begin(),result_.physical_effect_ids.end(),effect)==result_.physical_effect_ids.end())
        result_.physical_effect_ids.push_back(effect);
    }
    record.state_revision=std::max(record.state_revision,outcome.state_revision);
    result_.state_revision=std::max(result_.state_revision,outcome.state_revision);
  }
  void SetFailure(FailureKind kind,const std::string& detail) {
    auto& record=result_.tasks[index_];result_.success=false;
    record.status=kind==FailureKind::Cancelled?"cancelled":"failed";
    record.failure=Failure(kind);record.failed_phase=current_;record.elapsed_wall_s=Elapsed();
    Append(record.detail,detail);
    result_.failed_task_id=record.id;result_.failed_task_type=record.type;result_.failed_phase=current_;
    if(result_.detail.empty())result_.detail=record.detail;
    if(std::find(result_.failed_task_ids.begin(),result_.failed_task_ids.end(),record.id)==result_.failed_task_ids.end())
      result_.failed_task_ids.push_back(record.id);
  }
  void Escalate(FailureKind cause,const std::string& detail) {
    auto& record=result_.tasks[index_];
    if(record.failure==FailureKind::Ordinary || record.failure==FailureKind::None)record.failure=Failure(cause);
    if(record.failure==FailureKind::Cancelled)record.status="cancelled";
    Append(record.detail,detail);
  }
  void CaptureGuard() {
    try {Check();}
    catch(const Interrupted& interrupted){Escalate(interrupted.kind,interrupted.detail);}
    catch(const std::exception& error){Escalate(FailureKind::Fault,error.what());}
    catch(...){Escalate(FailureKind::Fault,"unknown runtime guard exception");}
  }
  StepResult InvokeBounded(const std::function<StepResult(double)>& hook,const std::string& label) {
    const double timeout=tasks_[index_].stop_timeout_s;
    double start=std::numeric_limits<double>::quiet_NaN(),end=start;
    try {start=RawTime();}catch(...){}
    StepResult outcome;
    try {outcome=hook(std::isfinite(timeout) && timeout>0.0?timeout:1.0);}
    catch(const std::exception& error){outcome={false,label+": "+error.what()};}
    catch(...){outcome={false,"unknown "+label+" exception"};}
    try {end=RawTime();}catch(...){}
    // This detects an overrun after return; the host must supervise/terminate blocked workers.
    if(!std::isfinite(timeout) || timeout<=0.0 || !std::isfinite(start) || !std::isfinite(end) ||
        end<start || end-start>=timeout) {
      outcome.ok=false;outcome.failure=FailureKind::UnconfirmedStop;outcome.stop=StopState::NotConfirmed;
      Append(outcome.detail,label+" did not return within its monotonic bound");
    }
    if(outcome.ok && outcome.failure!=FailureKind::None)outcome.ok=false;
    return outcome;
  }
  void Cleanup() {
    const auto& task=tasks_[index_];auto& record=result_.tasks[index_];
    if(task.stop && !stop_attempted_) {
      stop_attempted_=true;
      const auto stopped=InvokeBounded(task.stop,"stop");Absorb(stopped);record.stop=stopped.stop;
      if(!stopped.ok || stopped.stop!=StopState::Confirmed)
        Escalate(stopped.stop==StopState::Confirmed?Failure(stopped.failure):FailureKind::UnconfirmedStop,
                 "stop: "+stopped.detail);
      CaptureGuard();
    }
    if(task.cleanup && !cleanup_attempted_) {
      cleanup_attempted_=true;
      const auto cleaned=InvokeBounded(task.cleanup,"cleanup");Absorb(cleaned);
      if(!cleaned.ok) {
        record.stop=StopState::NotConfirmed;
        Escalate(Failure(cleaned.failure),"cleanup: "+cleaned.detail);
      }
      CaptureGuard();
    }
  }
  void SignalStopping() {
    if(stopping_notified_)return;
    stopping_notified_=true;
    try {ports_.phase("STOPPING");}
    catch(const std::exception& error){Escalate(FailureKind::Fault,"stopping notification: "+std::string(error.what()));}
    catch(...){Escalate(FailureKind::Fault,"unknown stopping notification exception");}
  }
  Result Terminal(FailureKind kind,const std::string& detail) {
    SetFailure(kind,detail);SignalStopping();Cleanup();
    auto& record=result_.tasks[index_];
    if((!tasks_[index_].stop || record.stop!=StopState::Confirmed) && !fallback_stop_attempted_) {
      fallback_stop_attempted_=true;
      try {ports_.stop();}
      catch(const std::exception& error){Append(record.detail,"stop: "+std::string(error.what()));}
      catch(...){Append(record.detail,"unknown stop exception");}
      CaptureGuard();
    }
    result_.success=false;result_.failure=record.failure;result_.stop=record.stop;
    result_.completion=record.failure==FailureKind::Cancelled?RunCompletion::Cancelled:RunCompletion::Failed;
    result_.detail=record.detail;record.elapsed_wall_s=Elapsed();
    try {ports_.task_event(record);}catch(...){}
    try {ports_.phase("FAILED");}catch(...){}
    return result_;
  }
  const std::vector<PreparedTask>& tasks_;
  const Ports& ports_;
  Result result_;
  std::size_t index_=0;
  std::string current_="VALIDATE";
  double started_=0.0,last_now_=std::numeric_limits<double>::quiet_NaN();
  bool stop_attempted_=false,cleanup_attempted_=false,stopping_notified_=false,fallback_stop_attempted_=false;
};
}  // namespace
Result Run(const std::vector<PreparedTask>& tasks,const Ports& ports) {return SequentialRun(tasks,ports).Execute();}
}  // namespace agx_task_workflow
