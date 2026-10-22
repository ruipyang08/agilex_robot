#include <cassert>
#include <stdexcept>
#include <algorithm>
#include "agx_task_workflow/task_registry.hpp"
using namespace agx_task_workflow;
namespace {
StepResult Stopped() {
  StepResult result{true,"confirmed stop"};result.stop=StopState::Confirmed;return result;
}
struct Rig {
  bool cancel=false,reconciled=true;
  FailureKind guard=FailureKind::None;
  double now=0.0;
  int global_stops=0;
  std::vector<std::string> calls,phases;
  Ports ports{[this]{return cancel;},[this]{++global_stops;},
    [this](const std::string& phase){phases.push_back(phase);},[](const TaskResult&){},
    [this]{return guard;},[this](const PreparedTask&,const StepResult&){calls.push_back("reconcile");return reconciled;},
    [this]{return now;}};
  PreparedTask Task(const std::string& id,bool succeeds=true) {
    PreparedTask task{id,"inert","TASK_RUNNING",[this,id,succeeds](const PhaseSink&){
      calls.push_back(id+":execute");StepResult result{succeeds,"native "+id};
      if(!succeeds){result.failure=FailureKind::Ordinary;result.physical_effect_ids={"effect-"+id};result.state_revision=3;}
      return result;
    }};
    task.metadata_complete=true;task.failure_policy=FailurePolicy::Continue;
    task.entry_check=[this,id]{calls.push_back(id+":entry");return StepResult{true,"entry clear"};};
    task.stop=[this,id](double timeout){assert(timeout>0);calls.push_back(id+":stop");return Stopped();};
    task.cleanup=[this,id](double timeout){assert(timeout>0);calls.push_back(id+":cleanup");return StepResult{true,"joined"};};
    task.stop_timeout_s=1.0;
    task.continuation_check=[](const StepResult&) {return true;};
    return task;
  }
};
void ReturnedFailureDetailSurvivesSimultaneousRuntimeGuard() {
  for(const auto kind:{FailureKind::Fault,FailureKind::Cancelled}) {
    Rig rig;auto task=rig.Task("release");
    task.execute=[&](const PhaseSink&) {
      rig.guard=kind;
      StepResult result{false,"tube release verified; withdrawal failed"};
      result.failure=FailureKind::Ordinary;
      result.physical_effect_ids={"released"};result.state_revision=4;
      return result;
    };
    const auto result=Run({task,rig.Task("next")},rig.ports);
    assert(!result.success && !result.tasks[1].dispatched);
    assert(result.detail.find("tube release verified; withdrawal failed")!=std::string::npos);
    assert(result.tasks[0].detail.find("tube release verified; withdrawal failed")!=std::string::npos);
    assert(result.physical_effect_ids==std::vector<std::string>{"released"});
    assert(result.failure==kind);
  }
}
void ContinuationTruthTable() {
  for(auto policy:{FailurePolicy::Stop,FailurePolicy::Continue})
    for(auto kind:{FailureKind::None,FailureKind::Ordinary,FailureKind::Cancelled,FailureKind::Deadline,FailureKind::Fault,FailureKind::UnconfirmedStop})
      for(auto stop:{StopState::NotConfirmed,StopState::Confirmed})
        for(bool reconciled:{false,true}) for(bool terminal:{false,true})
          assert(MayContinue(policy,kind,stop,reconciled,terminal)==
                 (policy==FailurePolicy::Continue && kind==FailureKind::Ordinary && stop==StopState::Confirmed && reconciled && !terminal));
}
void OrdinaryFailureContinuesWithHistoryAndFreshEntryChecks() {
  Rig rig;
  const auto result=Run({rig.Task("first",false),rig.Task("second")},rig.ports);
  assert(result.completion==RunCompletion::CompletedWithFailures && !result.success);
  assert(result.failure==FailureKind::Ordinary && result.failed_task_ids==std::vector<std::string>{"first"});
  assert(result.tasks[0].status=="failed" && result.tasks[1].status=="succeeded");
  assert(result.physical_effect_ids==std::vector<std::string>{"effect-first"} && result.state_revision==3);
  assert(result.tasks[0].stop==StopState::Confirmed && rig.global_stops==0);
  assert((rig.calls==std::vector<std::string>{"first:entry","first:execute","first:stop","first:cleanup","reconcile","second:entry","second:execute"}));
  assert(std::find(rig.phases.begin(),rig.phases.end(),"TASK_FAILED")!=rig.phases.end());
  assert(rig.phases.back()=="COMPLETED_WITH_FAILURES");
  for(const auto& forbidden:{"SUCCEEDED","FAILED","STOPPING"})
    assert(std::find(rig.phases.begin(),rig.phases.end(),forbidden)==rig.phases.end());
}
void UnsafeContinuationAndEntryRejectionNeverDispatchLaterMotion() {
  for(const auto& mode:{"stop-policy","unreconciled","no-eligibility","no-certificate","certificate-denied","no-runtime-guard","unconfirmed","fault","deadline","cancel"}) {
    Rig rig;auto first=rig.Task("first",false);
    const std::string value=mode;
    if(value=="stop-policy")first.failure_policy=FailurePolicy::Stop;
    if(value=="unreconciled")rig.reconciled=false;
    if(value=="no-eligibility")rig.ports.continuation_allowed={};
    if(value=="no-certificate")first.continuation_check={};
    if(value=="certificate-denied")first.continuation_check=[](const StepResult&){return false;};
    if(value=="no-runtime-guard")rig.ports.runtime_guard={};
    if(value=="unconfirmed")first.stop=[&](double){rig.calls.push_back("first:stop");return StepResult{true,"request only"};};
    if(value=="fault" || value=="deadline" || value=="cancel") {
      first.execute=[&,value](const PhaseSink&){StepResult r{false,value};r.failure=value=="fault"?FailureKind::Fault:value=="deadline"?FailureKind::Deadline:FailureKind::Cancelled;return r;};
    }
    const auto result=Run({first,rig.Task("second")},rig.ports);
    assert(!result.success && result.completion!=RunCompletion::CompletedWithFailures);
    assert(!result.tasks[1].dispatched);
    assert(std::count(rig.calls.begin(),rig.calls.end(),"first:stop")==1);
    assert(std::count(rig.calls.begin(),rig.calls.end(),"first:cleanup")==1);
    assert(rig.phases.back()=="FAILED");
    if(value=="unconfirmed")assert(result.stop==StopState::NotConfirmed && rig.global_stops==1);
  }
  Rig rig;auto second=rig.Task("second");
  second.entry_check=[&]{rig.calls.push_back("second:entry");return StepResult{false,"unsafe entry"};};
  const auto result=Run({rig.Task("first",false),second},rig.ports);
  assert(!result.tasks[1].dispatched && result.failed_task_id=="second");
  assert(result.tasks[0].status=="failed" && result.completion==RunCompletion::Failed);
}
void CancellationAndFaultsDuringCleanupOrReportingAreTerminal() {
  for(const auto& boundary:{"stop","cleanup","reconcile","event","phase"}) {
    Rig rig;auto first=rig.Task("first",false);const std::string where=boundary;
    if(where=="stop")first.stop=[&](double){rig.calls.push_back("first:stop");rig.cancel=true;return Stopped();};
    if(where=="cleanup")first.cleanup=[&](double){rig.calls.push_back("first:cleanup");rig.cancel=true;return StepResult{true,"joined"};};
    if(where=="reconcile")rig.ports.continuation_allowed=[&](const PreparedTask&,const StepResult&){rig.guard=FailureKind::Fault;return true;};
    if(where=="event")rig.ports.task_event=[](const TaskResult& task){if(task.status=="failed")throw std::runtime_error("report failed");};
    if(where=="phase")rig.ports.phase=[](const std::string& phase){if(phase=="TASK_FAILED")throw std::runtime_error("phase failed");};
    const auto result=Run({first,rig.Task("second")},rig.ports);
    assert(!result.tasks[1].dispatched && !result.success);
    assert(result.completion==(where=="stop" || where=="cleanup"?RunCompletion::Cancelled:RunCompletion::Failed));
    assert(std::count(rig.calls.begin(),rig.calls.end(),"first:stop")==1);
    assert(std::count(rig.calls.begin(),rig.calls.end(),"first:cleanup")==1);
    assert(result.physical_effect_ids==std::vector<std::string>{"effect-first"});
  }
}
void ReturnedEffectsPrecedeFinalCancellationAndProgressGuards() {
  Rig rig;auto first=rig.Task("first");
  first.execute=[&](const PhaseSink&){rig.cancel=true;StepResult r{true,"captured"};r.physical_effect_ids={"capture"};r.state_revision=7;return r;};
  const auto result=Run({first,rig.Task("second")},rig.ports);
  assert(result.completion==RunCompletion::Cancelled && result.physical_effect_ids==std::vector<std::string>{"capture"});
  assert(result.state_revision==7 && !result.tasks[1].dispatched);
  Rig progress;auto task=progress.Task("first");bool escaped=false;
  progress.ports.phase=[&](const std::string& phase){if(phase=="INNER")progress.guard=FailureKind::Fault;};
  task.execute=[&](const PhaseSink& phase){phase("INNER");escaped=true;return StepResult{true,"wrong"};};
  assert(!Run({task},progress.ports).success && !escaped);
}
void HookOverrunsAndExceptionsCannotConfirmContinuation() {
  for(const auto& boundary:{"stop-timeout","cleanup-timeout","stop-throw","cleanup-throw"}) {
    Rig rig;auto task=rig.Task("first",false);const std::string where=boundary;
    if(where.find("stop")==0)task.stop=[&](double){rig.calls.push_back("first:stop");if(where=="stop-throw")throw std::runtime_error("blocked stop");rig.now+=2;return Stopped();};
    else task.cleanup=[&](double){rig.calls.push_back("first:cleanup");if(where=="cleanup-throw")throw std::runtime_error("blocked cleanup");rig.now+=2;return StepResult{true,"late joined"};};
    const auto result=Run({task,rig.Task("second")},rig.ports);
    assert(!result.tasks[1].dispatched && !result.success && result.stop==StopState::NotConfirmed);
    assert(rig.global_stops==1);
    assert(std::count(rig.calls.begin(),rig.calls.end(),"first:stop")==1);
    assert(std::count(rig.calls.begin(),rig.calls.end(),"first:cleanup")==1);
  }
}
void CancellationFromRuntimeGuardCannotEscapeProgressBoundary() {
  Rig rig;auto task=rig.Task("one");bool inner=false,escaped=false;
  rig.ports.phase=[&](const std::string& phase){if(phase=="INNER")inner=true;};
  rig.ports.runtime_guard=[&]{if(inner)rig.cancel=true;return FailureKind::None;};
  task.execute=[&](const PhaseSink& phase){phase("INNER");escaped=true;return StepResult{true,"wrong"};};
  const auto result=Run({task},rig.ports);
  assert(!escaped && result.completion==RunCompletion::Cancelled);
}
void StopAndCleanupEffectsReachReconciliationBeforeContinuation() {
  Rig rig;auto task=rig.Task("first",false);
  task.stop=[](double){auto r=Stopped();r.physical_effect_ids={"release"};r.state_revision=4;return r;};
  task.cleanup=[](double){StepResult r{true,"withdrawn"};r.physical_effect_ids={"clear"};r.state_revision=5;return r;};
  rig.ports.continuation_allowed=[](const PreparedTask&,const StepResult& outcome){
    assert(outcome.state_revision==5 && outcome.stop==StopState::Confirmed);
    assert((outcome.physical_effect_ids==std::vector<std::string>{"effect-first","release","clear"}));
    return true;
  };
  const auto result=Run({task,rig.Task("second")},rig.ports);
  assert(result.completion==RunCompletion::CompletedWithFailures && result.state_revision==5);
  assert(result.physical_effect_ids.size()==3);
}
void TaskBeginPrecedesIdentityReportsAndNewDeadlineChecks() {
  Rig rig;std::string active;double deadline=0;bool advance=false;
  std::vector<std::string> trace;
  rig.ports.steady_now=[&]{if(advance){rig.now+=10;advance=false;}return rig.now;};
  rig.ports.task_begin=[&](const PreparedTask& task){active=task.id;deadline=rig.now+1;trace.push_back(active+":begin");};
  rig.ports.runtime_guard=[&]{return rig.now<deadline?FailureKind::None:FailureKind::Deadline;};
  rig.ports.phase=[&](const std::string& phase){if(phase=="TASK_RUNNING")trace.push_back(active+":phase");};
  rig.ports.task_event=[&](const TaskResult& task){
    assert(active==task.id);
    if(task.status=="running")trace.push_back(active+":event");
    if(task.id=="first" && task.status=="succeeded") {
      rig.ports.runtime_guard=[&] {
        const auto result=rig.now<deadline?FailureKind::None:FailureKind::Deadline;
        if(active=="first")advance=true;
        return result;
      };
    }
  };
  const auto result=Run({rig.Task("first"),rig.Task("second")},rig.ports);
  assert(result.success && result.tasks[1].dispatched);
  assert((trace==std::vector<std::string>{"first:begin","first:phase","first:event","second:begin","second:phase","second:event"}));
  for(const auto& boundary:{"before","inside","fault","throw"}) {
    Rig stop;int begins=0;const std::string where=boundary;
    stop.cancel=where=="before";
    stop.ports.task_begin=[&](const PreparedTask&) {
      ++begins;
      if(where=="inside")stop.cancel=true;
      if(where=="fault")stop.guard=FailureKind::Fault;
      if(where=="throw")throw std::runtime_error("admission failed");
    };
    const auto halted=Run({stop.Task("one")},stop.ports);
    assert(!halted.success && !halted.tasks[0].dispatched);
    assert(begins==(where=="before"?0:1));
    assert(std::find(stop.calls.begin(),stop.calls.end(),"one:entry")==stop.calls.end());
  }
}
}
int main() {
  ReturnedFailureDetailSurvivesSimultaneousRuntimeGuard();
  TaskBeginPrecedesIdentityReportsAndNewDeadlineChecks();
  ContinuationTruthTable();
  OrdinaryFailureContinuesWithHistoryAndFreshEntryChecks();
  UnsafeContinuationAndEntryRejectionNeverDispatchLaterMotion();
  CancellationAndFaultsDuringCleanupOrReportingAreTerminal();
  ReturnedEffectsPrecedeFinalCancellationAndProgressGuards();
  HookOverrunsAndExceptionsCannotConfirmContinuation();
  CancellationFromRuntimeGuardCannotEscapeProgressBoundary();
  StopAndCleanupEffectsReachReconciliationBeforeContinuation();
  for(int count:{1,2,3}) for(int failure:{-1,0,1,2}) {
    if(failure>=count)continue;
    std::vector<PreparedTask> tasks; std::vector<int> calls; int stops=0;
    for(int i=0;i<count;++i)tasks.push_back({std::to_string(i),"inert","TASK_RUNNING",
      [&,i](const PhaseSink& phase){calls.push_back(i);phase("INNER");return StepResult{i!=failure,"native"};}});
    Ports ports{[]{return false;},[&]{++stops;},[](const std::string&){},[](const TaskResult&){}};
    auto r=Run(tasks,ports); assert(r.success==(failure<0));assert(stops==(failure<0?0:1));
    assert(calls.size()==std::size_t(failure<0?count:failure+1));
    for(int i=0;i<count;++i) {
      assert(r.tasks[i].dispatched==(failure<0 || i<=failure));
      assert(r.tasks[i].status==(failure<0 || i<failure?"succeeded":i==failure?"failed":"not_run"));
    }
    if(failure>=0)assert(r.failed_task_id==std::to_string(failure) && r.failed_phase=="INNER" && r.detail=="native");
  }
  // Each observed boundary can cancel; no next callback may escape.
  for(const auto& boundary:{"before", "entry", "start", "execute", "finish"}) {
    bool cancel=std::string(boundary)=="before";int calls=0,stops=0;
    PreparedTask task{"one","inert","TASK_RUNNING",[&](const PhaseSink&){++calls;if(std::string(boundary)=="execute")cancel=true;return StepResult{true,"done"};}};
    Ports p{[&]{return cancel;},[&]{++stops;},[&](const std::string& s){if(s=="TASK_RUNNING" && std::string(boundary)=="entry")cancel=true;},
      [&](const TaskResult& r){if((r.status=="running" && std::string(boundary)=="start") || (r.status=="succeeded" && std::string(boundary)=="finish"))cancel=true;}};
    auto second=task;second.id="two";
    auto r=Run({task,second},p);
    assert(!r.success && stops==1 && calls==((std::string(boundary)=="execute" || std::string(boundary)=="finish")?1:0));
    assert(!r.tasks[1].dispatched);
  }
  for(const auto& boundary:{"entry","start","execute","finish"}) {
    int stops=0,calls=0;
    Ports p{[]{return false;},[&]{++stops;throw std::runtime_error("stop error");},
      [&](const std::string& s){if(s=="TASK_RUNNING" && std::string(boundary)=="entry")throw std::runtime_error("original");},
      [&](const TaskResult& r){if((r.status=="running" && std::string(boundary)=="start") || (r.status=="succeeded" && std::string(boundary)=="finish"))throw std::runtime_error("original");}};
    PreparedTask task{"one","inert","TASK_RUNNING",[&](const PhaseSink&){++calls;if(std::string(boundary)=="execute")throw std::runtime_error("original");return StepResult{true,"done"};}};
    auto r=Run({task,task},p);assert(!r.success && stops==1 && r.detail.find("original")!=std::string::npos);
    assert(calls<=1);
  }
  for(const auto& forbidden:{"SUCCEEDED","FAILED","STOPPING","VALIDATE","IDLE","TASK_FAILED","COMPLETED_WITH_FAILURES"}) {
    int stops=0;bool premature_success=false;
    Ports guarded{[]{return false;},[&]{++stops;},[&](const std::string& phase){if(phase=="SUCCEEDED")premature_success=true;},[](const TaskResult&){}};
    PreparedTask task{"one","inert","TASK_RUNNING",[&](const PhaseSink& phase){phase(forbidden);return StepResult{true,"done"};}};
    const auto result=Run({task},guarded);
    assert(!result.success && stops==1 && !premature_success);
  }
  int finishes=0;
  Ports once{[]{return false;},[]{},[](const std::string&){},[&](const TaskResult& r){if(r.status=="failed")++finishes;}};
  PreparedTask broken{"one","inert","TASK_RUNNING",[](const PhaseSink&){return StepResult{false,"native"};}};
  assert(!Run({broken},once).success && finishes==1);
  int stops=0; Ports p{[]{return false;},[&]{++stops;},[](const std::string&){},[](const TaskResult&){}};
  assert(!Run({},p).success && stops==0);
}
