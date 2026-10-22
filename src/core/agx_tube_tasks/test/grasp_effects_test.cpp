#include <cassert>
#include <memory>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <new>
#include <limits>
#include <stdexcept>
#include <thread>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>
#include "agx_tube_tasks/grasp_task.hpp"
// Test-only precise allocation failure. Disabled unless one fixture arms it.
namespace { std::atomic<bool> fail_next_allocation{false}; }
void* operator new(std::size_t size) {
  if (fail_next_allocation.exchange(false)) throw std::bad_alloc();
  if (void* result = std::malloc(size ? size : 1)) return result;
  throw std::bad_alloc();
}
void operator delete(void* value) noexcept { std::free(value); }
void operator delete(void* value, std::size_t) noexcept { std::free(value); }

using namespace agx_tube_tasks;
using namespace agxcore;
struct Harness {
  double source = 10.0, receipt = 20.0;
  std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state = std::make_shared<agx_robot_task_runtime::RobotStateStore>(
      [this]{return source;}, [this]{return receipt;}, 1.0);
  std::shared_ptr<GraspEffectAdapter> adapter = std::make_shared<GraspEffectAdapter>(
      state, "run", "grasp", "run/grasp/1", "tube13", 0);
  TubeEffectJournal journal{adapter->Sink()};
  bool Emit(TubeEffectKind kind) {
    source += .01; receipt += .01;
    return journal.Record(kind, {"gripper_state", source, receipt});
  }
  void Capture() {
    assert(Emit(TubeEffectKind::KnownEmpty));
    assert(Emit(TubeEffectKind::TransferBegun));
    assert(state->Snapshot().transport.pending);
    assert(state->Snapshot().gripper.occupancy == agx_robot_task_runtime::GripperOccupancy::Empty);
    assert(Emit(TubeEffectKind::Captured));
  }
};
void ObservationJournalCases();
void TryDrainJournalCases();
int main() {
  ObservationJournalCases();
  TryDrainJournalCases();
  { // Search failure retains measured empty; an uncaught observation is never used.
    Harness h; assert(h.Emit(TubeEffectKind::KnownEmpty));
    auto out = h.journal.Outcome(TubeMissionStatus::kSearchExhausted);
    out.worker_joined = true;
    const auto result = h.adapter->Finish(out);
    assert(!result.ok && result.physical_effect_ids.size() == 1);
    assert(result.failure == agx_task_workflow::FailureKind::Ordinary && result.stop == agx_task_workflow::StopState::NotConfirmed);
    assert(h.state->Snapshot().gripper.occupancy == agx_robot_task_runtime::GripperOccupancy::Empty);
  }
  { // Qualified missed-pick recovery establishes empty clearance, not capture.
    Harness h; assert(h.Emit(TubeEffectKind::KnownEmpty));
    assert(h.Emit(TubeEffectKind::EmptySafeClear));
    const auto out = h.journal.Outcome(TubeMissionStatus::kTargetAbsent);
    assert(out.payload == PayloadKnowledge::kKnownEmpty && out.clearance == TubeClearance::SafeClear);
    assert(h.state->Snapshot().transport.ready && !h.state->Snapshot().gripper.object);
  }
  { // Capture then failed extraction preserves physical custody, without travel permission.
    Harness h; h.Capture();
    const auto before = h.state->Snapshot();
    const auto out = h.journal.Outcome(TubeMissionStatus::kLiftFailed);
    assert(out.payload == PayloadKnowledge::kHolding);
    assert(out.clearance == TubeClearance::CapturedInFixture);
    assert(!h.adapter->Finish(out).ok);
    assert(h.state->Snapshot().revision == before.revision); // reporting is read-only
    assert(before.gripper.object->id == h.adapter->Object().id);
    assert(before.transport.eligibility == agx_robot_task_runtime::TravelEligibility::CapturedInFixture);
  }
  { // Return failure and a final cancellation cannot roll back verified extraction.
    Harness h; h.Capture(); assert(h.Emit(TubeEffectKind::ExtractedSafeClear));
    assert(h.Emit(TubeEffectKind::Retained));
    auto out = h.journal.Outcome(TubeMissionStatus::kPhaseOneExecutionFailed);
    assert(out.clearance == TubeClearance::SafeClear);
    assert(!h.adapter->Finish(out).ok);
    out.status = TubeMissionStatus::kInterrupted;
    const auto late = h.adapter->Finish(out);
    assert(late.failure == agx_task_workflow::FailureKind::Cancelled);
    assert(h.state->Snapshot().transport.ready);
    assert(h.state->Snapshot().effects.size() == 4);
  }
  { // Fresh uncertainty preserves last confirmed holder while invalidating current custody.
    Harness h; h.Capture(); assert(h.Emit(TubeEffectKind::RetentionUncertain));
    const auto out = h.journal.Outcome(TubeMissionStatus::kLiftFailed);
    const auto state = h.state->Snapshot();
    assert(out.payload == PayloadKnowledge::kUnknown);
    assert(state.gripper.occupancy == agx_robot_task_runtime::GripperOccupancy::Unknown && !state.transport.ready);
    assert(state.objects.at(h.adapter->Object().id).last_confirmed_holder->kind == agx_robot_task_runtime::HolderKind::Gripper);
  }
  { // Worker completion alone is not a measured physical stop.
    Harness h; auto out = h.journal.Outcome(TubeMissionStatus::kInterrupted);
    out.worker_joined = true;
    assert(h.adapter->Finish(out).stop == agx_task_workflow::StopState::NotConfirmed);
    out.measured_stop_confirmed = true;
    assert(h.adapter->Finish(out).stop == agx_task_workflow::StopState::Confirmed);
    out.worker_joined = false;
    assert(h.adapter->Finish(out).stop == agx_task_workflow::StopState::NotConfirmed);
  }
  { // A concurrent revision change rejects transfer before the close command boundary.
    Harness h; assert(h.Emit(TubeEffectKind::KnownEmpty));
    agx_robot_task_runtime::PhysicalEffect empty{"external", agx_robot_task_runtime::EffectKind::GripperEmpty};
    assert(h.state->ApplyEffect(empty, {"external", h.source, h.receipt, "run", "other", "other"},
                               "other", h.state->Snapshot().revision).status == agx_robot_task_runtime::UpdateStatus::Applied);
    assert(!h.Emit(TubeEffectKind::TransferBegun));
    assert(!h.Emit(TubeEffectKind::Captured));
    assert(h.state->Snapshot().gripper.occupancy == agx_robot_task_runtime::GripperOccupancy::Empty);
  }
}


namespace {
TubeEffect EmptyEvent(const std::string& source="gripper_state") {
  return {1,TubeEffectKind::KnownEmpty,{source,10.,20.}};
}
void SameControl(const agx_robot_task_runtime::RobotStateSnapshot& a,const agx_robot_task_runtime::RobotStateSnapshot& b) {
  assert(a.revision==b.revision && a.halted==b.halted && a.effects.size()==b.effects.size());
  assert(a.gripper.occupancy==b.gripper.occupancy && a.gripper.revision==b.gripper.revision &&
         a.gripper.fresh==b.gripper.fresh && a.gripper.object.has_value()==b.gripper.object.has_value());
  if(a.gripper.object)assert(a.gripper.object->id==b.gripper.object->id);
  assert(a.transport.load==b.transport.load && a.transport.eligibility==b.transport.eligibility &&
         a.transport.ready==b.transport.ready && a.transport.uncertain==b.transport.uncertain &&
         a.transport.pending==b.transport.pending && a.transport.revision==b.transport.revision);
  assert(a.objects.size()==b.objects.size());
  for(const auto& item:a.objects) {
    const auto& other=b.objects.at(item.first);
    assert(item.second.certainty==other.certainty && item.second.revision==other.revision &&
           item.second.pending.has_value()==other.pending.has_value() &&
           item.second.last_confirmed_holder.has_value()==other.last_confirmed_holder.has_value());
    if(item.second.last_confirmed_holder)
      assert(item.second.last_confirmed_holder->kind==other.last_confirmed_holder->kind);
  }
  for(std::size_t i=0;i<a.effects.size();++i) {
    const auto& x=a.effects[i];const auto& y=b.effects[i];
    assert(x.effect.id==y.effect.id && x.effect.kind==y.effect.kind && x.status==y.status &&
           x.expected_revision==y.expected_revision && x.resulting_revision==y.resulting_revision &&
           x.evidence.source==y.evidence.source && x.evidence.source_stamp_s==y.evidence.source_stamp_s &&
           x.evidence.receipt_steady_s==y.evidence.receipt_steady_s);
  }
}
void SameFinish(const agx_task_workflow::StepResult& a,const agx_task_workflow::StepResult& b) {
  assert(a.ok==b.ok && a.detail==b.detail && a.physical_effect_ids==b.physical_effect_ids &&
         a.state_revision==b.state_revision && a.stop==b.stop && a.failure==b.failure);
}
void OriginalEvidence(const GraspEffectObservation& row,const TubeEffect& event) {
  assert(row.source_event.sequence==event.sequence && row.source_event.kind==event.kind &&
         row.source_event.evidence.source==event.evidence.source &&
         row.source_event.evidence.source_stamp_s==event.evidence.source_stamp_s &&
         row.source_event.evidence.receipt_steady_s==event.evidence.receipt_steady_s);
  assert(row.run_id=="run" && row.task_id=="grasp" && row.operation_id=="run/grasp/1");
  assert(row.object.id=="run/grasp/1/selected-tube" && row.object.identity==agx_robot_task_runtime::ObjectIdentity::RunLocal &&
         row.object.container_profile=="tube13");
}
}

void ObservationJournalCases() {
  static_assert(GraspEffectAdapter::kObservationCapacity==64,"fixed software queue bound");
  static_assert(GraspEffectAdapter::kObservationStringBytes==512,"no unbounded identity copies");
  { // First delivery counts even if its original received_ vector allocation fails.
    Harness h;const auto event=EmptyEvent();bool threw=false;
    // Adapter/event allocations are complete. Disabled Apply first allocates
    // when its empty received_ vector copies this small-string source event.
    fail_next_allocation=true;
    try { h.adapter->Apply(event); } catch (const std::bad_alloc&) { threw=true; }
    assert(threw && !fail_next_allocation.load());
    assert(!h.adapter->EnableObservationJournal());
    assert(h.state->Snapshot().revision==0);
    assert(h.adapter->Apply(event)); // Original retry behavior remains unchanged.
    const auto rows=h.adapter->TakeObservations();assert(rows.count==0 && !rows.gap);
  }
  { // Disabled means no added Snapshot or drain clock reads.
    unsigned calls=0;
    auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([&]{++calls;return 10.;},[]{return 20.;},1.);
    auto adapter=std::make_shared<GraspEffectAdapter>(state,"run","grasp","run/grasp/1","tube13",0);
    assert(adapter->Apply(EmptyEvent()));
    assert(calls==1);
    const auto rows=adapter->TakeObservations();
    assert(rows.count==0 && !rows.gap && rows.last_sequence==0 && calls==1);
    assert(!adapter->EnableObservationJournal()); // No retroactive observation history.
    assert(calls==1);
  }
  { // Real original applications and later state projection, with disabled parity.
    Harness enabled,disabled;
    assert(enabled.adapter->EnableObservationJournal());
    assert(!enabled.adapter->EnableObservationJournal());
    const TubeEffectKind kinds[]={TubeEffectKind::KnownEmpty,TubeEffectKind::TransferBegun,
      TubeEffectKind::Captured,TubeEffectKind::ExtractedSafeClear,TubeEffectKind::Retained,
      TubeEffectKind::RetentionUncertain,TubeEffectKind::MeasuredStop};
    for(const auto kind:kinds) {
      assert(enabled.Emit(kind)==disabled.Emit(kind));
      SameControl(enabled.state->Snapshot(),disabled.state->Snapshot());
    }
    auto out=enabled.journal.Outcome(TubeMissionStatus::kInterrupted);out.worker_joined=true;
    auto other=disabled.journal.Outcome(TubeMissionStatus::kInterrupted);other.worker_joined=true;
    SameFinish(enabled.adapter->Finish(out),disabled.adapter->Finish(other));
    const auto rows=enabled.adapter->TakeObservations();
    assert(rows.count==6 && !rows.gap && rows.last_sequence==6);
    for(std::size_t i=0;i<rows.count;++i) {
      const auto& row=rows.records[i];
      assert(row.observation_sequence==i+1 && row.adapter_accepted);
      OriginalEvidence(row,out.effects[row.source_event.sequence-1]);
      if(row.source_event.kind==TubeEffectKind::MeasuredStop) {
        assert(!row.application && !row.matched_effect);continue;
      }
      assert(row.application && row.application->status==agx_robot_task_runtime::UpdateStatus::Applied);
      if(row.source_event.kind==TubeEffectKind::TransferBegun) {
        assert(!row.matched_effect && row.transport.pending);continue;
      }
      assert(row.matched_effect && row.matched_effect->status==agx_robot_task_runtime::UpdateStatus::Applied &&
             row.matched_effect->resulting_revision==row.application->revision &&
             row.matched_effect->effect.id=="run/grasp/1/effect/"+std::to_string(row.source_event.sequence));
    }
    const auto& captured=rows.records[2];
    assert(captured.gripper.occupancy==agx_robot_task_runtime::GripperOccupancy::Holding && captured.object_location &&
           captured.object_location->last_confirmed_holder->kind==agx_robot_task_runtime::HolderKind::Gripper &&
           captured.transport.eligibility==agx_robot_task_runtime::TravelEligibility::CapturedInFixture);
    const auto& extracted=rows.records[3];
    assert(extracted.transport.ready && extracted.transport.eligibility==agx_robot_task_runtime::TravelEligibility::SafeClear);
    const auto& uncertain=rows.records[4];
    assert(uncertain.gripper.occupancy==agx_robot_task_runtime::GripperOccupancy::Unknown && !uncertain.transport.ready &&
           uncertain.object_location->certainty==agx_robot_task_runtime::LocationCertainty::Uncertain &&
           uncertain.object_location->last_confirmed_holder->kind==agx_robot_task_runtime::HolderKind::Gripper);
    assert(disabled.adapter->TakeObservations().count==0);
  }
  { // Identical adapter replay has no new StateUpdate, even if it returns true.
    Harness h;assert(h.adapter->EnableObservationJournal());auto event=EmptyEvent();
    assert(h.adapter->Apply(event));const auto revision=h.state->Snapshot().revision;
    assert(h.adapter->Apply(event));
    const auto rows=h.adapter->TakeObservations();
    assert(rows.count==2 && !rows.gap && rows.records[0].application &&
           !rows.records[1].application && !rows.records[1].matched_effect && rows.records[1].adapter_accepted);
    assert(h.state->Snapshot().revision==revision);
    event.evidence.source="mutated";
    assert(!h.adapter->Apply(event));assert(!h.adapter->Apply(EmptyEvent()));
    const auto rejected=h.adapter->TakeObservations();
    assert(rejected.count==2 && !rejected.records[0].application && !rejected.records[1].application &&
           !rejected.records[0].adapter_accepted && !rejected.records[1].adapter_accepted);
  }
  { // Real store Duplicate does not invent a newly applied EffectRecord.
    Harness first;assert(first.adapter->Apply(EmptyEvent()));
    auto second=std::make_shared<GraspEffectAdapter>(first.state,"run","grasp","run/grasp/1","tube13",1);
    assert(second->EnableObservationJournal());assert(second->Apply(EmptyEvent()));
    const auto rows=second->TakeObservations();
    assert(rows.count==1 && !rows.gap && rows.records[0].adapter_accepted &&
           rows.records[0].application->status==agx_robot_task_runtime::UpdateStatus::Duplicate && !rows.records[0].matched_effect);
    assert(first.state->Snapshot().effects.size()==1);
  }
  { // Same ID with changed evidence must select the appended Conflict, not old Applied.
    Harness first;assert(first.adapter->Apply(EmptyEvent()));
    auto second=std::make_shared<GraspEffectAdapter>(first.state,"run","grasp","run/grasp/1","tube13",1);
    assert(second->EnableObservationJournal());auto event=EmptyEvent("changed_source");
    assert(!second->Apply(event));const auto rows=second->TakeObservations();
    assert(rows.count==1 && !rows.gap);
    const auto& row=rows.records[0];
    assert(!row.adapter_accepted && row.application->status==agx_robot_task_runtime::UpdateStatus::Conflict && row.application->revision==2);
    assert(row.matched_effect && row.matched_effect->status==agx_robot_task_runtime::UpdateStatus::Conflict &&
           row.matched_effect->resulting_revision==2 && row.matched_effect->evidence.source=="changed_source");
    assert(row.snapshot_halted && first.state->Snapshot().effects.size()==2);
  }
  { // Gapped sequence and unsupported kind have no store application.
    for(bool unsupported:{false,true}) {
      Harness h;assert(h.adapter->EnableObservationJournal());auto event=EmptyEvent();
      if(unsupported)event.kind=static_cast<TubeEffectKind>(999);else event.sequence=2;
      assert(!h.adapter->Apply(event));const auto rows=h.adapter->TakeObservations();
      assert(rows.count==1 && !rows.gap && !rows.records[0].application &&
             !rows.records[0].matched_effect && !rows.records[0].adapter_accepted);
      assert(h.state->Snapshot().revision==0);
    }
  }
  { // A later read can be stale while the original application truly succeeded.
    unsigned calls=0;
    auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([&]{return ++calls==1?10.:12.;},[]{return 20.;},1.);
    auto adapter=std::make_shared<GraspEffectAdapter>(state,"run","grasp","run/grasp/1","tube13",0);
    assert(adapter->EnableObservationJournal());assert(adapter->Apply(EmptyEvent()));
    const auto rows=adapter->TakeObservations();const auto& row=rows.records[0];
    assert(rows.count==1 && !rows.gap && row.application->status==agx_robot_task_runtime::UpdateStatus::Applied &&
           row.application->revision==1 && row.snapshot_revision==1 && !row.gripper.fresh);
    OriginalEvidence(row,EmptyEvent());
  }
  { // The row is captured at Apply, not reconstructed from mutable state on drain.
    Harness h;assert(h.adapter->EnableObservationJournal());auto event=EmptyEvent();
    assert(h.adapter->Apply(event));event.evidence.source="caller_mutated";
    assert(h.state->ApplyEffect({"later",agx_robot_task_runtime::EffectKind::GripperUncertain},
      {"later_source",10.,20.,"run","other","other"},"other",1).status==agx_robot_task_runtime::UpdateStatus::Applied);
    const auto rows=h.adapter->TakeObservations();const auto& row=rows.records[0];
    assert(rows.count==1 && !rows.gap && row.snapshot_revision==1 && row.application->revision==1 &&
           row.gripper.occupancy==agx_robot_task_runtime::GripperOccupancy::Empty && row.source_event.evidence.source=="gripper_state");
    assert(h.state->Snapshot().revision==2 && h.state->Snapshot().gripper.occupancy==agx_robot_task_runtime::GripperOccupancy::Unknown);
  }
  { // Fixed capacity preserves earliest records and sticky loss across drains/re-enable.
    Harness h;assert(h.adapter->EnableObservationJournal());
    for(std::size_t i=0;i<GraspEffectAdapter::kObservationCapacity+1;++i)assert(h.adapter->Apply(EmptyEvent()));
    const auto rows=h.adapter->TakeObservations();
    assert(rows.count==64 && rows.gap && rows.last_sequence==65);
    for(std::size_t i=0;i<rows.count;++i)assert(rows.records[i].observation_sequence==i+1);
    const auto empty=h.adapter->TakeObservations();assert(empty.count==0 && empty.gap && empty.last_sequence==65);
    assert(!h.adapter->EnableObservationJournal());assert(h.adapter->Apply(EmptyEvent()));
    const auto next=h.adapter->TakeObservations();assert(next.count==1 && next.gap &&
      next.last_sequence==66 && next.records[0].observation_sequence==66 && !next.records[0].application);
    assert(h.state->Snapshot().effects.size()==1);
  }
  { // Bounds affect only optional evidence, with no truncation or control rejection.
    for(std::size_t size:{std::size_t(512),std::size_t(513)}) {
      Harness h;assert(h.adapter->EnableObservationJournal());const std::string source(size,'s');
      assert(h.adapter->Apply(EmptyEvent(source)));const auto rows=h.adapter->TakeObservations();
      assert(h.state->Snapshot().effects.front().evidence.source==source);
      if(size==512)assert(rows.count==1 && !rows.gap && rows.records[0].source_event.evidence.source==source);
      else assert(rows.count==0 && rows.gap && rows.last_sequence==1);
    }
    auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([]{return 10.;},[]{return 20.;});
    auto adapter=std::make_shared<GraspEffectAdapter>(state,std::string(513,'r'),"grasp","operation","tube13",0);
    assert(adapter->EnableObservationJournal());assert(adapter->Apply(EmptyEvent()));
    const auto rows=adapter->TakeObservations();assert(rows.count==0 && rows.gap && state->Snapshot().revision==1);
  }
  { // Check nested projected strings, not just source event and adapter identity.
    Harness h;const auto object=h.adapter->Object();
    assert(h.state->ApplyEffect({"seed",agx_robot_task_runtime::EffectKind::ObjectAtHolder,object,
      agx_robot_task_runtime::HolderRef{agx_robot_task_runtime::HolderKind::FixtureSlot,std::string(513,'f'),"A1"}},
      {"seed",10.,20.,"run","seed","seed"},"seed",0).status==agx_robot_task_runtime::UpdateStatus::Applied);
    auto adapter=std::make_shared<GraspEffectAdapter>(h.state,"run","grasp","run/grasp/1","tube13",1);
    assert(adapter->EnableObservationJournal());assert(adapter->Apply(EmptyEvent()));
    const auto rows=adapter->TakeObservations();assert(rows.count==0 && rows.gap);
    assert(h.state->Snapshot().objects.at(object.id).last_confirmed_holder->fixture_instance.size()==513);
  }
  { // Invalid physical evidence remains an actual Conflict, never repaired.
    Harness h;assert(h.adapter->EnableObservationJournal());auto event=EmptyEvent();
    event.evidence.source_stamp_s=std::numeric_limits<double>::quiet_NaN();
    assert(!h.adapter->Apply(event));const auto rows=h.adapter->TakeObservations();
    assert(rows.count==1 && !rows.gap && std::isnan(rows.records[0].source_event.evidence.source_stamp_s));
    assert(rows.records[0].application->status==agx_robot_task_runtime::UpdateStatus::Conflict && rows.records[0].matched_effect &&
           std::isnan(rows.records[0].matched_effect->evidence.source_stamp_s));
  }
  { // Only the optional later Snapshot fails; actual mutation and true survive.
    unsigned calls=0;
    auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([&]{if(++calls==2)throw std::runtime_error("snapshot unavailable");return 10.;},[]{return 20.;});
    auto adapter=std::make_shared<GraspEffectAdapter>(state,"run","grasp","run/grasp/1","tube13",0);
    assert(adapter->EnableObservationJournal());assert(adapter->Apply(EmptyEvent()));
    const auto rows=adapter->TakeObservations();assert(rows.count==0 && rows.gap && rows.last_sequence==1);
    assert(state->Snapshot().revision==1 && state->Snapshot().gripper.occupancy==agx_robot_task_runtime::GripperOccupancy::Empty);
    assert(adapter->Apply(EmptyEvent())); // Replaying still retains original control semantics.
    const auto replay=adapter->TakeObservations();assert(replay.count==1 && replay.gap && !replay.records[0].application);
  }
  { // Original control exception is not swallowed by an optional observer.
    auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([]{throw std::runtime_error("original ApplyEffect failure");return 10.;},[]{return 20.;});
    auto adapter=std::make_shared<GraspEffectAdapter>(state,"run","grasp","run/grasp/1","tube13",0);
    assert(adapter->EnableObservationJournal());bool threw=false;
    try{adapter->Apply(EmptyEvent());}catch(const std::runtime_error&){threw=true;}
    assert(threw);
  }
  { // Concurrent draining cannot duplicate or corrupt the single producer's records.
    Harness h;assert(h.adapter->EnableObservationJournal());
    std::atomic<bool> done{false};std::vector<std::uint64_t> sequences;bool gap=false;std::uint64_t last=0;
    std::thread producer([&]{for(unsigned i=0;i<100;++i)assert(h.adapter->Apply(EmptyEvent()));done=true;});
    auto drain=[&]{const auto batch=h.adapter->TakeObservations();gap=gap||batch.gap;last=batch.last_sequence;
      assert(batch.count<=64);for(std::size_t i=0;i<batch.count;++i){
        const auto& row=batch.records[i];assert(row.adapter_accepted);OriginalEvidence(row,EmptyEvent());
        sequences.push_back(row.observation_sequence);}};
    while(!done.load())drain();producer.join();drain();
    assert(last==100 && !sequences.empty());
    for(std::size_t i=1;i<sequences.size();++i)assert(sequences[i]>sequences[i-1]);
    assert(gap || sequences.size()==100);
    assert(h.state->Snapshot().effects.size()==1);
  }
}


namespace {
GraspEffectObservations AcquireTryBatch(const std::shared_ptr<GraspEffectAdapter>& adapter) {
  // A try_lock is permitted to fail spuriously even without contention.
  // Retry only in this test after producer quiescence; the outer test watchdog
  // catches a broken implementation that never acquires. No timing threshold.
  for(;;) {auto batch=adapter->TryTakeObservations();if(batch)return std::move(*batch);}
}
}
void TryDrainJournalCases() {
  static_assert(std::is_same<decltype(std::declval<GraspEffectAdapter&>().TryTakeObservations()),
      std::optional<GraspEffectObservations>>::value,"busy must differ from an acquired empty batch");
  for(bool prior_gap:{false,true}) { // A real Apply holds both locks inside its optional Snapshot clock.
    std::mutex clock_mutex;std::condition_variable clock_changed;
    bool armed=false,entered=false,released=false,producer_done=false;std::atomic<unsigned> calls{0};
    auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([&]{
      ++calls;std::unique_lock<std::mutex> lock(clock_mutex);
      if(armed) {
        armed=false;entered=true;clock_changed.notify_all();
        clock_changed.wait(lock,[&]{return released;});
      }
      return 10.;
    },[]{return 20.;},1.);
    auto adapter=std::make_shared<GraspEffectAdapter>(state,"run","grasp","run/grasp/1","tube13",0);
    assert(adapter->EnableObservationJournal());assert(adapter->Apply(EmptyEvent()));
    assert(calls==2); // Original application, then journal Snapshot.
    if(prior_gap) {
      for(unsigned i=1;i<65;++i)assert(adapter->Apply(EmptyEvent()));
      const auto full=adapter->TakeObservations();assert(full.count==64 && full.gap && full.last_sequence==65);
      assert(full.records[0].application && full.records[0].application->revision==1);
      assert(adapter->Apply(EmptyEvent())); // Queue a real replay after overflow/drain.
    }
    const auto queued_sequence=prior_gap?66u:1u;
    const auto before_producer=calls.load();
    {std::lock_guard<std::mutex> lock(clock_mutex);armed=true;}
    const TubeEffect stopped{2,TubeEffectKind::MeasuredStop,{"measured_stop",10.,20.}};
    bool applied=false;std::exception_ptr producer_error;
    std::thread producer([&]{
      try{applied=adapter->Apply(stopped);}catch(...){producer_error=std::current_exception();}
      {std::lock_guard<std::mutex> lock(clock_mutex);producer_done=true;}
      clock_changed.notify_all();
    });
    {std::unique_lock<std::mutex> lock(clock_mutex);clock_changed.wait(lock,[&]{return entered||producer_done;});}
    // Main and Apply are different threads. No sleep or elapsed-time assertion.
    // The outer native runner must impose its normal deadlock watchdog.
    const auto before=calls.load();std::optional<GraspEffectObservations> busy;std::exception_ptr drain_error;
    try{busy=adapter->TryTakeObservations();}catch(...){drain_error=std::current_exception();}
    const auto after=calls.load();
    {std::lock_guard<std::mutex> lock(clock_mutex);released=true;}
    clock_changed.notify_all();producer.join(); // Always release/join before assertions or rethrow.
    if(producer_error)std::rethrow_exception(producer_error);
    if(drain_error)std::rethrow_exception(drain_error);
    assert(entered && !busy && before==after && after==before_producer+1 && applied);
    const auto rows=AcquireTryBatch(adapter);
    assert(rows.count==2 && rows.gap==prior_gap && rows.last_sequence==queued_sequence+1 && calls==after);
    assert(rows.records[0].observation_sequence==queued_sequence && rows.records[1].observation_sequence==queued_sequence+1);
    OriginalEvidence(rows.records[0],EmptyEvent());OriginalEvidence(rows.records[1],stopped);
    assert(rows.records[0].application.has_value()==!prior_gap);
    if(!prior_gap)assert(rows.records[0].application->revision==1);
    assert(!rows.records[1].application && !rows.records[1].matched_effect && rows.records[1].adapter_accepted);
    const auto empty=AcquireTryBatch(adapter);
    assert(empty.count==0 && empty.gap==prior_gap && empty.last_sequence==queued_sequence+1 && calls==after);
    assert(state->Snapshot().effects.size()==1); // No fabricated application while busy/draining.
  }
  { // Blocking and nonblocking drains share occupancy, sequence and sticky-gap semantics.
    Harness h;assert(h.adapter->EnableObservationJournal());
    for(unsigned i=0;i<65;++i)assert(h.adapter->Apply(EmptyEvent()));
    const auto first=AcquireTryBatch(h.adapter);
    assert(first.count==64 && first.gap && first.last_sequence==65);
    for(unsigned i=0;i<64;++i)assert(first.records[i].observation_sequence==i+1);
    const auto empty=h.adapter->TakeObservations();assert(empty.count==0 && empty.gap && empty.last_sequence==65);
    assert(h.adapter->Apply(EmptyEvent()));const auto blocking=h.adapter->TakeObservations();
    assert(blocking.count==1 && blocking.gap && blocking.last_sequence==66 &&
      blocking.records[0].observation_sequence==66 && !blocking.records[0].application);
    assert(h.adapter->Apply(EmptyEvent()));const auto next=AcquireTryBatch(h.adapter);
    assert(next.count==1 && next.gap && next.last_sequence==67 &&
      next.records[0].observation_sequence==67 && !next.records[0].application);
    const auto last=AcquireTryBatch(h.adapter);assert(last.count==0 && last.gap && last.last_sequence==67);
    assert(first.records[0].application && first.records[0].application->revision==1);
    OriginalEvidence(first.records[0],EmptyEvent());assert(h.state->Snapshot().effects.size()==1);
  }
  { // Disabled capture and both drains add no store Snapshot or clock read.
    unsigned calls=0;auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([&]{++calls;return 10.;},[]{return 20.;});
    auto adapter=std::make_shared<GraspEffectAdapter>(state,"run","grasp","run/grasp/1","tube13",0);
    const auto before=AcquireTryBatch(adapter);assert(before.count==0 && !before.gap && before.last_sequence==0 && calls==0);
    assert(adapter->Apply(EmptyEvent()) && calls==1);
    const auto after=AcquireTryBatch(adapter);assert(after.count==0 && !after.gap && after.last_sequence==0 && calls==1);
    assert(adapter->TakeObservations().count==0 && calls==1);
    assert(!adapter->EnableObservationJournal() && calls==1);
  }
}
