#include "agx_robot_task_runtime/robot_state.hpp"
#include "agx_task_workflow/runner.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>

using namespace agx_robot_task_runtime;

namespace {
const ObjectRef tube{"tube-1", ObjectIdentity::RunLocal, "capped-tube-v1"};
const HolderRef gripper{HolderKind::Gripper, "", ""};
const HolderRef slot{HolderKind::FixtureSlot, "rack-1", "A2"};

struct Rig {
  double source_now = 100.0, steady_now = 10.0;
  RobotStateStore state{[this] { return source_now; },
                        [this] { return steady_now; }, 1.0};
  Evidence EvidenceFor(const std::string& operation = "observe") const {
    return {"measured-gripper", source_now, steady_now, "run-1", "task-1", operation};
  }
  StateUpdate Apply(const PhysicalEffect& effect, const std::string& operation = "observe") {
    return state.ApplyEffect(effect, EvidenceFor(operation), operation, state.Snapshot().revision);
  }
  void Empty() {
    assert(Apply({"empty", EffectKind::GripperEmpty}).status == UpdateStatus::Applied);
  }
  void Capture() {
    Empty();
    assert(state.BeginTransfer(state.Snapshot().revision, "pick", tube,
                               std::nullopt, gripper).status == UpdateStatus::Applied);
    assert(Apply({"capture", EffectKind::GripperHolding, tube, std::nullopt,
                  TravelEligibility::CapturedInFixture}, "pick").status == UpdateStatus::Applied);
  }
  void Clear() {
    assert(Apply({"clear", EffectKind::TravelEligibility, tube, std::nullopt,
                  TravelEligibility::SafeClear}).status == UpdateStatus::Applied);
  }
};

void FreshEmptyAndHeldAreDistinctFromClearance() {
  Rig r;
  const auto startup = r.state.Snapshot();
  assert(startup.gripper.occupancy == GripperOccupancy::Unknown);
  assert(startup.transport.load == TransportLoad::Unknown && !startup.transport.ready);
  r.Capture();
  const auto captured = r.state.Snapshot();
  assert(captured.gripper.occupancy == GripperOccupancy::Holding);
  assert(captured.gripper.object->id == tube.id && captured.gripper.fresh);
  assert(captured.transport.load == TransportLoad::SingleHeld);
  assert(captured.transport.eligibility == TravelEligibility::CapturedInFixture);
  assert(!captured.transport.ready && !captured.transport.pending);
  r.Clear();
  assert(r.state.Snapshot().transport.ready);
  // Reading or editing a detached copy cannot update the store or earlier snapshots.
  auto copy = r.state.Snapshot();
  copy.gripper.object->id = "different";
  assert(r.state.Snapshot().gripper.object->id == tube.id);
  assert(startup.gripper.occupancy == GripperOccupancy::Unknown);
  assert(captured.transport.eligibility == TravelEligibility::CapturedInFixture);
}

void FreshnessUsesBothClocksWithoutRefreshingReads() {
  Rig r;
  r.Capture(); r.Clear();
  const auto receipt = r.state.Snapshot().gripper.evidence->receipt_steady_s;
  r.steady_now += 1.01;
  auto stale = r.state.Snapshot();
  assert(!stale.gripper.fresh && !stale.transport.ready && stale.transport.uncertain);
  assert(stale.gripper.evidence->receipt_steady_s == receipt);
  r.steady_now = 10.0; r.source_now += 1.01;
  assert(!r.state.Snapshot().transport.ready);
  r.source_now = 99.0;
  assert(!r.state.Snapshot().transport.ready);
  r.source_now = 100.0; r.steady_now = 9.0;
  assert(!r.state.Snapshot().transport.ready);
}

void InvalidEvidenceCannotConfirmCurrentCustody() {
  for (const double bad : {98.0, 101.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    Rig r;
    auto evidence = r.EvidenceFor(); evidence.source_stamp_s = bad;
    const auto result = r.state.ApplyEffect({"bad", EffectKind::GripperEmpty}, evidence, "observe", 0);
    assert(result.halt && result.status == UpdateStatus::Conflict);
    assert(r.state.Snapshot().gripper.occupancy == GripperOccupancy::Unknown);
    assert(!r.state.Snapshot().transport.ready);
  }
  for (const double bad : {8.0, 11.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    Rig r;
    auto evidence = r.EvidenceFor(); evidence.receipt_steady_s = bad;
    assert(r.state.ApplyEffect({"bad", EffectKind::GripperEmpty}, evidence, "observe", 0).halt);
    assert(!r.state.Snapshot().transport.ready);
  }
}

void PhysicalEffectsSurviveLaterTaskFailures() {
  Rig r;
  r.Capture();
  agx_task_workflow::StepResult failed_return{false, "return failed"};
  failed_return.physical_effect_ids = {"capture"};
  failed_return.state_revision = r.state.Snapshot().revision;
  assert(failed_return.failure == agx_task_workflow::FailureKind::Fault);  // Legacy failures are terminal.
  assert(failed_return.stop == agx_task_workflow::StopState::NotConfirmed);
  assert(r.state.Snapshot().gripper.object->id == tube.id);
  assert(r.state.Snapshot().objects.at(tube.id).certainty == LocationCertainty::Confirmed);

  assert(r.state.BeginTransfer(r.state.Snapshot().revision, "place", tube, gripper, slot).status == UpdateStatus::Applied);
  assert(r.state.Snapshot().transport.pending && !r.state.Snapshot().transport.ready);
  assert(r.Apply({"release", EffectKind::Released, tube, slot}, "place").status == UpdateStatus::Applied);
  agx_task_workflow::StepResult failed_withdrawal{false, "withdrawal failed"};
  failed_withdrawal.failure = agx_task_workflow::FailureKind::Cancelled;
  const auto released = r.state.Snapshot();
  assert(released.gripper.occupancy == GripperOccupancy::Empty && !released.gripper.object);
  assert(released.objects.at(tube.id).last_confirmed_holder->slot == "A2");
  assert(released.objects.at(tube.id).certainty == LocationCertainty::Confirmed);
  assert(!released.transport.ready);  // Still requires independent clearance evidence.
}

void UnknownReleaseKeepsHistoryWithoutInventingCurrentCustody() {
  Rig r; r.Capture();
  assert(r.state.BeginTransfer(r.state.Snapshot().revision, "drop", tube, gripper,
                               std::nullopt).status == UpdateStatus::Applied);
  assert(r.Apply({"release", EffectKind::Released, tube}, "drop").status == UpdateStatus::Applied);
  const auto released = r.state.Snapshot();
  assert(released.gripper.occupancy == GripperOccupancy::Empty && released.gripper.fresh);
  const auto& location = released.objects.at(tube.id);
  assert(location.certainty == LocationCertainty::Uncertain && !location.pending);
  assert(location.last_confirmed_holder->kind == HolderKind::Gripper);
  assert(location.last_confirmed_evidence->operation_id == "pick");
  assert(location.current_evidence->operation_id == "drop");
  assert(!released.transport.ready && released.transport.uncertain);
}

void IntendedReleaseDestinationIsNotMeasuredSupport() {
  Rig r; r.Capture();
  assert(r.state.BeginTransfer(r.state.Snapshot().revision, "place", tube, gripper, slot).status == UpdateStatus::Applied);
  assert(r.Apply({"release", EffectKind::Released, tube}, "place").status == UpdateStatus::Applied);
  assert(r.state.Snapshot().gripper.occupancy == GripperOccupancy::Empty);
  assert(r.state.Snapshot().objects.at(tube.id).certainty == LocationCertainty::Uncertain);
  assert(!r.state.Snapshot().objects.at(tube.id).pending);
}

void ContradictoryHolderObservationsCannotSilentlyReplaceCustody() {
  for (bool same_object : {true, false}) {
    Rig r; r.Capture();
    const ObjectRef other{"tube-2", ObjectIdentity::Registered, "capped-tube-v1"};
    const auto effect = same_object
        ? PhysicalEffect{"contradiction", EffectKind::ObjectAtHolder, tube, slot}
        : PhysicalEffect{"contradiction", EffectKind::ObjectAtHolder, other, gripper};
    assert(r.Apply(effect).status == UpdateStatus::Conflict);
    assert(r.state.Snapshot().halted);
    assert(r.state.Snapshot().objects.at(tube.id).last_confirmed_holder->kind == HolderKind::Gripper);
  }
}

void UnrelatedFixtureObservationCannotClearHeldTransport() {
  Rig r; r.Capture();
  const ObjectRef other{"tube-2", ObjectIdentity::Registered, "capped-tube-v1"};
  assert(r.Apply({"unrelated", EffectKind::ObjectAtHolder, other, slot,
                  TravelEligibility::SafeClear}).status == UpdateStatus::Conflict);
  assert(!r.state.Snapshot().transport.ready && r.state.Snapshot().halted);
  Rig ordinary; ordinary.Capture();
  assert(ordinary.Apply({"unrelated", EffectKind::ObjectAtHolder, other, slot}).status == UpdateStatus::Applied);
  assert(ordinary.state.Snapshot().transport.eligibility == TravelEligibility::CapturedInFixture);
  assert(!ordinary.state.Snapshot().transport.ready);
}

void DelayedCaptureCannotRestoreReleasedCustody() {
  for (int stale_clock = 0; stale_clock < 3; ++stale_clock) {
    Rig r; r.Capture();
    const auto captured = r.EvidenceFor("pick");
    assert(r.state.BeginTransfer(r.state.Snapshot().revision, "place", tube, gripper,
                                 std::nullopt).status == UpdateStatus::Applied);
    r.source_now += 0.2; r.steady_now += 0.2;
    assert(r.Apply({"release", EffectKind::Released, tube}, "place").status == UpdateStatus::Applied);
    auto late = r.EvidenceFor("late-capture");
    if (stale_clock != 1) late.source_stamp_s = captured.source_stamp_s;
    if (stale_clock != 0) late.receipt_steady_s = captured.receipt_steady_s;
    assert(r.state.ApplyEffect({"late-capture", EffectKind::GripperHolding, tube, std::nullopt,
                                TravelEligibility::SafeClear}, late, "late-capture",
                               r.state.Snapshot().revision).status == UpdateStatus::Conflict);
    assert(r.state.Snapshot().gripper.occupancy != GripperOccupancy::Holding);
    assert(r.state.Snapshot().objects.at(tube.id).certainty == LocationCertainty::Uncertain);
    assert(!r.state.Snapshot().transport.ready);
    assert(r.state.Snapshot().effects[2].effect.id == "release");
  }
}

void CompletedTransferOperationCannotReenterLaterCustody() {
  Rig r; r.Capture();
  assert(r.state.BeginTransfer(r.state.Snapshot().revision, "place", tube, gripper,
                               slot).status == UpdateStatus::Applied);
  r.source_now += 0.2; r.steady_now += 0.2;
  assert(r.Apply({"release", EffectKind::Released, tube, slot}, "place").status == UpdateStatus::Applied);
  assert(r.state.BeginTransfer(r.state.Snapshot().revision, "pick", tube, slot,
                               gripper).status == UpdateStatus::Rejected);
  r.source_now += 0.2; r.steady_now += 0.2;
  assert(r.Apply({"old-operation", EffectKind::GripperHolding, tube}, "pick").status == UpdateStatus::Conflict);
  assert(r.state.Snapshot().objects.at(tube.id).last_confirmed_holder->slot == "A2");
}

void DelayedClearanceCannotOverrideNewerUnsupportedObservation() {
  Rig r; r.Capture(); r.Clear();
  const auto old = r.EvidenceFor("clear");
  r.source_now += 0.2; r.steady_now += 0.2;
  assert(r.Apply({"unsupported", EffectKind::TravelEligibility, tube, std::nullopt,
                  TravelEligibility::Unsupported}).status == UpdateStatus::Applied);
  assert(r.state.ApplyEffect({"delayed-clear", EffectKind::TravelEligibility, tube, std::nullopt,
                              TravelEligibility::SafeClear}, old, "clear", r.state.Snapshot().revision).status == UpdateStatus::Conflict);
  assert(!r.state.Snapshot().transport.ready);
}

void FreshObservationRefreshKeepsSemanticTransportRevision() {
  Rig r; r.Capture(); r.Clear();
  const auto before=r.state.Snapshot();
  r.source_now+=0.2;r.steady_now+=0.2;
  assert(r.Apply({"retained",EffectKind::GripperHolding,tube}).status==UpdateStatus::Applied);
  const auto refreshed=r.state.Snapshot();
  assert(refreshed.revision>before.revision);
  assert(refreshed.transport.revision==before.transport.revision);
  assert(refreshed.transport.ready && refreshed.transport.eligibility==TravelEligibility::SafeClear);
  assert(refreshed.gripper.evidence->receipt_steady_s>before.gripper.evidence->receipt_steady_s);
  assert(refreshed.transport.clearance_evidence->receipt_steady_s==before.transport.clearance_evidence->receipt_steady_s);
  assert(r.Apply({"clear-refreshed",EffectKind::TravelEligibility,tube,std::nullopt,
                  TravelEligibility::SafeClear}).status==UpdateStatus::Applied);
  assert(r.state.Snapshot().transport.revision==before.transport.revision);
  assert(r.state.Snapshot().transport.clearance_evidence->receipt_steady_s>before.transport.clearance_evidence->receipt_steady_s);
  assert(r.Apply({"blocked",EffectKind::TravelEligibility,tube,std::nullopt,
                  TravelEligibility::Unknown}).status==UpdateStatus::Applied);
  assert(r.state.Snapshot().transport.revision>before.transport.revision);
  assert(!r.state.Snapshot().transport.ready);

  Rig empty;empty.Empty();
  assert(empty.Apply({"clear",EffectKind::TravelEligibility,std::nullopt,std::nullopt,
                      TravelEligibility::SafeClear}).status==UpdateStatus::Applied);
  const auto empty_before=empty.state.Snapshot();
  empty.source_now+=0.2;empty.steady_now+=0.2;
  assert(empty.Apply({"empty-refresh",EffectKind::GripperEmpty}).status==UpdateStatus::Applied);
  assert(empty.state.Snapshot().transport.revision==empty_before.transport.revision);
  assert(empty.state.Snapshot().transport.ready);
}

void RefreshAfterStalenessInvalidatesPriorSemanticRevision() {
  Rig r;r.Capture();r.Clear();
  const auto before=r.state.Snapshot();
  r.source_now+=1.1;r.steady_now+=1.1;
  const auto stale=r.state.Snapshot();
  assert(!stale.transport.ready && stale.transport.revision==before.transport.revision);
  assert(r.Apply({"retained-new",EffectKind::GripperHolding,tube}).status==UpdateStatus::Applied);
  assert(!r.state.Snapshot().transport.ready); // Custody does not refresh clearance.
  assert(r.Apply({"clear-new",EffectKind::TravelEligibility,tube,std::nullopt,
                  TravelEligibility::SafeClear}).status==UpdateStatus::Applied);
  assert(r.state.Snapshot().transport.ready && r.state.Snapshot().transport.revision>before.transport.revision);
}

void UncertainGripperPreservesHistoricalCustodyAndPendingTransfer() {
  for (bool pending : {false,true}) {
    Rig r;r.Capture();r.Clear();
    if (pending)
      assert(r.state.BeginTransfer(r.state.Snapshot().revision,"place",tube,gripper,slot).status==UpdateStatus::Applied);
    assert(r.Apply({"retention-uncertain",EffectKind::GripperUncertain},"monitor").status==UpdateStatus::Applied);
    const auto snapshot=r.state.Snapshot();
    assert(snapshot.gripper.occupancy==GripperOccupancy::Unknown && !snapshot.gripper.object);
    assert(snapshot.transport.eligibility==TravelEligibility::Unknown && !snapshot.transport.ready);
    assert(snapshot.objects.at(tube.id).certainty==LocationCertainty::Uncertain);
    assert(snapshot.objects.at(tube.id).last_confirmed_holder->kind==HolderKind::Gripper);
    assert(snapshot.objects.at(tube.id).pending.has_value()==pending);
    if(pending)assert(snapshot.objects.at(tube.id).pending->operation_id=="place");
  }
}

void ConflictingPhysicalEvidenceHaltsAndRetainsBothObservations() {
  Rig r; r.Capture(); r.Clear();
  const auto before = r.state.Snapshot();
  const auto conflict = r.state.ApplyEffect({"late-release", EffectKind::Released, tube},
                                           r.EvidenceFor("late"), "late", before.revision - 1);
  assert(conflict.status == UpdateStatus::Conflict && conflict.halt);
  const auto after = r.state.Snapshot();
  assert(after.revision > before.revision && after.halted);
  assert(after.gripper.occupancy == GripperOccupancy::Unknown && !after.transport.ready);
  assert(after.objects.at(tube.id).certainty == LocationCertainty::Uncertain);
  assert(after.objects.at(tube.id).last_confirmed_evidence->operation_id == "pick");
  assert(after.effects.back().effect.id == "late-release");
  assert(after.effects.back().evidence.operation_id == "late");
  assert(r.state.BeginTransfer(after.revision, "next", tube, gripper, slot).status == UpdateStatus::Rejected);
  // New evidence can record real custody after a conflict, but cannot clear the halt latch.
  assert(r.Apply({"observed-empty", EffectKind::GripperEmpty}).status == UpdateStatus::Applied);
  assert(r.state.Snapshot().halted && !r.state.Snapshot().transport.ready);
}

void DuplicateEffectsAreIdempotentButIdReuseIsNot() {
  Rig r;
  const PhysicalEffect effect{"empty", EffectKind::GripperEmpty};
  const auto evidence = r.EvidenceFor();
  assert(r.state.ApplyEffect(effect, evidence, "observe", 0).status == UpdateStatus::Applied);
  const auto before = r.state.Snapshot();
  r.steady_now += 3.0; r.source_now += 3.0;
  const auto duplicate = r.state.ApplyEffect(effect, evidence, "observe", 0);
  assert(duplicate.status == UpdateStatus::Duplicate && duplicate.revision == before.revision);
  assert(r.state.Snapshot().effects.size() == 1 && !r.state.Snapshot().gripper.fresh);
  assert(r.state.ApplyEffect({"empty", EffectKind::GripperHolding, tube},
                             r.EvidenceFor(), "observe", before.revision).status == UpdateStatus::Conflict);
  assert(r.state.Snapshot().halted);
}

void TransferRevisionAndOperationAreGuarded() {
  Rig r; r.Empty();
  const auto revision = r.state.Snapshot().revision;
  assert(r.state.BeginTransfer(revision - 1, "pick", tube, std::nullopt, gripper).status == UpdateStatus::Rejected);
  assert(r.state.Snapshot().revision == revision);
  assert(r.state.BeginTransfer(revision, "pick", tube, std::nullopt, gripper).status == UpdateStatus::Applied);
  assert(r.state.BeginTransfer(r.state.Snapshot().revision, "second", tube, std::nullopt, gripper).status == UpdateStatus::Rejected);
  assert(r.Apply({"capture", EffectKind::GripperHolding, tube}, "wrong-operation").status == UpdateStatus::Conflict);
  assert(r.state.Snapshot().objects.at(tube.id).pending->operation_id == "pick");
  assert(r.state.Snapshot().halted);
}

void UnexpectedEmptyAndUnsupportedLoadsBlockTravel() {
  Rig r; r.Capture(); r.Clear();
  assert(r.Apply({"unsupported", EffectKind::TravelEligibility, tube, std::nullopt,
                  TravelEligibility::Unsupported}).status == UpdateStatus::Applied);
  assert(r.state.Snapshot().transport.load == TransportLoad::Unsupported);
  assert(!r.state.Snapshot().transport.ready);
  assert(r.Apply({"lost", EffectKind::GripperEmpty}).status == UpdateStatus::Applied);
  assert(r.state.Snapshot().objects.at(tube.id).certainty == LocationCertainty::Uncertain);
  assert(!r.state.Snapshot().transport.ready);
}

void FreshEmptyNeedsClearanceAndRestartHasNoInventory() {
  Rig r; r.Empty();
  assert(r.state.Snapshot().transport.load == TransportLoad::Empty);
  assert(r.Apply({"clear-empty", EffectKind::TravelEligibility, std::nullopt, std::nullopt,
                  TravelEligibility::SafeClear}).status == UpdateStatus::Applied);
  assert(r.state.Snapshot().transport.ready);
  Rig restarted;
  assert(restarted.state.Snapshot().objects.empty());
  assert(restarted.state.Snapshot().effects.empty());
  assert(restarted.state.Snapshot().gripper.occupancy == GripperOccupancy::Unknown);
  assert(!restarted.state.Snapshot().transport.ready);
}

void ConcurrentUpdatesHaveOneWriteOwner() {
  Rig r;
  StateUpdate one, two;
  std::thread a([&] { one = r.state.ApplyEffect({"one", EffectKind::GripperEmpty}, r.EvidenceFor(), "observe", 0); });
  std::thread b([&] { two = r.state.ApplyEffect({"two", EffectKind::GripperEmpty}, r.EvidenceFor(), "observe", 0); });
  a.join(); b.join();
  assert((one.status == UpdateStatus::Applied) != (two.status == UpdateStatus::Applied));
  assert(r.state.Snapshot().halted && r.state.Snapshot().effects.size() == 2);
}

void InvalidConfigurationAndEffectShapesFailClosed() {
  for (double age : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
    bool threw = false;
    try { RobotStateStore invalid([] { return 100.0; }, [] { return 10.0; }, age); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
  }
  for (const auto& effect : {
      PhysicalEffect{"bad-shape", EffectKind::GripperEmpty, tube, slot},
      PhysicalEffect{"bad-shape", EffectKind::GripperHolding, tube, slot},
      PhysicalEffect{"bad-shape", static_cast<EffectKind>(99)},
      PhysicalEffect{"bad-shape", EffectKind::ObjectAtHolder, tube,
                     HolderRef{HolderKind::FixtureSlot, "rack-1", ""}}}) {
    Rig r;
    assert(r.Apply(effect).status == UpdateStatus::Conflict);
    assert(!r.state.Snapshot().transport.ready);
  }
  Rig r; r.Capture(); r.Clear();
  r.source_now = std::numeric_limits<double>::quiet_NaN();
  assert(!r.state.Snapshot().transport.ready);
  r.source_now = 100.0; r.steady_now = std::numeric_limits<double>::infinity();
  assert(!r.state.Snapshot().transport.ready);
  assert(agx_task_workflow::StepResult({true, "legacy success"}).failure == agx_task_workflow::FailureKind::None);
}
}  // namespace

int main() {
  FreshEmptyAndHeldAreDistinctFromClearance();
  FreshnessUsesBothClocksWithoutRefreshingReads();
  InvalidEvidenceCannotConfirmCurrentCustody();
  PhysicalEffectsSurviveLaterTaskFailures();
  UnknownReleaseKeepsHistoryWithoutInventingCurrentCustody();
  IntendedReleaseDestinationIsNotMeasuredSupport();
  ContradictoryHolderObservationsCannotSilentlyReplaceCustody();
  UnrelatedFixtureObservationCannotClearHeldTransport();
  DelayedCaptureCannotRestoreReleasedCustody();
  CompletedTransferOperationCannotReenterLaterCustody();
  DelayedClearanceCannotOverrideNewerUnsupportedObservation();
  FreshObservationRefreshKeepsSemanticTransportRevision();
  RefreshAfterStalenessInvalidatesPriorSemanticRevision();
  UncertainGripperPreservesHistoricalCustodyAndPendingTransfer();
  ConflictingPhysicalEvidenceHaltsAndRetainsBothObservations();
  DuplicateEffectsAreIdempotentButIdReuseIsNot();
  TransferRevisionAndOperationAreGuarded();
  UnexpectedEmptyAndUnsupportedLoadsBlockTravel();
  FreshEmptyNeedsClearanceAndRestartHasNoInventory();
  ConcurrentUpdatesHaveOneWriteOwner();
  InvalidConfigurationAndEffectShapesFailClosed();
  std::cout << "robot state tests passed\n";
}
