#include "agx_tube_tasks/grasp_task.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
namespace agx_tube_tasks {
namespace {
bool Bounded(const std::string& value) {
  return value.size() <= GraspEffectAdapter::kObservationStringBytes;
}
bool Bounded(const agx_robot_task_runtime::ObjectRef& value) {
  return Bounded(value.id) && Bounded(value.container_profile);
}
bool Bounded(const agx_robot_task_runtime::HolderRef& value) {
  return Bounded(value.fixture_instance) && Bounded(value.slot);
}
bool Bounded(const agx_robot_task_runtime::Evidence& value) {
  return Bounded(value.source) && Bounded(value.run_id) &&
         Bounded(value.task_id) && Bounded(value.operation_id);
}
bool Bounded(const agx_robot_task_runtime::PendingTransfer& value) {
  return Bounded(value.operation_id) && Bounded(value.object) &&
         (!value.source || Bounded(*value.source)) &&
         (!value.destination || Bounded(*value.destination));
}
bool Bounded(const agx_robot_task_runtime::ObjectLocation& value) {
  return Bounded(value.object) &&
         (!value.last_confirmed_holder || Bounded(*value.last_confirmed_holder)) &&
         (!value.last_confirmed_evidence || Bounded(*value.last_confirmed_evidence)) &&
         (!value.current_evidence || Bounded(*value.current_evidence)) &&
         (!value.pending || Bounded(*value.pending));
}
bool Bounded(const agx_robot_task_runtime::GripperView& value) {
  return (!value.object || Bounded(*value.object)) &&
         (!value.evidence || Bounded(*value.evidence));
}
bool Bounded(const agx_robot_task_runtime::TransportView& value) {
  return (!value.object || Bounded(*value.object)) &&
         (!value.custody_evidence || Bounded(*value.custody_evidence)) &&
         (!value.clearance_evidence || Bounded(*value.clearance_evidence));
}
bool Bounded(const agx_robot_task_runtime::EffectRecord& value) {
  return Bounded(value.effect.id) && Bounded(value.operation_id) &&
         Bounded(value.evidence) &&
         (!value.effect.object || Bounded(*value.effect.object)) &&
         (!value.effect.holder || Bounded(*value.effect.holder));
}
// Diagnostic identity only, matching the store's existing invalid-NaN equality.
// It neither validates, repairs nor admits physical evidence.
bool SameObservationStamp(double a, double b) {
  return a == b || (std::isnan(a) && std::isnan(b));
}
static_assert(std::is_nothrow_move_assignable<GraspEffectObservation>::value,
              "journal queue/drain moves must not allocate or throw");
static_assert(std::is_nothrow_move_constructible<GraspEffectObservations>::value,
              "returning a drained fixed batch must not throw");
}  // namespace

bool GraspEffectAdapter::EnableObservationJournal() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (observation_enabled_ || delivery_started_) return false;
  observation_enabled_ = true;
  return true;
}

GraspEffectObservations GraspEffectAdapter::TakeObservations() {
  std::lock_guard<std::mutex> lock(mutex_);
  return TakeObservationsLocked();
}

std::optional<GraspEffectObservations> GraspEffectAdapter::TryTakeObservations() {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return std::nullopt;
  return TakeObservationsLocked();
}

GraspEffectObservations GraspEffectAdapter::TakeObservationsLocked() {
  GraspEffectObservations result;
  result.count = observations_.count;
  result.gap = observations_.gap;
  result.last_sequence = observations_.last_sequence;
  for (std::size_t i = 0; i < result.count; ++i)
    result.records[i] = std::move(observations_.records[i]);
  observations_.count = 0;
  return result;
}

void GraspEffectAdapter::CaptureObservation(const agxcore::TubeEffect& event,
    const agx_robot_task_runtime::StateUpdate* update, bool returned) noexcept {
  // Retained refreshes are explicitly excluded from this compact journal.
  if (!observation_enabled_ || event.kind == agxcore::TubeEffectKind::Retained) return;
  if (observations_.last_sequence == std::numeric_limits<std::uint64_t>::max()) {
    observations_.gap = true;
    return;
  }
  const auto sequence = ++observations_.last_sequence;
  if (observations_.count == kObservationCapacity) {
    observations_.gap = true;
    return;
  }
  try {
    if (!Bounded(run_id_) || !Bounded(task_id_) || !Bounded(operation_id_) ||
        !Bounded(object_) || !Bounded(event.evidence.source) ||
        (update && !Bounded(update->detail))) {
      observations_.gap = true;
      return;
    }
    // This inherited API copies full store history under its lock. Fixed output
    // capacity/string limits do NOT make temporary allocation or capture time bounded.
    const auto actual = state_->Snapshot();
    const auto location = actual.objects.find(object_.id);
    const agx_robot_task_runtime::EffectRecord* matched = nullptr;
    const bool requires_record = update && event.kind != agxcore::TubeEffectKind::TransferBegun &&
        (update->status == agx_robot_task_runtime::UpdateStatus::Applied || update->status == agx_robot_task_runtime::UpdateStatus::Conflict);
    if (requires_record) {
      const auto effect_id = operation_id_ + "/effect/" + std::to_string(event.sequence);
      for (const auto& candidate : actual.effects) {
        const auto& evidence = candidate.evidence;
        if (candidate.effect.id == effect_id && candidate.operation_id == operation_id_ &&
            candidate.resulting_revision == update->revision && candidate.status == update->status &&
            evidence.run_id == run_id_ && evidence.task_id == task_id_ &&
            evidence.operation_id == operation_id_ && evidence.source == event.evidence.source &&
            SameObservationStamp(evidence.source_stamp_s, event.evidence.source_stamp_s) &&
            SameObservationStamp(evidence.receipt_steady_s, event.evidence.receipt_steady_s)) {
          matched = &candidate;
          break;
        }
      }
    }
    if ((requires_record && !matched) || !Bounded(actual.gripper) || !Bounded(actual.transport) ||
        (location != actual.objects.end() && !Bounded(location->second)) ||
        (matched && !Bounded(*matched))) {
      observations_.gap = true;
      return;
    }
    GraspEffectObservation row;
    row.observation_sequence = sequence;
    row.run_id = run_id_; row.task_id = task_id_; row.operation_id = operation_id_;
    row.object = object_; row.source_event = event;
    if (update) row.application = *update;
    row.adapter_accepted = returned;
    row.snapshot_revision = actual.revision; row.snapshot_halted = actual.halted;
    row.gripper = actual.gripper; row.transport = actual.transport;
    if (location != actual.objects.end()) row.object_location = location->second;
    if (matched) row.matched_effect = *matched;
    observations_.records[observations_.count] = std::move(row);
    ++observations_.count;
  } catch (...) {
    observations_.gap = true;
  }
}

GraspEffectAdapter::GraspEffectAdapter(std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state,
    std::string run, std::string task, std::string operation, std::string container,
    agx_robot_task_runtime::StateRevision revision) : state_(std::move(state)), run_id_(std::move(run)),
    task_id_(std::move(task)), operation_id_(std::move(operation)),
    object_{operation_id_ + "/selected-tube", agx_robot_task_runtime::ObjectIdentity::RunLocal, std::move(container)},
    revision_(revision) {
  if (!state_ || run_id_.empty() || task_id_.empty() || operation_id_.empty() ||
      object_.container_profile.empty())
    throw std::invalid_argument("grasp effects require state and complete operation identity");
}
agxcore::TubeEffectSink GraspEffectAdapter::Sink() {
  auto self = shared_from_this();
  return [self](const agxcore::TubeEffect& effect) { return self->Apply(effect); };
}
bool GraspEffectAdapter::Apply(const agxcore::TubeEffect& event) {
  std::lock_guard<std::mutex> lock(mutex_);
  delivery_started_ = true;
  if (!accepted_) { CaptureObservation(event, nullptr, false); return false; }
  if (event.sequence > 0 && event.sequence <= received_.size()) {
    const auto& old = received_[event.sequence - 1];
    if (old.kind == event.kind && old.evidence.source == event.evidence.source &&
        old.evidence.source_stamp_s == event.evidence.source_stamp_s &&
        old.evidence.receipt_steady_s == event.evidence.receipt_steady_s) {
      CaptureObservation(event, nullptr, true); return true;
    }
    accepted_ = false; detail_ = "grasp effect sequence reused with different evidence";
    CaptureObservation(event, nullptr, false); return false;
  }
  if (event.sequence != received_.size() + 1) {
    accepted_ = false; detail_ = "grasp effect sequence is not contiguous";
    CaptureObservation(event, nullptr, false); return false;
  }
  received_.push_back(event);
  using agxcore::TubeEffectKind;
  agx_robot_task_runtime::StateUpdate update;
  if (event.kind == TubeEffectKind::TransferBegun) {
    // Color selection identifies no measured exact fixture slot. Do not invent one.
    update = state_->BeginTransfer(revision_, operation_id_, object_, std::nullopt, agx_robot_task_runtime::HolderRef{});
  } else if (event.kind == TubeEffectKind::MeasuredStop) {
    CaptureObservation(event, nullptr, true);
    return true; // Stop evidence is reported separately from custody effects.
  } else {
    agx_robot_task_runtime::PhysicalEffect effect;
    effect.id = operation_id_ + "/effect/" + std::to_string(event.sequence);
    switch (event.kind) {
      case TubeEffectKind::KnownEmpty: effect.kind = agx_robot_task_runtime::EffectKind::GripperEmpty; break;
      case TubeEffectKind::Captured:
        effect.kind = agx_robot_task_runtime::EffectKind::GripperHolding;
        effect.object = object_;
        effect.eligibility = agx_robot_task_runtime::TravelEligibility::CapturedInFixture;
        break;
      case TubeEffectKind::Retained:
        effect.kind = agx_robot_task_runtime::EffectKind::GripperHolding; effect.object = object_; break;
      case TubeEffectKind::ExtractedSafeClear:
        effect.kind = agx_robot_task_runtime::EffectKind::TravelEligibility; effect.object = object_;
        effect.eligibility = agx_robot_task_runtime::TravelEligibility::SafeClear; break;
      case TubeEffectKind::EmptySafeClear:
        effect.kind = agx_robot_task_runtime::EffectKind::TravelEligibility;
        effect.eligibility = agx_robot_task_runtime::TravelEligibility::SafeClear; break;
      case TubeEffectKind::RetentionUncertain:
        effect.kind = agx_robot_task_runtime::EffectKind::GripperUncertain; effect.object = object_; break;
      default: accepted_ = false; detail_ = "unsupported grasp effect";
        CaptureObservation(event, nullptr, false); return false;
    }
    const auto& e = event.evidence;
    update = state_->ApplyEffect(effect,
        {e.source, e.source_stamp_s, e.receipt_steady_s, run_id_, task_id_, operation_id_},
        operation_id_, revision_);
    if (update.status == agx_robot_task_runtime::UpdateStatus::Applied || update.status == agx_robot_task_runtime::UpdateStatus::Duplicate)
      applied_ids_.push_back(effect.id);
  }
  revision_ = update.revision;
  accepted_ = !update.halt && (update.status == agx_robot_task_runtime::UpdateStatus::Applied || update.status == agx_robot_task_runtime::UpdateStatus::Duplicate);
  if (!accepted_) detail_ = update.detail;
  CaptureObservation(event, &update, accepted_);
  return accepted_;
}
agx_task_workflow::StepResult GraspEffectAdapter::Finish(const agxcore::TubeMissionOutcome& outcome) const {
  std::lock_guard<std::mutex> lock(mutex_);
  agx_task_workflow::StepResult result;
  result.ok = accepted_ && outcome.effects_accepted && outcome.status == agxcore::TubeMissionStatus::kSuccess;
  result.detail = accepted_ ? "tube mission status " + std::to_string(static_cast<int>(outcome.status)) : detail_;
  result.physical_effect_ids = applied_ids_;
  result.state_revision = revision_;
  result.stop = outcome.worker_joined && outcome.measured_stop_confirmed ? agx_task_workflow::StopState::Confirmed : agx_task_workflow::StopState::NotConfirmed;
  result.failure = result.ok ? agx_task_workflow::FailureKind::None :
      outcome.status == agxcore::TubeMissionStatus::kInterrupted && accepted_ ? agx_task_workflow::FailureKind::Cancelled : agx_task_workflow::FailureKind::Fault;
  // Ordinary is a cause classification, never a stop acknowledgement. The
  // runner still requires its qualified stop hook and fresh state reconciliation.
  if (!result.ok && accepted_ && outcome.effects_accepted && outcome.worker_joined &&
      outcome.payload == agxcore::PayloadKnowledge::kKnownEmpty &&
      (outcome.status == agxcore::TubeMissionStatus::kSearchExhausted ||
       outcome.status == agxcore::TubeMissionStatus::kTargetAbsent)) result.failure = agx_task_workflow::FailureKind::Ordinary;
  return result;
}
}
