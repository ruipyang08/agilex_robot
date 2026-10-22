#include "agx_robot_task_runtime/robot_state.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace agx_robot_task_runtime {
namespace {
bool Same(const ObjectRef& a, const ObjectRef& b) {
  return a.id == b.id && a.identity == b.identity && a.container_profile == b.container_profile;
}
bool Same(const HolderRef& a, const HolderRef& b) {
  return a.kind == b.kind && a.fixture_instance == b.fixture_instance && a.slot == b.slot;
}
template <typename T> bool Same(const std::optional<T>& a, const std::optional<T>& b) {
  return a.has_value() == b.has_value() && (!a || Same(*a, *b));
}
bool Same(const Evidence& a, const Evidence& b) {
  const auto same_time = [](double x, double y) { return x == y || (std::isnan(x) && std::isnan(y)); };
  return a.source == b.source && same_time(a.source_stamp_s, b.source_stamp_s) &&
         same_time(a.receipt_steady_s, b.receipt_steady_s) && a.run_id == b.run_id &&
         a.task_id == b.task_id && a.operation_id == b.operation_id;
}
bool Same(const PhysicalEffect& a, const PhysicalEffect& b) {
  return a.id == b.id && a.kind == b.kind && Same(a.object, b.object) &&
         Same(a.holder, b.holder) && a.eligibility == b.eligibility;
}
bool Valid(const ObjectRef& object) {
  return !object.id.empty() && !object.container_profile.empty() &&
         (object.identity == ObjectIdentity::Registered || object.identity == ObjectIdentity::RunLocal);
}
bool Valid(const HolderRef& holder) {
  return holder.kind == HolderKind::Gripper
      ? holder.fixture_instance.empty() && holder.slot.empty()
      : holder.kind == HolderKind::FixtureSlot && !holder.fixture_instance.empty() && !holder.slot.empty();
}
HolderRef Gripper() { return {}; }
bool NotBefore(const Evidence& evidence, const Evidence& previous) {
  return evidence.source_stamp_s >= previous.source_stamp_s &&
         evidence.receipt_steady_s >= previous.receipt_steady_s;
}
bool SameTransport(const TransportView& a, const TransportView& b) {
  return a.load==b.load && a.eligibility==b.eligibility && Same(a.object,b.object) &&
         a.pending==b.pending && a.uncertain==b.uncertain && a.ready==b.ready;
}
}  // namespace

RobotStateStore::RobotStateStore(Clock source_clock, Clock steady_clock, double max_age)
    : source_clock_(std::move(source_clock)), steady_clock_(std::move(steady_clock)),
      max_evidence_age_s_(max_age) {
  if (!source_clock_ || !steady_clock_ || !std::isfinite(max_age) || max_age <= 0.0)
    throw std::invalid_argument("state clocks and a finite positive evidence age are required");
}

bool RobotStateStore::Fresh(const Evidence& e, double source_now, double steady_now) const {
  const double source_age = source_now - e.source_stamp_s;
  const double receipt_age = steady_now - e.receipt_steady_s;
  return std::isfinite(source_now) && std::isfinite(steady_now) &&
         std::isfinite(e.source_stamp_s) && std::isfinite(e.receipt_steady_s) &&
         source_age >= 0.0 && source_age <= max_evidence_age_s_ &&
         receipt_age >= 0.0 && receipt_age <= max_evidence_age_s_;
}

RobotStateSnapshot RobotStateStore::SnapshotLocked(double source_now, double steady_now) const {
  auto snapshot = state_;
  auto& transport = snapshot.transport;
  const auto& gripper = snapshot.gripper;
  snapshot.gripper.fresh = gripper.occupancy != GripperOccupancy::Unknown && gripper.evidence &&
                           Fresh(*gripper.evidence, source_now, steady_now);
  transport.pending = false;
  transport.uncertain = snapshot.halted || !snapshot.gripper.fresh;
  for (auto& entry : snapshot.objects) {
    auto& location = entry.second;
    location.fresh = location.certainty == LocationCertainty::Confirmed && location.current_evidence &&
                     Fresh(*location.current_evidence, source_now, steady_now);
    transport.pending = transport.pending || location.pending.has_value();
    // Unresolved physical custody cannot be reinterpreted as an empty transport load.
    transport.uncertain = transport.uncertain || location.certainty != LocationCertainty::Confirmed;
  }
  transport.object = gripper.object;
  transport.custody_evidence = gripper.evidence;
  if (transport.eligibility == TravelEligibility::Unsupported) transport.load = TransportLoad::Unsupported;
  else if (gripper.occupancy == GripperOccupancy::Empty) transport.load = TransportLoad::Empty;
  else if (gripper.occupancy == GripperOccupancy::Holding) transport.load = TransportLoad::SingleHeld;
  else transport.load = TransportLoad::Unknown;
  transport.ready = !transport.uncertain && !transport.pending &&
                    transport.eligibility == TravelEligibility::SafeClear &&
                    transport.clearance_evidence && Fresh(*transport.clearance_evidence, source_now, steady_now) &&
                    (transport.load == TransportLoad::Empty || transport.load == TransportLoad::SingleHeld);
  return snapshot;
}

RobotStateSnapshot RobotStateStore::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return SnapshotLocked(source_clock_(), steady_clock_());
}

StateUpdate RobotStateStore::BeginTransfer(StateRevision expected, const std::string& operation,
                                           const ObjectRef& object, std::optional<HolderRef> source,
                                           std::optional<HolderRef> destination) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto reject = [&](const std::string& detail) {
    return StateUpdate{UpdateStatus::Rejected, state_.revision, state_.halted, detail};
  };
  if (state_.halted) return reject("state conflict requires reconciliation before further transfers");
  if (expected != state_.revision) return reject("transfer revision mismatch");
  if (transfer_operations_.count(operation)) return reject("transfer operation ID was already used");
  if (operation.empty() || !Valid(object) || (source && !Valid(*source)) ||
      (destination && !Valid(*destination)) || Same(source, destination))
    return reject("invalid transfer identity or holders");
  const auto snapshot = SnapshotLocked(source_clock_(), steady_clock_());
  if (snapshot.transport.pending) return reject("a transfer is already pending");
  const auto existing = state_.objects.find(object.id);
  if (existing != state_.objects.end()) {
    if (!Same(existing->second.object, object)) return reject("object identity cannot change");
    if (existing->second.certainty == LocationCertainty::Confirmed && source &&
        (!Same(existing->second.last_confirmed_holder, source) || !snapshot.objects.at(object.id).fresh))
      return reject("source is not fresh confirmed custody");
  }
  if (source && source->kind == HolderKind::Gripper &&
      (!snapshot.gripper.fresh || snapshot.gripper.occupancy != GripperOccupancy::Holding ||
       !snapshot.gripper.object || !Same(*snapshot.gripper.object, object)))
    return reject("gripper source is not fresh confirmed custody");
  if (destination && destination->kind == HolderKind::Gripper &&
      (!snapshot.gripper.fresh || snapshot.gripper.occupancy != GripperOccupancy::Empty))
    return reject("gripper destination is not confirmed empty");
  auto& location = state_.objects[object.id];
  location.object = object;
  location.pending = PendingTransfer{operation, object, std::move(source), std::move(destination)};
  transfer_operations_.insert(operation);
  location.revision = ++state_.revision;
  state_.transport.revision = state_.revision;
  state_.transport.eligibility = TravelEligibility::Unknown;
  state_.transport.clearance_evidence.reset();
  return {UpdateStatus::Applied, state_.revision, false, "transfer pending"};
}

StateUpdate RobotStateStore::Conflict(const PhysicalEffect& effect, const Evidence& evidence,
                                     const std::string& operation, StateRevision expected,
                                     const std::string& detail) {
  ++state_.revision;
  state_.halted = true;
  auto uncertain = [&](const ObjectRef& object) {
    if (!Valid(object)) return;
    auto& location = state_.objects[object.id];
    if (location.object.id.empty()) location.object = object;
    location.certainty = LocationCertainty::Uncertain;
    location.current_evidence = evidence;
    location.revision = state_.revision;
  };
  if (state_.gripper.object) uncertain(*state_.gripper.object);
  if (effect.object) uncertain(*effect.object);
  state_.gripper = {GripperOccupancy::Unknown, std::nullopt, evidence, state_.revision};
  state_.transport.eligibility = TravelEligibility::Unknown;
  state_.transport.clearance_evidence.reset();
  state_.transport.revision = state_.revision;
  state_.effects.push_back({effect, evidence, operation, expected, state_.revision, UpdateStatus::Conflict});
  return {UpdateStatus::Conflict, state_.revision, true, detail};
}

StateUpdate RobotStateStore::ApplyEffect(const PhysicalEffect& effect, const Evidence& evidence,
                                        const std::string& operation, StateRevision expected) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double source_now=source_clock_(), steady_now=steady_clock_();
  const auto transport_before=SnapshotLocked(source_now,steady_now).transport;
  auto conflict = [&](const std::string& detail) {
    const auto result=Conflict(effect, evidence, operation, expected, detail);
    if (SameTransport(transport_before,SnapshotLocked(source_now,steady_now).transport))
      state_.transport.revision=transport_before.revision;
    return result;
  };
  // Check replay before age: replaying old evidence must neither refresh it nor create a conflict.
  for (const auto& record : state_.effects) {
    if (record.effect.id != effect.id) continue;
    if (Same(record.effect, effect) && Same(record.evidence, evidence) && record.operation_id == operation)
      return {UpdateStatus::Duplicate, state_.revision, state_.halted, "effect already recorded"};
    return conflict("effect ID reused with different physical evidence");
  }
  if (expected != state_.revision) return conflict("physical effect revision mismatch");
  if (effect.id.empty() || operation.empty() || evidence.source.empty() || evidence.run_id.empty() ||
      evidence.task_id.empty() || evidence.operation_id != operation ||
      !Fresh(evidence, source_now, steady_now))
    return conflict("physical evidence is missing, stale, future dated or nonfinite");
  if ((effect.object && !Valid(*effect.object)) || (effect.holder && !Valid(*effect.holder)))
    return conflict("invalid object or exact holder reference");
  switch (effect.kind) {
    case EffectKind::GripperEmpty:
      if (effect.object || effect.holder) return conflict("empty observation cannot name an object or holder");
      break;
    case EffectKind::GripperHolding:
      if (effect.holder && effect.holder->kind != HolderKind::Gripper)
        return conflict("held observation cannot name a fixture holder");
      break;
    case EffectKind::GripperUncertain:
      if (effect.holder || effect.eligibility!=TravelEligibility::Unknown)
        return conflict("uncertain gripper cannot establish a holder or clearance");
      break;
    case EffectKind::Released:
    case EffectKind::ObjectAtHolder:
      break;
    case EffectKind::TravelEligibility:
      if (effect.holder) return conflict("clearance observation cannot change the holder");
      break;
    default:
      return conflict("unknown physical effect kind");
  }
  switch (effect.eligibility) {
    case TravelEligibility::Unknown:
    case TravelEligibility::CapturedInFixture:
    case TravelEligibility::SafeClear:
    case TravelEligibility::Unsupported:
      break;
    default:
      return conflict("unknown travel eligibility");
  }
  const bool needs_object = effect.kind == EffectKind::GripperHolding ||
                            effect.kind == EffectKind::Released || effect.kind == EffectKind::ObjectAtHolder;
  if (needs_object && !effect.object) return conflict("physical effect requires an object identity");
  if (effect.kind == EffectKind::ObjectAtHolder && !effect.holder)
    return conflict("holder observation requires an exact holder");
  if (effect.kind == EffectKind::ObjectAtHolder && effect.holder->kind == HolderKind::FixtureSlot &&
      effect.eligibility != TravelEligibility::Unknown)
    return conflict("fixture observation cannot establish gripper travel eligibility");
  if (effect.kind == EffectKind::Released && effect.holder && effect.holder->kind == HolderKind::Gripper)
    return conflict("release destination cannot be the gripper");
  if (effect.object) {
    const auto existing = state_.objects.find(effect.object->id);
    if (existing != state_.objects.end() && !Same(existing->second.object, *effect.object))
      return conflict("physical evidence changes object identity");
    if (state_.gripper.object && (effect.kind != EffectKind::ObjectAtHolder ||
                                effect.holder->kind == HolderKind::Gripper) &&
        !Same(*state_.gripper.object, *effect.object))
      return conflict("physical evidence disagrees with the held object");
  }
  const bool holding = effect.kind == EffectKind::GripperHolding ||
                       (effect.kind == EffectKind::ObjectAtHolder && effect.holder->kind == HolderKind::Gripper);
  const bool affects_gripper = effect.kind != EffectKind::TravelEligibility &&
      (effect.kind != EffectKind::ObjectAtHolder || holding ||
       (state_.gripper.object && Same(*state_.gripper.object, *effect.object)));
  const bool same_custody =
      (holding && state_.gripper.occupancy==GripperOccupancy::Holding && Same(state_.gripper.object,effect.object)) ||
      (effect.kind==EffectKind::GripperEmpty && state_.gripper.occupancy==GripperOccupancy::Empty);
  // A repeated custody observation does not independently re-observe or invalidate clearance.
  const bool custody_refresh_only=same_custody && effect.eligibility==TravelEligibility::Unknown;
  const bool affects_clearance = (affects_gripper && !custody_refresh_only) || effect.kind == EffectKind::TravelEligibility;
  if ((affects_gripper && gripper_high_water_ && !NotBefore(evidence, *gripper_high_water_)) ||
      (affects_clearance && clearance_high_water_ && !NotBefore(evidence, *clearance_high_water_)))
    return conflict("physical evidence predates current gripper or clearance evidence");
  if (effect.object && effect.kind != EffectKind::TravelEligibility) {
    const auto previous = custody_high_water_.find(effect.object->id);
    if (previous != custody_high_water_.end() && !NotBefore(evidence, previous->second))
      return conflict("physical evidence predates current object custody");
    const auto existing = state_.objects.find(effect.object->id);
    if (transfer_operations_.count(operation) && existing != state_.objects.end() &&
        !existing->second.pending && existing->second.current_evidence &&
        existing->second.current_evidence->operation_id != operation)
      return conflict("completed transfer operation cannot change later custody");
  }
  const auto destination = holding ? std::optional<HolderRef>(Gripper()) : effect.holder;
  if (effect.object && (holding || effect.kind == EffectKind::ObjectAtHolder)) {
    const auto existing = state_.objects.find(effect.object->id);
    if (existing != state_.objects.end() && existing->second.certainty == LocationCertainty::Confirmed &&
        !existing->second.pending && !Same(existing->second.last_confirmed_holder, destination))
      return conflict("holder observation contradicts confirmed custody without a pending transfer");
  }
  for (const auto& entry : state_.objects) {
    const auto& pending = entry.second.pending;
    if (!pending) continue;
    if (effect.kind==EffectKind::GripperUncertain) {
      const auto previous=custody_high_water_.find(entry.first);
      if (previous!=custody_high_water_.end() && !NotBefore(evidence,previous->second))
        return conflict("uncertainty evidence predates the pending object's custody");
      continue;
    }
    if (!effect.object || !Same(pending->object, *effect.object) || pending->operation_id != operation ||
        (!Same(pending->destination, destination) &&
         !(effect.kind == EffectKind::Released && !effect.holder)) ||
        effect.kind == EffectKind::TravelEligibility)
      return conflict("physical effect conflicts with the pending transfer");
  }
  if (effect.kind == EffectKind::TravelEligibility &&
      (state_.gripper.occupancy == GripperOccupancy::Unknown ||
       !Same(state_.gripper.object, effect.object)))
    return conflict("clearance observation requires known matching gripper custody");

  ++state_.revision;
  auto locate = [&](const ObjectRef& object, const std::optional<HolderRef>& holder) {
    auto& location = state_.objects[object.id];
    location.object = object;
    location.current_evidence = evidence;
    custody_high_water_[object.id] = evidence;
    location.revision = state_.revision;
    location.pending.reset();
    location.certainty = holder ? LocationCertainty::Confirmed : LocationCertainty::Uncertain;
    if (holder) {
      location.last_confirmed_holder = holder;
      location.last_confirmed_evidence = evidence;
    }
  };
  if (effect.kind != EffectKind::TravelEligibility) {
    if (affects_clearance) {
      state_.transport.eligibility = effect.eligibility;
      state_.transport.clearance_evidence = evidence;
    }
    if (effect.kind==EffectKind::GripperUncertain) {
      auto uncertain=[&](const ObjectRef& object) {
        auto& location=state_.objects[object.id];
        if (location.object.id.empty()) location.object=object;
        location.certainty=LocationCertainty::Uncertain;
        location.current_evidence=evidence;location.revision=state_.revision;
        custody_high_water_[object.id]=evidence;
      };
      if (state_.gripper.object) uncertain(*state_.gripper.object);
      if (effect.object) uncertain(*effect.object);
      for (const auto& entry:state_.objects) if (entry.second.pending) uncertain(entry.second.object);
      state_.gripper={GripperOccupancy::Unknown,std::nullopt,evidence,state_.revision};
    } else if (holding) {
      locate(*effect.object, Gripper());
      state_.gripper = {GripperOccupancy::Holding, effect.object, evidence, state_.revision};
    } else if (effect.kind == EffectKind::GripperEmpty || effect.kind == EffectKind::Released) {
      if (state_.gripper.object) locate(*state_.gripper.object, std::nullopt);
      if (effect.kind == EffectKind::Released) locate(*effect.object, effect.holder);
      state_.gripper = {GripperOccupancy::Empty, std::nullopt, evidence, state_.revision};
    } else {
      locate(*effect.object, effect.holder);
      if (state_.gripper.object && Same(*state_.gripper.object, *effect.object))
        state_.gripper = {GripperOccupancy::Empty, std::nullopt, evidence, state_.revision};
    }
  } else {
    state_.transport.eligibility = effect.eligibility;
    state_.transport.clearance_evidence = evidence;
  }
  if (!SameTransport(transport_before,SnapshotLocked(source_now,steady_now).transport))
    state_.transport.revision = state_.revision;
  if (affects_gripper) gripper_high_water_ = evidence;
  if (affects_clearance) clearance_high_water_ = evidence;
  state_.effects.push_back({effect, evidence, operation, expected, state_.revision, UpdateStatus::Applied});
  return {UpdateStatus::Applied, state_.revision, state_.halted, "physical effect recorded"};
}
}  // namespace agx_robot_task_runtime
