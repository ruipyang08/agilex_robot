#include "agx_place_tasks/place_execution.hpp"

#include <utility>
#include <vector>

namespace agx_place_tasks {
namespace {

namespace rt = agx_robot_task_runtime;
namespace wf = agx_task_workflow;

rt::Evidence EvidenceOf(const PlaceEvidence& evidence, const std::string& run_id,
                        const std::string& task_id, const std::string& operation) {
  return rt::Evidence{evidence.source, evidence.source_stamp_s, evidence.receipt_steady_s,
                      run_id, task_id, operation};
}

}  // namespace

wf::StepResult ExecutePlace(const PlaceParameters& request, const PlaceExecutionPorts& ports) {
  if (!ports.state || ports.run_id.empty() || ports.task_id.empty() || !ports.held_base ||
      !ports.refresh_custody || !ports.run || !ports.runtime_failure || !ports.stop_reason)
    return {false, "place execution bindings are incomplete"};
  if (!ports.held_base()) return {false, "place requires fresh held/stopped base evidence"};
  if (!request.rack || !request.rack->qualified)
    return {false, "place requires an installed qualified onboard rack profile"};

  ports.refresh_custody();
  const auto initial = ports.state->Snapshot();
  if (initial.halted || initial.transport.pending || !initial.gripper.fresh ||
      initial.gripper.occupancy != rt::GripperOccupancy::Holding || !initial.gripper.object)
    return {false, "place requires fresh confirmed held custody and no pending transfer"};
  const auto object = *initial.gripper.object;
  const std::string operation = ports.run_id + "/" + ports.task_id + "/place";
  const rt::HolderRef destination{rt::HolderKind::FixtureSlot, request.rack->profile_id,
                                  std::to_string(request.slot)};
  const auto begun = ports.state->BeginTransfer(initial.revision, operation, object, rt::HolderRef{},
                                                destination);
  if (begun.status != rt::UpdateStatus::Applied)
    return {false, "place transfer could not begin: " + begun.detail};

  const auto outcome = ports.run(request);
  const auto evidence = EvidenceOf(outcome.evidence, ports.run_id, ports.task_id, operation);
  auto revision = begun.revision;
  std::vector<std::string> applied;
  std::string detail = outcome.detail;
  bool confirmed = false;

  if (outcome.released && outcome.slot_occupancy_confirmed) {
    rt::PhysicalEffect effect;
    effect.id = operation + "/effect/release";
    effect.kind = rt::EffectKind::ObjectAtHolder;
    effect.object = object;
    effect.holder = destination;
    const auto update = ports.state->ApplyEffect(effect, evidence, operation, revision);
    revision = update.revision;
    confirmed = update.status == rt::UpdateStatus::Applied ||
                update.status == rt::UpdateStatus::Duplicate;
    if (confirmed) applied.push_back(effect.id);
    else detail = "measured slot occupancy was rejected: " + update.detail;
  } else if (outcome.released) {
    // The fingers opened but no sensor confirmed the slot. Record only what was
    // measured: the gripper is empty and the tube's location is uncertain.
    rt::PhysicalEffect effect;
    effect.id = operation + "/effect/release";
    effect.kind = rt::EffectKind::Released;
    effect.object = object;
    const auto update = ports.state->ApplyEffect(effect, evidence, operation, revision);
    revision = update.revision;
    detail = detail.empty() ? std::string{} : detail + "; ";
    detail += "tube released without measured slot occupancy; custody is uncertain";
  } else if (outcome.release_uncertain) {
    rt::PhysicalEffect effect;
    effect.id = operation + "/effect/uncertain";
    effect.kind = rt::EffectKind::GripperUncertain;
    effect.object = object;
    const auto update = ports.state->ApplyEffect(effect, evidence, operation, revision);
    revision = update.revision;
    detail = detail.empty() ? std::string{} : detail + "; ";
    detail += "gripper release is unconfirmed; custody is uncertain";
  } else {
    detail = detail.empty() ? std::string{} : detail + "; ";
    detail += "no measured release; transfer left pending";
  }

  wf::StepResult result;
  const bool stopped = outcome.worker_joined && outcome.measured_stop_confirmed;
  result.ok = confirmed && stopped;
  result.detail = detail;
  result.physical_effect_ids = applied;
  result.state_revision = revision;
  result.stop = stopped ? wf::StopState::Confirmed : wf::StopState::NotConfirmed;
  if (!result.ok) result.failure = stopped ? wf::FailureKind::Fault : wf::FailureKind::UnconfirmedStop;
  const auto reason = ports.stop_reason();
  if (!reason.empty()) result.detail += "; " + reason;
  const auto interrupted = ports.runtime_failure();
  if (interrupted != wf::FailureKind::None) {
    result.ok = false;
    result.failure = interrupted;
  }
  return result;
}

}  // namespace agx_place_tasks
