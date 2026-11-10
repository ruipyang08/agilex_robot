#include "agx_place_tasks/pick_execution.hpp"

#include <utility>

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

wf::StepResult ExecutePick(const PickParameters& request, const PickExecutionPorts& ports) {
  if (!ports.state || ports.run_id.empty() || ports.task_id.empty() || !ports.held_base ||
      !ports.refresh_custody || !ports.run || !ports.runtime_failure || !ports.stop_reason)
    return {false, "pick execution bindings are incomplete"};
  if (!ports.held_base()) return {false, "pick requires fresh held/stopped base evidence"};
  if (!request.rack || !request.rack->qualified)
    return {false, "pick requires an installed qualified onboard rack profile"};

  ports.refresh_custody();
  const auto initial = ports.state->Snapshot();
  if (initial.halted || initial.transport.pending || !initial.gripper.fresh ||
      initial.gripper.occupancy != rt::GripperOccupancy::Empty)
    return {false, "pick requires a fresh confirmed empty gripper and no pending transfer"};
  const std::string operation = ports.run_id + "/" + ports.task_id + "/pick";
  // Nothing in this system identifies a tube, so the picked one gets a run-local
  // identity. No transfer is opened: an unverified open transfer would leave the
  // store pending forever, and closing one requires evidence this robot does not
  // have. The grasp itself is the measured fact.
  const rt::ObjectRef object{operation + "/picked-tube", rt::ObjectIdentity::RunLocal,
                             "onboard_rack_tube"};

  const auto outcome = ports.run(request);
  auto revision = initial.revision;
  std::vector<std::string> applied;
  std::string detail = outcome.detail;
  bool recorded = false;

  if (outcome.grasped) {
    const auto evidence = EvidenceOf(outcome.evidence, ports.run_id, ports.task_id, operation);
    rt::PhysicalEffect holding;
    holding.id = operation + "/effect/holding";
    holding.kind = rt::EffectKind::GripperHolding;
    holding.object = object;
    const auto update = ports.state->ApplyEffect(holding, evidence, operation, revision);
    revision = update.revision;
    recorded = update.status == rt::UpdateStatus::Applied ||
               update.status == rt::UpdateStatus::Duplicate;
    if (!recorded) detail = "measured grasp was rejected: " + update.detail;

    if (recorded) {
      applied.push_back(holding.id);
      rt::PhysicalEffect clear;
      clear.id = operation + "/effect/clearance";
      clear.kind = rt::EffectKind::TravelEligibility;
      clear.object = object;
      clear.eligibility = rt::TravelEligibility::SafeClear;
      const auto cleared = ports.state->ApplyEffect(clear, evidence, operation, revision);
      revision = cleared.revision;
      if (cleared.status == rt::UpdateStatus::Applied ||
          cleared.status == rt::UpdateStatus::Duplicate)
        applied.push_back(clear.id);
      else
        detail = "clearance could not be certified: " + cleared.detail;
    }
  } else {
    detail = detail.empty() ? std::string{} : detail + "; ";
    detail += "no tube was grasped; no custody was recorded";
  }

  wf::StepResult result;
  const bool stopped = outcome.worker_joined && outcome.measured_stop_confirmed;
  result.ok = recorded && stopped;
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
