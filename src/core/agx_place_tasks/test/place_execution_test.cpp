#include "agx_place_tasks/place_execution.hpp"

#include <array>
#include <cassert>
#include <memory>
#include <string>

using namespace agx_place_tasks;
namespace rt = agx_robot_task_runtime;
namespace wf = agx_task_workflow;

namespace {

const rt::ObjectRef kTube{"run-1/task-1/place/selected-tube", rt::ObjectIdentity::RunLocal,
                          "capped-tube-v1"};

std::shared_ptr<const OnboardRackProfile> Rack(bool qualified = true) {
  auto rack = std::make_shared<OnboardRackProfile>();
  rack->profile_id = "onboard_rack_v1";
  rack->parent_frame = "link_base_lift0";
  rack->tip_link = "tcp_link";
  rack->descend_axis = {0.0, 0.0, -1.0};
  rack->approach_rise_m = 0.150;
  rack->tool_rpy_deg = std::array<double, 3>{{180.0, 0.0, 1.32}};
  for (std::size_t slot = 1; slot <= 6; ++slot)
    rack->slots.push_back(RackSlot{slot, "slot" + std::to_string(slot),
                                   {0.30, -0.10 + 0.04 * static_cast<double>(slot - 1), -0.078}});
  rack->qualified = qualified;
  rack->evidence_id = qualified ? "fixture-not-a-measurement" : "unqualified";
  std::string detail;
  assert(Validate(*rack, &detail));
  return rack;
}

struct Rig {
  double source_now = 100.0, steady_now = 10.0;
  std::shared_ptr<rt::RobotStateStore> state =
      std::make_shared<rt::RobotStateStore>([this] { return source_now; },
                                            [this] { return steady_now; }, 1.0);

  rt::Evidence E(const std::string& operation) const {
    return {"measured-gripper", source_now, steady_now, "run-1", "task-1", operation};
  }

  void Seed() {
    const auto empty = state->Snapshot().revision;
    assert(state->ApplyEffect({"seed-empty", rt::EffectKind::GripperEmpty}, E("seed"), "seed", empty)
               .status == rt::UpdateStatus::Applied);
    assert(state->BeginTransfer(state->Snapshot().revision, "seed-pick", kTube, std::nullopt,
                                rt::HolderRef{})
               .status == rt::UpdateStatus::Applied);
    const auto held = state->Snapshot().revision;
    assert(state->ApplyEffect({"seed-hold", rt::EffectKind::GripperHolding, kTube, std::nullopt,
                               rt::TravelEligibility::CapturedInFixture},
                              E("seed-pick"), "seed-pick", held)
               .status == rt::UpdateStatus::Applied);
  }
};

struct Scenario {
  wf::StepResult result;
  rt::RobotStateSnapshot snapshot;
  int runs = 0;
};

Scenario Run(PlaceMotionOutcome outcome, bool held_base = true,
             wf::FailureKind failure = wf::FailureKind::None, bool seed = true,
             bool qualified_rack = true) {
  Rig rig;
  if (seed) rig.Seed();
  PlaceParameters request;
  request.slot = 2;
  request.rack = Rack(qualified_rack);
  Scenario scenario;
  PlaceExecutionPorts ports;
  ports.state = rig.state;
  ports.run_id = "run-1";
  ports.task_id = "task-1";
  ports.held_base = [held_base] { return held_base; };
  ports.refresh_custody = [] {};
  ports.run = [&](const PlaceParameters&) {
    ++scenario.runs;
    return outcome;
  };
  ports.runtime_failure = [failure] { return failure; };
  ports.stop_reason = [] { return std::string{}; };
  scenario.result = ExecutePlace(request, ports);
  scenario.snapshot = rig.state->Snapshot();
  return scenario;
}

PlaceMotionOutcome Outcome(bool released, bool confirmed, bool uncertain = false,
                           bool stopped = true) {
  PlaceMotionOutcome outcome;
  outcome.released = released;
  outcome.release_uncertain = uncertain;
  outcome.slot_occupancy_measured = released;
  outcome.slot_occupancy_confirmed = confirmed;
  outcome.worker_joined = stopped;
  outcome.measured_stop_confirmed = stopped;
  outcome.detail = "measured";
  outcome.evidence = PlaceEvidence{"measured-gripper", 100.0, 10.0};
  return outcome;
}

}  // namespace

int main() {
  // A confirmed release resolves the custody transfer into the configured slot.
  {
    const auto scenario = Run(Outcome(true, true));
    assert(scenario.result.ok);
    assert(scenario.result.stop == wf::StopState::Confirmed);
    assert(scenario.result.physical_effect_ids.size() == 1);
    const auto& location = scenario.snapshot.objects.at(kTube.id);
    assert(location.certainty == rt::LocationCertainty::Confirmed);
    assert(location.last_confirmed_holder &&
           location.last_confirmed_holder->kind == rt::HolderKind::FixtureSlot &&
           location.last_confirmed_holder->fixture_instance == "onboard_rack_v1" &&
           location.last_confirmed_holder->slot == "2");
    assert(scenario.snapshot.gripper.occupancy == rt::GripperOccupancy::Empty);
    assert(!scenario.snapshot.transport.pending);
  }

  // Opening without a slot measurement is not a placement: the tube stays
  // uncertain and the task fails.
  {
    const auto scenario = Run(Outcome(true, false));
    assert(!scenario.result.ok);
    assert(scenario.result.failure == wf::FailureKind::Fault);
    assert(scenario.result.physical_effect_ids.empty());
    const auto& location = scenario.snapshot.objects.at(kTube.id);
    assert(location.certainty == rt::LocationCertainty::Uncertain);
    assert(!location.last_confirmed_holder ||
           location.last_confirmed_holder->kind == rt::HolderKind::Gripper);
    assert(scenario.snapshot.gripper.occupancy == rt::GripperOccupancy::Empty);
  }

  // An unconfirmed opening makes both the gripper and the tube unknown.
  {
    const auto scenario = Run(Outcome(false, false, true));
    assert(!scenario.result.ok);
    assert(scenario.snapshot.gripper.occupancy == rt::GripperOccupancy::Unknown);
    assert(scenario.snapshot.objects.at(kTube.id).certainty == rt::LocationCertainty::Uncertain);
  }

  // No release at all leaves the transfer pending rather than guessing custody.
  {
    const auto scenario = Run(Outcome(false, false));
    assert(!scenario.result.ok);
    assert(scenario.snapshot.transport.pending);
    assert(scenario.snapshot.objects.at(kTube.id).certainty == rt::LocationCertainty::Confirmed);
  }

  // A confirmed release without a confirmed stop is not success.
  {
    const auto scenario = Run(Outcome(true, true, false, false));
    assert(!scenario.result.ok);
    assert(scenario.result.failure == wf::FailureKind::UnconfirmedStop);
    assert(scenario.result.stop == wf::StopState::NotConfirmed);
  }

  // Entry guards run before any motion or state change.
  {
    const auto missing = Run(Outcome(true, true), true, wf::FailureKind::None, false);
    assert(!missing.result.ok && missing.runs == 0);
    const auto no_base = Run(Outcome(true, true), false);
    assert(!no_base.result.ok && no_base.runs == 0);
    const auto unqualified = Run(Outcome(true, true), true, wf::FailureKind::None, true, false);
    assert(!unqualified.result.ok && unqualified.runs == 0);
  }

  // An interruption keeps its own cause classification.
  {
    const auto scenario = Run(Outcome(true, true), true, wf::FailureKind::Cancelled);
    assert(!scenario.result.ok);
    assert(scenario.result.failure == wf::FailureKind::Cancelled);
  }
}
