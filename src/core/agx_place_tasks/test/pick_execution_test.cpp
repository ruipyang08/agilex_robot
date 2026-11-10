#include "agx_place_tasks/pick_execution.hpp"

#include <array>
#include <cassert>
#include <memory>
#include <string>

using namespace agx_place_tasks;
namespace rt = agx_robot_task_runtime;
namespace wf = agx_task_workflow;

namespace {

std::shared_ptr<const OnboardRackProfile> Rack(bool qualified = true) {
  auto rack = std::make_shared<OnboardRackProfile>();
  rack->profile_id = "onboard_rack_v1";
  rack->parent_frame = "link_base_lift0";
  rack->tip_link = "tcp_link";
  rack->descend_axis = {0.0, 0.0, -1.0};
  rack->approach_rise_m = 0.150;
  rack->tool_rpy_deg = std::array<double, 3>{{180.0, 0.0, 1.32}};
  rack->tool_yaw_offset_deg = 45.0;
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

  void Empty() {
    assert(state->ApplyEffect({"seed-empty", rt::EffectKind::GripperEmpty}, E("seed"), "seed",
                              state->Snapshot().revision)
               .status == rt::UpdateStatus::Applied);
  }

  void Holding() {
    Empty();
    const auto object = rt::ObjectRef{"seed-tube", rt::ObjectIdentity::RunLocal, "tube"};
    const auto before = state->Snapshot().revision;
    assert(state->BeginTransfer(before, "seed-pick", object, std::nullopt, rt::HolderRef{})
               .status == rt::UpdateStatus::Applied);
    assert(state->ApplyEffect({"seed-hold", rt::EffectKind::GripperHolding, object, std::nullopt,
                               rt::TravelEligibility::CapturedInFixture},
                              E("seed-pick"), "seed-pick", state->Snapshot().revision)
               .status == rt::UpdateStatus::Applied);
  }
};

struct Scenario {
  wf::StepResult result;
  rt::RobotStateSnapshot snapshot;
  int runs = 0;
};

PickMeasurement Measured(bool grasped, bool stopped = true, bool uncertain = false) {
  PickMeasurement measurement;
  measurement.grasped = grasped;
  measurement.grasp_uncertain = uncertain;
  measurement.worker_joined = stopped;
  measurement.measured_stop_confirmed = stopped;
  measurement.detail = grasped ? "fingers stopped at 299 on an object"
                               : "fingers closed to 3; nothing was between them";
  measurement.evidence = PlaceEvidence{"measured-gripper", 100.0, 10.0};
  return measurement;
}

Scenario Run(PickMeasurement measurement, bool held_base = true,
             wf::FailureKind failure = wf::FailureKind::None, bool seed_empty = true,
             bool qualified_rack = true) {
  Rig rig;
  if (seed_empty) rig.Empty();
  PickParameters request;
  request.slot = 1;
  request.rack = Rack(qualified_rack);
  Scenario scenario;
  PickExecutionPorts ports;
  ports.state = rig.state;
  ports.run_id = "run-1";
  ports.task_id = "task-1";
  ports.held_base = [held_base] { return held_base; };
  ports.refresh_custody = [] {};
  ports.run = [&](const PickParameters&) {
    ++scenario.runs;
    return measurement;
  };
  ports.runtime_failure = [failure] { return failure; };
  ports.stop_reason = [] { return std::string{}; };
  scenario.result = ExecutePick(request, ports);
  scenario.snapshot = rig.state->Snapshot();
  return scenario;
}

}  // namespace

int main() {
  // A measured grasp establishes custody and a travel clearance.
  {
    const auto scenario = Run(Measured(true));
    assert(scenario.result.ok);
    assert(scenario.result.stop == wf::StopState::Confirmed);
    assert(scenario.result.physical_effect_ids.size() == 2);
    assert(scenario.snapshot.gripper.occupancy == rt::GripperOccupancy::Holding);
    assert(scenario.snapshot.gripper.object.has_value());
    assert(scenario.snapshot.transport.eligibility == rt::TravelEligibility::SafeClear);
    assert(scenario.snapshot.transport.ready);
  }

  // An empty slot produces no custody at all.
  {
    const auto scenario = Run(Measured(false));
    assert(!scenario.result.ok);
    assert(scenario.result.failure == wf::FailureKind::Fault);
    assert(scenario.result.physical_effect_ids.empty());
    assert(scenario.snapshot.gripper.occupancy == rt::GripperOccupancy::Empty);
    assert(!scenario.snapshot.gripper.object.has_value());
  }

  // A grasp that could not be measured is a failure, not a silent success.
  {
    const auto scenario = Run(Measured(false, true, true));
    assert(!scenario.result.ok);
    assert(scenario.result.physical_effect_ids.empty());
  }

  // A confirmed grasp without a confirmed stop is not success.
  {
    const auto scenario = Run(Measured(true, false));
    assert(!scenario.result.ok);
    assert(scenario.result.failure == wf::FailureKind::UnconfirmedStop);
    assert(scenario.result.stop == wf::StopState::NotConfirmed);
  }

  // Entry guards run before any motion.
  {
    const auto not_empty = Run(Measured(true), true, wf::FailureKind::None, false);
    assert(!not_empty.result.ok);
    // The seeded store has an unknown gripper, so entry fails before moving.
    assert(not_empty.runs == 0);
    const auto no_base = Run(Measured(true), false);
    assert(!no_base.result.ok && no_base.runs == 0);
    const auto unqualified = Run(Measured(true), true, wf::FailureKind::None, true, false);
    assert(!unqualified.result.ok && unqualified.runs == 0);
  }

  // An interruption keeps its own cause classification.
  {
    const auto scenario = Run(Measured(true), true, wf::FailureKind::Cancelled);
    assert(!scenario.result.ok);
    assert(scenario.result.failure == wf::FailureKind::Cancelled);
  }
}
