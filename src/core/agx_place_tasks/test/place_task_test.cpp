#include "agx_place_tasks/place_task.hpp"

#include <array>
#include <cassert>
#include <memory>
#include <string>

#include "agx_robot_task_runtime/robot_sequence_plan.hpp"
#include "agx_task_workflow/config.hpp"

using namespace agx_place_tasks;
using namespace agx_task_workflow;

namespace {

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

PreparationState PlanWithPayload(bool retained = true, bool arrival = true) {
  agx_robot_task_runtime::RobotSequencePlan plan;
  if (arrival) plan.held_station = "wb-100";
  plan.payload_may_be_retained = retained;
  PreparationState state;
  state.planning.Set(agx_robot_task_runtime::kRobotSequencePlan, plan);
  return state;
}

TaskSpec Spec(const std::string& yaml) { return ParseConfig(yaml).tasks[0]; }

void ThrowsPrepare(const TaskDefinition& definition, const TaskSpec& spec, PreparationState state) {
  bool thrown = false;
  try {
    definition.prepare(spec, state);
    (void)state;
  } catch (const std::invalid_argument&) {
    thrown = true;
  }
  assert(thrown);
}

}  // namespace

int main() {
  int executions = 0, admissions = 0;
  PlaceParameters selected;
  PlaceTaskPorts ports;
  ports.cancelled = [] { return false; };
  ports.place = [&](const PlaceParameters& request) {
    ++executions;
    selected = request;
    return StepResult{true, "placed"};
  };
  ports.admit = [&](const std::string&, const PlaceParameters&) {
    ++admissions;
    return StepResult{true, "read-only admission"};
  };

  TaskRegistry registry;
  RegisterPlaceTasks(registry, PlaceInstallation{Rack()}, ports);
  const auto definition = DescribeTask(registry, "place_test_tube");
  assert(definition.type == "place_test_tube");

  const auto spec =
      Spec("schema_version: 2\nworkflow_id: place\ntasks: [{id: put, type: place_test_tube, slot: 2}]\n");
  auto state = PlanWithPayload();
  auto prepared = definition.prepare(spec, state);
  assert(executions == 0 && admissions == 0);
  const auto executed = prepared.execute({});
  assert(executed.ok && executions == 1);
  assert(selected.slot == 2 && selected.rack && selected.rack->profile_id == "onboard_rack_v1");
  assert(prepared.admission_check().ok && admissions == 1);
  // The tube leaves the gripper but stays on the robot, so the plan no longer
  // claims a payload retained by the gripper.
  assert(!state.planning.Get<agx_robot_task_runtime::RobotSequencePlan>(
              agx_robot_task_runtime::kRobotSequencePlan).payload_may_be_retained);

  // No preceding task may have produced a payload.
  ThrowsPrepare(definition, spec, PlanWithPayload(false));
  // No verified arrival.
  ThrowsPrepare(definition, spec, PlanWithPayload(true, false));
  // Slots outside the taught set, and non-integer slots.
  const auto far =
      Spec("schema_version: 2\nworkflow_id: place\ntasks: [{id: put, type: place_test_tube, slot: 7}]\n");
  ThrowsPrepare(definition, far, PlanWithPayload());
  const auto zero =
      Spec("schema_version: 2\nworkflow_id: place\ntasks: [{id: put, type: place_test_tube, slot: 0}]\n");
  ThrowsPrepare(definition, zero, PlanWithPayload());
  const auto fractional =
      Spec("schema_version: 2\nworkflow_id: place\ntasks: [{id: put, type: place_test_tube, slot: 1.5}]\n");
  ThrowsPrepare(definition, fractional, PlanWithPayload());
  // Unknown fields are refused rather than silently ignored.
  const auto extra =
      Spec("schema_version: 2\nworkflow_id: place\ntasks: [{id: put, type: place_test_tube, slot: 1, speed: 2}]\n");
  ThrowsPrepare(definition, extra, PlanWithPayload());
  // The named profile must be the installed one.
  const auto named = Spec(
      "schema_version: 2\nworkflow_id: place\ntasks: [{id: put, type: place_test_tube, slot: 1, rack_profile: other_rack}]\n");
  ThrowsPrepare(definition, named, PlanWithPayload());
  // A matching name is accepted.
  const auto matching = Spec(
      "schema_version: 2\nworkflow_id: place\ntasks: [{id: put, type: place_test_tube, slot: 1, rack_profile: onboard_rack_v1}]\n");
  auto matching_state = PlanWithPayload();
  (void)definition.prepare(matching, matching_state);

  // An unqualified rack cannot be prepared, and neither can a missing one.
  TaskRegistry unqualified_registry;
  RegisterPlaceTasks(unqualified_registry, PlaceInstallation{Rack(false)}, ports);
  ThrowsPrepare(DescribeTask(unqualified_registry, "place_test_tube"), spec, PlanWithPayload());
  TaskRegistry empty_registry;
  RegisterPlaceTasks(empty_registry, PlaceInstallation{}, ports);
  ThrowsPrepare(DescribeTask(empty_registry, "place_test_tube"), spec, PlanWithPayload());
}
