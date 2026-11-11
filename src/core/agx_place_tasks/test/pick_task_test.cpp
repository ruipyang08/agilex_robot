#include "agx_place_tasks/pick_task.hpp"

#include "agx_place_tasks/onboard_positions.hpp"

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

PreparationState Plan(bool arrival = true, bool payload = false) {
  agx_robot_task_runtime::RobotSequencePlan plan;
  if (arrival) plan.held_station = "wb-100";
  plan.payload_may_be_retained = payload;
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
  PickParameters selected;
  PickTaskPorts ports;
  PickTaskPorts policy_ports;
  ports.cancelled = [] { return false; };
  ports.pick = [&](const PickParameters& request) {
    ++executions;
    selected = request;
    return StepResult{true, "picked"};
  };
  ports.admit = [&](const std::string&, const PickParameters&) {
    ++admissions;
    return StepResult{true, "read-only admission"};
  };

  auto positions = std::make_shared<PositionTable>();
  positions->frame = "link_base_lift0";
  positions->grasp_z = -0.105;
  positions->approach_z = 0.040;
  positions->tool_rpy_deg = std::array<double, 3>{{180.0, 0.0, 1.32}};
  positions->tool_yaw_offset_deg = 45.0;
  positions->positions.emplace("1", PositionTarget{"1", 0.3240, 0.0670, -0.105, 0.040});
  positions->positions.emplace("4", PositionTarget{"4", 0.2524, 0.0716, -0.105, -0.020});
  policy_ports = ports;
  policy_ports.positions = positions;

  TaskRegistry registry;
  RegisterPickTasks(registry, PlaceInstallation{Rack()}, policy_ports);
  const auto definition = DescribeTask(registry, "pick_from_onboard");
  assert(definition.type == "pick_from_onboard");

  const auto spec =
      Spec("schema_version: 2\nworkflow_id: pick\ntasks: [{id: take, type: pick_from_onboard, slot: 3}]\n");
  auto state = Plan();
  auto prepared = definition.prepare(spec, state);
  assert(executions == 0 && admissions == 0);
  const auto executed = prepared.execute({});
  assert(executed.ok && executions == 1);
  assert(selected.slot == 3 && selected.rack && selected.rack->profile_id == "onboard_rack_v1");
  assert(prepared.admission_check().ok && admissions == 1);
  // Taking a tube means the gripper may now hold one, which is exactly what
  // place_test_tube requires as its precondition.
  assert(state.planning.Get<agx_robot_task_runtime::RobotSequencePlan>(
             agx_robot_task_runtime::kRobotSequencePlan).payload_may_be_retained);

  // An already-held payload cannot be picked again.
  ThrowsPrepare(definition, spec, Plan(true, true));
  // No verified arrival.
  ThrowsPrepare(definition, spec, Plan(false, false));
  // Slot outside the taught set, and non-integer slots.
  ThrowsPrepare(definition,
                Spec("schema_version: 2\nworkflow_id: pick\ntasks: [{id: take, type: pick_from_onboard, slot: 7}]\n"),
                Plan());
  ThrowsPrepare(definition,
                Spec("schema_version: 2\nworkflow_id: pick\ntasks: [{id: take, type: pick_from_onboard, slot: 0}]\n"),
                Plan());
  // Unknown fields and a foreign rack name are refused.
  ThrowsPrepare(definition,
                Spec("schema_version: 2\nworkflow_id: pick\ntasks: [{id: take, type: pick_from_onboard, slot: 1, speed: 2}]\n"),
                Plan());
  ThrowsPrepare(definition,
                Spec("schema_version: 2\nworkflow_id: pick\ntasks: [{id: take, type: pick_from_onboard, slot: 1, rack_profile: other}]\n"),
                Plan());
  auto matching_state = Plan();
  (void)definition.prepare(
      Spec("schema_version: 2\nworkflow_id: pick\ntasks: [{id: take, type: pick_from_onboard, slot: 1, rack_profile: onboard_rack_v1}]\n"),
      matching_state);

  TaskRegistry unqualified_registry;
  RegisterPickTasks(unqualified_registry, PlaceInstallation{Rack(false)}, policy_ports);
  ThrowsPrepare(DescribeTask(unqualified_registry, "pick_from_onboard"), spec, Plan());
  TaskRegistry empty_registry;
  RegisterPickTasks(empty_registry, PlaceInstallation{}, policy_ports);
  ThrowsPrepare(DescribeTask(empty_registry, "pick_from_onboard"), spec, Plan());

  // 用实测位置表代替示教槽位。
  {
    const auto by_position = Spec(
        "schema_version: 2\nworkflow_id: pick\n"
        "tasks: [{id: take, type: pick_from_onboard, position: 4}]\n");
    auto position_state = Plan();
    const auto prepared_by_position = definition.prepare(by_position, position_state);
    (void)prepared_by_position;
  }
  // slot 与 position 只能二选一。
  ThrowsPrepare(definition,
                Spec("schema_version: 2\nworkflow_id: pick\n"
                     "tasks: [{id: take, type: pick_from_onboard, slot: 1, position: 1}]\n"),
                Plan());
  // 位置表里没有这个名字。
  ThrowsPrepare(definition,
                Spec("schema_version: 2\nworkflow_id: pick\n"
                     "tasks: [{id: take, type: pick_from_onboard, position: 9}]\n"),
                Plan());
  // 没装位置表时不能按 position 取管。
  {
    TaskRegistry no_table;
    RegisterPickTasks(no_table, PlaceInstallation{Rack()}, ports);
    ThrowsPrepare(DescribeTask(no_table, "pick_from_onboard"),
                  Spec("schema_version: 2\nworkflow_id: pick\n"
                       "tasks: [{id: take, type: pick_from_onboard, position: 1}]\n"),
                  Plan());
  }
}
