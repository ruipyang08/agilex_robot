#include <cassert>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "agx_task_workflow/execution_context.hpp"
#include "agx_robot_task_runtime/robot_task_ports.hpp"

using namespace agx_robot_task_runtime;

namespace {

Evidence EvidenceFor(const RobotTaskPorts& ports, double now) {
  return {"fixture", now, now, ports.run_id, ports.task_id, ports.operation_id};
}

void IdentityDeadlineAndExpiryStayPinned() {
  double now = 10.0;
  bool cancelled = false;
  auto store = std::make_shared<RobotStateStore>([&] { return now; }, [&] { return now; });
  RobotTaskRuntimeBinding binding(store);
  auto first_context = std::make_shared<agx_task_workflow::ExecutionContext>(5.0, [&] { return now; }, [&] { return cancelled; });
  binding.Activate("run", "first", "run/first", first_context);
  const auto first = binding.Bind("first");
  assert(first.run_id == "run" && first.task_id == "first" && first.operation_id == "run/first");
  assert(first.deadline_steady_s() == 15.0 && first.check() == agx_task_workflow::FailureKind::None);
  assert(first.snapshot().revision == 0);

  now = 12.0;
  auto second_context = std::make_shared<agx_task_workflow::ExecutionContext>(20.0, [&] { return now; }, [] { return false; });
  binding.Activate("run", "second", "run/second", second_context);
  const auto second = binding.Bind("second");
  assert(second.deadline_steady_s() == 32.0);
  assert(first.check() == agx_task_workflow::FailureKind::Fault);
  bool expired = false;
  try { (void)first.deadline_steady_s(); }
  catch (const std::runtime_error&) { expired = true; }
  assert(expired);
  expired = false;
  try { (void)first.snapshot(); }
  catch (const std::runtime_error&) { expired = true; }
  assert(expired);
  bool wrong_task = false;
  try { (void)binding.Bind("first"); }
  catch (const std::runtime_error&) { wrong_task = true; }
  assert(wrong_task);
}

void IntentStopsAtCancellationButVerifiedEffectsSurvive() {
  double now = 20.0;
  bool cancelled = false;
  auto store = std::make_shared<RobotStateStore>([&] { return now; }, [&] { return now; });
  RobotTaskRuntimeBinding binding(store);
  auto context = std::make_shared<agx_task_workflow::ExecutionContext>(5.0, [&] { return now; }, [&] { return cancelled; });
  binding.Activate("run", "task", "run/task", context);
  const auto ports = binding.Bind("task");

  const ObjectRef object{"tube-1", ObjectIdentity::RunLocal, "tube"};
  PhysicalEffect empty;
  empty.id = "empty";
  empty.kind = EffectKind::GripperEmpty;
  const auto observed_empty = ports.apply_effect(empty, EvidenceFor(ports, now),
                                                 ports.operation_id, 0);
  assert(observed_empty.status == UpdateStatus::Applied && observed_empty.revision == 1);
  auto begun = ports.begin_transfer(1, ports.operation_id, object, std::nullopt,
                                    HolderRef{HolderKind::Gripper, {}, {}});
  assert(begun.status == UpdateStatus::Applied && begun.revision == 2);

  cancelled = true;
  auto denied = ports.begin_transfer(2, ports.operation_id, object, std::nullopt,
                                     HolderRef{HolderKind::Gripper, {}, {}});
  assert(denied.status == UpdateStatus::Rejected);
  assert(denied.detail.find("interrupted") != std::string::npos);

  PhysicalEffect captured;
  captured.id = "capture";
  captured.kind = EffectKind::GripperHolding;
  captured.object = object;
  const auto applied = ports.apply_effect(captured, EvidenceFor(ports, now),
                                          ports.operation_id, 2);
  assert(applied.status == UpdateStatus::Applied && applied.revision == 3);
  const auto snapshot = store->Snapshot();
  assert(snapshot.gripper.occupancy == GripperOccupancy::Holding);
  assert(snapshot.gripper.object && snapshot.gripper.object->id == "tube-1");

  auto wrong_evidence = EvidenceFor(ports, now);
  wrong_evidence.task_id = "other";
  const auto rejected = ports.apply_effect(captured, wrong_evidence,
                                           ports.operation_id, snapshot.revision);
  assert(rejected.status == UpdateStatus::Rejected);
  assert(rejected.detail.find("identity") != std::string::npos);
}

void OptionalHeldArrivalUsesOnlyTheActiveMeasuredCallback() {
  double now = 30.0;
  auto store = std::make_shared<RobotStateStore>([&] { return now; }, [&] { return now; });
  RobotTaskRuntimeBinding binding(store);
  auto context = std::make_shared<agx_task_workflow::ExecutionContext>(5.0, [&] { return now; }, [] { return false; });
  binding.Activate("run", "plain", "run/plain", context);
  const auto plain = binding.Bind("plain");
  assert(!plain.held_arrival);

  int checks = 0;
  binding.Activate("run", "held", "run/held", context,
      [&](const std::string& station) {
        ++checks;
        return agx_task_workflow::StepResult{station == "wb-100", station == "wb-100" ? "fresh measured hold" : "hold absent or stale"};
      });
  const auto held = binding.Bind("held");
  assert(held.held_arrival);
  assert(held.held_arrival("wb-100").ok);
  assert(!held.held_arrival("wb-200").ok);
  assert(checks == 2);
  binding.Expire();
  assert(!held.held_arrival("wb-100").ok);
  assert(checks == 2);
}

void ResourceOwnershipIsExplicit() {
  assert(!UsesSharedRobotActuators({}));
  assert(!UsesSharedRobotActuators({agx_robot_task_runtime::resources::Camera}));
  assert(UsesSharedRobotActuators({agx_robot_task_runtime::resources::Base}));
  assert(UsesSharedRobotActuators({agx_robot_task_runtime::resources::Arm}));
  assert(UsesSharedRobotActuators({agx_robot_task_runtime::resources::Lift}));
  assert(UsesSharedRobotActuators({agx_robot_task_runtime::resources::Gripper}));
}

}  // namespace

int main() {
  IdentityDeadlineAndExpiryStayPinned();
  IntentStopsAtCancellationButVerifiedEffectsSurvive();
  OptionalHeldArrivalUsesOnlyTheActiveMeasuredCallback();
  ResourceOwnershipIsExplicit();
}
