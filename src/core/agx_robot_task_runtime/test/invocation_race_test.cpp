#include <cassert>
#include <chrono>
#include <future>
#include <memory>
#include "agx_robot_task_runtime/robot_task_ports.hpp"

namespace task = agx_task_workflow;
namespace robot = agx_robot_task_runtime;
using namespace std::chrono_literals;

void EffectCompletesBeforeInvocationExpires(bool activate_next) {
  std::promise<void> entered, release;
  auto released = release.get_future().share();
  bool block_clock = true;  // Only the effect thread reads this until joined.
  auto state = std::make_shared<robot::RobotStateStore>([&] {
    if (block_clock) { entered.set_value(); released.wait(); }
    return 10.;
  }, [] { return 10.; });
  robot::RobotTaskRuntimeBinding binding(state);
  auto context = [] { return std::make_shared<task::ExecutionContext>(5, [] { return 10.; }, [] { return false; }); };
  binding.Activate("run", "first", "run/first", context());
  const auto first = binding.Bind("first");
  robot::PhysicalEffect empty;
  empty.id = "observed-empty"; empty.kind = robot::EffectKind::GripperEmpty;
  robot::Evidence evidence{"sensor", 10, 10, "run", "first", "run/first"};
  auto effect = std::async(std::launch::async, [&] {
    return first.apply_effect(empty, evidence, "run/first", 0);
  });
  entered.get_future().wait();  // Inside the authoritative update, with invocation lock held.
  std::promise<void> transition_started;
  auto transition = std::async(std::launch::async, [&] {
    transition_started.set_value();
    if (activate_next) binding.Activate("run", "next", "run/next", context());
    else binding.Expire();
  });
  transition_started.get_future().wait();
  // The explicit effect barrier, not elapsed simulation time, controls release.
  assert(transition.wait_for(50ms) == std::future_status::timeout);
  release.set_value();
  assert(effect.get().status == robot::UpdateStatus::Applied);
  transition.get();
  block_clock = false;
  assert(first.check() == task::FailureKind::Fault);
  empty.id = "late-effect";
  assert(first.apply_effect(empty, evidence, "run/first", 1).status == robot::UpdateStatus::Rejected);
  const auto snapshot = state->Snapshot();
  assert(snapshot.revision == 1 && snapshot.effects.size() == 1);
  assert(snapshot.gripper.occupancy == robot::GripperOccupancy::Empty);
  if (activate_next) assert(binding.Bind("next").snapshot().revision == 1);
}
int main() {
  EffectCompletesBeforeInvocationExpires(false);
  EffectCompletesBeforeInvocationExpires(true);
}
