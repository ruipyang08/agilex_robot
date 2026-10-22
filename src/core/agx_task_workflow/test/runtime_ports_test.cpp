#include <cassert>
#include <memory>
#include <stdexcept>
#include "agx_task_workflow/task_runtime_ports.hpp"

using namespace agx_task_workflow;
int main() {
  double now = 10;
  bool cancelled = false;
  TaskRuntimeBinding binding;
  auto context = std::make_shared<ExecutionContext>(5, [&] { return now; }, [&] { return cancelled; });
  binding.Activate("run", "first", "run/first", context);
  auto first = binding.Bind("first");
  assert(first.run_id == "run" && first.task_id == "first" && first.operation_id == "run/first");
  assert(first.check() == FailureKind::None && first.deadline_steady_s() == 15);
  bool rejected = false;
  try { (void)binding.Bind("other"); } catch (const std::runtime_error&) { rejected = true; }
  assert(rejected);
  cancelled = true;
  assert(first.check() == FailureKind::Cancelled);
  now = 12;
  binding.Activate("run", "second", "run/second",
      std::make_shared<ExecutionContext>(20, [&] { return now; }, [] { return false; }));
  assert(first.check() == FailureKind::Fault);
  rejected = false;
  try { (void)first.deadline_steady_s(); } catch (const InvocationExpired&) { rejected = true; }
  assert(rejected);
  auto second = binding.Bind("second");
  assert(second.deadline_steady_s() == 32);
  binding.Expire();
  assert(second.check() == FailureKind::Fault);
  binding.Activate("run", "second", "run/second",
      std::make_shared<ExecutionContext>(20, [&] { return now; }, [] { return false; }));
  assert(second.check() == FailureKind::Fault);  // Same labels, different invocation.
  assert(binding.Bind("second").check() == FailureKind::None);
  TaskRuntimePorts orphaned;
  {
    TaskRuntimeBinding temporary;
    temporary.Activate("run", "temporary", "run/temporary",
        std::make_shared<ExecutionContext>(20, [&] { return now; }, [] { return false; }));
    orphaned = temporary.Bind("temporary");
  }
  assert(orphaned.check() == FailureKind::Fault);
}
