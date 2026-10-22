#include <cstdlib>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "agx_task_workflow/config.hpp"
#include "agx_task_workflow/execution_context.hpp"
#include "agx_task_workflow/task_runtime_ports.hpp"
#include "agx_task_workflow/examples/record_value_task.hpp"

namespace wf = agx_task_workflow;
namespace example = agx_task_workflow::examples::record_value;

namespace {
double SteadyNow() {
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
}  // namespace

int main(int argc, char** argv) {
  const bool check_only = argc == 3 && std::string(argv[1]) == "--check";
  const int path_index = check_only ? 2 : 1;
  if (argc != path_index + 1) {
    std::cerr << "usage: record_value_example [--check] WORKFLOW.yaml\n";
    return 64;
  }
  try {
    wf::TaskRuntimeBinding binding;
    wf::TaskRegistry registry;
    example::RegisterRecordValueTask(
        registry, [&](const std::string& task_id) { return binding.Bind(task_id); },
        [](const std::string& value, const wf::TaskRuntimePorts& ports) {
          if (ports.check() != wf::FailureKind::None) {
            wf::StepResult result{false, "runtime interrupted before record"};
            result.failure = ports.check();
            return result;
          }
          std::cout << "record_value: " << value << '\n';
          return wf::StepResult{true, "Value recorded"};
        });
    const auto config = wf::LoadConfig(argv[path_index]);
    auto recipe = wf::PrepareRecipe(config, registry);
    if (check_only) {
      std::cout << "Validated " << recipe.tasks.size() << " task(s); no callbacks ran.\n";
      return 0;
    }
    wf::TaskRuntimePorts active;
    wf::Ports ports;
    ports.cancelled = [] { return false; };
    ports.stop = [] {};
    ports.phase = [](const std::string&) {};
    ports.task_event = [](const wf::TaskResult&) {};
    ports.steady_now = SteadyNow;
    ports.task_begin = [&](const wf::PreparedTask& task) {
      auto context = std::make_shared<wf::ExecutionContext>(5.0, SteadyNow,
                                                            [] { return false; });
      binding.Activate("record-value-example", task.id,
                       "record-value-example/" + task.id, context);
      active = binding.Bind(task.id);
    };
    ports.runtime_guard = [&] {
      return active.check ? active.check() : wf::FailureKind::Fault;
    };
    const auto result = wf::Run(recipe.tasks, ports);
    binding.Expire();
    if (!result.success) {
      std::cerr << result.detail << '\n';
      return 1;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
