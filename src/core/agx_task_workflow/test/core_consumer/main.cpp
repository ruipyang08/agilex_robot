#include <cassert>
#include <memory>
#include "agx_task_workflow/task_registry.hpp"
#include "agx_task_workflow/task_runtime_ports.hpp"
#include "agx_task_workflow/examples/record_value_task.hpp"

namespace task = agx_task_workflow;
int main() {
  task::TaskRuntimeBinding binding;
  task::TaskRegistry registry;
  int calls = 0;
  task::examples::record_value::RegisterRecordValueTask(registry,
      [&](const std::string& id) { return binding.Bind(id); },
      [&](const std::string& value, const task::TaskRuntimePorts& ports) {
        assert(value == "sample" && ports.check() == task::FailureKind::None);
        ++calls; return task::StepResult{true, "recorded"};
      });
  const auto recipe = task::PrepareRecipe(task::ParseConfig(
      "schema_version: 2\nworkflow_id: independent\ntasks: [{id: first, type: record_value, value: sample}]\n"), registry);
  assert(calls == 0);
  task::Ports ports;
  ports.task_begin = [&](const task::PreparedTask& prepared) {
    binding.Activate("run", prepared.id, "run/" + prepared.id,
        std::make_shared<task::ExecutionContext>(5, [] { return 1.; }, [] { return false; }));
  };
  ports.cancelled = [] { return false; };
  ports.stop = [] {};
  ports.phase = [](const std::string&) {};
  ports.task_event = [](const task::TaskResult&) {};
  assert(task::Run(recipe.tasks, ports).success && calls == 1);
  binding.Expire();
}
