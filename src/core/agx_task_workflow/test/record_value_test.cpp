#include <cassert>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "agx_task_workflow/execution_context.hpp"
#include "agx_task_workflow/task_runtime_ports.hpp"
#include "agx_task_workflow/examples/record_value_task.hpp"

namespace wf = agx_task_workflow;
namespace example = agx_task_workflow::examples::record_value;

int main() {
  double now = 10.0;
  bool cancelled = false;
  int callbacks = 0;
  std::vector<std::string> values;
  wf::TaskRuntimeBinding binding;
  wf::TaskRegistry registry;
  example::RegisterRecordValueTask(registry,
      [&](const std::string& task_id) { return binding.Bind(task_id); },
      [&](const std::string& value, const wf::TaskRuntimePorts& ports) {
        ++callbacks;
        values.push_back(value);
        wf::StepResult result{ports.check() == wf::FailureKind::None, "recorded " + value};
        if (!result.ok) result.failure = ports.check();
        return result;
      });

  const auto description = wf::DescribeTask(registry, "record_value");
  assert(description.contract_version == 1 && description.inputs.size() == 1);
  assert(description.resources.empty() && description.interfaces.empty());
  assert(callbacks == 0);

  const auto config = wf::ParseConfig(
      "schema_version: 2\nworkflow_id: record\ntasks:\n"
      "  - {id: first, type: record_value, value: sample-1}\n");
  auto recipe = wf::PrepareRecipe(config, registry);
  assert(callbacks == 0 && recipe.tasks.size() == 1);
  wf::Ports run;
  run.cancelled = [&] { return cancelled; };
  run.stop = [] {};
  run.phase = [](const std::string&) {};
  run.task_event = [](const wf::TaskResult&) {};
  run.steady_now = [&] { return now; };
  run.task_begin = [&](const wf::PreparedTask& task) {
    auto context = std::make_shared<wf::ExecutionContext>(5.0, [&] { return now; }, [&] { return cancelled; });
    binding.Activate("example-run", task.id, "example-run/" + task.id, context);
  };
  run.runtime_guard = [&] { return binding.Bind("first").check(); };
  const auto result = wf::Run(recipe.tasks, run);
  binding.Expire();
  assert(result.success && callbacks == 1 && values == std::vector<std::string>{"sample-1"});

  for (const auto& yaml : {
      "schema_version: 2\nworkflow_id: bad\ntasks: [{id: one,type: record_value}]\n",
      "schema_version: 2\nworkflow_id: bad\ntasks: [{id: one,type: record_value,value: 4}]\n",
      "schema_version: 2\nworkflow_id: bad\ntasks: [{id: one,type: record_value,value: ok,extra: no}]\n"}) {
    bool rejected = false;
    try { (void)wf::PrepareRecipe(wf::ParseConfig(yaml), registry); }
    catch (const wf::PreparationError&) { rejected = true; }
    assert(rejected && callbacks == 1);
  }
  auto configured = config;
  configured.tasks[0].config_path = "/tmp/unsupported.yaml";
  bool rejected = false;
  try { (void)wf::PrepareRecipe(configured, registry); }
  catch (const wf::PreparationError& error) {
    rejected = std::string(error.what()).find("does not accept config") != std::string::npos;
  }
  assert(rejected && callbacks == 1);
}
