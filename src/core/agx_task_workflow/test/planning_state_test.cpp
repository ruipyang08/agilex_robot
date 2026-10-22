#include <cassert>
#include <stdexcept>
#include <string>
#include "agx_task_workflow/planning_state.hpp"
#include "agx_task_workflow/task_registry.hpp"

using namespace agx_task_workflow;
namespace {
struct InstrumentPlan { std::string sample; };
template<class F> void Rejects(F call) {
  bool rejected = false;
  try { call(); } catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
}
PreparedTask Bind(const TaskSpec& spec) {
  PreparedTask task{spec.id, spec.type, "TASK_RUNNING",
      [](const PhaseSink&) { return StepResult{true, "recorded"}; }};
  task.entry_check = [] { return StepResult{true, "ready"}; };
  task.stop = [](double) { StepResult r{true, "stopped"}; r.stop = StopState::Confirmed; return r; };
  task.cleanup = [](double) { return StepResult{true, "clean"}; };
  return task;
}
void ProspectiveValuesAreDetached() {
  PlanningState before;
  assert(before.Get<InstrumentPlan>("instrument.sample").sample.empty());
  before.Set("instrument.sample", InstrumentPlan{"first"});
  auto after = before;
  after.Set("instrument.sample", InstrumentPlan{"second"});
  auto detached = after.Get<InstrumentPlan>("instrument.sample");
  detached.sample = "third";
  assert(detached.sample == "third");
  assert(before.Get<InstrumentPlan>("instrument.sample").sample == "first");
  assert(after.Get<InstrumentPlan>("instrument.sample").sample == "second");
  Rejects([&] { after.Set("instrument.sample", 1); });
  Rejects([&] { (void)after.Get<int>("instrument.sample"); });
  for (const auto* key : {"", "sample", ".sample", "instrument.", "instrument..sample", "instrument.bad key"}) {
    Rejects([&] { after.Set(key, 1); });
    Rejects([&] { (void)after.Get<int>(key); });
  }
}
void ContinuingFailureDoesNotPromiseAProducedSample() {
  for (bool continuing : {false, true}) {
    TaskRegistry registry;
    TaskDefinition produce;
    produce.type = "produce"; produce.description = "Plan a sample";
    produce.resources = {"instrument.loader"};
    produce.requirements = {{"instrument.ready", RequirementTiming::Entry, "Ready"}};
    produce.effects = {{"instrument.sample_available", EffectCondition::TaskSucceeded, "Sample"}};
    produce.prepare = [](const TaskSpec& spec, PreparationState& state) {
      state.planning.Set("instrument.sample", InstrumentPlan{"sample-1"});
      return Bind(spec);
    };
    RegisterTask(registry, produce);
    TaskDefinition inspect;
    inspect.type = "inspect"; inspect.description = "Inspect prospective state";
    inspect.prepare = [&](const TaskSpec& spec, PreparationState& state) {
      assert(state.planning.Get<InstrumentPlan>("instrument.sample").sample ==
             (continuing ? "" : "sample-1"));
      assert(state.conditional_effects.size() == (continuing ? 1u : 0u));
      if (continuing) assert(state.conditional_effects[0].effect.kind == "instrument.sample_available");
      return Bind(spec);
    };
    RegisterTask(registry, inspect);
    auto config = ParseConfig("schema_version: 2\nworkflow_id: samples\ntasks: [{id: first, type: produce}, {id: second, type: inspect}]\n");
    if (continuing) config.tasks[0].failure_policy = FailurePolicy::Continue;
    assert(PrepareRecipe(config, registry).tasks.size() == 2);
    assert(PrepareRecipe(config, registry).tasks.size() == 2);  // Fresh state on every admission.
  }
}
void DescriptionsDoNotAuthorizeContinuation() {
  TaskRegistry registry;
  int executions = 0;
  TaskDefinition definition;
  definition.type = "inspect"; definition.description = "Observe an instrument";
  definition.resources = {"lab.instrument"};
  definition.requirements = {{"lab.ready", RequirementTiming::Entry, "Ready"}};
  definition.effects = {{"lab.observed", EffectCondition::TaskSucceeded, "Observed"}};
  definition.prepare = [&](const TaskSpec& spec, PreparationState&) {
    auto task = Bind(spec);
    task.execute = [&](const PhaseSink&) {
      ++executions;
      StepResult result{false, "ordinary missed observation"};
      result.failure = FailureKind::Ordinary;
      return result;
    };
    return task;
  };
  RegisterTask(registry, definition);
  const auto recipe = PrepareRecipe(ParseConfig(
      "schema_version: 2\nworkflow_id: instrument\ntasks: [{id: first, type: inspect, on_failure: continue}, {id: second, type: inspect}]\n"), registry);
  Ports ports{[] { return false; }, [] {}, [](const std::string&) {}, [](const TaskResult&) {}};
  const auto result = Run(recipe.tasks, ports);  // No host reconciliation callback.
  assert(!result.success && executions == 1 && !result.tasks[1].dispatched);
  auto invalid = definition;
  invalid.type = "duplicate_resource"; invalid.resources.push_back("lab.instrument");
  Rejects([&] { RegisterTask(registry, invalid); });
}
}
int main() {
  ProspectiveValuesAreDetached();
  ContinuingFailureDoesNotPromiseAProducedSample();
  DescriptionsDoNotAuthorizeContinuation();
}
