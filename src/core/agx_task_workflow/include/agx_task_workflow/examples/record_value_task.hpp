#pragma once

#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#include "agx_task_workflow/task_registry.hpp"
#include "agx_task_workflow/task_runtime_ports.hpp"

namespace agx_task_workflow::examples::record_value {

using RecordCallback =
    std::function<StepResult(const std::string&, const TaskRuntimePorts&)>;

inline TaskDefinition MakeRecordValueTask(TaskRuntimePortsProvider runtime,
                                          RecordCallback record) {
  if (!runtime || !record)
    throw std::invalid_argument("record_value requires runtime and record callbacks");
  TaskDefinition definition;
  definition.type = "record_value";
  definition.contract_version = 1;
  definition.description = "Record a supplied value without owning actuators";
  definition.inputs = {{"value", InputType::String, true, "",
                        "Nonempty value to record"}};
  definition.effects = {{agx_task_workflow::effects::ObservationOnly,
                         EffectCondition::TaskSucceeded, "Value recorded"}};
  definition.prepare = [runtime = std::move(runtime), record = std::move(record)](
      const TaskSpec& spec, PreparationState&) {
    if (spec.config_path)
      throw std::invalid_argument("record_value does not accept config");
    const auto value = RequireString(spec.parameters["value"], "value");
    const auto task_id = spec.id;
    PreparedTask task{spec.id, spec.type, "TASK_RUNNING",
        [runtime, record, task_id, value](const PhaseSink& phase) {
          const auto ports = runtime(task_id);
          const auto before = ports.check ? ports.check() : FailureKind::Fault;
          if (before != FailureKind::None) {
            StepResult result{false, "record_value interrupted before callback"};
            result.failure = before;
            return result;
          }
          phase("RECORD_VALUE");
          auto result = record(value, ports);
          const auto after = ports.check ? ports.check() : FailureKind::Fault;
          if (after != FailureKind::None) {
            result.ok = false;
            result.failure = after;
            if (!result.detail.empty()) result.detail += "; ";
            result.detail += "record_value interrupted during callback";
          }
          return result;
        }};
    task.entry_check = [runtime, task_id] {
      const auto ports = runtime(task_id);
      const auto failure = ports.check ? ports.check() : FailureKind::Fault;
      if (failure == FailureKind::None)
        return StepResult{true, "No actuator prerequisites"};
      StepResult result{false, "record_value runtime is not active"};
      result.failure = failure;
      return result;
    };
    task.stop = [](double) {
      StepResult result{true, "No owned actuator or asynchronous work"};
      result.stop = StopState::Confirmed;
      return result;
    };
    task.cleanup = [](double) {
      return StepResult{true, "Record callback returned before task completion"};
    };
    task.continuation_check = [](const StepResult& result) {
      // This synchronous observation task owns no physical effects or workers.
      return result.failure == FailureKind::Ordinary &&
             result.stop == StopState::Confirmed && result.physical_effect_ids.empty();
    };
    return task;
  };
  return definition;
}

inline void RegisterRecordValueTask(TaskRegistry& registry,
                                    TaskRuntimePortsProvider runtime,
                                    RecordCallback record) {
  RegisterTask(registry,
               MakeRecordValueTask(std::move(runtime), std::move(record)));
}

}  // namespace agx_task_workflow::examples::record_value
