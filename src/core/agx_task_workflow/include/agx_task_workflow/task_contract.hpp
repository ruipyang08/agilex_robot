#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include "agx_task_workflow/config.hpp"

namespace agx_task_workflow {
struct PreparedTask;
struct PreparationState;
using TaskFactory = std::function<PreparedTask(const TaskSpec&, PreparationState&)>;

enum class InputType { String, Boolean, Integer, Number };
struct InputField {
  std::string name;
  InputType type = InputType::String;
  bool required = true;
  std::string unit, description;
};
enum class InterfaceKind { Service, Action, Topic };
enum class InterfaceDirection { Observe, Command };
struct InterfaceRequirement {
  InterfaceKind kind = InterfaceKind::Topic;
  std::string key, endpoint, ros_type;
  InterfaceDirection direction = InterfaceDirection::Observe;
};
// Modules own namespaced descriptive IDs. Executable guards remain explicit.
using TaskResource = std::string;
enum class RequirementTiming { Entry, Continuous };
using StateRequirementKind = std::string;
namespace requirements {
inline const StateRequirementKind CommonSupervision = "workflow.supervision";
}
struct StateRequirement {
  StateRequirementKind kind = agx_task_workflow::requirements::CommonSupervision;
  RequirementTiming timing = RequirementTiming::Entry;
  std::string description;
};
using ExpectedEffectKind = std::string;
namespace effects {
inline const ExpectedEffectKind ObservationOnly = "workflow.observation";
}
enum class EffectCondition { TaskSucceeded, VerifiedPhysicalEvent };
struct ExpectedEffect {
  ExpectedEffectKind kind = agx_task_workflow::effects::ObservationOnly;
  EffectCondition condition = EffectCondition::TaskSucceeded;
  std::string description;
};
struct PlannedEffect {
  std::string task_id;
  ExpectedEffect effect;
  bool conditional = false;
};
struct TaskDefinition {
  std::string type;
  std::uint32_t contract_version = 1;
  std::string description;
  std::vector<InputField> inputs;
  TaskFactory prepare;
  std::vector<InterfaceRequirement> interfaces;
  std::vector<TaskResource> resources;
  std::vector<StateRequirement> requirements;
  std::vector<ExpectedEffect> effects;
  bool metadata_complete = true;

  // Keeps existing registry.at(type)(spec, state) and factory-copy callers valid.
  PreparedTask operator()(const TaskSpec&, PreparationState&) const;
};
}  // namespace agx_task_workflow
