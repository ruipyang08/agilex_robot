#include "agx_task_workflow/task_registry.hpp"
#include <cmath>
#include <set>
#include <utility>

namespace agx_task_workflow {
namespace {
bool Same(const InterfaceRequirement& a, const InterfaceRequirement& b) {
  return a.kind==b.kind && a.key==b.key && a.endpoint==b.endpoint && a.ros_type==b.ros_type &&
         a.direction==b.direction;
}
std::vector<InterfaceRequirement> BindInterfaces(
    const std::vector<InterfaceRequirement>& declarations,
    const std::vector<InterfaceRequirement>& bindings) {
  if (bindings.empty()) return declarations;
  if (bindings.size()!=declarations.size())
    throw std::invalid_argument("bound interfaces must preserve every declared dependency");
  std::map<std::string,const InterfaceRequirement*> declared;
  for (const auto& dependency:declarations) declared.emplace(dependency.key,&dependency);
  std::set<std::string> seen;
  for (const auto& bound:bindings) {
    const auto found=declared.find(bound.key);
    if (found==declared.end() || !seen.insert(bound.key).second)
      throw std::invalid_argument("bound interface key is undeclared or duplicated: "+bound.key);
    const auto& original=*found->second;
    if (bound.kind!=original.kind || bound.ros_type!=original.ros_type || bound.direction!=original.direction ||
        (!original.endpoint.empty() && original.endpoint!=bound.endpoint))
      throw std::invalid_argument("bound interface changes its declared contract: "+bound.key);
  }
  return bindings;
}
void ValidateDefinition(const TaskDefinition& definition) {
  if (definition.type.empty() || !definition.prepare)
    throw std::invalid_argument("task type and factory are required");
  if (!definition.metadata_complete) return;
  if (!definition.contract_version || definition.description.empty())
    throw std::invalid_argument("complete task metadata needs version and description");
  std::set<std::string> fields;
  for (const auto& field:definition.inputs) {
    if (field.name.empty() || field.name=="id" || field.name=="type" ||
        field.name=="config" || field.name=="on_failure" || !fields.insert(field.name).second)
      throw std::invalid_argument("input schema contains an empty, duplicate or common field");
    switch (field.type) {
      case InputType::String: case InputType::Boolean: case InputType::Integer: case InputType::Number: break;
      default: throw std::invalid_argument("unknown input field type");
    }
  }
  std::set<std::string> interfaces;
  for (const auto& dependency:definition.interfaces) {
    if (dependency.key.empty() || dependency.ros_type.empty() || !interfaces.insert(dependency.key).second)
      throw std::invalid_argument("interface keys/types must be nonempty and keys unique");
    if (dependency.kind!=InterfaceKind::Service && dependency.kind!=InterfaceKind::Action &&
        dependency.kind!=InterfaceKind::Topic) throw std::invalid_argument("unknown interface kind");
    if (dependency.direction!=InterfaceDirection::Observe && dependency.direction!=InterfaceDirection::Command)
      throw std::invalid_argument("unknown interface direction");
    if (dependency.kind!=InterfaceKind::Topic && dependency.direction!=InterfaceDirection::Observe)
      throw std::invalid_argument("only topic dependencies have command direction");
  }
  std::set<TaskResource> resources;
  for (const auto& resource:definition.resources) {
    ValidateContractId(resource);
    if (!resources.insert(resource).second)
      throw std::invalid_argument("duplicate task resource: " + resource);
  }
  for (const auto& requirement:definition.requirements) {
    ValidateContractId(requirement.kind);
    if (requirement.timing!=RequirementTiming::Entry && requirement.timing!=RequirementTiming::Continuous)
      throw std::invalid_argument("unknown state requirement timing");
  }
  for (const auto& effect:definition.effects) {
    ValidateContractId(effect.kind);
    if (effect.condition!=EffectCondition::TaskSucceeded && effect.condition!=EffectCondition::VerifiedPhysicalEvent)
      throw std::invalid_argument("unknown expected effect condition");
  }
}
void ValidateInputs(const YAML::Node& parameters, const TaskDefinition& definition) {
  if (!definition.metadata_complete) return;
  if (!parameters.IsMap()) throw std::invalid_argument("task parameters must be a mapping");
  std::map<std::string,const InputField*> schema;
  for (const auto& field:definition.inputs) schema.emplace(field.name,&field);
  std::set<std::string> seen;
  for (const auto& entry:parameters) {
    const auto key=RequireString(entry.first,"parameter key");
    if (!seen.insert(key).second) throw std::invalid_argument("duplicate parameter: "+key);
    const auto field=schema.find(key);
    if (field==schema.end()) throw std::invalid_argument("unknown parameter: "+key);
    const auto& node=entry.second;
    if (field->second->type==InputType::String) {RequireString(node,key);continue;}
    if (!node.IsScalar() || node.Tag()=="!" || node.Tag()=="tag:yaml.org,2002:str")
      throw std::invalid_argument("parameter has wrong scalar type: "+key);
    const auto tag=node.Tag();
    const bool plain=tag.empty() || tag=="?";
    switch (field->second->type) {
      case InputType::Boolean:
        if ((!plain && tag!="tag:yaml.org,2002:bool") ||
            (node.Scalar()!="true" && node.Scalar()!="false"))
          throw std::invalid_argument("parameter must be true or false: "+key);
        break;
      case InputType::Integer: {
        long long value;
        if ((!plain && tag!="tag:yaml.org,2002:int") || !YAML::convert<long long>::decode(node,value))
          throw std::invalid_argument("parameter must be an integer: "+key);
        break;
      }
      case InputType::Number: {
        double value;
        if ((!plain && tag!="tag:yaml.org,2002:float" && tag!="tag:yaml.org,2002:int") ||
            !YAML::convert<double>::decode(node,value) || !std::isfinite(value))
          throw std::invalid_argument("parameter must be a finite number: "+key);
        break;
      }
      default: throw std::invalid_argument("unsupported parameter type");
    }
  }
  for (const auto& field:definition.inputs)
    if (field.required && !seen.count(field.name)) throw std::invalid_argument("missing parameter: "+field.name);
}
}  // namespace

PreparedTask TaskDefinition::operator()(const TaskSpec& task, PreparationState& state) const {
  return prepare(task,state);
}
void RegisterTask(TaskRegistry& registry, TaskDefinition definition) {
  ValidateDefinition(definition);
  const auto type=definition.type;
  if (!registry.emplace(type,std::move(definition)).second)
    throw std::invalid_argument("duplicate task registration: "+type);
}
TaskDefinition DescribeTask(const TaskRegistry& registry, const std::string& type) {
  const auto entry=registry.find(type);
  if (entry==registry.end()) throw std::invalid_argument("unknown task type: "+type);
  return entry->second;
}
void RegisterTask(TaskRegistry& registry, const std::string& type, TaskFactory factory) {
  TaskDefinition definition;
  definition.type=type;definition.prepare=std::move(factory);
  definition.contract_version=0;definition.metadata_complete=false;
  RegisterTask(registry,std::move(definition));
}
PreparedRecipe PrepareRecipe(const Config& config, const TaskRegistry& registry) {
  if (config.tasks.empty()) throw std::invalid_argument("tasks must be nonempty");
  PreparationState state;
  PreparedRecipe result;
  std::set<std::string> ids;
  for (const auto& task : config.tasks) {
    try {
      if (task.id.empty() || !ids.insert(task.id).second)
        throw std::invalid_argument("task IDs must be nonempty and unique");
      const auto found=registry.find(task.type);
      if (found==registry.end()) throw std::invalid_argument("unknown task type");
      const auto& definition=found->second;
      ValidateDefinition(definition);
      ValidateInputs(task.parameters,definition);
      if (!definition.metadata_complete && task.failure_policy==FailurePolicy::Continue)
        throw std::invalid_argument("legacy task metadata cannot authorize continuation");
      auto after_success=state;
      auto prepared=definition.prepare(task,after_success);
      if (prepared.id!=task.id || prepared.type!=task.type || !prepared.execute)
        throw std::invalid_argument("factory returned invalid task binding");
      if (prepared.entry_phase!="NAVIGATE" && prepared.entry_phase!="GRASP" && prepared.entry_phase!="TASK_RUNNING")
        throw std::invalid_argument("task entry phase must be NAVIGATE, GRASP or TASK_RUNNING");
      if (definition.metadata_complete && (!prepared.entry_check || !prepared.stop || !prepared.cleanup ||
          !std::isfinite(prepared.stop_timeout_s) || prepared.stop_timeout_s<=0.0))
        throw std::invalid_argument("full task binding requires entry, stop, cleanup and finite stop bound");
      prepared.failure_policy=task.failure_policy;
      prepared.metadata_complete=definition.metadata_complete;
      prepared.contract_version=definition.contract_version;
      prepared.required_interfaces=BindInterfaces(definition.interfaces,prepared.required_interfaces);
      prepared.resources=definition.resources;
      prepared.requirements=definition.requirements;
      prepared.expected_effects.clear();
      for (const auto& effect:definition.effects)
        prepared.expected_effects.push_back({task.id,effect,task.failure_policy==FailurePolicy::Continue ||
                                            effect.condition==EffectCondition::VerifiedPhysicalEvent});
      if (task.failure_policy==FailurePolicy::Continue) {
        state.conditional_effects.insert(state.conditional_effects.end(),
                                        prepared.expected_effects.begin(),prepared.expected_effects.end());
      } else state=std::move(after_success);
      result.tasks.push_back(std::move(prepared));
    } catch (const std::exception& e) { throw PreparationError(task,e.what()); }
    catch (...) { throw PreparationError(task,"unknown preparation exception"); }
  }
  // Union only fully prepared task dependencies; the host probes them before dispatch.
  std::map<std::string,InterfaceRequirement> interfaces;
  std::set<TaskResource> resources;
  for (std::size_t i=0;i<result.tasks.size();++i) {
    const auto& task=result.tasks[i];
    for (const auto& dependency:task.required_interfaces) {
      const auto inserted=interfaces.emplace(dependency.key,dependency);
      if (!inserted.second && !Same(inserted.first->second,dependency))
        throw PreparationError(config.tasks[i],"conflicting interface declaration: "+dependency.key);
    }
    resources.insert(task.resources.begin(),task.resources.end());
  }
  for (const auto& entry:interfaces) result.required_interfaces.push_back(entry.second);
  result.resources.assign(resources.begin(),resources.end());
  return result;
}
std::vector<PreparedTask> PrepareTasks(const Config& config, const TaskRegistry& registry) {
  return PrepareRecipe(config,registry).tasks;
}
}
