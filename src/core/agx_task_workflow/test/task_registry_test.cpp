#include <cassert>
#include <stdexcept>
#include "agx_task_workflow/task_registry.hpp"
using namespace agx_task_workflow;
namespace {
PreparedTask InertBinding(const TaskSpec& task, int& calls) {
  PreparedTask bound{task.id,task.type,"TASK_RUNNING",[&](const PhaseSink&){++calls;return StepResult{true,"observed"};}};
  bound.entry_check=[] { return StepResult{true,"no actuator prerequisites"}; };
  bound.stop=[](double timeout) { assert(timeout>0);StepResult r{true,"no owned motion"};r.stop=StopState::Confirmed;return r; };
  bound.cleanup=[](double timeout) { assert(timeout>0);return StepResult{true,"no pending work"}; };
  return bound;
}

void DescriptionsAndAdmissionDoNotDispatch() {
  int calls=0,preparations=0;
  TaskRegistry registry;
  TaskDefinition definition;
  definition.type="inspect";definition.contract_version=1;definition.description="Inspect an inert value";
  definition.inputs={{"value",InputType::String,true,"","Value to inspect"}};
  definition.interfaces={{InterfaceKind::Topic,"camera_state","/camera/state","std_msgs/String"}};
  definition.resources={"instrument.camera"};
  definition.requirements={{agx_task_workflow::requirements::CommonSupervision,RequirementTiming::Continuous,"workflow supervision"}};
  definition.effects={{agx_task_workflow::effects::ObservationOnly,EffectCondition::TaskSucceeded,"inspection recorded"}};
  definition.prepare=[&](const TaskSpec& task,PreparationState&) {++preparations;return InertBinding(task,calls);};
  RegisterTask(registry,definition);
  auto description=DescribeTask(registry,"inspect");
  assert(description.metadata_complete && description.contract_version==1 && preparations==0 && calls==0);
  description.description="modified detached copy";
  assert(DescribeTask(registry,"inspect").description=="Inspect an inert value");
  const auto recipe=ParseConfig("schema_version: 2\nworkflow_id: inspect\ntasks: [{id: first,type: inspect,value: one},{id: second,type: inspect,value: two}]\n");
  const auto prepared=PrepareRecipe(recipe,registry);
  assert(preparations==2 && calls==0 && prepared.tasks.size()==2);
  assert(prepared.required_interfaces.size()==1 && prepared.required_interfaces[0].key=="camera_state");
  assert(prepared.resources==std::vector<TaskResource>{"instrument.camera"});
  assert(prepared.tasks[0].metadata_complete && prepared.tasks[0].entry_check && prepared.tasks[0].stop && prepared.tasks[0].cleanup);
  Ports ports{[]{return false;},[]{},[](const std::string&){},[](const TaskResult&){}};
  assert(Run(prepared.tasks,ports).success && calls==2);
}

void ExactSchemaAndDependencyConflictsRejectBeforeDispatch() {
  int calls=0;
  TaskRegistry registry;
  TaskDefinition definition;
  definition.type="typed";definition.description="Typed input contract";
  definition.inputs={{"label",InputType::String},{"enabled",InputType::Boolean},
                     {"count",InputType::Integer},{"distance",InputType::Number,true,"m"},
                     {"optional",InputType::String,false}};
  definition.interfaces={{InterfaceKind::Topic,"shared","/camera/state","std_msgs/String"}};
  definition.prepare=[&](const TaskSpec& task,PreparationState&) {return InertBinding(task,calls);};
  RegisterTask(registry,definition);
  auto config=ParseConfig("schema_version: 2\nworkflow_id: typed\ntasks: [{id: one,type: typed,label: sample,enabled: true,count: 2,distance: 0.3}]\n");
  assert(PrepareRecipe(config,registry).tasks.size()==1);
  for(const auto& yaml:{"{label: a,enabled: true,count: 2,distance: .nan}",
                        "{label: a,enabled: 'true',count: 2,distance: 0.3}",
                        "{label: a,enabled: true,count: 2.5,distance: 0.3}",
                        "{label: a,enabled: true,count: 2,distance: 0.3,extra: 1}",
                        "{label: a,label: b,enabled: true,count: 2,distance: 0.3}",
                        "{label: 3,enabled: true,count: 2,distance: 0.3}",
                        "{label: a,enabled: true,count: 2}"}) {
    auto invalid=config;invalid.tasks[0].parameters=YAML::Load(yaml);
    try {PrepareRecipe(invalid,registry);assert(false);}catch(const PreparationError&){}
  }
  definition.type="conflict";
  definition.interfaces[0].endpoint="/other/state";
  RegisterTask(registry,definition);
  config.tasks.push_back(config.tasks[0]);config.tasks.back().id="two";config.tasks.back().type="conflict";
  try {PrepareRecipe(config,registry);assert(false);}catch(const PreparationError&){}
  assert(calls==0);
}

void ContinueKeepsSuccessEffectsConditional() {
  int calls=0;
  TaskRegistry registry;
  TaskDefinition arrival;
  arrival.type="arrival";arrival.description="Planned held arrival";
  arrival.effects={{"instrument.held_arrival",EffectCondition::TaskSucceeded,"station held"}};
  arrival.prepare=[&](const TaskSpec& task,PreparationState& state) {
    state.planning.Set("instrument.station", std::string("station-a"));return InertBinding(task,calls);
  };
  RegisterTask(registry,arrival);
  TaskDefinition inspect;
  inspect.type="inspect";inspect.description="Inspect planned alternatives";
  inspect.prepare=[&](const TaskSpec& task,PreparationState& state) {
    assert(state.planning.Get<std::string>("instrument.station").empty() && state.conditional_effects.size()==1);
    assert(state.conditional_effects[0].task_id=="first");
    return InertBinding(task,calls);
  };
  RegisterTask(registry,inspect);
  const auto recipe=PrepareRecipe(ParseConfig("schema_version: 2\nworkflow_id: condition\ntasks: [{id: first,type: arrival,on_failure: continue},{id: inspect,type: inspect}]\n"),registry);
  assert(recipe.tasks[0].failure_policy==FailurePolicy::Continue);
  assert(recipe.tasks[0].expected_effects[0].conditional);
  assert(calls==0);
}

void LegacyMetadataAndLifecycleRemainConservative() {
  int calls=0;
  TaskRegistry registry;
  RegisterTask(registry,"legacy",[&](const TaskSpec& task,PreparationState&) {return PreparedTask{task.id,task.type,"TASK_RUNNING",[&](const PhaseSink&){++calls;return StepResult{false,"terminal"};}};});
  assert(!DescribeTask(registry,"legacy").metadata_complete);
  const auto prepared=PrepareRecipe(ParseConfig("schema_version: 2\nworkflow_id: legacy\ntasks: [{id: one,type: legacy}]\n"),registry);
  assert(!prepared.tasks[0].metadata_complete && !prepared.tasks[0].stop);
  assert(prepared.tasks[0].execute({}).failure==FailureKind::Fault);
  TaskDefinition invalid;
  invalid.type="invalid";invalid.description="Missing lifecycle binding";
  invalid.prepare=[](const TaskSpec& task,PreparationState&) {return PreparedTask{task.id,task.type,"TASK_RUNNING",[](const PhaseSink&){return StepResult{true,"done"};}};};
  RegisterTask(registry,invalid);
  try {PrepareRecipe(ParseConfig("schema_version: 2\nworkflow_id: invalid\ntasks: [{id: one,type: invalid}]\n"),registry);assert(false);}catch(const PreparationError&){}
}

void UnknownContractVocabularyIsRejected() {
  int calls=0;
  const std::vector<std::function<void(TaskDefinition&)>> mutations{
    [](TaskDefinition& d){d.interfaces={{InterfaceKind::Topic,"topic","/topic","std_msgs/String",static_cast<InterfaceDirection>(99)}};},
    [](TaskDefinition& d){d.interfaces={{InterfaceKind::Service,"service","/service","std_srvs/Trigger",InterfaceDirection::Command}};},
    [](TaskDefinition& d){d.resources={"invalid_resource"};},
    [](TaskDefinition& d){d.requirements={{"invalid_requirement",RequirementTiming::Entry,"unknown"}};},
    [](TaskDefinition& d){d.requirements={{"instrument.held_arrival",static_cast<RequirementTiming>(99),"unknown"}};},
    [](TaskDefinition& d){d.effects={{"invalid_effect",EffectCondition::TaskSucceeded,"unknown"}};},
    [](TaskDefinition& d){d.effects={{"instrument.held_arrival",static_cast<EffectCondition>(99),"unknown"}};}
  };
  for(const auto& mutate:mutations) {
    TaskRegistry registry;
    TaskDefinition definition;
    definition.type="invalid";definition.description="Invalid contract vocabulary";
    definition.prepare=[&](const TaskSpec& spec,PreparationState&) {return InertBinding(spec,calls);};
    mutate(definition);
    bool rejected=false;
    try {RegisterTask(registry,definition);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected && registry.empty() && calls==0);
  }
}

void TopicDirectionsParticipateInDependencyAdmission() {
  int calls=0;
  TaskRegistry registry;
  TaskDefinition observe;
  observe.type="observe";observe.description="Observe a topic publisher";
  observe.interfaces={{InterfaceKind::Topic,"shared","/shared","std_msgs/String"}};
  observe.prepare=[&](const TaskSpec& spec,PreparationState&) {return InertBinding(spec,calls);};
  RegisterTask(registry,observe);
  auto command=observe;
  command.type="command";command.interfaces[0].direction=InterfaceDirection::Command;
  RegisterTask(registry,command);
  const auto command_only=PrepareRecipe(ParseConfig("schema_version: 2\nworkflow_id: command\ntasks: [{id: one,type: command}]\n"),registry);
  assert(command_only.required_interfaces[0].direction==InterfaceDirection::Command);
  bool rejected=false;
  try {PrepareRecipe(ParseConfig("schema_version: 2\nworkflow_id: mixed\ntasks: [{id: one,type: observe},{id: two,type: command}]\n"),registry);}
  catch(const PreparationError&){rejected=true;}
  assert(rejected && calls==0);
}

void BoundInterfacesMayResolveOnlyDeclaredDynamicEndpoints() {
  int calls=0;
  TaskRegistry registry;
  TaskDefinition definition;
  definition.type="bound";definition.description="Bind immutable configured endpoints";
  definition.interfaces={{InterfaceKind::Service,"sensor","","std_srvs/Trigger"},
                          {InterfaceKind::Topic,"status","/fixed/status","std_msgs/String"}};
  auto binding=definition.interfaces;
  binding[0].endpoint="/installed/sensor";
  const auto valid=binding;
  definition.prepare=[&](const TaskSpec& spec,PreparationState&) {
    auto task=InertBinding(spec,calls);task.required_interfaces=binding;return task;
  };
  RegisterTask(registry,definition);
  const auto config=ParseConfig("schema_version: 2\nworkflow_id: bound\ntasks: [{id: one,type: bound}]\n");
  const auto admitted=PrepareRecipe(config,registry);
  assert(admitted.required_interfaces[0].endpoint=="/installed/sensor");
  assert(admitted.tasks[0].required_interfaces[0].endpoint=="/installed/sensor");
  assert(DescribeTask(registry,"bound").interfaces[0].endpoint.empty());
  const std::vector<std::function<void(std::vector<InterfaceRequirement>&)>> invalid_bindings{
    [](auto& v){v.pop_back();},
    [](auto& v){v.push_back({InterfaceKind::Topic,"extra","/extra","std_msgs/String"});},
    [](auto& v){v[1]=v[0];},
    [](auto& v){v[0].kind=InterfaceKind::Action;},
    [](auto& v){v[0].ros_type="wrong/Type";},
    [](auto& v){v[1].direction=InterfaceDirection::Command;},
    [](auto& v){v[1].endpoint="/different/status";}
  };
  for(const auto& mutate:invalid_bindings) {
    binding=valid;mutate(binding);bool rejected=false;
    try {PrepareRecipe(config,registry);}catch(const PreparationError&){rejected=true;}
    assert(rejected && calls==0);
  }
  binding.clear();
  assert(PrepareRecipe(config,registry).required_interfaces[0].endpoint.empty());
}
}
int main() {
  DescriptionsAndAdmissionDoNotDispatch();
  ExactSchemaAndDependencyConflictsRejectBeforeDispatch();
  ContinueKeepsSuccessEffectsConditional();
  LegacyMetadataAndLifecycleRemainConservative();
  UnknownContractVocabularyIsRejected();
  TopicDirectionsParticipateInDependencyAdmission();
  BoundInterfacesMayResolveOnlyDeclaredDynamicEndpoints();
  int calls=0;
  TaskRegistry registry;
  RegisterTask(registry,"inert",[&](const TaskSpec& t, PreparationState& s) {
    assert(!s.planning.Get<bool>("example.prepared")); s.planning.Set("example.prepared",true);
    return PreparedTask{t.id,t.type,"TASK_RUNNING",[&](const PhaseSink&){++calls;return StepResult{true,"done"};}};
  });
  auto c=ParseConfig("schema_version: 2\nworkflow_id: demo\ntasks: [{id: one, type: inert}]\n");
  auto prepared=PrepareTasks(c,registry); assert(calls==0 && prepared.size()==1);
  PrepareTasks(c,registry); // Preparation must start with fresh state each time.
  bool duplicate=false;
  try { RegisterTask(registry,"inert",registry.at("inert")); } catch(const std::invalid_argument&) { duplicate=true; }
  assert(duplicate);
  c.tasks.push_back(TaskSpec{"bad","missing",YAML::Node(YAML::NodeType::Map)});
  try { PrepareTasks(c,registry); assert(false); }
  catch(const PreparationError& e) { assert(e.task_id=="bad" && e.task_type=="missing"); }
  assert(calls==0);
  RegisterTask(registry,"throws",[](const TaskSpec&,PreparationState&)->PreparedTask {throw std::runtime_error("native error");});
  c.tasks.back().type="throws";
  try { PrepareTasks(c,registry); assert(false); }
  catch(const PreparationError& e) { assert(e.task_id=="bad" && std::string(e.what()).find("native error")!=std::string::npos); }
  RegisterTask(registry,"empty",[](const TaskSpec& t,PreparationState&){return PreparedTask{t.id,t.type,"TASK_RUNNING",{}};});
  c.tasks.back().type="empty";
  try { PrepareTasks(c,registry); assert(false); } catch(const PreparationError&) {}
}
