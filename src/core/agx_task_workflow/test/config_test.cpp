#include <cassert>
#include <stdexcept>
#include <string>
#include "agx_task_workflow/config.hpp"
using namespace agx_task_workflow;
const std::string valid = R"(schema_version: 2
workflow_id: demo
tasks:
  - id: approach
    type: navigate
    target: wb-100
  - id: pick
    type: grasp_test_tube
    fixture_profile: table_grid_3x3_v1
    cap_color: green
)";
int main() {
  const auto parsed=ParseConfig(valid);
  assert(parsed.tasks.size()==2 && parsed.tasks[0].id=="approach");
  assert(parsed.tasks[0].type=="navigate" && parsed.tasks[1].type=="grasp_test_tube");
  assert(parsed.tasks[0].parameters["target"].as<std::string>()=="wb-100");
  assert(parsed.tasks[0].failure_policy==FailurePolicy::Stop);
  assert(!parsed.tasks[0].config_path);
  auto fails = [](const std::string& text) {
    try { ParseConfig(text); } catch (const std::exception&) { return; }
    assert(false && "invalid recipe was accepted");
  };
  auto replace = [](std::string input, const std::string& from, const std::string& to) {
    input.replace(input.find(from), from.size(), to); return input;
  };
  for (const auto& value : {"[]", "{}", "null", "true", "1.5", "''", "0x10"}) {
    fails(replace(valid,"workflow_id: demo",std::string("workflow_id: ")+value));
    fails(replace(valid,"id: approach",std::string("id: ")+value));
    fails(replace(valid,"type: navigate",std::string("type: ")+value));
  }
  for (const auto& value : {"1", "3", "'2'", "true", "null"})
    fails(replace(valid,"schema_version: 2",std::string("schema_version: ")+value));
  fails(valid+"workflow_id: duplicate\n");
  fails(valid+"unexpected: value\n");
  fails(valid+"---\n"+valid);
  fails(replace(valid,"id: pick","id: approach"));
  fails(replace(valid,"type: navigate","type: navigate\n    type: other"));
  fails(replace(valid,"target: wb-100","target: wb-100\n    target: wb-101"));
  fails(replace(valid,"id: approach","missing: approach"));
  fails(replace(valid,"type: navigate","missing: navigate"));
  for (const auto& value : {"[]", "{}", "null", "text", "[null]", "[[]]", "[1]"})
    fails(std::string("schema_version: 2\nworkflow_id: demo\ntasks: ")+value+"\n");
  for (const auto& key : {"schema_version: 2\n", "workflow_id: demo\n"})
    fails(replace(valid,key,""));
  ParseConfig(replace(valid,"workflow_id: demo","workflow_id: '123'"));
  ParseConfig(replace(valid,"target: wb-100","target: 12")); // Factory owns argument types.
  const std::string modular = R"(schema_version: 2
workflow_id: modular
tasks:
  - id: pick
    type: grasp_test_tube
    config: ../task_configs/rack_a.yaml
    cap_color: green
    on_failure: continue
)";
  fails(modular); // Relative paths cannot accidentally depend on process CWD.
  auto resolved=ParseConfig(modular,"/opt/lab/workflows/collect.yaml");
  assert(resolved.tasks[0].config_path->string()=="/opt/lab/task_configs/rack_a.yaml");
  assert(resolved.source_path->string()=="/opt/lab/workflows/collect.yaml");
  assert(resolved.tasks[0].failure_policy==FailurePolicy::Continue);
  assert(!resolved.tasks[0].parameters["config"] && !resolved.tasks[0].parameters["on_failure"]);
  assert(resolved.tasks[0].parameters["cap_color"].as<std::string>()=="green");
  for (const auto& policy:{"retry","true","3","[]","''"})
    fails(replace(replace(modular,"../task_configs/rack_a.yaml","/opt/lab/rack_a.yaml"),
                  "on_failure: continue",std::string("on_failure: ")+policy));
  const auto absolute=ParseConfig(replace(modular,"../task_configs/rack_a.yaml","/opt/lab/rack_a.yaml"));
  assert(absolute.tasks[0].config_path->string()=="/opt/lab/rack_a.yaml");
  fails(replace(modular,"config: ../task_configs/rack_a.yaml","config: false"));
  fails(replace(modular,"on_failure: continue","on_failure: stop\n    on_failure: continue"));
}
