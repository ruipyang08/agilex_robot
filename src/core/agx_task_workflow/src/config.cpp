#include "agx_task_workflow/config.hpp"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace agx_task_workflow {
std::string RequireString(const YAML::Node& node, const std::string& key) {
  if (!node.IsScalar()) throw std::invalid_argument(std::string(key)+" must be a string");
  if (!node.Tag().empty() && node.Tag() != "?" && node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str")
    throw std::invalid_argument(std::string(key)+" must be a string");
  const auto text=node.Scalar();
  if (text.empty() || text.find_first_not_of(" \t\r\n") == std::string::npos)
    throw std::invalid_argument(std::string(key)+" must be nonempty");
  // yaml-cpp's as<string>() also accepts numbers and booleans. Preserve
  // explicitly quoted strings, but reject other plain scalar types.
  if (node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str") {
    std::string lower=text;
    std::transform(lower.begin(),lower.end(),lower.begin(),[](unsigned char c){return std::tolower(c);});
    if (lower=="true" || lower=="false" || lower=="yes" || lower=="no" ||
        lower=="on" || lower=="off" || lower=="null" || lower=="~")
      throw std::invalid_argument(std::string(key)+" must be a string");
    double numeric; long long integer;
    if (YAML::convert<double>::decode(node,numeric) || YAML::convert<long long>::decode(node,integer))
      throw std::invalid_argument(std::string(key)+" must be a string, not a number");
  }
  return text;
}
void RequireKeys(const YAML::Node& node, const std::set<std::string>& fields,
                 const std::string& label) {
  if (!node.IsMap()) throw std::invalid_argument(label+" must be a mapping");
  std::set<std::string> seen;
  for (const auto& entry : node) {
    const auto key = RequireString(entry.first, label+" key");
    if (!seen.insert(key).second) throw std::invalid_argument(label+": duplicate field "+key);
    if (!fields.count(key)) throw std::invalid_argument(label+": unknown field "+key);
  }
  for (const auto& key : fields)
    if (!seen.count(key)) throw std::invalid_argument(label+": missing field "+key);
}
Config ParseConfig(const std::string& text, const std::filesystem::path& source_path) {
  const auto documents=YAML::LoadAll(text);
  if (documents.size() != 1) throw std::invalid_argument("exactly one workflow YAML document is required");
  const auto root=documents.front();
  if (!root.IsMap()) throw std::invalid_argument("workflow must be a YAML mapping");
  const auto version=root["schema_version"];
  if (!version.IsScalar() || version.Scalar() != "2" ||
      (version.Tag() != "?" && version.Tag() != "tag:yaml.org,2002:int"))
    throw std::invalid_argument("use schema_version: 2 with an ordered tasks list; migrate the old flat navigate/grasp recipe");
  RequireKeys(root, {"schema_version", "workflow_id", "tasks"}, "workflow");
  Config result;
  if (!source_path.empty()) result.source_path=std::filesystem::absolute(source_path).lexically_normal();
  result.workflow_id=RequireString(root["workflow_id"],"workflow_id");
  const auto tasks=root["tasks"];
  if (!tasks.IsSequence() || tasks.size()==0)
    throw std::invalid_argument("tasks must be a nonempty list");
  std::set<std::string> ids;
  for (std::size_t i=0; i<tasks.size(); ++i) {
    const auto node=tasks[i];
    const auto label="tasks["+std::to_string(i)+"]";
    if (!node.IsMap()) throw std::invalid_argument(label+" must be a mapping");
    std::set<std::string> keys;
    for (const auto& entry:node) {
      auto key=RequireString(entry.first,label+" key");
      if (!keys.insert(key).second) throw std::invalid_argument(label+": duplicate field "+key);
    }
    TaskSpec task;
    task.id=RequireString(node["id"],label+".id");
    task.type=RequireString(node["type"],label+".type");
    if (node["on_failure"]) {
      const auto policy=RequireString(node["on_failure"],label+".on_failure");
      if (policy=="continue") task.failure_policy=FailurePolicy::Continue;
      else if (policy!="stop") throw std::invalid_argument(label+".on_failure must be stop or continue");
    }
    if (node["config"]) {
      auto path=std::filesystem::path(RequireString(node["config"],label+".config"));
      if (path.is_relative()) {
        if (!result.source_path) throw std::invalid_argument(label+".config requires an explicit recipe source path");
        path=result.source_path->parent_path()/path;
      }
      task.config_path=path.lexically_normal();
    }
    if (!ids.insert(task.id).second) throw std::invalid_argument("duplicate task id: "+task.id);
    task.parameters=YAML::Node(YAML::NodeType::Map);
    for (const auto& entry:node) {
      auto key=entry.first.Scalar();
      if (key!="id" && key!="type" && key!="config" && key!="on_failure")
        task.parameters[key]=YAML::Clone(entry.second);
    }
    result.tasks.push_back(std::move(task));
  }
  return result;
}
Config LoadConfig(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read workflow YAML: "+path);
  return ParseConfig(std::string(std::istreambuf_iterator<char>(stream),{}),path);
}
}
