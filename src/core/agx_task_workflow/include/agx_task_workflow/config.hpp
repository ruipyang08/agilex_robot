#pragma once
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "agx_task_workflow/runner.hpp"

namespace agx_task_workflow {
struct TaskSpec {
  std::string id, type;
  YAML::Node parameters;
  std::optional<std::filesystem::path> config_path;
  FailurePolicy failure_policy = FailurePolicy::Stop;
};
struct Config {
  int schema_version = 2;
  std::string workflow_id;
  std::vector<TaskSpec> tasks;
  std::optional<std::filesystem::path> source_path;
};
// Shared strict YAML boundary helpers used by registered task validators.
std::string RequireString(const YAML::Node&, const std::string& label);
void RequireKeys(const YAML::Node&, const std::set<std::string>&, const std::string& label);
Config ParseConfig(const std::string& yaml, const std::filesystem::path& source_path = {});
Config LoadConfig(const std::string& path);
}
