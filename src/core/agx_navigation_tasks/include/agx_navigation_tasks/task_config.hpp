#pragma once
#include <filesystem>
#include <memory>
#include <map>
#include <optional>
#include <set>
#include "agx_task_workflow/config.hpp"
#include "agx_navigation_tasks/travel_profile.hpp"

namespace agx_navigation_tasks {
struct NavigateParameters {
  std::string target;
  std::shared_ptr<const TravelProfile> travel_profile;
};
// Snapshot installation parameters before preparation. Resolving a task reads
// files only: it never waits for ROS, observes live state, or issues commands.
struct NavigationInstallation {
  std::set<std::string> workbenches;
  std::map<std::string,std::shared_ptr<const TravelProfile>> admitted_navigation;
  std::filesystem::path registry_path, default_travel_profile;
  std::optional<TravelMode> execution_mode;
};
class NavigationConfigResolver {
 public:
  explicit NavigationConfigResolver(NavigationInstallation installation);
  NavigateParameters Resolve(const agx_task_workflow::TaskSpec&);
 private:
  NavigationInstallation installed_;
  bool registry_loaded_=false;
  std::map<std::string,std::shared_ptr<const TravelProfile>> snapshots_;
};
}  // namespace agx_navigation_tasks
