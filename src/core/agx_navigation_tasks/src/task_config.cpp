#include "agx_navigation_tasks/task_config.hpp"
#include <fstream>
#include <sstream>
#include <utility>

namespace agx_navigation_tasks {
NavigationConfigResolver::NavigationConfigResolver(NavigationInstallation installation)
    : installed_(std::move(installation)) {}
NavigateParameters NavigationConfigResolver::Resolve(const agx_task_workflow::TaskSpec& task) {
  agx_task_workflow::RequireKeys(task.parameters,{"target"},"navigate parameters");
  NavigateParameters request{agx_task_workflow::RequireString(task.parameters["target"],"target")};
  if (!registry_loaded_) {
    if (!installed_.registry_path.empty())
      for (const auto& station:YAML::LoadFile(installed_.registry_path.string())["workbenches"])
        installed_.workbenches.insert(agx_task_workflow::RequireString(station["id"],"registered target"));
    registry_loaded_=true;
  }
  if (!installed_.workbenches.count(request.target))
    throw std::invalid_argument("unknown registered target: "+request.target);
  const auto admitted=installed_.admitted_navigation.find(task.id);
  if(admitted!=installed_.admitted_navigation.end())request.travel_profile=admitted->second;
  else {
    const auto selected=task.config_path.value_or(installed_.default_travel_profile);
    if(!selected.empty()) {
      const auto path=std::filesystem::canonical(selected).string();
      auto found=snapshots_.find(path);
      if(found==snapshots_.end()) {
        std::ifstream input(path);std::stringstream contents;contents<<input.rdbuf();
        if(!input)throw std::invalid_argument("cannot read travel profile: "+path);
        found=snapshots_.emplace(path,std::make_shared<const TravelProfile>(ParseTravelProfile(contents.str(),path))).first;
      }
      request.travel_profile=found->second;
    }
  }
  if(request.travel_profile && installed_.execution_mode) {
    const auto mode=*installed_.execution_mode;
    if(!TravelEnabled(*request.travel_profile,mode))
      throw std::invalid_argument("travel profile "+request.travel_profile->source_path+" is not qualified for "+
          (mode==TravelMode::Simulation?"simulation":"hardware"));
    // A label cannot qualify the hardware JTC's initial desired-state bridge.
    if(mode==TravelMode::Hardware)
      throw std::invalid_argument("hardware travel initial trajectory bridge is not yet validated");
  }
  return request;
}
}  // namespace agx_navigation_tasks
