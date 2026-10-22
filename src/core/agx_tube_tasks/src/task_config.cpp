#include "agx_tube_tasks/task_config.hpp"
#include "agx_xarm_pick/tube_grasp_config.hpp"
#include <utility>

namespace agx_tube_tasks {
TubeConfigResolver::TubeConfigResolver(TubeInstallation installation)
    : installed_(std::move(installation)) {}
GraspParameters TubeConfigResolver::Resolve(const agx_task_workflow::TaskSpec& task) {
  agx_task_workflow::RequireKeys(task.parameters,task.config_path?std::set<std::string>{"cap_color"}:
      std::set<std::string>{"fixture_profile","cap_color"},"grasp_test_tube parameters");
  const auto color=agx_task_workflow::RequireString(task.parameters["cap_color"],"cap_color");
  GraspParameters request;
  const auto admitted=installed_.admitted_grasps.find(task.id);
  if(admitted!=installed_.admitted_grasps.end()) {
    request=admitted->second;
    if(!request.config || request.fixture_profile.empty() || request.cap_color!=color)
      throw std::invalid_argument("grasp invocation differs from its admitted configuration");
  } else {
    auto snapshot=installed_.legacy_config;
    if(task.config_path) {
      const auto path=std::filesystem::canonical(*task.config_path).string();
      auto found=snapshots_.find(path);
      if(found==snapshots_.end())found=snapshots_.emplace(path,agx_xarm_pick::ResolveTubeTaskConfig(path)).first;
      snapshot=found->second;
    } else if(!snapshot && installed_.legacy_error) std::rethrow_exception(installed_.legacy_error);
    if(snapshot)request={snapshot->fixture_profile_id,color,snapshot,snapshot->installation.station_id};
    else {
      request={agx_task_workflow::RequireString(task.parameters["fixture_profile"],"fixture_profile"),color};
      if(installed_.fixture_profile.empty() || request.fixture_profile!=installed_.fixture_profile)
        throw std::invalid_argument("fixture_profile is not the loaded supported grasp fixture");
      if(installed_.cap_color.empty() || color!=installed_.cap_color)
        throw std::invalid_argument("cap_color is not the loaded supported grasp color");
    }
  }
  if(!task.config_path && agx_task_workflow::RequireString(task.parameters["fixture_profile"],"fixture_profile")!=request.fixture_profile)
    throw std::invalid_argument("fixture_profile differs from the installed legacy profile");
  if(request.config && !agxcore::ValidateTubeMotionConfig(agx_xarm_pick::BindTubeTaskMotion(*request.config,color)))
    throw std::invalid_argument("grasp motion configuration is invalid");
  return request;
}
}  // namespace agx_tube_tasks
