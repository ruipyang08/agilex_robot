#pragma once
#include <exception>
#include <memory>
#include <map>
#include "agx_task_workflow/config.hpp"
#include "agx_xarm_pick/resolved_tube_config.hpp"

namespace agx_tube_tasks {
struct GraspParameters {
  std::string fixture_profile, cap_color;
  std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig> config;
  std::string station_id;
};
struct TubeInstallation {
  // Legacy embedded adapters may supply their supported inline selection.
  std::string fixture_profile, cap_color;
  std::map<std::string,GraspParameters> admitted_grasps;
  std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig> legacy_config;
  std::exception_ptr legacy_error;
};
class TubeConfigResolver {
 public:
  explicit TubeConfigResolver(TubeInstallation installation);
  GraspParameters Resolve(const agx_task_workflow::TaskSpec&);
 private:
  TubeInstallation installed_;
  std::map<std::string,std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig>> snapshots_;
};
}  // namespace agx_tube_tasks
