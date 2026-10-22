#include "agx_xarm_pick/resolved_tube_config.hpp"
#include <stdexcept>
#include <cmath>

namespace agx_xarm_pick {
TubeTaskMotionConfig BindTubeTaskMotion(const ResolvedTubeTaskConfig& frozen,
                                       const std::string& color) {
  if (frozen.config_id.empty() || frozen.fixture_instance_id.empty() ||
      frozen.fixture_profile_id.empty())
    throw std::invalid_argument("grasp requires an identified configuration snapshot");
  const auto classifier = frozen.behavior.color_classifiers.find(color);
  if (color.empty() || classifier == frozen.behavior.color_classifiers.end())
    throw std::invalid_argument("grasp color is not supported by this behavior profile");
  const auto& config = frozen.motion;
  if (config.tag_registration_required || frozen.installation.registration_required)
    throw std::invalid_argument("required fixture tag registration is not implemented");
  const auto& orientation=frozen.installation.measured_transform.rotation;
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      std::abs(orientation.x)>1e-6 || std::abs(orientation.y)>1e-6)
    throw std::invalid_argument("grasp supports horizontal rack installations only");
  if (config.task_type != "test_tube_pickup" || config.planning_frame != "link_base" ||
      config.color_frame != "cam1_color_optical_frame" ||
      (config.context_source != "configured_demo" && config.context_source != "resolved_installation"))
    throw std::invalid_argument("grasp configuration requests an unsupported motion adapter");
  auto bound = config;
  bound.target_cap_color = color;
  bound.profile_target_cap_color = color;
  bound.target_hsv_lower = classifier->second.hsv_lower;
  bound.target_hsv_upper = classifier->second.hsv_upper;
  return bound;
}
}
