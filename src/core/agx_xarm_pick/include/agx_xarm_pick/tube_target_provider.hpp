#ifndef AGX_XARM_PICK_TUBE_TARGET_PROVIDER_HPP
#define AGX_XARM_PICK_TUBE_TARGET_PROVIDER_HPP

#include <string>

#include <agx_pick_msg/SelectTubeTarget2D.h>
#include <agx_pick_msg/SelectTubeTarget2DConfigured.h>
#include <ros/ros.h>

#include "agx_xarm_pick/visual_alignment.hpp"
#include "agx_xarm_pick/resolved_tube_config.hpp"

namespace agx_xarm_pick {

class RosTubeTargetProvider final : public VisualTargetProvider {
 public:
  RosTubeTargetProvider(
      ros::NodeHandle* node,
      std::string fixture_profile_id,
      std::string expected_optical_frame,
      double service_wait_timeout_sec,
      double maximum_target_age_sec);
  RosTubeTargetProvider(ros::NodeHandle* node,
      std::shared_ptr<const ResolvedTubeTaskConfig> config,
      const std::string& target_cap_color);

  TargetProviderResult Select(std::uint64_t not_before_ns) override;

 private:
  ros::ServiceClient client_;
  std::string fixture_profile_id_;
  std::string expected_optical_frame_;
  ros::Duration service_wait_timeout_;
  ros::Duration maximum_target_age_;
  bool valid_ = false;
  std::optional<agx_pick_msg::SelectTubeTarget2DConfigured::Request> configured_request_;
};

}  // namespace agx_xarm_pick

#endif  // AGX_XARM_PICK_TUBE_TARGET_PROVIDER_HPP
