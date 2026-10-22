#pragma once
#include "agx_xarm_pick/resolved_tube_config.hpp"
#include <ros/ros.h>

namespace agxcore {
// Read the commissioned legacy parameters once before task dispatch. No
// actuator clients are created and no parameters are written by admission.
std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig>
SnapshotLegacyTubeTaskConfig(const ros::NodeHandle& node);

// Shared motion validation for legacy and modular configuration projections.
bool ValidateTubeMotionConfig(const agx_xarm_pick::TubeTaskMotionConfig& config);
}
