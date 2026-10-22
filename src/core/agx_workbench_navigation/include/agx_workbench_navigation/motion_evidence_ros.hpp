#pragma once
#include <agx_workbench_navigation/motion_preflight.hpp>
#include <ros/node_handle.h>
#include <memory>
namespace agx_workbench_navigation {
struct MotionCostmapSnapshot {
  MotionGrid grid;
  double observed_steady_s=0,deadline_steady_s=0;
};
// Read-only adapter. Owns sensor callbacks on a separate queue and never grants
// motion. The costmap callback supplies raw obstacle cells (0 or 254), copied
// under their owning locks. Static cells come from the verified live map;
// inflated costs and footprint-cleared master cells are never its authority.
class RosMotionEvidence {
 public:
  using GridSource=std::function<MotionCostmapSnapshot()>;
  RosMotionEvidence(ros::NodeHandle node,std::string policy_path,std::string map_path,
      std::string registry_path,std::string lidar_config_path,GridSource costmap,
      std::function<double()> steady_clock);
  ~RosMotionEvidence();
  MotionPlanningEvidence Snapshot();
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
// Immutable owner-published cache. Reading cannot renew original deadlines.
void PublishSharedMotionEvidence(MotionPlanningEvidence evidence);
std::shared_ptr<const MotionPlanningEvidence> SharedMotionEvidence();
}
