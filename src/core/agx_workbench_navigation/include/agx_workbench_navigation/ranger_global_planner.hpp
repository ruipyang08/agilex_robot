#ifndef AGX_WORKBENCH_NAVIGATION_RANGER_GLOBAL_PLANNER_HPP_
#define AGX_WORKBENCH_NAVIGATION_RANGER_GLOBAL_PLANNER_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <agx_workbench_navigation/motion_geometry.hpp>
#include <agx_workbench_navigation/motion_policy_io.hpp>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/publisher.h>
#include <ros/wall_timer.h>

namespace agx_workbench_navigation {
class RosMotionEvidence;
class MotionPlanService;
struct MotionCostmapSnapshot;

class RangerGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  RangerGlobalPlanner();
  ~RangerGlobalPlanner() override;
  RangerGlobalPlanner(
      std::string name,
      costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(
      std::string name,
      costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan) override;

 private:
  void publishEffectiveConfiguration(const ros::WallTimerEvent&);
  void captureRouteCostmap(const ros::WallTimerEvent&);
  MotionCostmapSnapshot routeCostmapSnapshot();
  bool initialized_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  ros::Publisher global_plan_publisher_;
  std::string effective_config_session_;
  ros::Publisher effective_config_publisher_;
  ros::WallTimer effective_config_timer_;
  ros::WallTimer route_costmap_timer_;
  std::mutex route_costmap_mutex_;
  std::shared_ptr<const MotionGrid> route_costmap_;
  double route_costmap_receipt_s_=0;
  std::string route_source_error_;
  std::unique_ptr<InstalledMotionPolicy> route_policy_;
  std::unique_ptr<RosMotionEvidence> route_evidence_;
  // Destroy/drain the search queue before its evidence source.
  std::unique_ptr<MotionPlanService> route_service_;
};

}  // namespace agx_workbench_navigation

#endif  // AGX_WORKBENCH_NAVIGATION_RANGER_GLOBAL_PLANNER_HPP_
