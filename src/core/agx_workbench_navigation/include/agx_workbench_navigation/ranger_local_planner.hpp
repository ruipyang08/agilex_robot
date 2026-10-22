#ifndef AGX_WORKBENCH_NAVIGATION_RANGER_LOCAL_PLANNER_HPP_
#define AGX_WORKBENCH_NAVIGATION_RANGER_LOCAL_PLANNER_HPP_

#include <agx_workbench_navigation/motion_route_follower.hpp>
#include <agx_workbench_navigation/motion_policy_io.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>
#include <ros/wall_timer.h>
#include <tf2_ros/buffer.h>

#include <agx_workbench_navigation/CoarseNavigationState.h>
#include <agx_workbench_navigation/LidarSafetyState.h>
#include <agx_workbench_navigation/ranger_control.hpp>
#include <agx_workbench_navigation/command_source_gate.hpp>
#include <agx_workbench_navigation/AuthorizedBaseCommand.h>
#include <agx_workbench_navigation/BaseAuthorityState.h>
#include <agx_workbench_navigation/BindBaseCommandSource.h>
#include <agx_workbench_navigation/effective_planner_config.hpp>
#include <agx_workbench_navigation/navigation_progress_guard.hpp>

namespace agx_workbench_navigation {

class RangerLocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  RangerLocalPlanner();

  void initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(
      const std::vector<geometry_msgs::PoseStamped>& global_plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

 private:
  friend class RangerLocalPlannerProgressTest;
  bool checkNavigationProgress(const CommandBinding&,const ros::Time&);
  void publishEffectiveConfiguration(const ros::WallTimerEvent&);
  bool bindSource(BindBaseCommandSource::Request&,BindBaseCommandSource::Response&);
  bool setCurrentPlan(const std::vector<geometry_msgs::PoseStamped>&);
  bool computeRouteCommand(geometry_msgs::Twist&,const CommandBinding&,const ros::Time&);
  bool currentGoalReached(const CommandBinding&,const ros::Time&);
  void authorityCallback(const BaseAuthorityState::ConstPtr& message);
  void publishAuthorized(const CommandBinding&,std::uint64_t,const ros::Time&,
                         const geometry_msgs::Twist&);
  void publishDecision(
      const RangerControlDecision& decision,
      bool arc_collision_free,
      bool rotation_costmap_clear,
      bool rotation_lidar_clear,
      double path_heading_error_rad);

  bool initialized_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  RangerPlannerConfig config_;
  std::unique_ptr<RangerCommandLimiter> command_limiter_;
  std::unique_ptr<NavigationProgressGuard> navigation_progress_;
  // First accepted current TF resolution for this operation's original sample.
  // A later TF2 interpolation path must not reinterpret that same observation.
  NavigationProgressPose resolved_progress_pose_;
  std::string progress_reason_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped last_goal_;
  bool last_goal_available_;
  ros::Time plan_receipt_time_;
  ros::Publisher state_publisher_;
  ros::Publisher local_plan_publisher_;
  ros::Subscriber authority_subscriber_;
  CanonicalMotionFeedback motion_feedback_;
  mutable std::mutex evidence_mutex_;
  CommandSourceGate command_gate_;
  std::unique_ptr<InstalledMotionPolicy> route_policy_;
  std::unique_ptr<MotionRouteFollower> route_follower_;
  FollowerObservation route_observation_;
  bool route_complete_=false;
  CommandBinding route_binding_;
  ros::Publisher authorized_publisher_;
  ros::ServiceServer binding_service_;
  RoverExecutionSnapshot effective_rover_execution_;
  std::string effective_config_session_;
  ros::Publisher effective_config_publisher_;
  ros::WallTimer effective_config_timer_;  // destroyed before report state
};

}  // namespace agx_workbench_navigation

#endif  // AGX_WORKBENCH_NAVIGATION_RANGER_LOCAL_PLANNER_HPP_
