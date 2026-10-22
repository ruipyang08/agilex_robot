#pragma once
#include <agx_workbench_navigation/motion_preflight.hpp>
#include <agx_workbench_navigation/PlanNavigationRoute.h>
#include <agx_workbench_navigation/ManageNavigationRoute.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/spinner.h>
#include <atomic>

namespace agx_workbench_navigation {
// Owned callback queue: search cannot block TF, odometry, health or controller
// callbacks. One job executes; a concurrent service request receives BUSY
// (EvidenceUnavailable), without starting a second search.
class MotionPlanService {
 public:
  MotionPlanService(ros::NodeHandle node,MotionPreflightConfig config,
                    MotionRouteStore& store,MotionPreflight::EvidenceSource evidence,
                    std::function<double()> steady_clock);
  ~MotionPlanService();
  MotionPlanService(const MotionPlanService&)=delete;
  MotionPlanService& operator=(const MotionPlanService&)=delete;
 private:
  bool Handle(PlanNavigationRoute::Request& request,PlanNavigationRoute::Response& response);
  bool Manage(ManageNavigationRoute::Request&,ManageNavigationRoute::Response&);
  ros::CallbackQueue queue_;
  ros::NodeHandle node_;
  MotionPreflight planner_;
  MotionRouteStore& store_;
  MotionPreflight::EvidenceSource evidence_;
  std::string frame_;
  std::atomic<bool> stopping_{false};
  ros::ServiceServer service_;
  ros::ServiceServer management_;
  ros::AsyncSpinner spinner_;
};
}  // namespace agx_workbench_navigation
