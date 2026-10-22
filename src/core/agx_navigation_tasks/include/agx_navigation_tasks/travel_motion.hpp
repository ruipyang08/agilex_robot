#pragma once
#include "agx_navigation_tasks/bounded_motion.hpp"
#include "agx_navigation_tasks/travel_profile.hpp"
#include "agx_navigation_tasks/travel_policy.hpp"
#include "agx_xarm_pick/tube_live_observer.hpp"
namespace ros {class NodeHandle;}
namespace agx_navigation_tasks {
// ROS actuator adapter. Only the policy-qualified start/default path may call
// this; it never opens, closes or initializes the gripper.
BoundedMotionResult MoveToTravelPosture(ros::NodeHandle node,const TravelProfile&,
    const StartClearance&,const agx_robot_task_runtime::TransportView&,
    std::shared_ptr<agxcore::TubeLiveObserver>,std::function<agx_task_workflow::FailureKind()> guard,
    std::function<void()> stop);
}
