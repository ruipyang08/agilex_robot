#include "agx_navigation_tasks/travel_motion.hpp"
#include "agx_navigation_tasks/travel_motion_sequence.hpp"
#include "agx_navigation_tasks/travel_spline.hpp"
#include "agx_xarm_pick/tube_lift_contract.hpp"
#include "agx_xarm_pick/tube_preparation_contract.hpp"
#include <bt_task_msgs/LiftMotorSrv.h>
#include <geometric_shapes/shapes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <algorithm>
#include <set>

namespace agx_navigation_tasks {
namespace {
bool ArmAtDefault(const TravelProfile& p,const agxcore::TubeLiveSnapshot& s) {
  if(!s.arm.fresh || !s.arm.valid || !s.arm.stationary || p.arm.size()!=6)return false;
  for(std::size_t i=0;i<6;++i)
    if(!std::isfinite(s.arm.positions[i]) || !std::isfinite(s.arm.velocities[i]) ||
        std::abs(s.arm.positions[i]-p.arm[i].position_rad)>p.arm[i].tolerance_rad ||
        std::abs(s.arm.velocities[i])>p.arm[i].stopped_velocity_rad_s)return false;
  return true;
}
bool AtNamedStart(const TravelProfile& p,const TravelStartPose& start,const agxcore::TubeLiveSnapshot& s) {
  if(!s.arm.valid || !s.arm.fresh || !s.arm.stationary || !s.lift.valid || !s.lift.fresh ||
      !s.lift.stationary || !std::isfinite(s.lift.position_m) ||
      std::abs(s.lift.position_m-start.lift_position_m)>start.lift_tolerance_m)return false;
  for(std::size_t i=0;i<p.arm.size();++i)
    if(!std::isfinite(s.arm.positions[i]) || !std::isfinite(s.arm.velocities[i]) ||
        std::abs(s.arm.positions[i]-start.arm_positions_rad.at(p.arm[i].name))>start.arm_tolerances_rad.at(p.arm[i].name) ||
        std::abs(s.arm.velocities[i])>p.arm[i].stopped_velocity_rad_s)return false;
  return true;
}
bool InMeasuredCorridor(const TravelProfile& p,const TravelStartPose& start,
    const agxcore::TubeLiveSnapshot& s,bool lifting) {
  if(!s.arm.valid || !s.arm.fresh || !s.lift.valid || !s.lift.fresh || !std::isfinite(s.lift.position_m))return false;
  std::vector<std::string> names;std::vector<double> positions;
  for(std::size_t i=0;i<p.arm.size();++i){names.push_back(p.arm[i].name);positions.push_back(s.arm.positions[i]);}
  if(!InTravelJointCorridor(p,start,names,positions))return false;
  if(!lifting)return std::abs(s.lift.position_m-start.lift_position_m)<=start.lift_tolerance_m;
  return ArmAtDefault(p,s) &&
      s.lift.position_m>=std::min(start.lift_position_m-start.lift_tolerance_m,p.lift_position_m-p.lift_tolerance_m) &&
      s.lift.position_m<=std::max(start.lift_position_m+start.lift_tolerance_m,p.lift_position_m+p.lift_tolerance_m);
}
bool MatchesMeasured(const TravelProfile& p,const moveit::core::RobotState& state,
    const agxcore::TubeLiveSnapshot& observed) {
  const auto& variables=state.getRobotModel()->getVariableNames();
  std::map<std::string,double> positions;
  for(const auto& joint:p.arm) {
    if(std::find(variables.begin(),variables.end(),joint.name)==variables.end())return false;
    positions[joint.name]=state.getVariablePosition(joint.name);
  }
  if(std::find(variables.begin(),variables.end(),"lifting_joint")==variables.end())return false;
  positions["lifting_joint"]=state.getVariablePosition("lifting_joint");
  return TravelModelMatchesMeasured(p,positions,observed);
}
bool AttachQualifiedLoad(const TravelProfile& p,const agx_robot_task_runtime::TransportView& load,moveit::core::RobotState& state) {
  std::map<std::string,std::string> parents;
  for(const auto* link:state.getRobotModel()->getLinkModels())
    parents[link->getName()]=link->getParentLinkModel()?link->getParentLinkModel()->getName():"";
  if(!TravelPayloadLinksAllowed(p,parents))return false;
  std::vector<const moveit::core::AttachedBody*> existing;
  state.getAttachedBodies(existing);
  if(!existing.empty())return false;  // This adapter supports exactly the declared empty/single load.
  if(load.load==agx_robot_task_runtime::TransportLoad::Empty)return true;
  if(load.load!=agx_robot_task_runtime::TransportLoad::SingleHeld || !load.object)return false;
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
  pose.translation()=Eigen::Vector3d(p.held_center_xyz_m[0],p.held_center_xyz_m[1],p.held_center_xyz_m[2]);
  pose.linear()=Eigen::Quaterniond(p.held_orientation_xyzw[3],p.held_orientation_xyzw[0],
      p.held_orientation_xyzw[1],p.held_orientation_xyzw[2]).toRotationMatrix();
  std::vector<shapes::ShapeConstPtr> shapes{shapes::ShapeConstPtr(new shapes::Cylinder(p.held_max_radius_m,p.held_max_length_m))};
  EigenSTL::vector_Isometry3d shape_poses{Eigen::Isometry3d::Identity()};
  const std::set<std::string> touch(p.held_touch_links.begin(),p.held_touch_links.end());
  const std::string id="qualified_travel_payload_"+load.object->id;
  state.attachBody(id,pose,shapes,shape_poses,touch,p.held_attachment_link);
  state.update();
  return state.getAttachedBody(id)!=nullptr;
}
std::shared_ptr<planning_scene::PlanningScene> CurrentScene(ros::NodeHandle& node,
    const moveit::core::RobotModelConstPtr& model) {
  auto client=node.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  moveit_msgs::GetPlanningScene request;
  using Components=moveit_msgs::PlanningSceneComponents;
  request.request.components.components=Components::SCENE_SETTINGS|Components::ROBOT_STATE|
      Components::ROBOT_STATE_ATTACHED_OBJECTS|Components::WORLD_OBJECT_GEOMETRY|Components::OCTOMAP|
      Components::TRANSFORMS|Components::ALLOWED_COLLISION_MATRIX|Components::LINK_PADDING_AND_SCALING;
  if(!client.call(request))return {};
  auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  if(!scene->setPlanningSceneMsg(request.response.scene))return {};
  std::vector<const moveit::core::AttachedBody*> attached;
  scene->getCurrentState().getAttachedBodies(attached);
  if(!attached.empty())return {};  // Do not silently drop an independently attached load.
  return scene;
}
bool CollisionFree(const planning_scene::PlanningScene& scene,moveit::core::RobotState& state) {
  state.update();return state.satisfiesBounds() && !scene.isStateColliding(state);
}
bool ArmPathSafe(const TravelProfile& p,const TravelStartPose& start,const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& initial,const moveit_msgs::RobotTrajectory& trajectory) {
  auto state=initial;const auto& joint=trajectory.joint_trajectory;
  if(joint.points.size()<2 || !CollisionFree(scene,state))return false;
  for(std::size_t segment=1;segment<joint.points.size();++segment) {
    const auto& previous=joint.points[segment-1];const auto& point=joint.points[segment];
    const double duration=(point.time_from_start-previous.time_from_start).toSec();
    std::vector<TravelQuinticCurve> curves;double variation=0;
    for(std::size_t i=0;i<joint.joint_names.size();++i) {
      auto curve=TravelQuintic(previous.positions[i],previous.velocities[i],previous.accelerations[i],
          point.positions[i],point.velocities[i],point.accelerations[i],duration);
      if(!curve)return false;
      variation=std::max(variation,curve->VelocityBound()*duration);curves.push_back(*curve);
    }
    // Sample the actual full-state quintic, not straight waypoint chords.
    // Continuous collision clearance/controller tracking error still belongs
    // to qualification of the complete named joint corridor.
    const double count=std::ceil(variation/.02);
    if(!std::isfinite(count) || count>10000)return false;
    const auto steps=std::max(1,static_cast<int>(count));
    for(int k=0;k<=steps;++k) {
      std::vector<double> positions;
      for(std::size_t i=0;i<curves.size();++i) {
        positions.push_back(curves[i].Sample(static_cast<double>(k)/steps)[0]);
        state.setVariablePosition(joint.joint_names[i],positions.back());
      }
      if(!InTravelJointCorridor(p,start,joint.joint_names,positions))return false;
      if(!CollisionFree(scene,state))return false;
    }
  }
  for(std::size_t i=0;i<joint.joint_names.size();++i) {
    const auto target=std::find_if(p.arm.begin(),p.arm.end(),
        [&](const TravelJointTarget& value){return value.name==joint.joint_names[i];});
    if(target==p.arm.end() || std::abs(joint.points.back().positions[i]-target->position_rad)>target->tolerance_rad)return false;
  }
  return true;
}
bool LiftPathSafe(const planning_scene::PlanningScene& scene,moveit::core::RobotState& state,
    double target,const MotionStopToken& token) {
  const double current=state.getVariablePosition("lifting_joint");
  const double count=std::ceil(std::abs(target-current)/.001);
  if(!std::isfinite(count) || count>2000)return false;
  const auto steps=std::max(1,static_cast<int>(count));
  for(int i=0;i<=steps;++i) {
    if(token->load())return false;
    state.setVariablePosition("lifting_joint",current+(target-current)*i/steps);
    if(!CollisionFree(scene,state))return false;
  }
  state.setVariablePosition("lifting_joint",current);state.update();return true;
}
bool LimitTrajectory(const TravelProfile& p,const TravelStartPose& start,moveit_msgs::RobotTrajectory* trajectory) {
  auto& joint=trajectory->joint_trajectory;
  if(!trajectory->multi_dof_joint_trajectory.points.empty() || !trajectory->multi_dof_joint_trajectory.joint_names.empty() ||
      joint.points.size()<2 || joint.joint_names.size()!=6 || !joint.points.front().time_from_start.isZero())return false;
  double factor=1,previous=-1;
  for(const auto& point:joint.points) {
    const double time=point.time_from_start.toSec();
    if(!std::isfinite(time) || time<0 || time<=previous || point.velocities.size()!=6 || point.accelerations.size()!=6 ||
        !InTravelJointCorridor(p,start,joint.joint_names,point.positions))return false;
    previous=time;
    for(std::size_t i=0;i<6;++i) {
      if(!std::isfinite(point.velocities[i]) || !std::isfinite(point.accelerations[i]))return false;
      factor=std::max(factor,std::abs(point.velocities[i])/p.arm_motion_velocity_rad_s);
      factor=std::max(factor,std::sqrt(std::abs(point.accelerations[i])/p.arm_motion_acceleration_rad_s2));
    }
  }
  for(std::size_t segment=1;segment<joint.points.size();++segment) {
    const auto& from=joint.points[segment-1];const auto& to=joint.points[segment];
    const double duration=(to.time_from_start-from.time_from_start).toSec();
    for(std::size_t i=0;i<6;++i) {
      const auto curve=TravelQuintic(from.positions[i],from.velocities[i],from.accelerations[i],
          to.positions[i],to.velocities[i],to.accelerations[i],duration);
      const auto target=std::find_if(p.arm.begin(),p.arm.end(),
          [&](const TravelJointTarget& item){return item.name==joint.joint_names[i];});
      if(!curve || target==p.arm.end())return false;
      const auto& name=joint.joint_names[i];
      const double low=std::min(start.arm_positions_rad.at(name)-start.arm_tolerances_rad.at(name),
          target->position_rad-target->tolerance_rad);
      const double high=std::max(start.arm_positions_rad.at(name)+start.arm_tolerances_rad.at(name),
          target->position_rad+target->tolerance_rad);
      if(!curve->Inside(low,high))return false;
      factor=std::max(factor,curve->TimeScale(p.arm_motion_velocity_rad_s,p.arm_motion_acceleration_rad_s2));
    }
  }
  if(!std::isfinite(factor) || joint.points.back().time_from_start.toSec()*factor>p.preparation_timeout_s)return false;
  for(auto& point:joint.points) {
    point.time_from_start=ros::Duration(point.time_from_start.toSec()*factor);
    for(auto& v:point.velocities)v/=factor;
    for(auto& a:point.accelerations)a/=(factor*factor);
  }
  // ROS duration quantization must not erase the strict derivative bounds.
  for(std::size_t segment=1;segment<joint.points.size();++segment) {
    const auto& from=joint.points[segment-1];const auto& to=joint.points[segment];
    for(std::size_t i=0;i<6;++i) {
      const auto curve=TravelQuintic(from.positions[i],from.velocities[i],from.accelerations[i],
          to.positions[i],to.velocities[i],to.accelerations[i],(to.time_from_start-from.time_from_start).toSec());
      if(!curve || curve->VelocityBound()>p.arm_motion_velocity_rad_s ||
          curve->AccelerationBound()>p.arm_motion_acceleration_rad_s2)return false;
      const auto& name=joint.joint_names[i];
      const auto target=std::find_if(p.arm.begin(),p.arm.end(),
          [&](const TravelJointTarget& item){return item.name==name;});
      if(target==p.arm.end() || !curve->Inside(
          std::min(start.arm_positions_rad.at(name)-start.arm_tolerances_rad.at(name),target->position_rad-target->tolerance_rad),
          std::max(start.arm_positions_rad.at(name)+start.arm_tolerances_rad.at(name),target->position_rad+target->tolerance_rad)))return false;
    }
  }
  return true;
}
}
BoundedMotionResult MoveToTravelPosture(ros::NodeHandle node,const TravelProfile& profile,
    const StartClearance& clearance,const agx_robot_task_runtime::TransportView& load,std::shared_ptr<agxcore::TubeLiveObserver> observer,
    std::function<agx_task_workflow::FailureKind()> guard,std::function<void()> stop) {
  ValidateTravelProfile(profile);
  const auto start=ResolveTravelMotionStart(profile,clearance,load);
  const auto wire=TravelLiftWireValues(profile);
  if(!observer || !guard || !start || !wire || profile.arm.size()!=6)
    return {{false,"travel requires qualified start/load, representable lift command and live six-joint observer"}};
  for(std::size_t i=0;i<6;++i)if(profile.arm[i].name!="joint"+std::to_string(i+1))
    return {{false,"travel adapter requires ordered xarm6 joints"}};
  if(!AtNamedStart(profile,*start,observer->Snapshot()))return {{false,"measured qualified starting posture changed"}};
  auto lifting=std::make_shared<std::atomic<bool>>(false);
  auto watched=[profile,start,observer,lifting,guard=std::move(guard)] {
    const auto failure=guard();if(failure!=agx_task_workflow::FailureKind::None)return failure;
    return InMeasuredCorridor(profile,*start,observer->Snapshot(),lifting->load())?agx_task_workflow::FailureKind::None:agx_task_workflow::FailureKind::Fault;
  };
  return RunBoundedMotion([node,profile,start,wire,load,observer,lifting](const MotionStopToken& interrupted) mutable -> agx_task_workflow::StepResult {
    moveit::core::RobotStatePtr state;
    TravelMotionSequencePorts ports;
    ports.arm=[&]() -> agx_task_workflow::StepResult {
      moveit::planning_interface::MoveGroupInterface arm(profile.robot.arm_group);
      if(interrupted->load())return {false,"travel stopped during arm setup"};
      arm.setPlanningTime(std::min(10.0,profile.preparation_timeout_s));
      state=arm.getCurrentState(1.0);
      if(!state || !MatchesMeasured(profile,*state,observer->Snapshot()) || !AttachQualifiedLoad(profile,load,*state))
        return {false,"MoveIt model/start/held geometry disagrees with qualified measured state"};
      if(ArmAtDefault(profile,observer->Snapshot()))return {true,"arm already at measured default"};
      arm.setStartState(*state);
      std::map<std::string,double> target;for(const auto& j:profile.arm)target[j.name]=j.position_rad;
      if(!arm.setJointValueTarget(target))return {false,"default arm target is invalid"};
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if(interrupted->load())return {false,"travel stopped before planning"};
      if(arm.plan(plan)!=moveit::planning_interface::MoveItErrorCode::SUCCESS)return {false,"qualified arm path planning failed"};
      if(interrupted->load())return {false,"travel stopped after planning"};
      robot_trajectory::RobotTrajectory path(arm.getRobotModel(),profile.robot.arm_group);
      path.setRobotTrajectoryMsg(*state,plan.trajectory_);
      trajectory_processing::IterativeParabolicTimeParameterization retime;
      if(!retime.computeTimeStamps(path,1.0,1.0))return {false,"travel trajectory timing failed"};
      path.getRobotTrajectoryMsg(plan.trajectory_);
      if(!LimitTrajectory(profile,*start,&plan.trajectory_))return {false,"travel trajectory violates qualified corridor or timing"};
      const auto scene=CurrentScene(node,arm.getRobotModel());
      if(!scene || !ArmPathSafe(profile,*start,*scene,*state,plan.trajectory_))
        return {false,"travel arm/payload path is not collision clear"};
      const auto before=observer->Snapshot();
      if(interrupted->load() || !MatchesMeasured(profile,*state,before))return {false,"travel start changed before arm execution"};
      if(arm.execute(plan)!=moveit::planning_interface::MoveItErrorCode::SUCCESS)return {false,"default arm execution failed"};
      while(!interrupted->load()) {
        const auto measured=observer->Snapshot();
        if(!measured.arm.valid || !measured.arm.fresh)return {false,"arm feedback lost after execution"};
        if(ArmAtDefault(profile,measured) && measured.arm.evidence.source_stamp_s>before.source_now_s &&
            measured.arm.evidence.receipt_steady_s>before.steady_now_s)return {true,"measured arm endpoint"};
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
      return {false,"travel interrupted while measuring arm endpoint"};
    };
    ports.lift_needed=[&]()->std::optional<bool> {
      const auto measured=observer->Snapshot();
      if(!state || !ArmAtDefault(profile,measured) || !measured.lift.valid || !measured.lift.fresh ||
          !measured.lift.stationary || !std::isfinite(measured.lift.position_m))return {};
      for(std::size_t i=0;i<profile.arm.size();++i)state->setVariablePosition(profile.arm[i].name,measured.arm.positions[i]);
      state->setVariablePosition("lifting_joint",measured.lift.position_m);state->update();
      const auto scene=CurrentScene(node,state->getRobotModel());
      if(!scene || interrupted->load() || !LiftPathSafe(*scene,*state,wire->position_mm/1000.0,interrupted))return {};
      lifting->store(true);
      return std::abs(measured.lift.position_m-profile.lift_position_m)>profile.lift_tolerance_m;
    };
    auto lift=node.serviceClient<bt_task_msgs::LiftMotorSrv>(profile.robot.lift_service);
    ports.lift_speed=[&] {
      bt_task_msgs::LiftMotorSrv request;request.request.mode=-5;request.request.val=wire->speed_mm_s;
      if(!lift.call(request) || !agx_xarm_pick::accepted_lift_speed_service_state(request.response.state))
        return agx_task_workflow::StepResult{false,"travel lift speed request failed"};
      return agx_task_workflow::StepResult{true,"travel lift speed accepted"};
    };
    ports.lift_position=[&] {
      const auto measured=observer->Snapshot();
      if(interrupted->load() || !InMeasuredCorridor(profile,*start,measured,true) ||
          !measured.lift.stationary || std::abs(measured.lift.position_m-state->getVariablePosition("lifting_joint"))>profile.lift_tolerance_m)
        return agx_task_workflow::StepResult{false,"travel posture changed before lift position request"};
      for(std::size_t i=0;i<profile.arm.size();++i)state->setVariablePosition(profile.arm[i].name,measured.arm.positions[i]);
      state->setVariablePosition("lifting_joint",measured.lift.position_m);state->update();
      const auto scene=CurrentScene(node,state->getRobotModel());
      if(!scene || interrupted->load() || !LiftPathSafe(*scene,*state,wire->position_mm/1000.0,interrupted))
        return agx_task_workflow::StepResult{false,"lift/payload path is not collision clear before dispatch"};
      bt_task_msgs::LiftMotorSrv request;request.request.mode=0;request.request.val=wire->position_mm;
      if(!lift.call(request) || !agx_xarm_pick::accepted_lift_service_state(request.response.state))
        return agx_task_workflow::StepResult{false,"travel lift position request failed"};
      while(!interrupted->load()) {
        const auto actual=observer->Snapshot();
        if(!actual.lift.valid || !actual.lift.fresh)return agx_task_workflow::StepResult{false,"lift feedback lost after position request"};
        if(actual.lift.stationary && std::isfinite(actual.lift.position_m) &&
            std::abs(actual.lift.position_m-profile.lift_position_m)<=profile.lift_tolerance_m &&
            actual.lift.evidence.source_stamp_s>measured.source_now_s &&
            actual.lift.evidence.receipt_steady_s>measured.steady_now_s)
          return agx_task_workflow::StepResult{true,"measured lift endpoint"};
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
      return agx_task_workflow::StepResult{false,"travel interrupted while measuring lift endpoint"};
    };
    return RunTravelMotionSequence(interrupted,ports);
  },std::move(watched),std::move(stop),profile.preparation_timeout_s);
}
}  // namespace agx_navigation_tasks
