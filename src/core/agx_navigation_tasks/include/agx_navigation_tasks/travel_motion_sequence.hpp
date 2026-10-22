#pragma once
#include <algorithm>
#include <limits>
#include <set>
#include "agx_navigation_tasks/bounded_motion.hpp"
#include "agx_navigation_tasks/travel_policy.hpp"

namespace agx_navigation_tasks {
// These are the actual adapter's command boundaries, kept ROS-free so delayed
// replies and interruption traces are testable without an actuator connection.
struct TravelMotionSequencePorts {
  std::function<agx_task_workflow::StepResult()> arm,lift_speed,lift_position;
  std::function<std::optional<bool>()> lift_needed;
};
struct TravelLiftWire {std::int64_t speed_mm_s=0,position_mm=0;};
inline bool TravelPayloadLinksAllowed(const TravelProfile& profile,const std::map<std::string,std::string>& parents) {
  const std::string root="dh_robotics_ag95_base_link";
  const auto descends=[&](std::string link) {
    std::set<std::string> visited;
    while(parents.count(link) && visited.insert(link).second) {
      if(link==root)return true;
      link=parents.at(link);
    }
    return false;
  };
  if(!descends(profile.held_attachment_link) || profile.held_touch_links.empty())return false;
  for(const auto& link:profile.held_touch_links)
    if(link==root || !descends(link))return false;
  return true;
}
inline bool TravelModelMatchesMeasured(const TravelProfile& profile,
    const std::map<std::string,double>& positions,const agxcore::TubeLiveSnapshot& observed) {
  if(!observed.arm.valid || !observed.arm.fresh || !observed.lift.valid || !observed.lift.fresh ||
      profile.arm.size()>observed.arm.positions.size())return false;
  for(std::size_t i=0;i<profile.arm.size();++i) {
    const auto value=positions.find(profile.arm[i].name);
    if(value==positions.end() || !std::isfinite(value->second) || !std::isfinite(observed.arm.positions[i]) ||
        std::abs(value->second-observed.arm.positions[i])>profile.arm[i].tolerance_rad)return false;
  }
  const auto lift=positions.find("lifting_joint");
  return lift!=positions.end() && std::isfinite(lift->second) && std::isfinite(observed.lift.position_m) &&
      std::abs(lift->second-observed.lift.position_m)<=profile.lift_tolerance_m;
}
inline std::optional<TravelStartPose> ResolveTravelMotionStart(
    const TravelProfile& profile,const StartClearance& start,const agx_robot_task_runtime::TransportView& load) {
  if(!start.ok || (!profile.simulation.enabled && !profile.hardware.enabled) ||
      load.pending || load.uncertain ||
      (load.eligibility!=agx_robot_task_runtime::TravelEligibility::Unknown && load.eligibility!=agx_robot_task_runtime::TravelEligibility::SafeClear))return {};
  const bool empty=load.load==agx_robot_task_runtime::TransportLoad::Empty && !load.object;
  const bool held=load.load==agx_robot_task_runtime::TransportLoad::SingleHeld && load.object && !load.object->id.empty() &&
      load.object->container_profile==profile.container_profile_id &&
      (load.object->identity==agx_robot_task_runtime::ObjectIdentity::Registered || load.object->identity==agx_robot_task_runtime::ObjectIdentity::RunLocal);
  if(!empty && !held)return {};
  for(const auto& candidate:profile.starts)
    if(candidate.id==start.start_pose_id && candidate.path_id==start.path_id &&
        ((empty && candidate.empty_allowed) || (held && candidate.single_held_allowed)))return candidate;
  return {};
}
inline bool InTravelJointCorridor(const TravelProfile& profile,const TravelStartPose& start,
    const std::vector<std::string>& names,const std::vector<double>& positions) {
  if(names.size()!=profile.arm.size() || positions.size()!=names.size())return false;
  std::set<std::string> seen;
  for(std::size_t i=0;i<names.size();++i) {
    if(!seen.insert(names[i]).second || !std::isfinite(positions[i]))return false;
    const auto target=std::find_if(profile.arm.begin(),profile.arm.end(),
        [&](const TravelJointTarget& joint){return joint.name==names[i];});
    const auto origin=start.arm_positions_rad.find(names[i]);
    const auto tolerance=start.arm_tolerances_rad.find(names[i]);
    if(target==profile.arm.end() || origin==start.arm_positions_rad.end() ||
        tolerance==start.arm_tolerances_rad.end())return false;
    const double lower=std::min(origin->second-tolerance->second,target->position_rad-target->tolerance_rad);
    const double upper=std::max(origin->second+tolerance->second,target->position_rad+target->tolerance_rad);
    if(!std::isfinite(lower) || !std::isfinite(upper) || positions[i]<lower || positions[i]>upper)return false;
  }
  return true;
}
inline std::optional<TravelLiftWire> TravelLiftWireValues(const TravelProfile& profile) {
  const double speed=profile.lift_motion_velocity_mps*1000.0;
  const double position=profile.lift_position_m*1000.0;
  const long double maximum=std::numeric_limits<std::int64_t>::max();
  if(!std::isfinite(speed) || !std::isfinite(position) || speed<1 || position<0 ||
      static_cast<long double>(speed)>=maximum || static_cast<long double>(position)>=maximum)return {};
  // Never round a maximum speed upward. Endpoint quantization must still fit
  // the qualified tolerance and its physical path envelope.
  const auto wire_speed=static_cast<std::int64_t>(std::floor(speed));
  const auto wire_position=std::llround(position);
  if(!std::isfinite(profile.lift_tolerance_m) || profile.lift_tolerance_m<=0 ||
      std::abs(static_cast<double>(wire_position)/1000.0-profile.lift_position_m)>profile.lift_tolerance_m)return {};
  return TravelLiftWire{wire_speed,wire_position};
}
inline agx_task_workflow::StepResult RunTravelMotionSequence(const MotionStopToken& token,const TravelMotionSequencePorts& ports) {
  if(!token || !ports.arm || !ports.lift_needed || !ports.lift_speed || !ports.lift_position)
    return {false,"travel command ports are incomplete"};
  const auto interrupted=[] {return agx_task_workflow::StepResult{false,"travel producer interrupted"};};
  if(token->load())return interrupted();
  auto result=ports.arm();
  if(token->load())return interrupted();
  if(!result.ok)return result;
  const auto needed=ports.lift_needed();
  if(token->load())return interrupted();
  if(!needed)return {false,"lift or arm endpoint evidence is unavailable"};
  if(*needed) {
    result=ports.lift_speed();
    if(token->load())return interrupted();
    if(!result.ok)return result;
    result=ports.lift_position();
    if(token->load())return interrupted();
    if(!result.ok)return result;
  }
  return {true,"travel producers returned; post-command measured dwell remains required"};
}
}
