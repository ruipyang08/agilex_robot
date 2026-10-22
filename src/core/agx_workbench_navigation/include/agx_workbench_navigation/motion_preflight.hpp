#pragma once
#include <agx_workbench_navigation/motion_search.hpp>
#include <agx_workbench_navigation/motion_route_store.hpp>
#include <agx_workbench_navigation/motion_departure.hpp>
#include <mutex>

namespace agx_workbench_navigation {
// Evidence is supplied by the owning ROS adapter, not by service request fields.
// Deadlines use its original receipts/source ages converted to the steady clock.
struct MotionPlanningEvidence {
  MotionGrid grid;
  MotionPose measured_pose;
  std::string frame_id, static_map_digest, registry_digest;
  std::string unavailable_detail;
  double observed_at_s=0, pose_deadline_s=0, obstacle_deadline_s=0;
  MotionDeparturePolicy departure_policy;
  std::vector<RegisteredMotionDeparture> registered_departures;
  bool departure_stop_observed=false;
};
struct MotionPreflightRequest {
  std::string invocation_id, existing_route_id, model_digest, registry_digest, frame_id;
  MotionPose measured_start, coarse_start, goal;
  std::vector<MotionSegment> departure_prefix;
  std::string departure_source_id;
};
struct MotionPreflightConfig {
  std::string model_digest, frame_id;
  MotionGeometryPolicy geometry;
  MotionLimits limits;
  MotionSearchOptions search;
  double start_position_tolerance_m=.01, start_yaw_tolerance_rad=.01;
};
// Read-only planning and pre-release revalidation. No actuator/grant API.
// Registered departure uses installed directional geometry and source stations.
// Its owner must prove a fresh stopped endpoint before coarse route binding.
class MotionPreflight {
 public:
  using EvidenceSource=std::function<MotionPlanningEvidence()>;
  MotionPreflight(MotionPreflightConfig config,MotionRouteStore& store,
                  std::function<double()> steady_clock);
  RouteResult Plan(const MotionPreflightRequest& request,const EvidenceSource& evidence,
                   const std::function<bool()>& cancelled={});
  RouteResult CompleteDeparture(const std::string& route_id,const std::string& invocation_id,
                               const EvidenceSource& evidence);
 private:
  bool Fresh(const MotionPlanningEvidence& evidence,double now) const;
  bool StartMatches(const MotionPose& observed,const MotionPose& requested) const;
  MotionPreflightConfig config_;
  MotionRouteStore& store_;
  std::function<double()> clock_;
  std::mutex work_mutex_;
  std::string session_;
  std::uint64_t next_route_=0;
};
}  // namespace agx_workbench_navigation
