#pragma once
#include <agx_workbench_navigation/motion_geometry.hpp>

namespace agx_workbench_navigation {
struct RegisteredMotionDeparture {
  std::string station_id;
  MotionPose marker, staging;
};
// Installed by the evidence owner. No request can supply this geometry or
// relax it. The rear corridor is the existing directional docking phase;
// ordinary travel still uses its full body outline, including at the join.
struct MotionDeparturePolicy {
  bool enabled=false;
  MotionGeometryPolicy geometry;
  double max_distance_m=2., max_speed_mps=.05, acceleration_mps2=.1;
  double stop_dwell_s=.5, max_cross_track_m=.025, max_yaw_drift_rad=.035;
};
struct MotionDepartureResult {
  RouteStatus status=RouteStatus::EvidenceUnavailable;
  MotionSegment segment;
  std::string detail;
};
struct MotionDeparturePose {MotionPose pose;double source_stamp_s=0;};
// Original ordered measured poses, not reported command velocity or repeated
// callback time, establish the half-second stopped interval.
bool ObservedDepartureStop(const std::vector<MotionDeparturePose>&,double latest_source_stamp_s);
MotionDepartureResult AdmitRegisteredDeparture(const MotionGrid&,
    const MotionDeparturePolicy&, const std::vector<RegisteredMotionDeparture>&,
    const std::string& source_id, const MotionPose& start, const MotionPose& endpoint);
}  // namespace agx_workbench_navigation
