#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace agx_workbench_navigation {
// Experimental simulation tuning ceiling; installed policy still owns the
// actual limit and execution qualification remains a separate gate.
constexpr double kMotionYawRateCeilingRadps=.55;
enum class MotionKind : std::uint8_t { Forward, Spin, DockDeparture, VerifyStop };
enum class RouteStatus : std::uint8_t {
  Ready, NoRoute, SearchLimit, EvidenceUnavailable, StartMismatch,
  RouteInvalidated, InvalidRequest
};
struct MotionPose { double x_m=0, y_m=0, yaw_rad=0; };
struct MotionSample {
  MotionPose pose;
  double curvature_per_m=0, speed_ceiling_mps=0, yaw_ceiling_radps=0, time_s=0;
};
struct MotionSegment {
  MotionKind kind=MotionKind::Forward;
  std::vector<MotionSample> samples;
  bool stop_before=true, stop_after=true;
  double estimated_time_s=0;
  double geometry_error_m=0;  // Conservative curve-to-sampled-path bound.
  double curvature_bound_per_m=0;  // Includes curvature between samples.
};
struct MotionRoute {
  std::string route_id, invocation_id, model_digest, map_digest, registry_digest;
  std::string departure_source_id;
  MotionPose measured_start, coarse_start, goal;
  std::vector<MotionSegment> segments;
  double estimated_time_s=0;
};
struct RouteResult {
  RouteStatus status=RouteStatus::InvalidRequest;
  MotionRoute route;
  std::string detail;
};
struct MotionLimits {
  double max_speed_mps=.5, acceleration_mps2=.25, deceleration_mps2=.3;
  double max_yaw_rate_radps=.4, angular_acceleration_radps2=.4;
  double curvature_rate_per_m_s=.5, lateral_acceleration_mps2=.2;
  double min_turn_radius_m=.4764, steering_settle_s=2., stop_dwell_s=.12;
};
double MotionAngleDifference(double to, double from);
bool ValidMotionPose(const MotionPose& pose);
bool SameMotionPose(const MotionPose& a, const MotionPose& b, double tolerance=1e-6);
// Empty string means structurally valid; collision and fresh-state checks are separate.
std::string ValidateMotionLimits(const MotionLimits& limits);
std::string ValidateMotionRoute(const MotionRoute& route, const MotionLimits& limits);
}  // namespace agx_workbench_navigation
