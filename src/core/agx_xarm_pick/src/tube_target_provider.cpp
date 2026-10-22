#include "agx_xarm_pick/tube_target_provider.hpp"
#include "agx_xarm_pick/tube_perception_projection.hpp"

#include <cmath>
#include <utility>

namespace agx_xarm_pick {
namespace {

TargetProviderStatus MapStatus(std::uint8_t status) {
  using Response = agx_pick_msg::SelectTubeTarget2D::Response;
  switch (status) {
    case Response::SUCCESS:
      return TargetProviderStatus::kSuccess;
    case Response::INVALID_REQUEST:
      return TargetProviderStatus::kInvalidRequest;
    case Response::TARGET_ABSENT:
      return TargetProviderStatus::kAbsent;
    case Response::AMBIGUOUS_TARGET:
      return TargetProviderStatus::kAmbiguous;
    case Response::STALE_FRAME:
      return TargetProviderStatus::kStale;
    case Response::PROFILE_MISMATCH:
      return TargetProviderStatus::kProfileMismatch;
    case Response::INCONSISTENT_TARGET:
      return TargetProviderStatus::kInconsistent;
    case Response::INVALID_OBSERVATION:
      return TargetProviderStatus::kInvalidObservation;
    default:
      return TargetProviderStatus::kInvalidObservation;
  }
}

}  // namespace

RosTubeTargetProvider::RosTubeTargetProvider(
    ros::NodeHandle* node,
    std::string fixture_profile_id,
    std::string expected_optical_frame,
    double service_wait_timeout_sec,
    double maximum_target_age_sec)
    : fixture_profile_id_(std::move(fixture_profile_id)),
      expected_optical_frame_(std::move(expected_optical_frame)),
      service_wait_timeout_(service_wait_timeout_sec),
      maximum_target_age_(maximum_target_age_sec) {
  valid_ = node != nullptr && !fixture_profile_id_.empty() &&
      !expected_optical_frame_.empty() &&
      std::isfinite(service_wait_timeout_sec) &&
      service_wait_timeout_sec > 0.0 &&
      std::isfinite(maximum_target_age_sec) && maximum_target_age_sec > 0.0;
  if (valid_) {
    client_ = node->serviceClient<agx_pick_msg::SelectTubeTarget2D>(
        "/select_tube_target_2d", true);
  }
}

RosTubeTargetProvider::RosTubeTargetProvider(ros::NodeHandle* node,
    std::shared_ptr<const ResolvedTubeTaskConfig> config, const std::string& color)
    : RosTubeTargetProvider(node, config ? config->fixture_profile_id : "",
        config ? config->motion.color_frame : "",
        config ? config->motion.detection_timeout_sec : 1.0,
        config ? config->motion.maximum_frame_age_sec : 1.0) {
  if (!valid_ || !config) return;
  configured_request_.emplace();
  FillConfiguredTubePerceptionRequest(*config,color,&*configured_request_);
  client_ = node->serviceClient<agx_pick_msg::SelectTubeTarget2DConfigured>(
      "/select_tube_target_2d_configured",true);
}

TargetProviderResult RosTubeTargetProvider::Select(
    std::uint64_t not_before_ns) {
  TargetProviderResult result;
  if (!valid_ || not_before_ns == 0) {
    result.status = TargetProviderStatus::kInvalidRequest;
    result.detail = "invalid ROS tube-target provider request";
    return result;
  }
  if (!client_.waitForExistence(service_wait_timeout_)) {
    result.status = TargetProviderStatus::kUnavailable;
    result.detail = "target service is unavailable";
    return result;
  }

  agx_pick_msg::SelectTubeTarget2D service;
  service.request.fixture_profile_id = fixture_profile_id_;
  service.request.not_before.fromNSec(not_before_ns);
  bool received = false;
  if (configured_request_) {
    agx_pick_msg::SelectTubeTarget2DConfigured configured;
    configured.request=*configured_request_;
    configured.request.not_before=service.request.not_before;
    received=client_.call(configured);
    if (received && (configured.response.config_id!=configured.request.config_id ||
        configured.response.fixture_instance_id!=configured.request.fixture_instance_id ||
        configured.response.fixture_profile_id!=configured.request.fixture_profile_id ||
        configured.response.projection_sha256!=configured.request.projection_sha256)) {
      result.status=TargetProviderStatus::kProfileMismatch;
      result.detail="target service returned a different configuration identity";
      return result;
    }
    service.response.status=configured.response.status;
    service.response.detail=configured.response.detail;
    service.response.target=configured.response.target;
    service.response.target_candidate_count=configured.response.target_candidate_count;
  } else {
    received=client_.call(service);
  }
  if (!received) {
    result.status = TargetProviderStatus::kUnavailable;
    result.detail = "target service transport failed";
    return result;
  }

  result.status = MapStatus(service.response.status);
  result.detail = service.response.detail;
  if (result.status != TargetProviderStatus::kSuccess) {
    return result;
  }

  const auto& target = service.response.target;
  result.observation.stamp_ns = target.header.stamp.toNSec();
  result.observation.frame_id = target.header.frame_id;
  result.observation.u_px = target.u_px;
  result.observation.v_px = target.v_px;
  result.observation.reference_u_px = target.reference_u_px;
  result.observation.reference_v_px = target.reference_v_px;
  result.observation.image_width = target.image_width;
  result.observation.image_height = target.image_height;

  const ros::Time now = ros::Time::now();
  const bool time_order_valid = !target.header.stamp.isZero() &&
      target.header.stamp > service.request.not_before &&
      !now.isZero() && target.header.stamp <= now;
  const bool age_valid = time_order_valid &&
      now - target.header.stamp <= maximum_target_age_;
  if (service.response.target_candidate_count != 1 ||
      target.header.frame_id != expected_optical_frame_ ||
      !valid_visual_target_observation(result.observation) ||
      !age_valid) {
    result.status = TargetProviderStatus::kInvalidObservation;
    result.detail = "target service returned an invalid or stale success payload";
  }
  return result;
}

}  // namespace agx_xarm_pick
