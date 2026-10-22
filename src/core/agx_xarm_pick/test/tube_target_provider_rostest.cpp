#include "agx_xarm_pick/tube_target_provider.hpp"

#include <agx_pick_msg/SelectTubeTarget2D.h>
#include "agx_xarm_pick/resolved_tube_config.hpp"
#include <agx_pick_msg/SelectTubeTarget2DConfigured.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using agx_xarm_pick::TargetProviderStatus;

enum class Mutation {
  kNone,
  kZeroStamp,
  kEqualStamp,
  kWrongFrame,
  kNonFinite,
  kZeroDimension,
  kOutOfBounds,
  kWrongCandidateCount,
  kTooOld,
};

struct FakeService {
  std::uint8_t status = agx_pick_msg::SelectTubeTarget2D::Response::SUCCESS;
  Mutation mutation = Mutation::kNone;
  std::string captured_profile;
  std::uint64_t captured_not_before_ns = 0;

  bool Handle(
      agx_pick_msg::SelectTubeTarget2D::Request& request,
      agx_pick_msg::SelectTubeTarget2D::Response& response) {
    captured_profile = request.fixture_profile_id;
    captured_not_before_ns = request.not_before.toNSec();
    response.status = status;
    response.detail = "fake target response";
    response.target_candidate_count = 1;
    response.target.header.stamp = ros::Time::now();
    response.target.header.frame_id = "cam1_color_optical_frame";
    response.target.u_px = 365.0F;
    response.target.v_px = 215.0F;
    response.target.reference_u_px = 319.5F;
    response.target.reference_v_px = 239.5F;
    response.target.image_width = 640;
    response.target.image_height = 480;
    switch (mutation) {
      case Mutation::kNone:
        break;
      case Mutation::kZeroStamp:
        response.target.header.stamp = ros::Time(0);
        break;
      case Mutation::kEqualStamp:
        response.target.header.stamp = request.not_before;
        break;
      case Mutation::kWrongFrame:
        response.target.header.frame_id = "wrong_frame";
        break;
      case Mutation::kNonFinite:
        response.target.u_px = std::numeric_limits<float>::quiet_NaN();
        break;
      case Mutation::kZeroDimension:
        response.target.image_width = 0;
        break;
      case Mutation::kOutOfBounds:
        response.target.v_px = 480.0F;
        break;
      case Mutation::kWrongCandidateCount:
        response.target_candidate_count = 2;
        break;
      case Mutation::kTooOld:
        response.target.header.stamp = ros::Time::now() - ros::Duration(2.0);
        break;
    }
    return true;
  }
};

struct ConfiguredService {
  int mismatched_field=-1;
  std::string captured_config,captured_instance,captured_projection,captured_color;
  bool Handle(agx_pick_msg::SelectTubeTarget2DConfigured::Request& request,
              agx_pick_msg::SelectTubeTarget2DConfigured::Response& response) {
    captured_config=request.config_id;captured_instance=request.fixture_instance_id;
    captured_projection=request.projection_sha256;captured_color=request.target_cap_color;
    response.config_id=request.config_id;response.fixture_instance_id=request.fixture_instance_id;
    response.fixture_profile_id=request.fixture_profile_id;response.projection_sha256=request.projection_sha256;
    std::string* identities[]={&response.config_id,&response.fixture_instance_id,
                              &response.fixture_profile_id,&response.projection_sha256};
    if(mismatched_field>=0)*identities[mismatched_field]="different-context";
    agx_pick_msg::SelectTubeTarget2D::Request legacy;
    legacy.not_before=request.not_before;
    agx_pick_msg::SelectTubeTarget2D::Response payload;
    FakeService{}.Handle(legacy,payload);
    response.status=payload.status;response.detail=payload.detail;
    response.target=payload.target;response.target_candidate_count=payload.target_candidate_count;
    return true;
  }
};

void Require(bool condition, const std::string& detail) {
  if (!condition) {
    throw std::runtime_error(detail);
  }
}

}  // namespace

TEST(VisualTargetProvider, PreservesAndValidatesRosTransport) {
  ros::NodeHandle node;
  FakeService fake;
  auto server = node.advertiseService(
      "/select_tube_target_2d", &FakeService::Handle, &fake);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  agx_xarm_pick::RosTubeTargetProvider provider(
      &node, "table_grid_3x3_v1", "cam1_color_optical_frame", 0.2, 1.0);
  const std::uint64_t request_ns =
      (ros::Time::now() - ros::Duration(0.01)).toNSec();
  auto result = provider.Select(request_ns);
  Require(!(result.status != TargetProviderStatus::kSuccess ||
      fake.captured_profile != "table_grid_3x3_v1" ||
      fake.captured_not_before_ns != request_ns ||
      result.observation.frame_id != "cam1_color_optical_frame" ||
      result.observation.stamp_ns <= request_ns ||
      result.observation.u_px != 365.0 ||
      result.observation.v_px != 215.0 ||
      result.observation.reference_u_px != 319.5 ||
      result.observation.reference_v_px != 239.5 ||
      result.observation.image_width != 640 ||
      result.observation.image_height != 480),
      "valid target response was not preserved exactly");

  for (const auto mutation : {
           Mutation::kZeroStamp,
           Mutation::kEqualStamp,
           Mutation::kWrongFrame,
           Mutation::kNonFinite,
           Mutation::kZeroDimension,
           Mutation::kOutOfBounds,
           Mutation::kWrongCandidateCount,
           Mutation::kTooOld}) {
    fake.status = agx_pick_msg::SelectTubeTarget2D::Response::SUCCESS;
    fake.mutation = mutation;
    result = provider.Select(request_ns);
    Require(
        result.status == TargetProviderStatus::kInvalidObservation,
        "invalid success payload was accepted");
  }

  fake.mutation = Mutation::kNone;
  const std::vector<std::pair<std::uint8_t, TargetProviderStatus>> mappings = {
      {agx_pick_msg::SelectTubeTarget2D::Response::INVALID_REQUEST,
       TargetProviderStatus::kInvalidRequest},
      {agx_pick_msg::SelectTubeTarget2D::Response::TARGET_ABSENT,
       TargetProviderStatus::kAbsent},
      {agx_pick_msg::SelectTubeTarget2D::Response::AMBIGUOUS_TARGET,
       TargetProviderStatus::kAmbiguous},
      {agx_pick_msg::SelectTubeTarget2D::Response::STALE_FRAME,
       TargetProviderStatus::kStale},
      {agx_pick_msg::SelectTubeTarget2D::Response::PROFILE_MISMATCH,
       TargetProviderStatus::kProfileMismatch},
      {agx_pick_msg::SelectTubeTarget2D::Response::INCONSISTENT_TARGET,
       TargetProviderStatus::kInconsistent},
      {agx_pick_msg::SelectTubeTarget2D::Response::INVALID_OBSERVATION,
       TargetProviderStatus::kInvalidObservation},
  };
  for (const auto& mapping : mappings) {
    fake.status = mapping.first;
    result = provider.Select(request_ns);
    Require(
        result.status == mapping.second,
        "service status did not retain a distinct typed mapping");
  }

  server.shutdown();
  result = provider.Select(request_ns);
  Require(
      result.status == TargetProviderStatus::kUnavailable,
      "transport failure did not map to provider unavailable");
  spinner.stop();
}

TEST(VisualTargetProvider, PinsConfiguredProjectionAndRejectsAllMismatchedIdentities) {
  ros::NodeHandle node,private_node("~");std::string path;
  ASSERT_TRUE(private_node.getParam("configuration_fixture",path));
  const auto config=agx_xarm_pick::ResolveTubeTaskConfig(path);
  ConfiguredService service;
  auto server=node.advertiseService("/select_tube_target_2d_configured",&ConfiguredService::Handle,&service);
  ros::AsyncSpinner spinner(2);spinner.start();
  agx_xarm_pick::RosTubeTargetProvider provider(&node,config,"green");
  const auto cutoff=(ros::Time::now()-ros::Duration(.01)).toNSec();
  EXPECT_EQ(TargetProviderStatus::kSuccess,provider.Select(cutoff).status);
  EXPECT_EQ(config->config_id,service.captured_config);
  EXPECT_EQ(config->fixture_instance_id,service.captured_instance);
  EXPECT_EQ("green",service.captured_color);
  EXPECT_EQ(71u,service.captured_projection.size());
  EXPECT_EQ("sha256:",service.captured_projection.substr(0,7));
  EXPECT_EQ(std::string::npos,service.captured_projection.substr(7).find_first_not_of("0123456789abcdef"));
  for(int i=0;i<4;++i) {
    service.mismatched_field=i;
    EXPECT_EQ(TargetProviderStatus::kProfileMismatch,provider.Select(cutoff).status);
  }
  server.shutdown();spinner.stop();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tube_target_provider_rostest");
  return RUN_ALL_TESTS();
}
