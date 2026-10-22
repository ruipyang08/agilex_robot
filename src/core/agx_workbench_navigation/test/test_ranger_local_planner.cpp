#include <agx_workbench_navigation/ranger_local_planner.hpp>
#include <agx_workbench_navigation/ranger_global_planner.hpp>
#include <agx_workbench_navigation/rover_execution.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace agx_workbench_navigation {
namespace {

TEST(CommandProvenance, InstalledTfPreservesNonzeroStampForSameMapFrame) {
  ros::Time::init();
  tf2_ros::Buffer buffer;
  geometry_msgs::PoseStamped original,transformed;
  original.header.frame_id="map";
  original.header.stamp=ros::Time(123,456);
  original.header.seq=19;
  original.pose.position.x=1.25;
  original.pose.orientation.w=1;
  // This is the transform overload used by installed move_base 1.17.3's
  // goalToGlobalFrame. The endpoint must already use the map frame.
  buffer.transform(original,transformed,"map",ros::Duration(0));
  EXPECT_EQ(transformed.header.stamp,original.header.stamp);
  EXPECT_EQ(transformed.header.frame_id,"map");
  EXPECT_DOUBLE_EQ(transformed.pose.position.x,original.pose.position.x);
}

TEST(RoverExecutionPolicy, GlobalAndSelectedTaskScaleReferenceLimits) {
  XmlRpc::XmlRpcValue policy;
  policy["velocity_scale"] = .8;
  policy["acceleration_scale"] = .6;
  policy["default_coefficient"] = .5;
  policy["task_coefficients"]["coarse_navigation"] = .25;
  const auto snapshot=snapshotRoverExecution(policy);
  EXPECT_DOUBLE_EQ(snapshot.velocity_scale,.8);
  EXPECT_DOUBLE_EQ(snapshot.acceleration_scale,.6);
  EXPECT_DOUBLE_EQ(snapshot.default_coefficient,.5);
  EXPECT_DOUBLE_EQ(snapshot.task_coefficients.at("coarse_navigation"),.25);
  EXPECT_EQ(snapshot.task_coefficients.size(),1u);
  auto scales = resolveRoverScales(policy, "coarse_navigation");
  EXPECT_DOUBLE_EQ(scales.first, .2);
  EXPECT_DOUBLE_EQ(scales.second, .15);
  EXPECT_DOUBLE_EQ(resolveRoverScales(policy, "rollback").first, .4);
  const auto valid = policy;
  for (const char* field : {"velocity_scale", "acceleration_scale", "default_coefficient"}) {
    for (const auto& invalid : {XmlRpc::XmlRpcValue(true), XmlRpc::XmlRpcValue("0.5"),
         XmlRpc::XmlRpcValue(0.0), XmlRpc::XmlRpcValue(-.1), XmlRpc::XmlRpcValue(1.1),
         XmlRpc::XmlRpcValue(std::numeric_limits<double>::infinity()),
         XmlRpc::XmlRpcValue(std::numeric_limits<double>::quiet_NaN())}) {
      policy = valid;
      policy[field] = invalid;
      EXPECT_THROW(resolveRoverScales(policy, "coarse_navigation"), std::invalid_argument);
      EXPECT_THROW(snapshotRoverExecution(policy), std::invalid_argument);
      policy = valid;
      policy["task_coefficients"]["rollback"] = invalid;
      EXPECT_THROW(resolveRoverScales(policy, "coarse_navigation"), std::invalid_argument);
    }
  }
  policy = valid;
  policy["task_coefficients"]["typo"] = .5;
  EXPECT_THROW(resolveRoverScales(policy, "coarse_navigation"), std::invalid_argument);
  policy = valid;
  policy["velocity_scale"] = 1e-300;
  policy["default_coefficient"] = 1e-300;
  EXPECT_THROW(resolveRoverScales(policy, "coarse_navigation"), std::invalid_argument);
}

}  // namespace
}  // namespace agx_workbench_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
