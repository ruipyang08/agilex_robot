#include "agx_xarm_pick/tube_grasp_config.hpp"
#include "agx_xarm_pick/tube_perception_projection.hpp"
#include "tube_config_parity.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <gtest/gtest.h>

TEST(TubeConfiguration, LegacySnapshotAndImmutableBinding) {
  ros::NodeHandle node;
  const auto frozen=agxcore::SnapshotLegacyTubeTaskConfig(node);
  const auto bound=agx_xarm_pick::BindTubeTaskMotion(*frozen,"green");
  assert(agxcore::ValidateTubeMotionConfig(bound));
  assert(bound.active_profile_id=="table_grid_3x3_v1");
  assert(bound.rows==3 && bound.columns==3 && bound.minimum_layout_inliers==6);
  assert(bound.table_surface_height_above_floor_m==0.985);
  assert(bound.arm_base_height_at_zero_lift_above_floor_m==0.885);
  assert(bound.gripper_force==30.0);
  assert((bound.gripper_speeds==std::array<double,3>{30.0,20.0,10.0}));
  assert(bound.lift_target_tolerance_mm==2);
  assert(bound.link6_to_grasp_tcp.translation.z==0.18);
  const auto projection=agx_xarm_pick::MakeTubePerceptionProjection(*frozen,"green");
  assert((projection.roi_normalized==std::array<double,4>{.10,.10,.95,.90}));
  assert(projection.minimum_layout_saturation==70);
  assert(projection.cap_diameter_m==bound.cap_diameter_m);
  const auto same=agxcore::SnapshotLegacyTubeTaskConfig(node);
  assert(same->config_id==frozen->config_id);

  node.setParam("/tube_grasp/motion/gripper_force",35.0);
  const auto changed=agxcore::SnapshotLegacyTubeTaskConfig(node);
  assert(changed->config_id!=frozen->config_id);
  assert(changed->motion.gripper_force==35.0);
  assert(frozen->motion.gripper_force==30.0);
  assert(bound.gripper_force==30.0);
  node.setParam("/tube_grasp/motion/gripper_force",0.0);
  bool rejected=false;
  try { agxcore::SnapshotLegacyTubeTaskConfig(node); }
  catch (const std::runtime_error&) { rejected=true; }
  assert(rejected);
  assert(agxcore::ValidateTubeMotionConfig(bound));
  node.setParam("/tube_grasp/motion/gripper_force",30.0);
  std::cout << "legacy snapshot and immutable binding: PASS\n";
}

TEST(TubeConfiguration, AllLegacyControlAndPerceptionFieldsMatchModularInput) {
  ros::NodeHandle node, private_node("~");
  std::string task_path;
  ASSERT_TRUE(private_node.getParam("parity_task_config",task_path));
  const auto legacy=agxcore::SnapshotLegacyTubeTaskConfig(node);
  const auto modular=agx_xarm_pick::ResolveTubeTaskConfig(task_path);
  auto legacy_motion=agx_xarm_pick::BindTubeTaskMotion(*legacy,"green");
  const auto modular_motion=agx_xarm_pick::BindTubeTaskMotion(*modular,"green");

  // Different admission paths intentionally retain different context labels,
  // identities and provenance. They are not physical/control differences.
  ASSERT_EQ("configured_demo",legacy_motion.context_source);
  ASSERT_EQ("resolved_installation",modular_motion.context_source);
  EXPECT_NE(legacy->config_id,modular->config_id);
  EXPECT_NE(legacy->task_config_id,modular->task_config_id);
  EXPECT_NE(legacy->fixture_instance_id,modular->fixture_instance_id);
  EXPECT_EQ(legacy->fixture_profile_id,modular->fixture_profile_id);
  ASSERT_EQ(1u,legacy->sources.size());
  ASSERT_EQ(7u,modular->sources.size());
  EXPECT_EQ("legacy_parameters",legacy->sources.front().kind);
  for (const auto& source:modular->sources) EXPECT_NE("legacy_parameters",source.kind);
  legacy_motion.context_source=modular_motion.context_source;
  for (const auto& group:tube_config_test::MotionDifferences(legacy_motion,modular_motion))
    ADD_FAILURE() << "legacy/modular control parity: " << group;

  const auto robot_fields=[](const auto& r) {
    return std::tie(r.planning_frame,r.color_frame,r.detector_service,
        r.joint_state_topic,r.gripper_command_topic,r.gripper_state_topic);
  };
  EXPECT_EQ(robot_fields(legacy->robot),robot_fields(modular->robot));
  EXPECT_NE(legacy->robot.id,modular->robot.id);
  EXPECT_NE(legacy->behavior.id,modular->behavior.id);
  EXPECT_EQ(legacy->behavior.task_type,modular->behavior.task_type);
  const auto& legacy_colors=legacy->behavior.color_classifiers;
  const auto& modular_colors=modular->behavior.color_classifiers;
  ASSERT_EQ(legacy_colors.size(),modular_colors.size());
  for (const auto& color:legacy_colors) {
    const auto other=modular_colors.find(color.first);
    ASSERT_NE(modular_colors.end(),other);
    EXPECT_EQ(color.second.hsv_lower,other->second.hsv_lower);
    EXPECT_EQ(color.second.hsv_upper,other->second.hsv_upper);
  }

  // Compare every wire field via the production canonical encoding. Use common
  // test identity arguments only for this content comparison, never for dispatch.
  const auto legacy_projection=agx_xarm_pick::MakeTubePerceptionProjection(*legacy,"green");
  const auto modular_projection=agx_xarm_pick::MakeTubePerceptionProjection(*modular,"green");
  EXPECT_EQ(agx_xarm_pick::TubePerceptionProjectionBytes(legacy_projection,"parity","rack","green"),
            agx_xarm_pick::TubePerceptionProjectionBytes(modular_projection,"parity","rack","green"));

  const auto container_fields=[](const auto& c) {
    return std::tie(c.body_diameter_m,c.body_height_m,c.cap_diameter_m,
        c.cap_height_m,c.diameter_tolerance_m,c.height_tolerance_m);
  };
  EXPECT_EQ(container_fields(legacy->container),container_fields(modular->container));
  EXPECT_EQ(legacy->support.frame_id,modular->support.frame_id);
  EXPECT_DOUBLE_EQ(legacy->support.surface_height_above_floor_m,modular->support.surface_height_above_floor_m);
  EXPECT_DOUBLE_EQ(legacy->support.height_tolerance_m,modular->support.height_tolerance_m);
  EXPECT_EQ(legacy->installation.parent_frame,modular->installation.parent_frame);
  EXPECT_TRUE(tube_config_test::TupleEqual(
      tube_config_test::TransformFields(legacy->installation.measured_transform),
      tube_config_test::TransformFields(modular->installation.measured_transform)));
  EXPECT_EQ(legacy->installation.registration_required,modular->installation.registration_required);
  EXPECT_EQ(legacy->installation.tag_id,modular->installation.tag_id);
  // Legacy never had physical rack slots/station identities/dimensions. Do not
  // manufacture them while checking its complete existing execution projection.
  EXPECT_TRUE(legacy->rack.slots.empty());
  EXPECT_TRUE(legacy->installation.station_id.empty());
  EXPECT_TRUE(legacy->support.station_id.empty());
}

int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  ros::init(argc,argv,"tube_config_snapshot_test");
  return RUN_ALL_TESTS();
}
