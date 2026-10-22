#include "agx_xarm_pick/tube_grasp_mission.hpp"

#include <cstdlib>
#include <exception>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_tube_search");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  std::string fixture_profile_id;
  std::string target_cap_color;
  private_node.param<std::string>(
      "fixture_profile_id",
      fixture_profile_id,
      "table_grid_3x3_v1");
  private_node.param<std::string>(
      "target_cap_color", target_cap_color, "green");

  ros::AsyncSpinner spinner(4);
  spinner.start();
  try {
    agxcore::TubeGraspMission mission(&node);
    const auto status =
        mission.RunSearchOnly(fixture_profile_id, target_cap_color);
    const int exit_code = agxcore::tube_mission_exit_code(status);
    if (exit_code == EXIT_SUCCESS) {
      ROS_INFO(
          "demo_tube_search completed successfully; no grasp was attempted");
    } else {
      ROS_ERROR(
          "demo_tube_search failed: %s; no grasp was attempted",
          agxcore::tube_mission_status_name(status));
    }
    spinner.stop();
    return exit_code;
  } catch (const std::exception& error) {
    ROS_ERROR(
        "demo_tube_search aborted by exception; "
        "no grasp was attempted: %s",
        error.what());
    spinner.stop();
    return EXIT_FAILURE;
  }
}
