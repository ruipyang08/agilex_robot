#include "agx_xarm_pick/tube_grasp_mission.hpp"
#include "agx_xarm_pick/tube_mission_runner.hpp"
#include <csignal>

#include <cstdlib>
#include <exception>
#include <string>

static volatile std::sig_atomic_t interrupted = 0;
static void Signal(int) { interrupted = 1; }
int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_tube_pick", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, Signal);
  std::signal(SIGTERM, Signal);
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  std::string fixture_profile_id;
  std::string target_cap_color;
  private_node.param<std::string>(
      "fixture_profile_id",
      fixture_profile_id,
      "table_grid_3x3_v1");
  private_node.param<std::string>("target_cap_color", target_cap_color, "green");

  ros::AsyncSpinner spinner(4);
  spinner.start();
  try {
    const auto result = agxcore::RunTubeMissionMonitored(
        node, fixture_profile_id, target_cap_color, [] { return !interrupted; });
    const auto status = result.status;
    const int exit_code = agxcore::tube_mission_exit_code(status);
    if (exit_code == EXIT_SUCCESS) {
      ROS_INFO("demo_tube_pick completed successfully");
    } else {
      ROS_ERROR(
          "demo_tube_pick failed: %s",
          agxcore::tube_mission_status_name(status));
    }
    if (result.exit_required) { std::fflush(nullptr); std::_Exit(EXIT_FAILURE); }
    spinner.stop();
    return exit_code;
  } catch (const std::exception& error) {
    ROS_ERROR(
        "demo_tube_pick aborted by exception; gripper command unchanged: %s",
        error.what());
    spinner.stop();
    return EXIT_FAILURE;
  }
}
