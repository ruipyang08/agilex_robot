#include "agx_xarm_pick/tube_grasp_mission.hpp"
#include <ros/ros.h>
#include <stdexcept>
#include <string>
#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tube_speed_config_rejection_test");
  ros::NodeHandle node;
  try {
    agxcore::TubeGraspMission mission(&node);
  } catch (const std::runtime_error& error) {
    if (std::string(error.what()) == "invalid /tube_grasp configuration") {
      std::cout << "invalid speed configuration rejected before actuator setup: PASS\n";
      return 0;
    }
  }
  return 1;
}
