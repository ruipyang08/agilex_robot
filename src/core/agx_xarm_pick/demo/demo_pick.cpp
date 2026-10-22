#include "agx_xarm_pick/agx_xarm_pick.hpp"

#include <cstdlib>
using namespace agxcore;

int main(int argc, char** argv)
{
  ros::init(argc,argv,"demo_pick");
  ROS_INFO_STREAM("demo_pick");
  ros::NodeHandle node("");
  ros::AsyncSpinner s(4);
  AgxXarmPick robot(&node,&s);
  const MissionStatus status = robot.RunStandaloneMission(0, true);
  const int exit_code = mission_exit_code(status);
  if (exit_code == EXIT_SUCCESS)
  {
    ROS_INFO("demo_pick completed successfully");
    return EXIT_SUCCESS;
  }
  ROS_ERROR("demo_pick failed: %s", mission_status_name(status));
  return EXIT_FAILURE;
}
