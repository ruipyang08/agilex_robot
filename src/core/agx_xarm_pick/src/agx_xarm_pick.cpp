#include "agx_xarm_pick/agx_xarm_pick.hpp"


//#include <signal.h>
#include <iostream>
#include <csignal>
#include <unistd.h>
using namespace agxcore;
using namespace std;

void Stop(int signal)
{
  terminate();
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"agx_xarm_pick");
  ros::NodeHandle node("");
  ros::AsyncSpinner s(4);
  AgxXarmPick robot(&node,&s);
  signal(SIGINT, Stop);
  robot.Setup();
  //robot.SetObstruction();
  robot.SetHomePose();//move arms into ready pose
  robot.OpenGripper();
  ros::Rate r(50);
  robot.init();
  ROS_INFO("runing");

 
  //robot.GetPointMsg(0,1,0);
  //robot.task_identified_tag = 0;
  //robot.ExecuteMission();

  while(ros::ok())
  {

    //robot.SetPlace(robot.A,robot.B);
    //robot.RobotWork();
    //robot.SetPlace(robot.B,robot.A);
    //robot.RobotWork();

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
