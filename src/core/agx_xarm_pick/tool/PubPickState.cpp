#include <ros/ros.h>
#include <string>
#include <dh_gripper_msgs/GripperState.h>
#include <tf/LinearMath/Quaternion.h>

int main(int argc, char** argv )
{
  ros::init(argc,argv,"test");
  ros::NodeHandle node("");
  ros::Publisher pub =node.advertise<dh_gripper_msgs::GripperState>("/gripper/state",1);
  dh_gripper_msgs::GripperState msg;
  int state;
  ros::Rate r(0.5);
  r.sleep();
  std::cout<<"input gripper state"<<std::endl;
  std::cout<<"1 stop ; 0 active ; 2 grap"<<std::endl;
  std::cin >> state;
  msg.grip_state = state;
  pub.publish(msg);
  ros::shutdown();
  return 0;
}
