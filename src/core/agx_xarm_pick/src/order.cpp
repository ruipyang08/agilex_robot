#include <ros/ros.h>
#include <string>
#include <move_base_msgs/MoveBaseActionResult.h>

int main(int argc, char** argv )
{
  ros::init(argc,argv,"testorder");
  ros::NodeHandle node("");
  move_base_msgs::MoveBaseActionResult msg;
  ros::Publisher pub = node.advertise<move_base_msgs::MoveBaseActionResult>("/move_base/result",1);
  msg.status.status = 3;
  ros::Rate r(0.3);
  r.sleep();
  ROS_INFO("test");
  pub.publish(msg);
//  r.sleep();
//  if (client.call(srv))
//  {
//    ROS_INFO("call once");
//  }
//  else
//  {
//    ROS_ERROR("Failed to call service ");
//    return 1;
//  }

  return 0;
}
