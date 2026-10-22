#include <ros/ros.h>
#include <string>
#include "agx_pick_msg/AgxPickSrv.h"
#include "agx_pick_msg/OrderSrv.h"
#include <tf/LinearMath/Quaternion.h>

int main(int argc, char** argv )
{
  ros::init(argc,argv,"test");
  ros::NodeHandle node("");
  ros::ServiceClient client = node.serviceClient<agx_pick_msg::OrderSrv>("goal_service");
  double A_x,A_y,A_z,A_qx,A_qy,A_qz,A_qw;
  double B_x,B_y,B_z,B_qx,B_qy,B_qz,B_qw;
  node.getParam("test/A_x",A_x);
  node.getParam("test/A_y",A_y);
  node.getParam("test/A_z",A_z);
  node.getParam("test/A_qx",A_qx);
  node.getParam("test/A_qy",A_qy);
  node.getParam("test/A_qz",A_qz);
  node.getParam("test/A_qw",A_qw);
  node.getParam("test/B_x",B_x);
  node.getParam("test/B_y",B_y);
  node.getParam("test/B_z",B_z);
  node.getParam("test/B_qx",B_qx);
  node.getParam("test/B_qy",B_qy);
  node.getParam("test/B_qz",B_qz);
  node.getParam("test/B_qw",B_qw);
  agx_pick_msg::OrderSrv srv;
  geometry_msgs::PoseStamped a;
  geometry_msgs::PoseStamped b;

  tf::Quaternion q;
  q.setRPY(0,0,1.12224669);
  srv.request.id = 0;
  a.header.frame_id = "world";
  a.pose.position.x = A_x;
  a.pose.position.y = A_y;
  a.pose.position.z = A_z;
  a.pose.orientation.x = A_qx;
  a.pose.orientation.y = A_qy;
  a.pose.orientation.z = A_qz;
  a.pose.orientation.w = A_qw;
  srv.request.A_goal = a;

  q.setRPY(0,0,4.39822964);
  b.header.frame_id = "world";
  b.pose.position.x = B_x;
  b.pose.position.y = B_y;
  b.pose.position.z = B_z;
  b.pose.orientation.x = B_qx;
  b.pose.orientation.y = B_qy;
  b.pose.orientation.z = B_qz;
  b.pose.orientation.w = B_qw;
  srv.request.B_goal = b;
  ROS_INFO("test");
  client.call(srv);
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
