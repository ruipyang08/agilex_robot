#include <ros/ros.h>
#include <string>
#include "agx_pick_msg/AgxPickSrv.h"
#include "agx_pick_msg/AgxPickSrv.h"
#include <tf/LinearMath/Quaternion.h>

int main(int argc, char** argv )
{
  ros::init(argc,argv,"test");
  ros::NodeHandle node("");
  ros::ServiceClient pont_client = node.serviceClient<agx_pick_msg::AgxPickSrv>("pick_point");
  agx_pick_msg::AgxPickSrv srv;
  srv.request.item = 0;
  srv.request.handpose = 1;
  srv.request.Prepose =0;
  if(pont_client.call(srv))
  {
    ROS_INFO("%f",srv.response.position.x);
    ROS_INFO("%f",srv.response.position.y);
    ROS_INFO("%f",srv.response.position.z);
  }
  else {
    ROS_INFO("can not see this mark!!");
  }

  return 0;
}
