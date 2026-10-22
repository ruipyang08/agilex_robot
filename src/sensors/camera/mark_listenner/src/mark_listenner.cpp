#include "mark_listenner/mark_listenner.hpp"

using namespace agxcore;

int main(int argc, char** argv)
{
  ros::init(argc,argv,"mark_listener");
  ros::NodeHandle node("");
  MarkListenner ml(&node);
  //ml.initial();
  ml.Setup();
  ros::Rate r(10);
  ROS_INFO("start");
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
