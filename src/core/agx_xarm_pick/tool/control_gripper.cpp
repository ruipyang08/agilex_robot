#include <ros/ros.h>
#include <string>
#include <dh_gripper_msgs/GripperCtrl.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"control_gripper");
  ros::NodeHandle node("");
  ros::Publisher pub =node.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/ctrl",1);
  dh_gripper_msgs::GripperCtrl cmd;
  ros::Rate r(1);
  std::string x;
  while (ros::ok()) {
    r.sleep();
    std::cout << "1 open ; 0 close" << std::endl;
    std::cin >> x;
    if(!x.compare("q"))break;
    int d = atoi(x.c_str());
    cmd.initialize = false;
    cmd.position = d * 1000;
    cmd.force = 50;
    cmd.speed = 0;
    pub.publish(cmd);
    std::cout << ros::ok() << std::endl;
  }
  ros::shutdown();
}
