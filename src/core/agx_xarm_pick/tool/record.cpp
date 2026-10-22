#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"record");
  ros::NodeHandle node("");
  tf::TransformListener listener;
  tf::StampedTransform transform_1;
  tf::StampedTransform transform_2;
  std::cout<<"recording first place"<<std::endl;
  getchar();
  try{
	listener.waitForTransform("/dummy_base_link", "/world",ros::Time(0),ros::Duration(10.0));
        listener.lookupTransform("/world", "/dummy_base_link",
                                 ros::Time(0), transform_1);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
  std::cout<<"recording second place"<<std::endl;
  getchar();
  try{
	listener.waitForTransform("/dummy_base_link", "/world",ros::Time(0),ros::Duration(10.0));
        listener.lookupTransform("/world", "/dummy_base_link",
                                 ros::Time(0), transform_2);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

  FILE *f =fopen("place.yaml", "w");

  fprintf(f, "A_x: %f\nA_y: %f\nA_z: %f\nA_qx: %f\nA_qy: %f\nA_qz: %f\nA_qw: %f\nB_x: %f\nB_y: %f\nB_z: %f\nB_qx: %f\nB_qy: %f\nB_qz: %f\nB_qw: %f\n\n",
          transform_1.getOrigin().getX(),transform_1.getOrigin().getY(),transform_1.getOrigin().getZ(),
          transform_1.getRotation().x(),transform_1.getRotation().y(),transform_1.getRotation().z(),transform_1.getRotation().w(),
          transform_2.getOrigin().getX(),transform_2.getOrigin().getY(),transform_2.getOrigin().getZ(),
          transform_2.getRotation().x(),transform_2.getRotation().y(),transform_2.getRotation().z(),transform_2.getRotation().w());

  fclose(f);
}
