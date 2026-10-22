#ifndef MARKLISTENNER_HPP
#define MARKLISTENNER_HPP
#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include "agx_pick_msg/AgxPickSrv.h"
#include "agx_pick_msg/PickStateSrv.h"
#include <array>
#include <chrono>
#include <map>

#define HOME_X -0.15
#define HOME_Y -0.29
#define HOME_Z 0.66
#define PRE_OFFSET_Y 0.28
#define PRE_OFFSET_Z 0.06
#define PLACE_PRE_OFFSET_Y 0.30
#define PLACE_OFFSET_Z 0.12
#define GRIPPER_LENGTH 0.18
#define TABLE_Z 0.16
#define TABLE_Y 0.40
#define HAND_BASE_LEN 0.15
#define FILLTER_N 3

const float theta=(90/180.0)*M_PI;

namespace agxcore{
class MarkListenner{
public:
  explicit MarkListenner(ros::NodeHandle *nh);
  void Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void GetPickupPoint(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void PubPickupPoint(geometry_msgs::Pose &msg);
  bool MarkCallback(agx_pick_msg::AgxPickSrv::Request &req,agx_pick_msg::AgxPickSrv::Response &res);
  int CountData(int i,int id);
  void SetPose(geometry_msgs::Pose msg);
  void text(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void yolo_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  int GetPickItemType(std::string neme);
 // void Clear(T msg);
  void Clear(geometry_msgs::PoseStamped *msg);
  void Clear(geometry_msgs::Pose *msg);
  void ClearMarkState();
  enum Item{OBJECT,PICK,PLACE,APPLE,APPLE_CAM};
  enum { ITEM_COUNT = 5 };
  enum Realorpre{REAL,PRE};
  enum Prepose{FRONT,UPSIDE};
  void Setup();
  void initial(geometry_msgs::PoseStamped msg[]);
  void initial(geometry_msgs::Pose msg[]);
  ros::ServiceServer service;



private:
  bool has_fresh_marker(int item) const;
  void reset_stale_partial(
      int item,
      const std::chrono::steady_clock::time_point &now);

  ros::NodeHandle *nh_;
  ros::Subscriber mark;
  ros::Subscriber place_mark;
  ros::Subscriber yolo_sub_;
  ros::Subscriber mark2cam_;
  ros::Publisher pub;
  geometry_msgs::PoseStamped temp[ITEM_COUNT];
  geometry_msgs::Pose real_position[ITEM_COUNT];
  std::map<int,bool> is_mark_empty;
  std::array<std::chrono::steady_clock::time_point, ITEM_COUNT> last_marker_arrival_;
  std::array<bool, ITEM_COUNT> has_marker_arrival_;
  std::array<std::chrono::steady_clock::time_point, ITEM_COUNT> last_partial_arrival_;
  std::array<bool, ITEM_COUNT> has_partial_arrival_;
  double marker_freshness_timeout_sec_;
  // Distance the fingertip pads advance past a front-facing marker plane so
  // the target lies between the jaws instead of merely touching their tips.
  // Keep this parameterized for different real and simulated object depths.
  double front_grasp_insertion_m_ = 0.015;
  geometry_msgs::Pose point_msg;

};

}
#endif // MARKLISTENNER_HPP
