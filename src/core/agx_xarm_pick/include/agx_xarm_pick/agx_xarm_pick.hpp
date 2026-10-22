#ifndef AGXXARMPICK_HPP
#define AGXXARMPICK_HPP
#include <ros/ros.h>
#include <chrono>
#include <functional>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <queue>
#include <functional>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <mutex>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Geometry>
#include <queue>
#include <move_base_msgs/MoveBaseActionResult.h>
#include "agx_pick_msg/PickStateSrv.h"
#include "agx_pick_msg/AgxPickSrv.h"
#include "agx_pick_msg/OrderSrv.h"
#include "agx_pick_msg/TaskCmd.h"
#include "dh_gripper_msgs/GripperState.h"
#include "dh_gripper_msgs/GripperCtrl.h"
#include <tf/transform_listener.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include "agx_xarm_pick/mission_contract.hpp"

/*#define HOME_X 0.202
#define HOME_Y 0.116
#define HOME_Z 0.698*/
#define HOME_X 0.228
#define HOME_Y 0.0
#define HOME_Z 0.285
#define A_x 0
#define A_y 0
#define B_x 0
#define B_y 0
#define pub_a_goal 0
#define pick 1
#define pub_b_goal 2
#define place 3
#define calibration 4
#define ar_distance 0.6
#define search_dis 0.10
using namespace std;



namespace agxcore{

class AgxXarmPick{
  typedef enum
  {
    go2pick = 0,
    go2place = 1,
  }navstatus;

  typedef struct
  {
    uint navstatus = 0;
  }missionstatus;

public:
  explicit AgxXarmPick(ros::NodeHandle *nh,ros::AsyncSpinner *spinner);
  void ReadyPointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  bool MoveToA(geometry_msgs::Pose msg);
  void Gripper(int moterID,int pose,int force);
  bool Gripper(dh_gripper_msgs::GripperCtrl &cmd);
  bool OpenGripper();
  bool Setup();
  bool SetHomePose();
  bool PickAction();
  bool PlaceAction();
  bool GetPointMsg(int item,int handpose,int Prepose);//0--bottle;1--A;2--B
  void init();
  void RobotWork();
  void SetPlace(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b);
  void SetObstruction();
  int MovePath(vector<geometry_msgs::Pose> waypoints);
  void GenerateWayPoint(geometry_msgs::Pose beginpoint,geometry_msgs::Pose targetpoint,int flag);
  void GeneratePath(geometry_msgs::Pose beginpoint,geometry_msgs::Pose targetpoint,int flag);
  void GenerateFinalPath(geometry_msgs::Pose beginpoint,geometry_msgs::Pose targetpoint);
  void settestpoint();
  bool ReturnReadyPose();
  bool set_goal_signal(agx_pick_msg::OrderSrv::Request &req,
                       agx_pick_msg::OrderSrv::Response &res);

  bool TaskCallback(agx_pick_msg::TaskCmd::Request &req,
                    agx_pick_msg::TaskCmd::Response &res);
  void SetGoal(geometry_msgs::PoseStamped msg);
  void CatchAppleAction();
  void CorrectionHandPose();
  void GoCatch();
  void RemoveObject();

  void Calibration(geometry_msgs::Pose targetpoint);
  bool SearchMaker();
  void MoveToCatchPose();
  bool ExecuteMission();
  MissionStatus RunStandaloneMission(int marker_id, bool open_before_mission);
  MissionStatus last_status() const { return last_status_; }
  void InitMoveit()
  {
    static const std::string PLANNING_GROUP  = "manipulator_i5";


    // Create a planning group interface object and set up a planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("arm_base");


    // Create a planning scene interface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create an object of the visualization class
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("arm_base");
    visual_tools.deleteAllMarkers();
  };

  int count = 0;
  bool data_flag = false;
  bool work_flag = false;
  int flag_grap = 0;
  int flag_cal = 0;		//
  int task_states = 0;
  int task_identified_tag = 0; //该任务要识别的标签
  enum Realorpre{REAL,PRE};
  enum Prepose{FRONT,UPSIDE};
  int object_num = 0;
  int place_num = 2;
  navstatus status;
  missionstatus mstatus;
//  agx_xarm_pick::PickStateSrv pick_date;
  ros::ServiceClient client;
  ros::ServiceClient pont_client;
  ros::ServiceServer goal_service;
  ros::ServiceServer task_service;
  geometry_msgs::PoseStamped A;//抓取点
  geometry_msgs::PoseStamped B;//投放点
  geometry_msgs::PoseStamped pick_place;//抓取点
  geometry_msgs::PoseStamped drop_place;//投放点
  geometry_msgs::PoseStamped base_link_state;
  std::queue<geometry_msgs::PoseStamped> q;
  std::vector<geometry_msgs::Pose> waypoints;
  dh_gripper_msgs::GripperState *gripperstate_;

  enum Item{OBJECT,PICK,PLACE,APPLE,APPLE_CAM};
  enum Route{go_ahead,go_back,correct};
  enum grip_states{active,stop,grap};
  bool first_flag = false;
private:
  MissionStatus FinishStandaloneFailure(bool attempt_home_recovery);
  void GripperStateCallback(const dh_gripper_msgs::GripperState::ConstPtr &msg);
  bool WaitForGripperState(
      int expected_state,
      const ros::Time &issued_ros_time,
      const std::chrono::steady_clock::time_point &issued_steady_time);

  ros::NodeHandle *nh_;
  ros::AsyncSpinner *spinner_;
 /*--------define publisher---------*/
  ros::Publisher pub;
  ros::Publisher goal;
  ros::Publisher vel;
  ros::Publisher grip_ctl_pub_;

/*--------define subcriber---------*/
  ros::Subscriber sub;
  ros::Subscriber grip_state_sub_;

  geometry_msgs::Pose temp;
  geometry_msgs::Pose prepose;
  ros::Subscriber goal_state;
  geometry_msgs::Pose home_pose;
  bool base_flag = false;
  std::mutex base_state_mutex_;
  std::mutex gripper_state_mutex_;
  dh_gripper_msgs::GripperState latest_gripper_state_;
  bool has_gripper_state_ = false;
  ros::Time latest_gripper_ros_time_;
  std::chrono::steady_clock::time_point latest_gripper_steady_time_;
  double setup_timeout_sec_ = 10.0;
  double moveit_timeout_sec_ = 10.0;
  double gripper_timeout_sec_ = 15.0;
  double marker_search_timeout_sec_ = 30.0;
  double marker_settle_sec_ = 2.0;
  double cartesian_velocity_scaling_ = 0.1;
  double cartesian_acceleration_scaling_ = 1.0;
  double post_grasp_lift_m_ = 0.06;
  int max_search_attempts_ = 6;
  MissionStatus last_status_ = MissionStatus::kSuccess;
  int waypoint_num = 5;
  // std::mutex goal_state_mutex_
  int x=0;
  const float theta=(00/180.0)*M_PI;

};

}
#endif // AGXXARMPICK_HPP
