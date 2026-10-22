#include "mark_listenner/mark_listenner.hpp"
#include <algorithm>
#include <math.h>
using namespace agxcore;



MarkListenner::MarkListenner(ros::NodeHandle *nh) : nh_(nh){}

void MarkListenner::Setup()
{
   nh_->param("marker_freshness_timeout_sec", marker_freshness_timeout_sec_, 1.0);
   nh_->param("front_grasp_insertion_m", front_grasp_insertion_m_, front_grasp_insertion_m_);
   if(marker_freshness_timeout_sec_ <= 0.0)
   {
     ROS_WARN("marker_freshness_timeout_sec must be positive; using 1.0 second");
     marker_freshness_timeout_sec_ = 1.0;
   }
   front_grasp_insertion_m_ = std::max(0.0, std::min(
       front_grasp_insertion_m_, 0.05));
   has_marker_arrival_.fill(false);
   has_partial_arrival_.fill(false);

   mark = nh_->subscribe<ar_track_alvar_msgs::AlvarMarkers>("/cam1/ar_pose_marker",5,&MarkListenner::Callback,this);
   place_mark = nh_->subscribe<ar_track_alvar_msgs::AlvarMarkers>("/cam2/ar_pose_marker",5,&MarkListenner::Callback,this);

   //pub = nh_->advertise<geometry_msgs::Pose>("pick_point",50);
   service = nh_->advertiseService("pick_point",&MarkListenner::MarkCallback,this);
   yolo_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>("yolo3_pose",5,&MarkListenner::yolo_callback,this);
   mark2cam_ = nh_->subscribe<geometry_msgs::PoseStamped>("mark2cam",5,&MarkListenner::yolo_callback,this);
   initial(temp);
   initial(real_position);
   ClearMarkState();
}

void MarkListenner::yolo_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  int type = GetPickItemType(msg->header.frame_id);
  if(type < 0 || type >= ITEM_COUNT)
  {
    ROS_WARN_THROTTLE(3, "Ignoring unknown detector frame '%s'", msg->header.frame_id.c_str());
    return;
  }
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  reset_stale_partial(type, now);
  temp[type].header.seq++;
  temp[type].pose.position.x += msg->pose.position.x;
  temp[type].pose.position.y += msg->pose.position.y;
  temp[type].pose.position.z += msg->pose.position.z;
  if (temp[type].header.seq == FILLTER_N)
  {
    real_position[type].position.x = temp[type].pose.position.x / FILLTER_N;
    real_position[type].position.y = temp[type].pose.position.y / FILLTER_N;
    real_position[type].position.z = temp[type].pose.position.z / FILLTER_N;

    Clear(&temp[type]);
    has_partial_arrival_[type] = false;
    is_mark_empty[type] = false;
    last_marker_arrival_[type] = now;
    has_marker_arrival_[type] = true;
  }
  else
  {
    last_partial_arrival_[type] = now;
    has_partial_arrival_[type] = true;
  }
}

int MarkListenner::GetPickItemType(std::string name)
{
  if(std::strcmp(name.c_str(),"apple") == 0)
  {
    return APPLE;
  }
  if(std::strcmp(name.c_str(),"apple_to_cam") == 0)
  {
    return APPLE_CAM;
  }
  return -1;
}

bool MarkListenner::has_fresh_marker(int item) const
{
  if(item < 0 || item >= ITEM_COUNT || !has_marker_arrival_[item])
  {
    return false;
  }
  const std::chrono::duration<double> age =
      std::chrono::steady_clock::now() - last_marker_arrival_[item];
  return age.count() <= marker_freshness_timeout_sec_;
}

void MarkListenner::reset_stale_partial(
    int item,
    const std::chrono::steady_clock::time_point &now)
{
  if(item < 0 || item >= ITEM_COUNT || !has_partial_arrival_[item])
  {
    return;
  }
  const std::chrono::duration<double> age = now - last_partial_arrival_[item];
  if(age.count() > marker_freshness_timeout_sec_)
  {
    Clear(&temp[item]);
    has_partial_arrival_[item] = false;
  }
}

bool MarkListenner::MarkCallback(agx_pick_msg::AgxPickSrv::Request &req,agx_pick_msg::AgxPickSrv::Response &res)
{
  int item_num = req.item;
  const double front_gripper_length =
      GRIPPER_LENGTH - front_grasp_insertion_m_;
  ROS_INFO("item_num:%d",item_num);

  //判断获取的标签是否不在当前的摄像头可视范围之内
  if(item_num < 0 || item_num >= ITEM_COUNT ||
     is_mark_empty[item_num] || !has_fresh_marker(item_num))
  {
    ROS_WARN("No fresh, complete observation for item %d", item_num);
    return false;
  }

  //根据标签id和夹爪姿势返回坐标
  switch(item_num)
  {
    case OBJECT:
    {
      if(req.Prepose == FRONT && req.handpose == PRE)
      {
          ROS_INFO("FRONT,PRE");
          res.position.y = real_position[item_num].position.y - PRE_OFFSET_Y*cos(theta); 
          res.position.x = real_position[item_num].position.x - PRE_OFFSET_Y*sin(theta);                 
          res.position.z = real_position[item_num].position.z - PRE_OFFSET_Z;

          ROS_INFO("%f",real_position[item_num].position.x);
          ROS_INFO("%f",real_position[item_num].position.y);
          ROS_INFO("%f",real_position[item_num].position.z);
      }
      else if (req.Prepose == UPSIDE && req.handpose == PRE) {
          ROS_INFO("UPSIDE,PRE");
          res.position.z = real_position[item_num].position.z + PRE_OFFSET_Z;
          res.position.y = real_position[item_num].position.y + PRE_OFFSET_Y;
      }
      else {
          ROS_INFO("else");
          res.position.y = real_position[item_num].position.y - front_gripper_length*cos(theta);
          res.position.x = real_position[item_num].position.x - front_gripper_length*sin(theta);
          res.position.z = real_position[item_num].position.z;
      }

    }break;
    case PICK:
    {
      res.position.y = real_position[item_num].position.y;
      res.position.z = real_position[item_num].position.z;
    }break;
    case PLACE:
    {
      if(req.Prepose == FRONT && req.handpose == PRE)
      {
          ROS_INFO("FRONT,PRE");
          res.position.y = real_position[item_num].position.y - PLACE_PRE_OFFSET_Y*cos(theta); 
          res.position.x = real_position[item_num].position.x - PLACE_PRE_OFFSET_Y*sin(theta);         
          res.position.z = real_position[item_num].position.z - PRE_OFFSET_Z;

          ROS_INFO("%f",real_position[item_num].position.x);
          ROS_INFO("%f",real_position[item_num].position.y);
          ROS_INFO("%f",real_position[item_num].position.z);
      }
      else if (req.Prepose == UPSIDE && req.handpose == PRE) {
          ROS_INFO("UPSIDE,PRE");
          res.position.z = real_position[item_num].position.z + PRE_OFFSET_Z;
          res.position.y = real_position[item_num].position.y + PRE_OFFSET_Y;
      }
      else {
          ROS_INFO("else");
          res.position.y = real_position[item_num].position.y - front_gripper_length*cos(theta);
          res.position.x = real_position[item_num].position.x - front_gripper_length*sin(theta);
          res.position.z = real_position[item_num].position.z + PLACE_OFFSET_Z;
      }
    }break;
    case APPLE:
    {
      if(req.Prepose == FRONT && req.handpose == PRE)
        res.position.x = real_position[item_num].position.x - PRE_OFFSET_Y;
      else

      res.position.y = real_position[item_num].position.y;

      if(req.Prepose == UPSIDE && req.handpose == PRE)
        res.position.z = real_position[item_num].position.z + PRE_OFFSET_Z;
      else
        res.position.z = real_position[item_num].position.z;

    }break;
    case APPLE_CAM:
    {
      res.position.y = real_position[item_num].position.y;
      res.position.z = real_position[item_num].position.z;
    }break;
    default:
    {
      ROS_INFO("use no valid num");
      return false;
    };
  }

  // res.position.x = real_position[item_num].position.x;

  Clear(&real_position[item_num]);
  is_mark_empty[item_num] = true;
  has_marker_arrival_[item_num] = false;
  tf::Quaternion q;
  if(req.Prepose == FRONT)
    // q.setRPY(1.8486912250518799, -1.570712924003601, 1.8164114952087402);
    q.setRPY(-0.5838785767555237, -1.5707074403762817, -2.557819366455078);
  else
    q.setRPY(3.14,0,0);
  res.orientation.x = q.getX();
  res.orientation.y = q.getY();
  res.orientation.z = q.getZ();
  res.orientation.w = q.getW();
  return true;
}

void MarkListenner::Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    if(msg->markers.size() > 0)
    {
        GetPickupPoint(msg);
    }
    else
    {
        ROS_DEBUG_STREAM_THROTTLE(3, "no data!");
    }

}

void MarkListenner::GetPickupPoint(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  for(int i=0;i<msg->markers.size();i++)
  {
    int id_num = msg->markers[i].id;
    if(id_num < OBJECT || id_num > PLACE)
    {
      ROS_WARN_THROTTLE(3, "Ignoring unsupported AR marker id %d", id_num);
      continue;
    }
    //ROS_INFO("%d(%f,%f,%f)",msg->markers[i].id,msg->markers[i].pose.pose.position.x,msg->markers[i].pose.pose.position.y,msg->markers[i].pose.pose.position.z);
    //only save dates of marks which id is 0,1,2
    switch(id_num)
    {
      case OBJECT:
      case PICK:
      case PLACE:
      {
        const std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now();
        reset_stale_partial(id_num, now);
        temp[id_num].pose.position.x += msg->markers[i].pose.pose.position.x;
        temp[id_num].pose.position.y += msg->markers[i].pose.pose.position.y;
        temp[id_num].pose.position.z += msg->markers[i].pose.pose.position.z;
        temp[id_num].header.seq++;
        if(temp[id_num].header.seq == FILLTER_N)
        {
          real_position[id_num].position.x = temp[id_num].pose.position.x / 3;
          real_position[id_num].position.y = temp[id_num].pose.position.y / 3;
          real_position[id_num].position.z = temp[id_num].pose.position.z / 3;
          ROS_INFO("%d:%f,%f,%f",id_num,real_position[id_num].position.x,real_position[id_num].position.y,real_position[id_num].position.z);
          Clear(&temp[id_num]);
          has_partial_arrival_[id_num] = false;
          is_mark_empty[id_num] = false;
          last_marker_arrival_[id_num] = now;
          has_marker_arrival_[id_num] = true;
        }
        else
        {
          last_partial_arrival_[id_num] = now;
          has_partial_arrival_[id_num] = true;
        }
      }break;
      default:
      {
        ROS_INFO("Invalid number ");
      }break;
    }
  }
}

//void MarkListenner::PubPickupPoint(geometry_msgs::Pose &msg)
//{
//  tf::Quaternion q;
//  q.setRPY(HOME_X,HOME_Y,HOME_Z);
//  msg.orientation.x = q.x();
//  msg.orientation.y = q.y();
//  msg.orientation.z = q.z();
//  msg.orientation.w = q.w();
//  point_msg.position.x = msg.position.x;
//  point_msg.position.y = msg.position.y;
//  point_msg.position.z = msg.position.z;
//  point_msg.orientation.x = msg.orientation.x;
//  point_msg.orientation.y = msg.orientation.y;
//  point_msg.orientation.z = msg.orientation.z;
//  point_msg.orientation.w = msg.orientation.w;


//  //pub.publish(msg);
//}

void MarkListenner::initial(geometry_msgs::PoseStamped msg[])
{
  for(int i=0;i<ITEM_COUNT;i++)
  {
    Clear(&msg[i]);
  }
}

void MarkListenner::initial(geometry_msgs::Pose msg[])
{
  for(int i=0;i<ITEM_COUNT;i++)
  {
    Clear(&msg[i]);
  }
}

void MarkListenner::SetPose(geometry_msgs::Pose msg)
{
  tf::Quaternion q;
  q.setRPY(1.57,0,0);
  msg.orientation.x = q.getX();
  msg.orientation.y = q.getY();
  msg.orientation.z = q.getZ();
  msg.orientation.w = q.getW();
}

void MarkListenner::Clear(geometry_msgs::PoseStamped *msg)
{

  msg->pose.position.x = 0;
  msg->pose.position.y = 0;
  msg->pose.position.z = 0;
  msg->header.seq = 0;
}

void MarkListenner::Clear(geometry_msgs::Pose *msg)
{
  msg->position.x = 0;
  msg->position.y = 0;
  msg->position.z = 0;
}

void MarkListenner::ClearMarkState()
{
  for(int i = 0;i < ITEM_COUNT;i++)
  {
     is_mark_empty[i] = true;
     has_marker_arrival_[i] = false;
     has_partial_arrival_[i] = false;
  }
}
