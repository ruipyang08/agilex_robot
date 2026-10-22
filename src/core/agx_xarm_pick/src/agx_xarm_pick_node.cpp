#include "agx_xarm_pick/agx_xarm_pick.hpp"
#include <algorithm>
#include<cmath>

using namespace agxcore;
using namespace std;
//ros::AsyncSpinner spinner(1);
AgxXarmPick::AgxXarmPick(ros::NodeHandle *nh,ros::AsyncSpinner *spinner) : nh_(nh),spinner_(spinner){}


ros::Duration d(3);
bool AgxXarmPick::Setup()
{
   nh_->param("setup_timeout_sec", setup_timeout_sec_, setup_timeout_sec_);
   nh_->param("moveit_timeout_sec", moveit_timeout_sec_, moveit_timeout_sec_);
   nh_->param("gripper_timeout_sec", gripper_timeout_sec_, gripper_timeout_sec_);
   nh_->param("marker_search_timeout_sec", marker_search_timeout_sec_, marker_search_timeout_sec_);
   nh_->param("marker_settle_sec", marker_settle_sec_, marker_settle_sec_);
   nh_->param("cartesian_velocity_scaling", cartesian_velocity_scaling_, cartesian_velocity_scaling_);
   nh_->param("cartesian_acceleration_scaling", cartesian_acceleration_scaling_, cartesian_acceleration_scaling_);
   nh_->param("post_grasp_lift_m", post_grasp_lift_m_, post_grasp_lift_m_);
   nh_->param("max_search_attempts", max_search_attempts_, max_search_attempts_);
   setup_timeout_sec_ = std::max(0.1, setup_timeout_sec_);
   moveit_timeout_sec_ = std::max(0.1, moveit_timeout_sec_);
   gripper_timeout_sec_ = std::max(0.1, gripper_timeout_sec_);
   marker_search_timeout_sec_ = std::max(0.1, marker_search_timeout_sec_);
   marker_settle_sec_ = std::max(0.0, marker_settle_sec_);
   cartesian_velocity_scaling_ = std::clamp(cartesian_velocity_scaling_, 0.01, 1.0);
   cartesian_acceleration_scaling_ = std::clamp(cartesian_acceleration_scaling_, 0.01, 1.0);
   post_grasp_lift_m_ = std::clamp(post_grasp_lift_m_, 0.01, 0.20);
   max_search_attempts_ = std::max(1, max_search_attempts_);

    //initial services
   //sub = nh_->subscribe<geometry_msgs::Pose>("pick_point",1,&AgxXarmPick::Callback,this);
   client = nh_->serviceClient<agx_pick_msg::PickStateSrv>("pick_state");
   pont_client = nh_->serviceClient<agx_pick_msg::AgxPickSrv>("pick_point");//call 1 to get pick point
//   goal_service = nh_->advertiseService("goal_service",&AgxXarmPick::set_goal_signal,this);
   task_service = nh_->advertiseService("send_task",&AgxXarmPick::TaskCallback,this);

   //initial topic
   //pub = nh_->advertise<geometry_msgs::Pose>("/pick_point",50);
   //subA = nh_->subscribe<geometry_msgs::PoseStamped>("A",1,&AgxXarmPick::ReadyPointCallback,this);
   //goal_state = nh_->subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result",1,&AgxXarmPick::IsReachGoal,this);
   goal = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
   std::string base_cmd_vel_topic;
   nh_->param<std::string>("base_cmd_vel_topic", base_cmd_vel_topic, "/cmd_vel");
   vel = nh_->advertise<geometry_msgs::Twist>(base_cmd_vel_topic,1);
   grip_ctl_pub_ = nh_->advertise<dh_gripper_msgs::GripperCtrl>("/gripper/ctrl",1);
   grip_state_sub_ = nh_->subscribe<dh_gripper_msgs::GripperState>("/gripper/states",5,&AgxXarmPick::GripperStateCallback,this);

   if (!pont_client.waitForExistence(ros::Duration(setup_timeout_sec_)))
   {
       last_status_ = MissionStatus::kSetupUnavailable;
       ROS_ERROR("pick_point service was not available within %.1f seconds", setup_timeout_sec_);
       return false;
   }
   last_status_ = MissionStatus::kSuccess;
   return true;
}

void AgxXarmPick::GripperStateCallback(
    const dh_gripper_msgs::GripperState::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    latest_gripper_state_ = *msg;
    latest_gripper_ros_time_ = msg->header.stamp;
    latest_gripper_steady_time_ = std::chrono::steady_clock::now();
    has_gripper_state_ = true;
}

bool AgxXarmPick::WaitForGripperState(
    int expected_state,
    const ros::Time &issued_ros_time,
    const std::chrono::steady_clock::time_point &issued_steady_time)
{
    const auto deadline = issued_steady_time +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(gripper_timeout_sec_));
    bool received_fresh_state = false;

    while (ros::ok() && std::chrono::steady_clock::now() < deadline)
    {
        ros::spinOnce();
        {
            std::lock_guard<std::mutex> lock(gripper_state_mutex_);
            const bool fresh_steady = has_gripper_state_ &&
                latest_gripper_steady_time_ >= issued_steady_time;
            const bool fresh_ros = has_gripper_state_ &&
                !latest_gripper_ros_time_.isZero() &&
                latest_gripper_ros_time_ >= issued_ros_time;
            if (fresh_steady && fresh_ros)
            {
                received_fresh_state = true;
                if (latest_gripper_state_.grip_state == expected_state)
                {
                    last_status_ = MissionStatus::kSuccess;
                    return true;
                }
            }
        }
        ros::WallDuration(0.02).sleep();
    }

    if (!ros::ok())
    {
        last_status_ = MissionStatus::kInterrupted;
    }
    else if (!received_fresh_state)
    {
        last_status_ = MissionStatus::kGripperTimeout;
    }
    else if (expected_state == 2)
    {
        last_status_ = MissionStatus::kCaughtStateRequired;
    }
    else
    {
        last_status_ = MissionStatus::kOpenStateRequired;
    }
    ROS_ERROR("gripper did not reach required state %d: %s",
              expected_state, mission_status_name(last_status_));
    return false;
}


bool AgxXarmPick::TaskCallback(agx_pick_msg::TaskCmd::Request &req,
                               agx_pick_msg::TaskCmd::Response &res)
{
    task_states = req.taskID;
    switch(task_states)
    {
        case pub_a_goal:
        {
            task_identified_tag = OBJECT;
            SetPlace(req.A_goal,req.B_goal);
            ROS_INFO("-----pubA-----");
            SetGoal(q.front());
            if(q.size() > 0)
            {
                q.pop();
            }
            res.result = true;
        }break;

        case pick:
        {
            ROS_INFO("-----pick-----");
            /*给两秒钟的识别标签时间*/
            ros::WallDuration(marker_settle_sec_).sleep();
            const bool marker_found = GetPointMsg(task_identified_tag,PRE,FRONT);

            /*判断是否识别到标签，没有识别到移动摄像头寻找标签*/
            if(!marker_found && !SearchMaker())
            {
                res.result = false;
                return true;
            }

            /*判断机械臂是否由于误差而无法抓取到东西，而再次进入校准状态*/
            if(ExecuteMission())
                res.result = true;
            else
                res.result = false;
        }break;

        case pub_b_goal:
        {
            task_identified_tag = PLACE;
            ROS_INFO("-----pubB-----");
            SetGoal(q.front());

            if(q.size() > 0)
            {
                q.pop();
            }
            res.result = true;

         // d.sleep();
        }break;

        case place:
        {
            ROS_INFO("------place-----");
            /*给两秒钟的识别标签时间*/
            ros::WallDuration(marker_settle_sec_).sleep();
            const bool marker_found = GetPointMsg(task_identified_tag,PRE,FRONT);

            /*判断是否识别到标签，没有识别到移动摄像头寻找标签*/
            if(!marker_found && !SearchMaker())
            {
                res.result = false;
                return true;
            }

            /*判断机械臂是否由于误差而无法放置到东西，而再次进入校准状态*/
            if(ExecuteMission())
                res.result = true;
            else
                res.result = false;
        }break;

        case calibration:
        {
            d.sleep();
            ROS_INFO("-----calibration-----");
            ROS_INFO("get_%s_mark",task_identified_tag == 0 ? "object" : "place");
            GetPointMsg(task_identified_tag,REAL,FRONT);
            if(data_flag == false)
            {
                SearchMaker();
                res.result  = true;
            }
            waypoints.clear();
	    
	    std::string mark_frame = "ar_marker_"+std::to_string(task_identified_tag);
            tf::TransformListener listener;
            tf::StampedTransform transform;
            try{
            listener.waitForTransform(mark_frame, "/base_link",ros::Time(0),ros::Duration(3.0));
            listener.lookupTransform("/base_link", mark_frame,
                     ros::Time(0), transform);
               }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            geometry_msgs::Pose dis;
            dis.position.x = transform.getOrigin().x();
            dis.position.y = transform.getOrigin().y();
            dis.position.z = transform.getOrigin().z();
            Calibration(dis);
            x++;
        }
    }
    return true;
}

bool AgxXarmPick::SetHomePose()
{
    /*填充home_pose的点*/
    home_pose.position.x = HOME_X;
    home_pose.position.y = HOME_Y;
    home_pose.position.z = HOME_Z;
    tf::Quaternion q;
    q.setRPY(-0.5838785767555237, -1.5707074403762817, -2.557819366455078);
    // q.setRPY(180/180.0*M_PI, -90/180.0*M_PI, 30/180.0*M_PI);
    home_pose.orientation.x = q.getX();
    home_pose.orientation.y = q.getY();
    home_pose.orientation.z = q.getZ();
    home_pose.orientation.w = q.getW();

    /*限制机械臂的关节回到抓取姿势*/
    return MoveToA(home_pose);
    //MoveToCatchPose();
}


void AgxXarmPick::ReadyPointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    base_link_state.header.frame_id = msg->header.frame_id;
    base_link_state.pose.position.x = msg->pose.position.x;
    base_link_state.pose.position.y = msg->pose.position.y;
}

bool AgxXarmPick::PickAction()
{
    ROS_INFO("excute pick");
    /*机械臂移动到预瞄点*/
    if (MovePath(waypoints) == 0)
    {
        waypoints.clear();
        return false;
    }
    waypoints.clear();

    /*给两秒钟的识别标签时间*/
    ros::WallDuration(marker_settle_sec_).sleep();

    /*直接移动到抓取点*/
    flag_cal = 1;
    if (!GetPointMsg(OBJECT,REAL,FRONT))
    {
        flag_cal = 0;
        return false;
    }
    int reach_target = MovePath(waypoints);
    waypoints.clear();
    data_flag = false;
    flag_cal = 0;

    /*判断抓取点有没有超出抓取范围,若超出之后不进行爪子闭合*/
    if(reach_target)
    {
        dh_gripper_msgs::GripperCtrl cmd;
        cmd.initialize = false;
        cmd.force = 50;
        cmd.position = 0.0;
        cmd.speed = 10;
        if (!Gripper(cmd))
            return false;
        return ReturnReadyPose();
    }

    /*回到home_pose*/
    const MissionStatus failure = last_status_;
    ReturnReadyPose();
    last_status_ = failure;
    return false;
}

bool AgxXarmPick::ExecuteMission()
{
    int mission_type = task_identified_tag;
    ROS_INFO("excute %s",mission_type == OBJECT ? "pick" : "place");
    /*机械臂移动到预瞄点*/
    if (MovePath(waypoints) == 0)
    {
        waypoints.clear();
        return false;
    }
    waypoints.clear();

    /*给两秒钟的识别标签时间*/
    ros::WallDuration(marker_settle_sec_).sleep();

    /*直接移动到抓取点*/
    flag_cal = 1;
    if (!GetPointMsg(task_identified_tag,REAL,FRONT))
    {
        flag_cal = 0;
        waypoints.clear();
        return false;
    }
    int reach_target = MovePath(waypoints);
    waypoints.clear();
    data_flag = false;
    flag_cal = 0;

    /*判断抓取点有没有超出工作范围,若超出之后爪子不张合*/
    if(reach_target)
    {
        dh_gripper_msgs::GripperCtrl cmd;
        cmd.initialize = false;
        cmd.force = 50;
        cmd.position = mission_type == OBJECT ? 0 : 1000;
        cmd.speed = 10;
        if (!Gripper(cmd))
            return false;
        return ReturnReadyPose();

    }

    /*回到home_pose*/
    const MissionStatus failure = last_status_;
    ReturnReadyPose();
    last_status_ = failure;
    return false;
}

bool AgxXarmPick::MoveToA(geometry_msgs::Pose msg)
{
    // Start a thread
    double time1 = ros::Time::now().toSec();
    //ros::AsyncSpinner spinner(1);
    spinner_->start();



    // Define the planning group name
    static const std::string PLANNING_GROUP  = "xarm6";


    // Create a planning group interface object and set up a planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("link_base");
    move_group.setPlanningTime(moveit_timeout_sec_);
    const auto current_state = move_group.getCurrentState(moveit_timeout_sec_);
    if (!current_state)
    {
        last_status_ = MissionStatus::kMoveItUnavailable;
        ROS_ERROR("MoveIt current state was unavailable within %.1f seconds", moveit_timeout_sec_);
        spinner_->stop();
        return false;
    }
    move_group.setStartState(*current_state);
    //move_group.getPoseTarget();


    // Get the coordinate system of the basic information
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());


    // Get the end of the basic information
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Call the planner for planning calculations Note: This is just planning

    bool success  = false;


    geometry_msgs::Pose target_pose;
    target_pose.position.x  = msg.position.x ;
    target_pose.position.y  = msg.position.y;
    target_pose.position.z  = msg.position.z;
    target_pose.orientation.x  = msg.orientation.x;
    target_pose.orientation.y  = msg.orientation.y;
    target_pose.orientation.z  = msg.orientation.z;
    target_pose.orientation.w  = msg.orientation.w;
    move_group.setPoseTarget(target_pose);
    //std::cout<<"target position is ("<< target_pose.position.x<<","<<target_pose.position.y<<","<<target_pose.position.z<<")"<<std::endl;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    success  = (move_group.plan(my_plan)  ==  moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");

    // Perform planning actions
    if(success  ==  true)
    {
        const auto result = move_group.execute(my_plan);
        if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            last_status_ = MissionStatus::kExecutionFailed;
            ROS_ERROR("MoveIt execution failed with code %d", result.val);
            spinner_->stop();
            return false;
        }
    }
    else
    {
        last_status_ = MissionStatus::kPlanFailed;
        std::cout<<"error state"<<std::endl;
        spinner_->stop();
        return false;
    }
    spinner_->stop();
    double time2 = ros::Time::now().toSec();
    ROS_INFO("%f",time2-time1);
    last_status_ = MissionStatus::kSuccess;
    return true;
}

void AgxXarmPick::Gripper(int moterID,int pose,int force)
{

    bool state;
    agx_pick_msg::PickStateSrv srv;
    srv.request.moterID = moterID;
    srv.request.pose = pose;
    srv.request.force = force;
    if(ros::ok())
    {
        state = client.call(srv);
        std::cout <<"state:"<< state;
    }

}

bool AgxXarmPick::OpenGripper()
{
    dh_gripper_msgs::GripperCtrl cmd;
    cmd.initialize = false;
    cmd.force = 50;
    cmd.position = 1000.0;
    cmd.speed = 10;
    return Gripper(cmd);
}

bool AgxXarmPick::Gripper(dh_gripper_msgs::GripperCtrl &cmd)
{
    const auto connection_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(gripper_timeout_sec_));
    while (ros::ok() && grip_ctl_pub_.getNumSubscribers() == 0 &&
           std::chrono::steady_clock::now() < connection_deadline)
    {
        ros::spinOnce();
        ros::WallDuration(0.02).sleep();
    }
    if (!ros::ok())
    {
        last_status_ = MissionStatus::kInterrupted;
        return false;
    }
    if (grip_ctl_pub_.getNumSubscribers() == 0)
    {
        last_status_ = MissionStatus::kGripperTimeout;
        ROS_ERROR("no /gripper/ctrl subscriber appeared within %.1f seconds", gripper_timeout_sec_);
        return false;
    }

    const ros::Time issued_ros_time = ros::Time::now();
    const auto issued_steady_time = std::chrono::steady_clock::now();
    grip_ctl_pub_.publish(cmd);
    const int expected_state = cmd.position <= 0.0 ? 2 : 1;
    return WaitForGripperState(expected_state, issued_ros_time, issued_steady_time);
}

bool AgxXarmPick::GetPointMsg(int item,int handpose,int Prepose)
{
    ROS_INFO("get %d mark",item);
    agx_pick_msg::AgxPickSrv srv;
    srv.request.item = item;
    srv.request.handpose = handpose;
    srv.request.Prepose = Prepose;
    geometry_msgs::Pose begin;
    begin = home_pose;  //P  是home pose 初始姿态
    if(flag_cal == 1)
    {
        begin = temp;
    }

    if(pont_client.call(srv))
    {
        temp.position = srv.response.position;
        temp.orientation = srv.response.orientation;
        data_flag = true;
        if(flag_cal == 1)
        {
            GenerateFinalPath(begin,temp);
        }
        else
        {
	        prepose = temp;
            GeneratePath(begin,temp,go_ahead);//go_ahead  机械臂往前运动
        }
    }
    else
    {
        data_flag = false;
        last_status_ = MissionStatus::kMarkerUnavailable;
        ROS_WARN("can not see this mark");
    }
    return data_flag;
}

void AgxXarmPick::SetGoal(geometry_msgs::PoseStamped msg)
{
    ROS_INFO("set nav_goal");
    goal.publish(msg);
}

void AgxXarmPick::init()
{
    A.pose.position.x = A_x;
    A.pose.position.y = A_y;
    B.pose.position.x = B_x;
    B.pose.position.y = B_y;
}

bool AgxXarmPick::PlaceAction()
{
    ROS_INFO("excute place");
    /*机械臂移动到预瞄点*/
    if (MovePath(waypoints) == 0)
    {
        waypoints.clear();
        return false;
    }
    waypoints.clear();

    /*给两秒钟的识别标签时间*/
    ros::WallDuration(marker_settle_sec_).sleep();

    /*直接移动到抓取点*/
    flag_cal = 1;
    if (!GetPointMsg(PLACE,REAL,FRONT))
    {
        flag_cal = 0;
        return false;
    }
    int reach_target = MovePath(waypoints);
    waypoints.clear();
    data_flag = false;
    flag_cal = 0;

    if(reach_target)
    {
        dh_gripper_msgs::GripperCtrl cmd;
        cmd.initialize = false;
        cmd.force = 30;
        cmd.position = 1000.0;
        cmd.speed = 100;
        if (!Gripper(cmd))
            return false;
        //Gripper(1,100,30);
        return ReturnReadyPose();
    }

//    if(place_num == 1)
//    {
//        place_num = 2;
//    }
//    else
//    {
//        place_num = 1;
//    }

    /*回到home_pose*/
    const MissionStatus failure = last_status_;
    ReturnReadyPose();
    last_status_ = failure;
    return false;

}

void AgxXarmPick::RobotWork()
{
  ROS_INFO("robotwork");
    ROS_INFO("dected mark");
    GetPointMsg(object_num,PRE,FRONT);
    if(data_flag)
    {
      PickAction();
    }
    SetHomePose();
    task_states = place;
    GetPointMsg(PLACE,PRE,FRONT);
    if(data_flag)
    {
      work_flag = true;

      PlaceAction();
    }
}


void AgxXarmPick::SetPlace(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
{
    q.push(a);
    q.push(b);
}

void AgxXarmPick::SetObstruction()
{
    //ROS_INFO("1111111111111111");
    //ros::AsyncSpinner spinner(1);
    spinner_->start();
    static const std::string PLANNING_GROUP  = "xarm6";


    // Create a planning group interface object and set up a planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("link_base");


    // Create a planning scene interface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create an object of the visualization class
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link_base");
    visual_tools.deleteAllMarkers();


    // Load remote control tool
    visual_tools.loadRemoteControl();

    // Create a robot model information object
    const robot_state::JointModelGroup* joint_model_group  = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    Eigen::Isometry3d text_pose  = Eigen::Isometry3d::Identity();
    text_pose.translation().z()  = 1.2;

    // Define a collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();


    // Set the ID of the object
    collision_object.id = "box1";


    // ********************************Defining a box (obstacle) added to the world
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.20;
    primitive.dimensions[1] = 0.15;
    primitive.dimensions[2] = 0.60;

    // 设置盒子(障碍物)的位置和姿态
    // Set the box (obstacle) pose
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0.40;
    box_pose.position.z = 0.975;


    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    //--------------------------------------------------------------------
    // Define a collision object
    moveit_msgs::CollisionObject collision_object1;
    collision_object1.header.frame_id = move_group.getPlanningFrame();


    // Set the ID of the object
    collision_object1.id = "box3";


    // ********************************Defining a box (obstacle) added to the world
    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 0.09;
    primitive1.dimensions[1] = 0.1;
    primitive1.dimensions[2] = 0.1;

    // 设置盒子(障碍物)的位置和姿态
    // Set the box (obstacle) pose
    geometry_msgs::Pose box_pose1;
    box_pose1.orientation.w = 1.0;
    box_pose1.position.x = -0.09;
    box_pose1.position.y = -0.1;
    box_pose1.position.z = 0.675;


    collision_object1.primitives.push_back(primitive1);
    collision_object1.primitive_poses.push_back(box_pose1);
    collision_object1.operation = collision_object1.ADD;


    //------------------------------------------------------------

    // *********************************Defining a box (desktop) added in the world
    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group.getPlanningFrame();

    // Set the ID of the object
    collision_object2.id = "box2";

    // Set length, width and height
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 1.7;
    primitive2.dimensions[1] = 1.7;
    primitive2.dimensions[2] = 0.05;


    // Set the box (Desktop) pose
    geometry_msgs::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x = 0.0;
    box_pose2.position.y = 0.0;
    box_pose2.position.z = 0.0;

    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object2.ADD;


    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    collision_objects.push_back(collision_object2);

    planning_scene_interface.addCollisionObjects(collision_objects);


    // Show text in RViz of status
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Xarm Add object Example5", rvt::RED, rvt::XLARGE);
    visual_tools.trigger();

    spinner_->stop();
    //  visual_tools.prompt("Press 'next'6 in the RvizVisualToolsGui window to once the collision object appears in RViz");
}

int AgxXarmPick::MovePath(vector<geometry_msgs::Pose> waypoints)
{

    spinner_->start();
    first_flag = true;
//  ROS_INFO("point_size=",waypoints.size());
    std::cout<<"path_size="<<waypoints.size()<<std::endl;

    // Define the planning group name
    static const std::string PLANNING_GROUP  = "xarm6";


    // Create a planning group interface object and set up a planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    move_group.setPoseReferenceFrame("link_base");
    move_group.setPlanningTime(moveit_timeout_sec_);

    if (waypoints.empty())
    {
        last_status_ = MissionStatus::kPlanFailed;
        ROS_ERROR("cannot plan an empty Cartesian path");
        spinner_->stop();
        return 0;
    }

    // Create a planning scene interface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create a robot model information object
    const auto current_state = move_group.getCurrentState(moveit_timeout_sec_);
    if (!current_state)
    {
        last_status_ = MissionStatus::kMoveItUnavailable;
        ROS_ERROR("MoveIt current state was unavailable within %.1f seconds", moveit_timeout_sec_);
        spinner_->stop();
        return 0;
    }
    move_group.setStartState(*current_state);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Reduce the speed of the robot arm by the scaling factor of the maximum speed of each joint. Please note that this is not the speed of the final effector.
    move_group.setMaxVelocityScalingFactor(cartesian_velocity_scaling_);
    move_group.setMaxAccelerationScalingFactor(cartesian_acceleration_scaling_);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm.
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold  = 0.0;           //(The jump threshold is set to 0.0)
    const double eef_step  = 0.01;               //(interpolation step)


    // Calculate Cartesian interpolation path: return path score (0~1, -1 stands for error)
    double fraction  = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


    if(fraction < 1.0 - 1e-6)
    {
        last_status_ = MissionStatus::kPlanFailed;
        spinner_->stop();
        return 0;
    }

    robot_trajectory::RobotTrajectory timed_trajectory(
        current_state->getRobotModel(), PLANNING_GROUP);
    timed_trajectory.setRobotTrajectoryMsg(*current_state, trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(
            timed_trajectory,
            cartesian_velocity_scaling_,
            cartesian_acceleration_scaling_))
    {
        last_status_ = MissionStatus::kPlanFailed;
        ROS_ERROR("failed to time-parameterize Cartesian path");
        spinner_->stop();
        return 0;
    }
    timed_trajectory.getRobotTrajectoryMsg(trajectory);

    //Move to the home point position
    my_plan.trajectory_ = trajectory;
    double time1 = ros::Time::now().toSec();
    const auto result = move_group.execute(my_plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        last_status_ = MissionStatus::kExecutionFailed;
        ROS_ERROR("Cartesian path execution failed with code %d", result.val);
        spinner_->stop();
        return 0;
    }
    spinner_->stop();
    double time2 = ros::Time::now().toSec();
    ROS_INFO("move_path_time:%f",time2-time1);

    last_status_ = MissionStatus::kSuccess;
    return waypoints.size();
}

void AgxXarmPick::GenerateWayPoint(geometry_msgs::Pose beginpoint,geometry_msgs::Pose targetpoint,int flag)
{
  //flag=1:go ahead;flag=0:go back
    geometry_msgs::Pose waypoint;
    tf::Quaternion q;
    // q.setRPY(1.57,0,0);
    q.setRPY(-0.5838785767555237, -1.5707074403762817, -2.557819366455078);
    ROS_INFO("begin(%f,%f,%f)",beginpoint.position.x,beginpoint.position.y,beginpoint.position.z);
    ROS_INFO("target(%f,%f,%f)",targetpoint.position.x,targetpoint.position.y,targetpoint.position.z);

    for(int i=0;i<5;i++)
    {
        if(flag == 0 && i == 0)
        {
            ROS_INFO("go back!!!!!!!!!!!!!!");
            continue;
        }else {
            ROS_INFO("go ahead!!!!!!!!!!!!!!!");
        }
        switch (i) {
        case 1:
        {
            waypoint.position.x = beginpoint.position.x;
            if(flag == 1)
            {
            //ahead
            waypoint.position.y = beginpoint.position.y-0.06;
            waypoint.position.z = beginpoint.position.z;
            }else
            {
            //back
            waypoint.position.y = beginpoint.position.y+0.08;
            waypoint.position.z = beginpoint.position.z+0.03;
            }

        }break;
        case 2:
        {
          waypoint.position.x = targetpoint.position.x+0.01;
          //waypoint.position.y = beginpoint.position.y;
         // waypoint.position.z = beginpoint.position.z;
        }break;
        case 3:
        {
          //waypoint.position.x = targetpoint.position.x;
          waypoint.position.y = (beginpoint.position.y+targetpoint.position.y)/2;
          waypoint.position.z = (beginpoint.position.z+targetpoint.position.z)/2;
        }break;
        case 4:
        {
          //waypoint.position.x = targetpoint.position.x;
          waypoint.position.y = targetpoint.position.y+0.03;
          waypoint.position.z = targetpoint.position.z;
        }break;
        default:
        {
          waypoint.position.x = beginpoint.position.x;
          waypoint.position.y = beginpoint.position.y;
          waypoint.position.z = beginpoint.position.z;
        } break;
        }
        waypoint.orientation.x = q.getX();
        waypoint.orientation.y = q.getY();
        waypoint.orientation.z = q.getZ();
        waypoint.orientation.w = q.getW();
        //    ROS_INFO("waypoint:(%f,%f,%f)",waypoint.position.x,waypoint.position.y,waypoint.position.z);
        waypoints.push_back(waypoint);

    }
}

void AgxXarmPick::settestpoint()
{
    geometry_msgs::Pose p;
    tf::Quaternion q;
    // q.setRPY(1.57,0,0);
    q.setRPY(-0.5838785767555237, -1.5707074403762817, -2.557819366455078);
    p.position.x = /*-0.15;*/ HOME_X;
    p.position.y = /*-0.29;*/ HOME_Y;
    p.position.z = /*0.66;*/  HOME_Z;
    p.orientation.x = q.getX();
    p.orientation.y = q.getY();
    p.orientation.z = q.getZ();
    p.orientation.w = q.getW();
    waypoints.push_back(p);
    p.position.x += 0.1;
    //  p.position.x = 0;
    waypoints.push_back(p);
    p.position.z -= 0.1;
    //  p.position.y = -0.4;
    //  p.position.z = 0.48;
    waypoints.push_back(p);
    p.position.x -= 0.1;
    //  p.position.y = -0.5;
    //  p.position.z = 0.33;
    waypoints.push_back(p);
    p.position.z += 0.1;
    waypoints.push_back(p);
}

bool AgxXarmPick::ReturnReadyPose()
{
    ROS_INFO("return reday pose");
    geometry_msgs::Pose lift_pose = temp;
    lift_pose.position.z = temp.position.z + post_grasp_lift_m_;
    prepose.position.z = lift_pose.position.z;
    GeneratePath(temp,lift_pose,go_back);
    GeneratePath(lift_pose,prepose,go_back);
    GeneratePath(prepose,home_pose,go_back);
    const bool success = MovePath(waypoints) > 0;
    waypoints.clear();
    //SetHomePose();
    return success;
}

void AgxXarmPick::CatchAppleAction()
{
    GetPointMsg(APPLE,PRE,UPSIDE);
    MovePath(waypoints);
    waypoints.clear();
    CorrectionHandPose();
    GoCatch();
    ReturnReadyPose();
}

void AgxXarmPick::GeneratePath(geometry_msgs::Pose beginpoint,geometry_msgs::Pose targetpoint,int flag)
{
    geometry_msgs::Pose waypoint;
    tf::Quaternion q;
    ROS_INFO("begin(%f,%f,%f)",beginpoint.position.x,beginpoint.position.y,beginpoint.position.z);
    ROS_INFO("target(%f,%f,%f)",targetpoint.position.x,targetpoint.position.y,targetpoint.position.z);
    double step_pose_R = 0,pose_R = 0,begin_pose_R = 0;  //pose 为最终的pose
    double step_pose_P = 0,pose_P = 0,begin_pose_P = 0;
    double step_pose_Y = 0,pose_Y = 0,begin_pose_Y = 0;
    ROS_WARN("%d",task_states);
    if(flag == go_ahead)
    {
        ROS_WARN("go_ahead");
        pose_R = 1.8486912250518799;// 1.57为弧度制，转化为角度是90°，运动关节为第六关节
        begin_pose_R = -3.1415927410125732;// 
        pose_P = -1.570712924003601;
        begin_pose_P = -1.0471999645233154;
        pose_Y = 1.8164114952087402;
        begin_pose_Y = 0.5235999822616577;
    }
    else
    {
        ROS_WARN("go_back");
        pose_R = -3.1415927410125732;
        begin_pose_R = 1.8486912250518799;
        pose_P = -1.0471999645233154;
        begin_pose_P = -1.570712924003601;
        pose_Y = 0.5235999822616577;
        begin_pose_Y = 1.8164114952087402;
    }
    for(int i = 0;i < waypoint_num;i++)
    {
	    if(i==0)continue;
        waypoint.position.x = beginpoint.position.x + ((targetpoint.position.x - beginpoint.position.x) / (waypoint_num - 1) * i);  //targetpoint 目标点
        waypoint.position.y = beginpoint.position.y + ((targetpoint.position.y - beginpoint.position.y) / (waypoint_num - 1) * i);
        waypoint.position.z = beginpoint.position.z + ((targetpoint.position.z - beginpoint.position.z) / (waypoint_num - 1) * i);
        step_pose_R = begin_pose_R + ((pose_R - begin_pose_R) / (waypoint_num - 1) * i);
	    step_pose_P = begin_pose_P + ((pose_P - begin_pose_P) / (waypoint_num - 1) * i);
	    step_pose_Y = begin_pose_Y + ((pose_Y - begin_pose_Y) / (waypoint_num - 1) * i);
        // q.setRPY(1.8486912250518799,-1.570712924003601,1.8164114952087402);
        q.setRPY(-0.5838785767555237, -1.5707074403762817, -2.557819366455078);
        waypoint.orientation.x = q.getX();
        waypoint.orientation.y = q.getY();
        waypoint.orientation.z = q.getZ();
        waypoint.orientation.w = q.getW();
        waypoints.push_back(waypoint);
        std::cout <<"("<< waypoint.position.x << "," << waypoint.position.y << "," << waypoint.position.z << ")" << std::endl;
    }
}

void AgxXarmPick::GenerateFinalPath(geometry_msgs::Pose beginpoint,geometry_msgs::Pose targetpoint)
{

    geometry_msgs::Pose waypoint;
    tf::Quaternion step_quaternion;
    ROS_INFO("begin(%f,%f,%f)",beginpoint.position.x,beginpoint.position.y,beginpoint.position.z);
    ROS_INFO("target(%f,%f,%f)",targetpoint.position.x,targetpoint.position.y,targetpoint.position.z);
    double step_pose = 0;
    //step_pose = 1.57;
    // step_quaternion.setRPY(1.8486912250518799,-1.57071292400360,1.8164114952087402);
    step_quaternion.setRPY(-0.5838785767555237, -1.5707074403762817, -2.557819366455078);

    //align z
    waypoint.position.x = beginpoint.position.x;
    waypoint.position.y = beginpoint.position.y;
    waypoint.position.z = beginpoint.position.z;
    waypoint.orientation.x = step_quaternion.getX();
    waypoint.orientation.y = step_quaternion.getY();
    waypoint.orientation.z = step_quaternion.getZ();
    waypoint.orientation.w = step_quaternion.getW();
    //waypoints.push_back(waypoint);

    waypoint.position.x = beginpoint.position.x;
    waypoint.position.y = beginpoint.position.y;
    waypoint.position.z = targetpoint.position.z;
    waypoint.orientation.x = step_quaternion.getX();
    waypoint.orientation.y = step_quaternion.getY();
    waypoint.orientation.z = step_quaternion.getZ();
    waypoint.orientation.w = step_quaternion.getW();
    waypoints.push_back(waypoint);

    //align x axex
    waypoint.position.x = targetpoint.position.x-0.12*cos(theta);
    waypoint.position.y = targetpoint.position.y-0.12*sin(theta);
    waypoint.position.z = targetpoint.position.z;
    waypoint.orientation.x = step_quaternion.getX();
    waypoint.orientation.y = step_quaternion.getY();
    waypoint.orientation.z = step_quaternion.getZ();
    waypoint.orientation.w = step_quaternion.getW();
    waypoints.push_back(waypoint);

    //align y
    waypoint.position.x = targetpoint.position.x;
    waypoint.position.y = targetpoint.position.y;
    waypoint.position.z = targetpoint.position.z;
    waypoint.orientation.x = step_quaternion.getX();
    waypoint.orientation.y = step_quaternion.getY();
    waypoint.orientation.z = step_quaternion.getZ();
    waypoint.orientation.w = step_quaternion.getW();
    waypoints.push_back(waypoint);
}

void AgxXarmPick::CorrectionHandPose()
{
    GetPointMsg(APPLE,UPSIDE,REAL);
}

void AgxXarmPick::GoCatch()
{
    MovePath(waypoints);
    waypoints.clear();
    Gripper(1,0,30);
}

void AgxXarmPick::Calibration(geometry_msgs::Pose targetpoint)
{
    //geometry_msgs::Pose waypoint;
    double del_x = 0;
    tf::Quaternion step_quaternion;
    // ROS_INFO("begin(%f,%f,%f)",beginpoint.position.x,beginpoint.position.y,beginpoint.position.z);
    ROS_INFO("target(%f)",targetpoint.position.x);

    del_x = targetpoint.position.x-ar_distance;

    int ticks;

    geometry_msgs::Twist speed;

    double rate = 10 ;
    ros::Rate loopRate(rate);
    double linear_speed = 0;
    linear_speed =  del_x > 0 ? 0.2 : -0.2;
    double goal_distance = del_x;
    double linear_duration = abs(goal_distance / linear_speed);

    speed.linear.x = linear_speed;
    ticks = int(linear_duration*rate);
    ROS_WARN("tick:%d",ticks);
    ROS_WARN("linear_duration:%f",linear_duration);
    ROS_WARN("del_x:%f",del_x);

    for(int i = 0;i<ticks; i++)
    {
        vel.publish(speed);
        loopRate.sleep();
    }
    speed.linear.x = 0;
    vel.publish(speed);
    sleep(2+ceil(linear_duration));
}

bool AgxXarmPick::SearchMaker()            //在看不到标签的情况下，执行标签寻找函数
{
    ROS_INFO("I am looking the Mark");
    const auto search_started = std::chrono::steady_clock::now();
    const auto search_deadline = search_started +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(marker_search_timeout_sec_));
    tf::TransformListener listener;
    tf::StampedTransform transform_1;
    double x = search_dis;
    geometry_msgs::Pose deta_dis;
    //获取link6 到 link_base 的tf关系，数值存在transfirm_1
    try
    {
        const double tf_timeout = std::min(10.0, marker_search_timeout_sec_);
        listener.waitForTransform("/link6", "/link_base",ros::Time(0),ros::Duration(tf_timeout));
        listener.lookupTransform("/link_base", "/link6",
                                 ros::Time(0), transform_1);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        last_status_ = MissionStatus::kSearchExhausted;
        return false;
    }

    bool flag = false;
    int cont = 0;
    int attempts = 0;

    /*data_flag为true就是看到标签，反之为false*/
    while (!data_flag && ros::ok() && attempts < max_search_attempts_ &&
           std::chrono::steady_clock::now() < search_deadline)
    {
        ROS_INFO("x:%f  flag:%d  cont:%d",x,flag,cont);
        if (cont==2)
        {
            /*判断flag是否为false*/
            if(flag)
                flag = false;
            else
                flag = true;
            cont = 0;
        }

        if(flag==false)
        {
            /*X方向偏移*/
            deta_dis.position.x = transform_1.getOrigin().getX();
            deta_dis.position.y = transform_1.getOrigin().getY();
            deta_dis.position.z = transform_1.getOrigin().getZ();
            tf::Quaternion q;
            // q.setRPY(180/180.0*M_PI, -90/180.0*M_PI, (30+x*100)/180.0*M_PI);
            q.setRPY(-0.5838785767555237, -1.5707074403762817, 2*x-2.557819366455078);
            deta_dis.orientation.x = q.getX();
            deta_dis.orientation.y = q.getY();
            deta_dis.orientation.z = q.getZ();
            deta_dis.orientation.w = q.getW();
        }

        else
        {
            /*Z方向偏移*/
            deta_dis.position.x = transform_1.getOrigin().getX();
            deta_dis.position.y = transform_1.getOrigin().getY();
            deta_dis.position.z = transform_1.getOrigin().getZ();
            if(x>0){
	    	    deta_dis.position.z+=x;
            	deta_dis.orientation.x = transform_1.getRotation().getX();
            	deta_dis.orientation.y = transform_1.getRotation().getY();
            	deta_dis.orientation.z = transform_1.getRotation().getZ();
            	deta_dis.orientation.w = transform_1.getRotation().getW();
	        }
	        else{
            	tf::Quaternion q;
            	// q.setRPY(180/180.0*M_PI, -80/180.0*M_PI, 30/180.0*M_PI);
                q.setRPY(-0.5838785767555237, -1.5707074403762817, -2.557819366455078);
            	deta_dis.orientation.x = q.getX();
            	deta_dis.orientation.y = q.getY();
            	deta_dis.orientation.z = q.getZ();
            	deta_dis.orientation.w = q.getW();
	        }
        }
        
        if (!MoveToA(deta_dis))
            return false;
        ros::WallDuration(marker_settle_sec_).sleep();
        /*判断是否看到标签，看到标签data_flag为true*/
        GetPointMsg(task_identified_tag,PRE,FRONT);
        if(data_flag)
        {
            break;
        }
        waypoints.clear();
        cont++;
        attempts++;
        /*x值反转*/
        x=x*(-1.0);
    }
    if (data_flag)
    {
        last_status_ = MissionStatus::kSuccess;
        ROS_INFO("I saw it");
        return true;
    }
    last_status_ = ros::ok() ? MissionStatus::kSearchExhausted
                             : MissionStatus::kInterrupted;
    ROS_ERROR("marker search ended after %d/%d attempts: %s",
              attempts, max_search_attempts_, mission_status_name(last_status_));
    return false;
}

MissionStatus AgxXarmPick::FinishStandaloneFailure(
    bool attempt_home_recovery)
{
    const MissionStatus failure = last_status_;
    waypoints.clear();
    data_flag = false;
    flag_cal = 0;

    if (attempt_home_recovery &&
        failure != MissionStatus::kInterrupted && ros::ok())
    {
        ROS_WARN("releasing the gripper and returning home after standalone "
                 "mission failure: %s",
                 mission_status_name(failure));
        if (!OpenGripper())
        {
            ROS_ERROR("gripper release also failed after: %s",
                      mission_status_name(failure));
        }
        if (!SetHomePose())
        {
            ROS_ERROR("home recovery also failed after: %s",
                      mission_status_name(failure));
        }
    }

    waypoints.clear();
    data_flag = false;
    flag_cal = 0;
    last_status_ = failure;
    return failure;
}

MissionStatus AgxXarmPick::RunStandaloneMission(
    int marker_id, bool open_before_mission)
{
    last_status_ = MissionStatus::kSuccess;
    if (!Setup())
        return FinishStandaloneFailure(false);
    if (!SetHomePose())
        return FinishStandaloneFailure(false);
    if (open_before_mission && !OpenGripper())
        return FinishStandaloneFailure(false);

    init();
    task_identified_tag = marker_id;
    if (!GetPointMsg(marker_id, PRE, FRONT) && !SearchMaker())
        return FinishStandaloneFailure(true);
    if (!ExecuteMission())
        return FinishStandaloneFailure(true);

    last_status_ = MissionStatus::kSuccess;
    return last_status_;
}

void AgxXarmPick::MoveToCatchPose()
{
    // Start a thread
    double time1 = ros::Time::now().toSec();
    //ros::AsyncSpinner spinner(1);
    spinner_->start();



    // Define the planning group name
    static const std::string PLANNING_GROUP  = "xarm6";


    // Create a planning group interface object and set up a planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("link_base");
    //move_group.getPoseTarget();


    // Get the coordinate system of the basic information
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());


    // Get the end of the basic information
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Call the planner for planning calculations Note: This is just planning

    bool success  = false;

    double target_pose[6] = {0.5235999822616577, -0.8726999759674072, -1.0471999645233154, -3.469446951953614e-18, 0.8726999759674072, 8.673617379884035e-19};
    std::vector<double> joint_group_position(6);
    joint_group_position[0] = target_pose[0];
    joint_group_position[1] = target_pose[1];
    joint_group_position[2] = target_pose[2];
    joint_group_position[3] = target_pose[3];
    joint_group_position[4] = target_pose[4];
    joint_group_position[5] = target_pose[5];
    move_group.setJointValueTarget(joint_group_position);
    move_group.move();
    spinner_->stop();
}
