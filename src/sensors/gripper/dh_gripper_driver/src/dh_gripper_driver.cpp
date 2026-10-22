#include <iostream>
#include <unistd.h>
#include "dh_gripper_factory.h"

#include "ros/ros.h"
#include "dh_gripper_msgs/GripperCtrl.h"
#include "dh_gripper_msgs/GripperState.h"
#include "dh_gripper_msgs/GripperRotCtrl.h"
#include "dh_gripper_msgs/GripperRotState.h"

#include "sensor_msgs/JointState.h"

std::string _gripper_ID;
std::string _gripper_model;
std::string _gripper_connect_port;
std::string _gripper_Baudrate;
DH_Gripper *_gripper;

void update_gripper_control(const dh_gripper_msgs::GripperCtrl::ConstPtr& msg)
{
    
    if(msg->initialize)
    {
      _gripper->Initialization();  
    }
    else
    {
        ROS_INFO("speed:[%f],force: [%f], position: [%f]", msg->speed,msg->force, msg->position);
        _gripper->SetTargetSpeed((int)msg->speed);
        _gripper->SetTargetForce((int)msg->force);
        _gripper->SetTargetPosition((int)msg->position);
    }
}

void update_rotation_control(const dh_gripper_msgs::GripperRotCtrl::ConstPtr& msg)
{
    ROS_INFO("r_speed:[%f],r_force: [%f], r_angle: [%f]", msg->speed,msg->force, msg->angle);
    if(_gripper_model.find("RGI")!= _gripper_model.npos)
    {
        dynamic_cast<DH_RGI *>(_gripper)->SetTargetRotationTorque((int)msg->force); 
        dynamic_cast<DH_RGI *>(_gripper)->SetTargetRotationSpeed((int)msg->speed); 
        dynamic_cast<DH_RGI *>(_gripper)->SetTargetRotation((int)msg->angle); 

    }
    else if(_gripper_model.find("DH3_CAN")!= _gripper_model.npos)
    {
        dynamic_cast<DH_DH3_CAN *>(_gripper)->SetTargetRotation((int)msg->angle); 
    }

}


void update_gripper_state(dh_gripper_msgs::GripperState& msg)
{
    static long seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.seq =seq; 
    int tmp_state[5] = {0};
    _gripper->GetRunStates(tmp_state);
    if(tmp_state[0] == 1)
        msg.is_initialized = true;
    else
        msg.is_initialized = false;
        
    msg.grip_state      = tmp_state[1];
    msg.position        = tmp_state[2];
    msg.target_position = tmp_state[3];
    msg.target_force    = tmp_state[4];
    seq++;
}

void update_gripper_joint_state(sensor_msgs::JointState& msg, int position)
{
    static long seq = 0;
    msg.header.frame_id = "";
    msg.header.stamp = ros::Time::now();
    msg.header.seq = seq;
    
    msg.name.resize(1);
    msg.position.resize(1);

    // Position already came from GetRunStates. A second register read here
    // only added another serial round trip on the same thread.
    msg.position[0] = (1000 - position) / 1000.0 * 0.637;
    msg.name[0] = "gripper_finger1_joint"; 

    seq++;
}



void update_rotation_state(dh_gripper_msgs::GripperRotState& msg)
{
    static long seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.seq =seq; 
    int tmp_state[9] = {0};
    _gripper->GetRunStates(tmp_state); 
    msg.rot_state       = tmp_state[5];
    msg.angle           = tmp_state[6];
    msg.target_angle    = tmp_state[7];
    msg.target_force    = tmp_state[8];
    seq++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dh_gripper_driver");
    ros::NodeHandle n("~");

    n.param<std::string>("Gripper_ID", _gripper_ID,"1");
    n.param<std::string>("Gripper_Model", _gripper_model,"AG95_MB");
    n.param<std::string>("Connect_port", _gripper_connect_port,"/dev/ttyUSB0");
    n.param<std::string>("BaudRate", _gripper_Baudrate, "115200");
    
    ROS_INFO("Gripper_ID : %s", _gripper_ID.c_str());
    ROS_INFO("Gripper_model : %s", _gripper_model.c_str());
    ROS_INFO("Connect_port: %s", _gripper_connect_port.c_str());
    ROS_INFO("BaudRate : %s (In TCP/IP Mode , BaudRate is unuse)", _gripper_Baudrate.c_str());


    DH_Gripper_Factory* _gripper_Factory = new DH_Gripper_Factory();
    _gripper_Factory->Set_Parameter(atoi(_gripper_ID.c_str()), _gripper_connect_port, atoi(_gripper_Baudrate.c_str()));
    

    _gripper = _gripper_Factory->CreateGripper(_gripper_model);
    if(_gripper == NULL)
    {
        ROS_ERROR("No this Model :%s", _gripper_model.c_str());
        return -1;
    }   

     
    if(_gripper->open()<0)
    {
        ROS_ERROR("Unable to open commport to %s", _gripper_connect_port.c_str());
        return -1;
    }

    // Communication failure leaves initstate untouched, so it must not be
    // treated as "not initialized" (that would start a real finger motion).
    int initstate = 0;
    if (!_gripper->GetInitState(initstate))
    {
        ROS_ERROR("读取夹爪初始化状态失败，请检查串口 %s", _gripper_connect_port.c_str());
        _gripper->close();
        return -1;
    }
    if (initstate != DH_Gripper::S_INIT_FINISHED)
    {
        ROS_INFO("夹爪未初始化 (state=%d)，开始初始化", initstate);
        if (!_gripper->Initialization())
        {
            ROS_ERROR("发送夹爪初始化指令失败");
            _gripper->close();
            return -1;
        }

        int comm_fail = 0;
        ros::Time deadline = ros::Time::now() + ros::Duration(30.0);
        ros::Rate wait_rate(5);
        while (ros::ok() && initstate != DH_Gripper::S_INIT_FINISHED)
        {
            if (ros::Time::now() > deadline)
            {
                ROS_ERROR("夹爪初始化超时 (state=%d)", initstate);
                _gripper->close();
                return -1;
            }
            if (!_gripper->GetInitState(initstate))
            {
                comm_fail += 1;
                ROS_WARN("读取夹爪初始化状态失败 (%d/3)", comm_fail);
                if (comm_fail >= 3)
                {
                    ROS_ERROR("夹爪初始化期间通信连续失败，退出");
                    _gripper->close();
                    return -1;
                }
            }
            else
            {
                comm_fail = 0;
            }
            wait_rate.sleep();
        }
        if (!ros::ok())
        {
            _gripper->close();
            return 0;
        }
        ROS_INFO("夹爪初始化完成 (state=%d)", initstate);
    }
    else
    {
        ROS_INFO("夹爪已初始化，跳过初始化动作");
    }

    ros::Subscriber _sub_grip ;
    ros::Publisher _gripper_state_pub;
    ros::Subscriber _sub_rot ;
    ros::Publisher _rot_state_pub;

    // Queue of 1 keeps only the newest command. A length of 50 replayed
    // stale targets for as long as each serial write was blocked.
    _sub_grip = n.subscribe("/gripper/ctrl", 1, update_gripper_control);
    _gripper_state_pub = n.advertise<dh_gripper_msgs::GripperState>("/gripper/states", 50);

    if(_gripper->GetGripperAxiNumber()==2)
    {
        _sub_rot = n.subscribe("/gripper/rot_ctrl", 50, update_rotation_control);
        _rot_state_pub = n.advertise<dh_gripper_msgs::GripperRotState>("/gripper/rot_states", 50);
    }
    //TODO
    ros::Publisher _gripper_joint_state_pub = n.advertise<sensor_msgs::JointState>("/gripper/joint_states", 50); 

    ros::Rate loop_rate(10);
     while (ros::ok())
     {
        // Apply the latest command before the status polls, so a slider
        // release is not stuck behind five register reads.
        ros::spinOnce();

         dh_gripper_msgs::GripperState msg_g_state;
         update_gripper_state(msg_g_state);
        _gripper_state_pub.publish(msg_g_state);

        sensor_msgs::JointState msg_g_joint_state;
        update_gripper_joint_state(msg_g_joint_state, (int)msg_g_state.position);
        _gripper_joint_state_pub.publish(msg_g_joint_state);

        if(_gripper->GetGripperAxiNumber()==2)
        {
            dh_gripper_msgs::GripperRotState msg_r_state;
            update_rotation_state(msg_r_state);
            _rot_state_pub.publish(msg_r_state);

        }

        loop_rate.sleep();
    }


    _gripper->close();


    return 0;

}