#!/usr/bin/env python
import rospy
import os,sys
import roslib
from agx_pick_msg.srv import *
import time
import node
from move_base_msgs.msg import MoveBaseActionResult
from dh_gripper_msgs.msg import GripperRotState

def callback(req):
    id = req.taskID
    if id == 0:
        print("start PubA")
        time.sleep(1)#excute pubA
        return TaskCmdResponse(True)

    elif id == 1:
        print("start pick")
        time.sleep(2)#excute pickz
        msg=GripperRotState()

    elif id == 2:
        print("start pubB")
        time.sleep(3)#excute pubB
        return TaskCmdResponse(True)

    else:
        print("start place")
        time.sleep(4)#excute place
        return TaskCmdResponse(True)

    

def IsReachGoal(msg):
    if msg.status.status == 3:
        print("reach goal")
        node.goal_flag = True

def main():
    rospy.init_node('respons_test')
    rospy.Service('send_task',TaskCmd,callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, IsReachGoal)
    print("ready to excute task!")
    rospy.spin()
   

if __name__ == '__main__':
    main()
