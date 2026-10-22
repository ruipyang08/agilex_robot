'''
Author: your name
Date: 2021-08-16 10:34:27
LastEditTime: 2021-08-20 15:21:10
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /agx_aubo_smach/script/node.py
'''
#!/usr/bin/env python
import rospy
import os,sys
import roslib
from geometry_msgs.msg import PoseStamped
from agx_pick_msg.srv import *
import time
from enum import Enum

A = PoseStamped()
B = PoseStamped()
flag = False
goal_flag = False
flag_grap = 0
flag_grap_position = 0
task_states = 0
count = 0
client = rospy.ServiceProxy('send_task', TaskCmd)

task = {'pubA':0,'pick':1,'pubB':2,'place':3,'arrival':4}
grap_state = {'active':0,'open':1,'grap':2}