#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os,sys
import roslib
import smach
import smach_ros
import sys
from agx_pick_msg.srv import *
from dh_gripper_msgs.msg import *
from geometry_msgs.msg import PoseStamped
import time
import node
from move_base_msgs.msg import MoveBaseActionResult
import signal

class WaitFororder(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['receive','wait'])

    def execute(self, userdata):
        rospy.loginfo('Executing state waitfororder')
        time.sleep(1)
        if node.flag == True:
            node.flag = False
            return 'receive'
        else:
            return 'wait'

class PubAGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pubA'])

    def execute(self, userdata):
        node.task_states = node.task["pick"]
        rospy.loginfo('Executing state publish Agoal')
        rospy.wait_for_service('/send_task')
        print("connect service")
        try:
           #client = rospy.ServiceProxy('send_task', TaskCmd)
           #taskid = node.task.pubA
           resp1 = node.client(node.task['pubA'],node.A,node.B)
        except rospy.ServiceException as e:
               print ("Service call failed: %s"%e)
        return 'pubA'

class WaitForReachGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reachA','no','reachB'])

    def execute(self, userdata):
        rospy.loginfo('waiting for reach goal')
        time.sleep(1)
        if node.goal_flag == True:
            if node.task_states == node.task['pick']:
                node.goal_flag = False
                return 'reachA'
            else:
                node.goal_flag = False
                return 'reachB'
        else:
            return 'no'

class Calibration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrivalA' , 'arrivalB' , 'data_empty'])


    def execute(self, userdata):
        rospy.loginfo('Calibration the car states')

        rospy.wait_for_service('/send_task')
        print("connect service")

        try:
            resp1 = node.client(node.task['arrival'],node.A,node.B)
               
        except rospy.ServiceException as e:
             
           print ("Service call failed: %s",e)
           return 'data_empty'

        if node.task_states == node.task['pick']:
            return 'arrivalA'
        else:
            return 'arrivalB'

class Pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['mission_success','mission_fail'])

    def execute(self, userdata):
        rospy.loginfo('Executing state pick')
        #time.sleep(0.5)
        rospy.wait_for_service('/send_task')
        print("connect service")
        try:
           resp1 = node.client(node.task['pick'],node.A,node.B)
        #    if resp1 == False:
        #         return 'finish'

           if resp1.result == False:
                return 'mission_fail'
        except rospy.ServiceException as e:
               print ("Service call failed: %s"%e)

        node.goal_flag = False
        return 'mission_success'



'''
description: 订阅夹爪状态的话题，查看夹爪的状态，由于夹爪张开和夹爪没有抓取到东西时夹爪的状态都为1
所以用夹爪之间的距离来判断夹爪张开还是闭合。
param {* }
return {*}
'''
class ChechGrapState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grap','open','active','grap_nothing'])

    def execute(self, userdata):
        rospy.loginfo('checking gripper states')
        time.sleep(1)
        if node.flag_grap == node.grap_state['grap']:
            return 'grap'

        elif node.flag_grap == node.grap_state['open']:
            return 'open'

        elif node.flag_grap == node.grap_state['open'] & node.flag_grap_position < 1000 :
                    return 'grap_nothing'

        else:
            return 'active'

class PubBGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pubB'])

    def execute(self, userdata):
        node.task_states = node.task["place"]
        rospy.loginfo('Executing state publish Bgoal')
        node.flag_grap = node.grap_state['active']
        #time.sleep(1)
        rospy.wait_for_service('/send_task')
        print("connect service")
        try:
           #client = rospy.ServiceProxy('send_task', TaskCmd)
           #taskid = node.task.pubB
           resp1 = node.client(node.task['pubB'],node.A,node.B)
        except rospy.ServiceException as e:
               print ("Service call failed: %s"%e)
        return 'pubB'

class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['place'])

    def execute(self, userdata):
        node.task_states = node.task["place"]
        rospy.loginfo('Executing state pick')
        rospy.wait_for_service('/send_task')
        print("connect service")
        try:
           resp1 = node.client(node.task['place'],node.A,node.B)
           if resp1 == False:
             return 'finish'
        except rospy.ServiceException as e:
               print ("Service call failed: %s"%e)
        node.goal_flag = False
        return 'place'

def start(req):
    print("test")

    node.A = req.A_goal
    node.B = req.B_goal
    if req.id == 0:
        node.flag = True
    return OrderSrvResponse(1)

def IsReachGoal(msg):
    print(msg.status.status)
    if msg.status.status == 3:
        print("reach goal")
        node.goal_flag = True

def GripperStateCallback(msg):
    node.flag_grap = msg.grip_state
    node.flag_grap_position = msg.position

def main():
    rospy.init_node('smach_demo')
    rospy.Service('/goal_service',OrderSrv,start)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, IsReachGoal)
    rospy.Subscriber('/gripper/states', GripperState, GripperStateCallback)
    #signal.signal(signal.SIGINT, sys.exit())

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['over', 'outcome5'])



    # Open the container
    with sm:
        # Add states to the container
        # Create a sub SMACH state machine
        sm_action = smach.StateMachine(outcomes=['finish'])

        smach.StateMachine.add('WAITFORORDER', WaitFororder(),
                               transitions={
                                   'receive':'PUBAGOAL',
                                    'wait':'WAITFORORDER'})

        smach.StateMachine.add('PUBAGOAL', PubAGoal(),
                               transitions={'pubA':'WAITFORREACHGOAL' })

        smach.StateMachine.add('CALIBRATION', Calibration(),
                               transitions={
                                   'arrivalA' : 'PICK',
                                    'arrivalB' : 'PLACE',
                                    'data_empty' : 'over' })

        smach.StateMachine.add('WAITFORREACHGOAL', WaitForReachGoal(),
                                transitions={
                                    'no':'WAITFORREACHGOAL',
                                    'reachA':'CALIBRATION',
                                    'reachB':'CALIBRATION'})

        smach.StateMachine.add('CHECHGRAPSTATE', ChechGrapState(),
                                transitions={
                                    'grap':'PUBBGOAL',
                                    'open':'WAITFORORDER',
                                    'active':'CHECHGRAPSTATE',
                                    'grap_nothing':'CALIBRATION'})

        smach.StateMachine.add('PICK', Pick(),
                               transitions={
                                   'mission_success':'CHECHGRAPSTATE',
                                   'mission_fail':'CALIBRATION'})

        smach.StateMachine.add('PUBBGOAL', PubBGoal(),
                               transitions={'pubB':'WAITFORREACHGOAL'})


        smach.StateMachine.add('PLACE', Place(),
                               transitions={'place':'CHECHGRAPSTATE'})

        #smach.StateMachine.add('SUB',sm_action,transitions={''})

        #with sm_action:


        intro_server = smach_ros.IntrospectionServer('sm_IntrospectionServer', sm, '/SM_ROOT')
        intro_server.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
