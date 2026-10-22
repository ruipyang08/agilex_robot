#!/usr/bin/env python3
"""Stop the task workflow through stock ROS1 interfaces if its process disappears.

No task dispatch, recovery or hold release lives here. A fault stays latched
until application restart, and the gripper command is deliberately preserved.
"""
import threading
import time

import rospy
from actionlib_msgs.msg import GoalID
from bt_task_msgs.srv import LiftMotorSrv
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class Watchdog:
    def __init__(self):
        self.last_seen = 0.0
        self.active = False
        self.actuator_stop_armed = False
        self.failed = False
        self.rpc_lock = threading.Lock()
        self.hold_busy = threading.Event()
        self.lift_busy = threading.Event()
        self.arm = rospy.Publisher('/trajectory_execution_event', String, queue_size=1)
        self.cancel = rospy.Publisher('/workbench_nav/navigate/cancel', GoalID, queue_size=1)
        self.stop = rospy.Publisher('/workbench_workflow/stop', Bool, queue_size=1, latch=True)
        self.ready = rospy.Publisher('/workbench_workflow/watchdog_ready', Bool, queue_size=1)
        self.hold = rospy.ServiceProxy('/workbench_nav/hold_for_manipulation', Trigger)
        service = rospy.get_param('/tube_grasp/lift/service_name', '/lifter_1/LiftingMotorService')
        self.lift = rospy.ServiceProxy(service, LiftMotorSrv)
        self.sub = rospy.Subscriber('/workbench_workflow/phase', String, self.phase, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def phase(self, message):
        self.last_seen = time.monotonic()
        if message.data in ('NAVIGATE', 'VERIFY_STOP', 'HOLD_BASE', 'GRASP', 'TASK_RUNNING', 'TASK_FAILED'):
            self.active = True
            # Any registered task may use the known arm/lift interfaces.
            self.actuator_stop_armed = True
        if message.data in ('STOPPING', 'FAILED') and self.active:
            self.failed = True
        if message.data in ('SUCCEEDED', 'COMPLETED_WITH_FAILURES'):
            self.active = False

    def hold_rpc(self):
        try:
            self.hold()
        except rospy.ServiceException:
            pass
        finally:
            self.hold_busy.clear()

    def lift_rpc(self):
        try:
            self.lift(mode=-2, val=0)
        except rospy.ServiceException:
            pass
        finally:
            self.lift_busy.clear()

    def request_stop(self):
        self.stop.publish(Bool(data=True))
        self.cancel.publish(GoalID())
        if self.actuator_stop_armed:
            self.arm.publish(String(data='stop'))
        # Each service gets at most one blocked worker. Neither can suppress
        # the other stop path or the independent publishers above.
        with self.rpc_lock:
            for enabled, busy, rpc in ((True, self.hold_busy, self.hold_rpc),
                                       (self.actuator_stop_armed, self.lift_busy, self.lift_rpc)):
                if enabled and not busy.is_set():
                    busy.set()
                    threading.Thread(target=rpc, daemon=True).start()

    def shutdown(self):
        if self.active:
            self.request_stop()

    def run(self):
        while not rospy.is_shutdown():
            if self.active and time.monotonic() - self.last_seen > 1.0:
                if not self.failed:
                    rospy.logerr('workflow heartbeat lost; latching stop and base hold')
                self.failed = True
            if self.failed:
                self.request_stop()
            self.ready.publish(Bool(data=not self.failed))
            time.sleep(0.1)  # Wall time: a paused simulation clock cannot suspend supervision.


if __name__ == '__main__':
    rospy.init_node('workbench_workflow_watchdog')
    Watchdog().run()
