#!/usr/bin/env python3
"""Run the real host against isolated ROS fakes, never a robot/simulator graph.

Success paths through physical MoveIt execution belong to simulation acceptance.
These tests cover full admission, repeatable recipe composition, and failures
before motion; fake joint telemetry does not certify physical travel posture.
"""
import copy
import json
from pathlib import Path
import subprocess
import signal
import time
import tempfile
import threading
import unittest
import uuid

import actionlib
import rospkg
import rospy
import rostest
import yaml
from actionlib_msgs.msg import GoalStatusArray
from agx_workbench_navigation.msg import (BaseAuthorityState, NavigateToWorkbenchAction,
    TravelReadiness, PlannedMotionSegment, PlannedMotionSample)
from agx_workbench_navigation.srv import (AcquireBaseHold, AcquireBaseHoldResponse,
    PrepareBaseDeparture, PrepareBaseDepartureResponse, ReleaseBaseHold, ReleaseBaseHoldResponse,
    PrepareNavigationRoute, PrepareNavigationRouteResponse)
from bt_task_msgs.msg import LiftMotorMsg
from bt_task_msgs.srv import LiftMotorSrv, LiftMotorSrvResponse
from dh_gripper_msgs.msg import GripperState
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from moveit_msgs.msg import MoveGroupActionGoal, ExecuteTrajectoryActionGoal
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneResponse
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


class TaskWorkflowRos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.directory = tempfile.TemporaryDirectory(prefix='task_workflow_ros_')
        cls.root = Path(cls.directory.name)
        nav = Path(rospkg.RosPack().get_path('agx_workbench_navigation'))
        package = Path(rospkg.RosPack().get_path('agx_robot_workflow'))
        cls.profile = yaml.safe_load((package/'test/workflow_ros_travel.yaml').read_text())
        cls.report_configs = yaml.safe_load((package/'test/workflow_ros_effective.yaml').read_text())
        cls.report_bytes = {}
        cls.report_publishers = {role: rospy.Publisher('/workbench_nav/effective_config/'+role,
            String, queue_size=1, latch=True) for role in cls.report_configs}
        (cls.root/'registry.yaml').write_text(yaml.safe_dump({'workbenches': [{'id': 'wb-100'}, {'id': 'wb-101'}]}))
        rospy.set_param('/workbench_mission/registry_path', str(cls.root/'registry.yaml'))
        rospy.set_param('/base_command_arbiter/config_file', str(nav/'config/authority.yaml'))
        rospy.set_param('/workbench_mission/parking_config', str(nav/'config/parking.yaml'))
        cls.publishers = {
            'authority': rospy.Publisher('/workbench_nav/authority_state', BaseAuthorityState, queue_size=10),
            'odom': rospy.Publisher('/odom', Odometry, queue_size=10),
            'command': rospy.Publisher('/cmd_vel', Twist, queue_size=10),
            'watchdog': rospy.Publisher('/workbench_workflow/watchdog_ready', Bool, queue_size=10),
            'goal': rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10),
            'status': rospy.Publisher('/move_base/status', GoalStatusArray, queue_size=10),
            'execution': rospy.Publisher('/execute_trajectory/status', GoalStatusArray, queue_size=10),
            'planning': rospy.Publisher('/move_group/status', GoalStatusArray, queue_size=10),
            'joints': rospy.Publisher('/test/joints', JointState, queue_size=10),
            'gripper': rospy.Publisher('/test/gripper', GripperState, queue_size=10),
            'lift': rospy.Publisher('/test/lift_state', LiftMotorMsg, queue_size=10),
        }
        cls.services = [
            rospy.Service('/workbench_nav/prepare_navigation_route', PrepareNavigationRoute, cls.route),
            rospy.Service('/workbench_nav/hold_for_manipulation', Trigger, cls.hold),
            rospy.Service('/workbench_nav/acquire_base_hold', AcquireBaseHold, cls.acquire),
            rospy.Service('/workbench_nav/prepare_base_departure', PrepareBaseDeparture, cls.prepare),
            rospy.Service('/workbench_nav/release_base_hold', ReleaseBaseHold, cls.release),
            rospy.Service('/test/lift_command', LiftMotorSrv, cls.lift_command),
            rospy.Service('/get_planning_scene', GetPlanningScene, cls.scene),
        ]
        cls.subscribers = [
            rospy.Subscriber('/trajectory_execution_event', String, cls.stop_event),
            rospy.Subscriber('/workbench_nav/travel_readiness', TravelReadiness, lambda _: None),
            rospy.Subscriber('/move_group/goal', MoveGroupActionGoal, cls.actuator),
            rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, cls.actuator),
        ]
        cls.server = actionlib.SimpleActionServer('/workbench_nav/navigate', NavigateToWorkbenchAction,
                                                  execute_cb=cls.navigate, auto_start=False)
        cls.server.start()
        cls.reset()
        cls.timer = rospy.Timer(rospy.Duration(.02), cls.publish_state)

    @classmethod
    def tearDownClass(cls):
        cls.timer.shutdown()
        cls.directory.cleanup()

    @classmethod
    def reset(cls):
        with cls.lock:
            cls.calls = dict(route=0, navigate=0, acquire=0, prepare=0, release=0, actuator=0, scene=0,
                             legacy_hold=0, stop=0, lift=0)
            cls.held = False
            cls.route_status = PrepareNavigationRouteResponse.READY
            cls.route_wrong_invocation = False
            cls.route_delay_s = 0
            cls.acquire_mode = 'reject'
            cls.stale_authority = cls.stale_status = False
            cls.grip_state = 1
            cls.watchdog_enabled = True
            cls.block_stop_rpc = ''
            cls.stop_rpc_entered = threading.Event()
            cls.stop_rpc_release = threading.Event()
            cls.stop_rpc_finished = threading.Event()
            cls.report_bytes = {role: json.dumps(dict(schema_version=1, component=role,
                session_id='isolated-fake-'+role, configuration=config), sort_keys=True)
                for role, config in cls.report_configs.items()}

    def setUp(self):
        if rospy.get_param('~stop_rpc_only', False) and not self._testMethodName.startswith('test_stop_rpc_'):
            self.skipTest('focused stop-RPC regression launch')
        self.reset()
        self.travel = self.root/'travel.yaml'
        self.travel.write_text(yaml.safe_dump(self.profile))
        # Navigation admission is deliberately independent of grasp configuration.
        if rospy.has_param('/tube_grasp'):
            rospy.delete_param('/tube_grasp')

    @classmethod
    def route(cls, request):
        with cls.lock:
            cls.calls['route'] += 1
            status, wrong, delay = cls.route_status, cls.route_wrong_invocation, cls.route_delay_s
        time.sleep(delay)
        response = PrepareNavigationRouteResponse(status=status, detail='isolated read-only route')
        if status == response.READY:
            response.route.header.frame_id = 'map'
            response.route.header.stamp = rospy.Time.now()
            response.route.invocation_id = 'another-invocation' if wrong else request.invocation_id
            response.route.route_id = 'fake-route-'+request.invocation_id
            response.route.model_digest = 'installed-fake-model'
            response.route.map_digest = 'installed-fake-map'
            response.route.registry_digest = 'installed-fake-registry'
            segment = PlannedMotionSegment()
            segment.samples = [PlannedMotionSample(), PlannedMotionSample()]
            response.route.segments = [segment]
        return response

    @classmethod
    def hold(cls, _request):
        with cls.lock:
            cls.calls['legacy_hold'] += 1
            cls.held = True
        cls.wait_stop_reply('hold')
        return TriggerResponse(success=True, message='fake stopped hold')

    @classmethod
    def wait_stop_reply(cls, role):
        with cls.lock:
            blocked = cls.block_stop_rpc == role
            entered, release, finished = cls.stop_rpc_entered, cls.stop_rpc_release, cls.stop_rpc_finished
        if blocked:
            entered.set()
            try:
                if not release.wait(15):
                    raise rospy.ServiceException('test stop reply was not released within its bound')
            finally: finished.set()

    @classmethod
    def acquire(cls, _request):
        with cls.lock:
            cls.calls['acquire'] += 1
            cls.held = cls.acquire_mode != 'reject'
            return AcquireBaseHoldResponse(accepted=cls.held, stopped_confirmed=cls.held,
                session_id='wrong-session' if cls.acquire_mode == 'mismatch' else 'fake-base',
                hold_epoch=1, reason='injected hold rejection' if not cls.held else 'fake hold')

    @classmethod
    def prepare(cls, _request):
        with cls.lock: cls.calls['prepare'] += 1
        return PrepareBaseDepartureResponse(accepted=False, reason='test never permits departure')

    @classmethod
    def release(cls, _request):
        with cls.lock: cls.calls['release'] += 1
        return ReleaseBaseHoldResponse(accepted=False, reason='test never releases hold')

    @classmethod
    def lift_command(cls, request):
        with cls.lock:
            cls.calls['lift'] += 1
            if request.mode != -2: cls.calls['actuator'] += 1
        if request.mode == -2: cls.wait_stop_reply('lift')
        return LiftMotorSrvResponse(state=1)

    @classmethod
    def scene(cls, _request):
        with cls.lock: cls.calls['scene'] += 1
        return GetPlanningSceneResponse()

    @classmethod
    def stop_event(cls, _message):
        with cls.lock: cls.calls['stop'] += 1

    @classmethod
    def actuator(cls, _message):
        with cls.lock: cls.calls['actuator'] += 1

    @classmethod
    def navigate(cls, _goal):
        with cls.lock: cls.calls['navigate'] += 1
        cls.server.set_aborted(text='unexpected navigation dispatch in pre-actuator test')

    @classmethod
    def publish_state(cls, _event):
        now = rospy.Time.now()
        with cls.lock:
            held, old_authority, old_status, grip = cls.held, cls.stale_authority, cls.stale_status, cls.grip_state
            reports = dict(cls.report_bytes)
            watchdog_enabled = cls.watchdog_enabled
        authority = BaseAuthorityState()
        authority.header.stamp = now-rospy.Duration(10) if old_authority else now
        authority.state = BaseAuthorityState.NONE
        authority.session_id = 'fake-base'
        authority.hold_epoch = 1
        authority.manipulation_hold_active = held
        cls.publishers['authority'].publish(authority)
        odom = Odometry(); odom.header.stamp = now; odom.pose.pose.orientation.w = 1.
        cls.publishers['odom'].publish(odom)
        cls.publishers['command'].publish(Twist())
        cls.publishers['watchdog'].publish(Bool(data=watchdog_enabled))
        status = GoalStatusArray(); status.header.stamp = now-rospy.Duration(10) if old_status else now
        cls.publishers['status'].publish(status)
        status = GoalStatusArray(); status.header.stamp = now
        cls.publishers['execution'].publish(status); cls.publishers['planning'].publish(status)
        joints = JointState(); joints.header.stamp = now
        joints.name = ['joint'+str(i) for i in range(1,7)]
        joints.position = [0.]*6; joints.velocity = [0.]*6
        cls.publishers['joints'].publish(joints)
        gripper = GripperState(); gripper.header.stamp = now; gripper.is_initialized = True
        gripper.grip_state = grip
        gripper.position = gripper.target_position = 0. if grip == 2 else 1000.
        cls.publishers['gripper'].publish(gripper)
        lift = LiftMotorMsg(); lift.header.stamp = now; lift.initState = True
        cls.publishers['lift'].publish(lift)
        for role, payload in reports.items(): cls.report_publishers[role].publish(String(data=payload))

    @staticmethod
    def tasks():
        return [dict(id='a', type='navigate', target='wb-100'),
                dict(id='b', type='navigate', target='wb-101'),
                dict(id='a_again', type='navigate', target='wb-100')]

    def invoke(self, tasks=None, check=False, mode='simulation', include_profile=True, task_timeout=None):
        # ROS private parameters survive process exit; each admission starts clean.
        if rospy.has_param("/workbench_workflow"):
            rospy.delete_param("/workbench_workflow")
        self.publish_state(None)
        config = self.root/'recipe.yaml'; summary = self.root/'summary.json'
        if summary.exists(): summary.unlink()
        config.write_text(yaml.safe_dump(dict(schema_version=2, workflow_id='isolated_fake',
                                             tasks=self.tasks() if tasks is None else tasks)))
        self.expected_run_id = uuid.uuid4().hex
        command = ['rosrun', 'agx_robot_workflow', 'run_robot_workflow',
                   '_run_id:='+self.expected_run_id,
                   '_config:='+str(config), '_summary:='+str(summary),
                   '_check_only:='+str(check).lower(), '_execution_mode:='+mode]
        if include_profile: command.append('_travel_profile:='+str(self.travel))
        if task_timeout is not None: command.append('_task_timeout_s:='+task_timeout)
        result = subprocess.run(command, capture_output=True, text=True, timeout=40)
        self.assertTrue(summary.exists(), result.stdout+result.stderr)
        if check and result.returncode == 0:
            self.assertIn('Validation passed. No tasks were executed.', result.stdout)
        return result.returncode, json.loads(summary.read_text())

    def assert_no_dispatch(self):
        with self.lock: self.assertEqual(self.calls, {key: 0 for key in self.calls})

    def assert_no_actuation(self):
        with self.lock:
            for key in ('navigate', 'prepare', 'release', 'actuator', 'scene'):
                self.assertEqual(self.calls[key], 0, (key, self.calls))

    def test_check_only_repeated_navigation_needs_no_grasp_configuration(self):
        code, report = self.invoke(check=True)
        self.assertEqual(code, 0, report)
        self.assertEqual(report['run_id'], self.expected_run_id)
        self.assertTrue(report['valid'] and report['validation_only'])
        self.assertEqual(report['completion'], 'Validated')
        self.assertEqual(report['task_timeout_s'], 900.0)
        self.assertEqual(report['task_timeout_clock'], 'steady')
        self.assertFalse(report['success'] or report['grasp_dispatched'])
        self.assertEqual([row['id'] for row in report['tasks']], ['a','b','a_again'])
        self.assertTrue(all(row['status']=='not_run' and not row['dispatched'] for row in report['tasks']))
        self.assert_no_dispatch()

    def test_private_task_timeout_report_and_rejection(self):
        for value, expected in [('1800', 1800.0), ('3600.25', 3600.25)]:
            with self.subTest(value=value):
                code, report = self.invoke(check=True, task_timeout=value)
                self.assertEqual(code, 0, report)
                self.assertTrue(report['valid'] and report['validation_only'])
                self.assertEqual(report['task_timeout_s'], expected)
                self.assertEqual(report['task_timeout_clock'], 'steady')
                self.assert_no_dispatch()
        for value in ['0', '-1', 'true', '"1800"', '[]', '.nan', '.inf', '-.inf', '1.0e+309', '1.0e-300']:
            with self.subTest(value=value):
                code, report = self.invoke(check=True, task_timeout=value)
                self.assertNotEqual(code, 0, report)
                self.assertFalse(report['valid'])
                self.assertEqual(report['completion'], 'Failed')
                self.assertIsNone(report['task_timeout_s'])
                self.assertIn('task_timeout_s', report['detail'])
                self.assert_no_dispatch()

    def test_check_only_does_not_replace_active_workflow_node(self):
        type(self).acquire_mode='mismatch'
        config=self.root/'active.yaml'; summary=self.root/'active_summary.json'
        config.write_text(yaml.safe_dump(dict(schema_version=2, workflow_id='active', tasks=self.tasks()[:1])))
        process=subprocess.Popen(['rosrun','agx_robot_workflow','run_robot_workflow',
            '_config:='+str(config),'_summary:='+str(summary), '_execution_mode:=simulation',
            '_travel_profile:='+str(self.travel)], stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        try:
            deadline=time.monotonic()+5
            while self.calls['acquire']==0 and time.monotonic()<deadline:
                time.sleep(.02)
            self.assertGreater(self.calls['acquire'],0)
            code,report=self.invoke(check=True)
            self.assertEqual(code,0,report)
            self.assertIsNone(process.poll(), 'check-only replaced or signalled the executing ROS node')
            self.assert_no_actuation()
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=20)

    def test_entry_waits_for_initial_watchdog_handshake(self):
        with self.lock: type(self).watchdog_enabled = False
        started = threading.Event()
        timers = []
        def become_ready():
            with self.lock: type(self).watchdog_enabled = True
        def phase(message):
            if message.data == 'NAVIGATE' and not started.is_set():
                started.set()
                timer = threading.Timer(.5, become_ready)
                timers.append(timer); timer.start()
        subscriber = rospy.Subscriber('/workbench_workflow/phase', String, phase, queue_size=10)
        try:
            code, report = self.invoke()
            self.assertTrue(started.is_set())
            self.assertNotEqual(code, 0)
            self.assertNotIn('entry check:', report['detail'])
            with self.lock: self.assertGreater(self.calls['acquire'], 0)
            self.assert_no_actuation()
        finally:
            subscriber.unregister()
            for timer in timers: timer.join()

    def test_invalid_later_task_prevents_every_command(self):
        tasks=self.tasks(); tasks[1]['type']='not_implemented'
        code, report=self.invoke(tasks)
        self.assertNotEqual(code,0); self.assertEqual(report['failed_task_id'],'b')
        self.assertEqual(report['failed_phase'],'VALIDATE'); self.assert_no_dispatch()

    def test_missing_travel_profile_prevents_every_command(self):
        code, report=self.invoke(include_profile=False)
        self.assertNotEqual(code,0); self.assertIn('travel_profile',report['detail'])
        self.assert_no_dispatch()

    def test_simulation_fixture_cannot_admit_hardware(self):
        code, report=self.invoke(mode='hardware')
        self.assertNotEqual(code,0); self.assertIn('not qualified',report['detail'])
        self.assert_no_dispatch()

    def test_hardware_label_cannot_enable_unvalidated_controller_splice(self):
        profile=copy.deepcopy(self.profile)
        profile['hardware']={'enabled': True, 'evidence_id': 'claimed_hardware_qualification'}
        self.travel.write_text(yaml.safe_dump(profile))
        code, report=self.invoke(mode='hardware')
        self.assertNotEqual(code,0); self.assertIn('initial trajectory bridge',report['detail'])
        self.assert_no_dispatch()

    def test_unqualified_later_profile_prevents_first_task(self):
        profile=copy.deepcopy(self.profile); profile['simulation']['enabled']=False
        other=self.root/'disabled.yaml'; other.write_text(yaml.safe_dump(profile))
        tasks=self.tasks(); tasks[1]['config']=str(other)
        code, report=self.invoke(tasks)
        self.assertNotEqual(code,0); self.assertEqual(report['failed_task_id'],'b')
        self.assert_no_dispatch()

    def test_too_fast_effective_controls_prevent_every_command(self):
        with self.lock:
            payload=json.loads(self.report_bytes['base_arbiter'])
            payload['configuration']['max_linear_x_mps']=.5
            self.report_bytes['base_arbiter']=json.dumps(payload,sort_keys=True)
        code, report=self.invoke(check=True)
        self.assertNotEqual(code,0); self.assertIn('speed',report['detail'])
        self.assert_no_dispatch()

    def test_rejected_typed_hold_never_reaches_preparation(self):
        code, report=self.invoke(self.tasks()[:1])
        self.assertNotEqual(code,0); self.assertIn('injected hold rejection',report['detail'])
        self.assertEqual(report['failed_phase'],'PREPARE_DEPARTURE')
        self.assertGreater(self.calls['acquire'],0); self.assert_no_actuation()

    def test_no_route_prevents_hold_and_physical_preparation(self):
        type(self).route_status = PrepareNavigationRouteResponse.NO_ROUTE
        code, report = self.invoke()
        self.assertNotEqual(code, 0)
        self.assertEqual(report['failed_phase'], 'PLAN_ROUTE', report)
        self.assertIn('23:', report['detail'])
        self.assertEqual(self.calls['route'], 1)
        self.assertEqual(self.calls['acquire'], 0)
        self.assert_no_actuation()

    def test_other_invocations_route_cannot_prepare_or_release(self):
        type(self).route_wrong_invocation = True
        code, report = self.invoke()
        self.assertNotEqual(code, 0)
        self.assertEqual(report['failed_phase'], 'PLAN_ROUTE', report)
        self.assertEqual(self.calls['acquire'], 0)
        self.assert_no_actuation()

    def test_late_route_response_cannot_prepare_or_release(self):
        type(self).route_delay_s = 2
        code, report = self.invoke(task_timeout='0.5')
        self.assertNotEqual(code, 0)
        self.assertEqual(report['failed_phase'], 'PLAN_ROUTE', report)
        self.assertEqual(self.calls['route'], 1)
        self.assertEqual(self.calls['acquire'], 0)
        self.assert_no_actuation()

    def test_latched_watchdog_stop_still_allows_measured_stop_confirmation(self):
        armed = threading.Event()
        def phase(message):
            if message.data == 'NAVIGATE': armed.set()
            if message.data == 'STOPPING' and armed.is_set():
                with self.lock: type(self).watchdog_enabled = False
        subscriber = rospy.Subscriber('/workbench_workflow/phase', String, phase, queue_size=10)
        try:
            code, report = self.invoke(self.tasks()[:1])
            self.assertNotEqual(code, 0)
            self.assertEqual(report['failed_phase'], 'PREPARE_DEPARTURE')
            self.assertTrue(report['tasks'][0]['stop_confirmed'], report)
            with self.lock:
                self.assertFalse(type(self).watchdog_enabled)
                self.assertGreater(self.calls['legacy_hold'], 0)
            self.assert_no_actuation()
        finally:
            subscriber.unregister()

    def test_different_session_hold_response_never_starts_preparation(self):
        type(self).acquire_mode='mismatch'
        code, report=self.invoke(self.tasks()[:1])
        self.assertNotEqual(code,0); self.assertEqual(report['failed_phase'],'PREPARE_DEPARTURE')
        self.assertGreater(self.calls['acquire'],0); self.assert_no_actuation()

    def test_fresh_receipt_cannot_hide_stale_authority_source(self):
        type(self).acquire_mode='accept'; type(self).stale_authority=True
        code, report=self.invoke(self.tasks()[:1])
        self.assertNotEqual(code,0); self.assertEqual(report['failed_phase'],'PREPARE_DEPARTURE')
        self.assert_no_actuation()

    def test_fresh_receipt_cannot_hide_stale_action_status_source(self):
        type(self).acquire_mode='accept'; type(self).stale_status=True
        code, report=self.invoke(self.tasks()[:1])
        self.assertNotEqual(code,0); self.assertEqual(report['failed_phase'],'PREPARE_DEPARTURE')
        self.assert_no_actuation()

    def test_unidentified_caught_gripper_never_dispatches_arm_motion(self):
        type(self).acquire_mode='accept'; type(self).grip_state=2
        code, report=self.invoke(self.tasks()[:1])
        self.assertNotEqual(code,0); self.assertEqual(report['failed_phase'],'PREPARE_TRAVEL')
        self.assert_no_actuation()

    def stop_rpc_reply_case(self, role, timeout=False):
        with self.lock: type(self).block_stop_rpc = role
        results=[];errors=[]
        def run():
            try: results.append(self.invoke())
            except BaseException as error: errors.append(error)
        worker=threading.Thread(target=run)
        worker.start()
        try:
            self.assertTrue(self.stop_rpc_entered.wait(8), 'owned stop RPC did not arrive')
            if timeout:
                worker.join(timeout=12)
                self.assertFalse(worker.is_alive(), 'blocked stop RPC defeated bounded process exit')
            else:
                # Fresh zero/held/terminal arm/lift telemetry continues throughout
                # this wait. Only the owned RPC reply is withheld.
                time.sleep(1.2)
                self.assertTrue(worker.is_alive(), 'host confirmed stop before its stop RPC returned')
                self.assertFalse((self.root/'summary.json').exists(), 'summary escaped pending stop RPC')
                self.stop_rpc_release.set()
                worker.join(timeout=8)
                self.assertFalse(worker.is_alive())
            self.assertFalse(errors, errors)
            self.assertEqual(len(results),1)
            code,report=results[0]
            self.assertNotEqual(code,0)
            self.assertEqual(report['failed_phase'],'PREPARE_DEPARTURE')
            self.assertFalse(any(task['dispatched'] for task in report['tasks'][1:]), report)
            self.assertEqual(report['tasks'][0]['stop_confirmed'],not timeout,report)
            self.assertEqual(report['blocked_child_exit'],timeout,report)
            if timeout:
                self.assertIn(('lift' if role=='lift' else 'hold')+'_stop_rpc_pending=true',report['detail'])
            with self.lock:
                self.assertEqual(self.calls['lift'],1,self.calls)
                self.assertEqual(self.calls['legacy_hold'],1,self.calls)
            self.assert_no_actuation()
        finally:
            self.stop_rpc_release.set()
            worker.join(timeout=40)
            if self.stop_rpc_entered.is_set(): self.assertTrue(self.stop_rpc_finished.wait(2))

    def test_stop_rpc_lift_reply_must_join_before_confirmation(self):
        self.stop_rpc_reply_case('lift')

    def test_stop_rpc_hold_reply_must_join_before_confirmation(self):
        self.stop_rpc_reply_case('hold')

    def test_stop_rpc_lift_timeout_is_terminal(self):
        self.stop_rpc_reply_case('lift',timeout=True)

    def test_stop_rpc_hold_timeout_is_terminal(self):
        self.stop_rpc_reply_case('hold',timeout=True)


if __name__ == '__main__':
    rospy.init_node('workbench_mission')
    rostest.rosrun('agx_robot_workflow', 'task_workflow_ros', TaskWorkflowRos)
