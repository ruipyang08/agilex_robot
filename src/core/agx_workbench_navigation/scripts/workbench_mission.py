#!/usr/bin/env python3
"""ROS1 action wrapper for the registered workbench mission coordinator."""

from __future__ import annotations

import os
import json
import time

import actionlib
import rospkg
import rospy
import tf2_ros
import yaml
from std_msgs.msg import String

from agx_workbench_navigation.contracts import ActionResultCode
from agx_workbench_navigation.docking import DockingConfig
from agx_workbench_navigation.effective_config import docking_configuration, start_ros_report
from agx_workbench_navigation.rover_execution import RoverExecutionPolicy
from agx_workbench_navigation.mission import (
    MissionConfig,
    MissionCoordinator,
    internal_failure_result,
)
from agx_workbench_navigation.msg import (
    AuthorizedBaseCommand,
    NavigateToWorkbenchAction,
    NavigateToWorkbenchFeedback,
    NavigateToWorkbenchResult,
)
from agx_workbench_navigation.ros_mission_ports import (
    AuthorityPort,
    CameraPort,
    DockingPort,
    LidarPort,
    LocalizationPort,
    MoveBasePort,
    OdometryPort,
    RegistryPort,
    ReverseDeparturePort,
    TagPort,
)
from agx_workbench_navigation.wall_runtime import WallRate
from agx_workbench_navigation.ros_route_preflight import RoutePreflightPort
from agx_workbench_navigation.route_preflight import RouteStatus, route_failure_code


ACTION_TOPIC = "/workbench_nav/navigate"
DOCKING_COMMAND_TOPIC = "/workbench_nav/cmd_vel/docking"
CONTROL_RATE_HZ = 20.0
FINAL_ERROR_SOURCE = "pre_handoff_alvar_plus_odom"


def _load_docking_runtime():
    package = rospkg.RosPack().get_path("agx_workbench_navigation")
    path = os.path.join(package, "config/docking.yaml")
    with open(path, "r") as stream:
        document = yaml.safe_load(stream)
    values = dict(document["controller"])
    for name in (
        "tag_reacquisition_timeout_s",
        "visual_handoff_x_m",
        "visual_phase_timeout_s",
        "visual_timeout_s",
        "blind_timeout_s",
    ):
        values[name] = rospy.get_param(
            "~{}".format(name),
            values[name],
        )
    config = DockingConfig(**values)
    values = rospy.get_param('/rover_execution', None)
    if values is None:
        values = dict(velocity_scale=1.0, acceleration_scale=1.0,
                      default_coefficient=1.0, task_coefficients={})
    policy = RoverExecutionPolicy.from_mapping(values)
    return policy.apply_docking(config), policy


def _load_docking_config():
    return _load_docking_runtime()[0]


class WorkbenchMissionNode:
    def __init__(self):
        self._registry = RegistryPort()
        self._docking_config, self._execution_policy = _load_docking_runtime()
        self._command_operation_pub = rospy.Publisher('/workbench_nav/command_operation', String, queue_size=20)
        self._authority = AuthorityPort()
        self._navigation = MoveBasePort(self._authority)
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._localization = LocalizationPort(self._tf_buffer)
        self._camera = CameraPort()
        self._lidar = LidarPort()
        odometry_receipt_timeout_s = rospy.get_param("~odometry_receipt_timeout_s", 0.25)
        self._odometry = OdometryPort(
            self._docking_config,
            odometry_receipt_timeout_s,
        )
        self._tags = TagPort(self._tf_buffer)
        self._docking_pub = rospy.Publisher(
            DOCKING_COMMAND_TOPIC,
            AuthorizedBaseCommand,
            queue_size=1,
        )
        self._docking = DockingPort(
            self._docking_config,
            self._docking_pub,
            self._tags,
            self._odometry,
            self._lidar,
            self._registry.mode,
            self._registry.commissioning,
            self._authority,
        )
        self._departure = ReverseDeparturePort(
            self._docking_config,
            self._docking_pub,
            self._registry,
            self._localization,
            self._odometry,
            self._lidar,
            self._authority,
        )
        self._route_preflight = RoutePreflightPort(self._registry, self._localization,
                                                  self._docking_config.max_rollback_travel_m)
        self._server = actionlib.SimpleActionServer(
            ACTION_TOPIC,
            NavigateToWorkbenchAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()
        self._effective_report = start_ros_report("docking",
            docking_configuration(self._docking_config, self._execution_policy))

    def _coordinator(self):
        return MissionCoordinator(
            MissionConfig(
                localization_timeout_s=rospy.get_param(
                    "~localization_timeout_s",
                    10.0,
                ),
                camera_timeout_s=rospy.get_param(
                    "~camera_timeout_s",
                    10.0,
                ),
                departure_timeout_s=rospy.get_param(
                    "~departure_timeout_s",
                    60.0,
                ),
                navigation_timeout_s=rospy.get_param(
                    "~navigation_timeout_s",
                    120.0,
                ),
                settling_timeout_s=rospy.get_param(
                    "~settling_timeout_s",
                    5.0,
                ),
                tag_acquisition_timeout_s=rospy.get_param(
                    "~tag_acquisition_timeout_s",
                    10.0,
                ),
                authority_timeout_s=rospy.get_param(
                    "~authority_timeout_s",
                    3.0,
                ),
                docking_timeout_s=rospy.get_param(
                    "~docking_timeout_s",
                    240.0,
                ),
                final_settling_timeout_s=rospy.get_param(
                    "~final_settling_timeout_s",
                    3.0,
                ),
            ),
            registry=self._registry,
            navigation=self._navigation,
            localization=self._localization,
            camera=self._camera,
            tags=self._tags,
            lidar=self._lidar,
            authority=self._authority,
            docking=self._docking,
            departure=self._departure,
            odometry=self._odometry,
        )

    def _grant_observer(self, action_goal_id):
        # Capture once at invocation entry, never consult a later current_goal.
        def observed(event):
            self._command_operation_pub.publish(String(data=json.dumps(dict(event, action_goal_id=action_goal_id))))
        return observed

    def _execute(self, goal):
        coordinator = None
        last_feedback = None
        result = None
        try:
            action_goal_id = self._server.current_goal.get_goal_id().id
            if not goal.invocation_id and not goal.route_id:
                goal.task_deadline_s=time.monotonic()+float(rospy.get_param('~navigation_timeout_s',120.))
            self._authority.set_grant_observer(self._grant_observer(action_goal_id))
            admitted = self._route_preflight.admit_goal(goal, self._server.is_preempt_requested)
            if self._server.is_preempt_requested():
                self._server.set_preempted(NavigateToWorkbenchResult(code=int(ActionResultCode.CANCELLED),
                    message='cancelled during route admission'), 'cancelled during route admission')
                return
            if admitted.status != RouteStatus.READY:
                self._server.set_aborted(NavigateToWorkbenchResult(code=int(route_failure_code(admitted.status)),
                    message=admitted.detail), admitted.detail)
                return
            if not self._route_preflight.activate(goal, self._server.is_preempt_requested):
                self._server.set_aborted(NavigateToWorkbenchResult(code=int(ActionResultCode.ROUTE_INVALIDATED),
                    message='route could not be pinned to the current task deadline'))
                return
            self._navigation.configure_route(goal.invocation_id,goal.route_id,goal.task_deadline_s)
            self._departure.configure_route(admitted.route, self._route_preflight, goal,
                lambda: self._server.is_preempt_requested() or time.monotonic() >= goal.task_deadline_s)
            coordinator = self._coordinator()
            coordinator.start(
                goal.workbench_id,
                now_s=rospy.Time.now().to_sec(),
            )
            rate = WallRate(CONTROL_RATE_HZ)
            while not rospy.is_shutdown() and result is None:
                result = coordinator.step(
                    now_s=rospy.Time.now().to_sec(),
                    cancelled=(self._server.is_preempt_requested() or time.monotonic() >= goal.task_deadline_s),
                )
                feedback = coordinator.feedback
                if feedback is not None and feedback != last_feedback:
                    self._server.publish_feedback(self._feedback(feedback))
                    last_feedback = feedback
                if result is None:
                    rate.sleep()
            if result is None:
                result = coordinator.step(
                    now_s=rospy.Time.now().to_sec(),
                    cancelled=True,
                )
            if result is None:
                raise RuntimeError("mission ended without a terminal result")
            action_result = self._result(result)
            if result.code == ActionResultCode.OK:
                self._server.set_succeeded(action_result, result.message)
            elif result.code == ActionResultCode.CANCELLED:
                self._server.set_preempted(action_result, result.message)
            else:
                self._server.set_aborted(action_result, result.message)
        except Exception as error:
            rospy.logerr("workbench mission wrapper failure: %s", error)
            if coordinator is not None and not coordinator.terminal:
                try:
                    result = coordinator.abort_internal(
                        now_s=rospy.Time.now().to_sec()
                    )
                except Exception:
                    result = internal_failure_result(
                        "wrapper_cleanup_failure"
                    )
            else:
                result = internal_failure_result("wrapper_execution_failure")
            try:
                self._server.set_aborted(
                    self._result(result),
                    result.message,
                )
            except Exception as terminal_error:
                rospy.logerr(
                    "unable to terminate workbench action: %s",
                    terminal_error,
                )
        finally:
            self._authority.set_grant_observer(None)
            # Bounded, best-effort exact-zero cleanup; the coordinator retains
            # the primary failure and records any cleanup diagnostics first.
            if coordinator is not None:
                try:
                    self._docking.stop()
                except Exception:
                    pass
                try:
                    self._navigation.cancel()
                except Exception:
                    pass
                try:
                    if not coordinator.cleanup_done:
                        self._authority.release()
                except Exception:
                    pass
            self._route_preflight.release(goal)
            # Failure-only formatting follows terminal reporting and every
            # physical cleanup attempt, outside the command/safety poll lock.
            try:
                self._docking.log_motion_progress_failure()
            except Exception:
                pass

    @staticmethod
    def _feedback(value):
        message = NavigateToWorkbenchFeedback()
        message.phase = int(value.phase)
        message.phase_name = value.phase_name
        message.detail = value.detail
        message.position_error_m = value.position_error_m
        message.yaw_error_rad = value.yaw_error_rad
        message.docking_attempt = value.docking_attempt
        return message

    @staticmethod
    def _result(value):
        if (
            value.final_error_is_estimated
            and value.final_error_source != FINAL_ERROR_SOURCE
        ):
            raise ValueError("invalid final error evidence source")
        message = NavigateToWorkbenchResult()
        message.code = int(value.code)
        message.message = value.message
        message.final_position_error_m = value.final_position_error_m
        message.final_yaw_error_rad = value.final_yaw_error_rad
        message.final_error_is_estimated = value.final_error_is_estimated
        message.final_error_source = value.final_error_source
        return message


def main():
    rospy.init_node("workbench_mission")
    WorkbenchMissionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
