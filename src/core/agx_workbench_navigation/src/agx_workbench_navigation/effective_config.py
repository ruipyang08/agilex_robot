"""Read-only reports of cached producer settings, frozen for a process instance."""

import json
from dataclasses import asdict
import threading
import uuid


BASE_FIELDS = (
    'max_linear_x_mps', 'max_linear_y_mps', 'max_angular_z_radps',
    'max_linear_speed_lead_mps', 'max_angular_speed_lead_radps',
    'min_turn_radius_m', 'odometry_receipt_s', 'odometry_header_s',
    'maximum_header_future_skew_s',
)
DOCKING_FIELDS = (
    'max_yaw_speed_radps', 'max_lateral_speed_mps', 'max_visual_forward_speed_mps',
    'max_blind_speed_mps', 'max_rollback_speed_mps',
    'max_visual_linear_acceleration_mps2', 'max_visual_angular_acceleration_radps2',
    'max_blind_acceleration_mps2', 'max_rollback_acceleration_mps2',
)
LIDAR_FIELDS = ('front_x_m', 'rear_x_m', 'left_y_m', 'right_y_m',
                'min_obstacle_z_m', 'max_obstacle_z_m', 'self_padding_m')


def base_configuration(config):
    values = {name: getattr(config, name) for name in BASE_FIELDS}
    values['pose_feedback'] = asdict(config.pose_feedback)
    values['final_command_policy'] = 'measured_progress_contraction'
    values['normal_slew_clock'] = 'producer_source_time'
    return values


def docking_configuration(config, policy):
    values = {name: getattr(config, name) for name in DOCKING_FIELDS}
    values['rover_execution'] = {
        'velocity_scale': policy.velocity_scale,
        'acceleration_scale': policy.acceleration_scale,
        'default_coefficient': policy.default_coefficient,
        'task_coefficients': dict(policy.task_coefficients),
    }
    return values


def lidar_configuration(config,self_exclusion_xy_m=None):
    result={name: getattr(config.envelope,name) for name in LIDAR_FIELDS}
    if self_exclusion_xy_m is not None:
        result['self_exclusion_xy_m']=[list(p) for p in self_exclusion_xy_m]
    return result


class EffectiveConfigReporter:
    """Serialize once; repeat identical bytes on a wall-clock heartbeat.

    A new instance gets a new session. Reports are observations of loaded
    settings, not a facility for changing the producer or granting motion.
    """

    def __init__(self, component, configuration, publish):
        if component not in ('base_arbiter', 'docking', 'lidar'):
            raise ValueError('unknown effective configuration component')
        self._payload = json.dumps({
            'schema_version': 1, 'component': component,
            'session_id': str(uuid.uuid4()), 'configuration': configuration,
        }, sort_keys=True, separators=(',', ':'), allow_nan=False)
        self._publish = publish
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run,
            name='effective-config-' + component, daemon=True)

    def publish_once(self):
        self._publish(self._payload)

    def start(self):
        self.publish_once()
        self._thread.start()

    def _run(self):
        while not self._stop.wait(.2):
            self.publish_once()

    def close(self):
        self._stop.set()
        if self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=.5)


def start_ros_report(component, configuration):
    """Attach one latched ROS report stream; no ROS-time timer or param reads."""
    import rospy
    from std_msgs.msg import String
    publisher = rospy.Publisher('/workbench_nav/effective_config/' + component,
                                String, queue_size=1, latch=True)
    reporter = EffectiveConfigReporter(component, configuration,
                                        lambda payload: publisher.publish(String(data=payload)))
    rospy.on_shutdown(reporter.close)
    reporter.start()
    return reporter
