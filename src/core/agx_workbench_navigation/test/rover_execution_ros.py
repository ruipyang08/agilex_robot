#!/usr/bin/env python3
"""Exercise actual Noetic launch parsing and docking parameter loading."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import roslaunch.config
import rospkg
import rospy
import rostest
import yaml


class RoverExecutionRosTest(unittest.TestCase):
    def test_atomic_policy_reload_and_production_docking_loader(self):
        package = Path(rospkg.RosPack().get_path('agx_workbench_navigation'))
        spec = importlib.util.spec_from_file_location('mission_entrypoint', str(package / 'scripts/workbench_mission.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        original = yaml.safe_load((package / 'config/rover_execution.yaml').read_text())
        rospy.set_param('/rover_execution', original)
        baseline = module._load_docking_config()
        policy = dict(velocity_scale=.8, acceleration_scale=.6,
                      default_coefficient=.5, task_coefficients={'final_approach': .75})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'policy.yaml'
            for overrides, expected in (({'final_approach': .75}, .6), ({}, .4)):
                policy['task_coefficients'] = overrides
                path.write_text(yaml.safe_dump(policy))
                config = roslaunch.config.load_config_default(
                    [(str(package / 'launch/workbench_application.launch'),
                      ['map_file:=unused.yaml', 'rover_execution_config_file:=' + str(path)])],
                    11311, verbose=False)
                policy_params = {name: param.value for name, param in config.params.items()
                                 if name == '/rover_execution' or name.startswith('/rover_execution/')}
                self.assertEqual(policy_params, {'/rover_execution': policy})
                # roslaunch uploads each Param with setParam; preserve the
                # exact parsed key/value boundaries on a real parameter server.
                for name, value in policy_params.items():
                    rospy.set_param(name, value)
                self.assertEqual(rospy.get_param('/rover_execution'), policy)
                current = module._load_docking_config()
                self.assertAlmostEqual(current.max_blind_speed_mps, baseline.max_blind_speed_mps * expected)
                self.assertEqual(current.min_blind_speed_mps, baseline.min_blind_speed_mps)
                self.assertEqual(current.position_tolerance_m, baseline.position_tolerance_m)
                self.assertEqual(current.stop_linear_speed_mps, baseline.stop_linear_speed_mps)
        policy['task_coefficients'] = {'final_approach': .01}
        rospy.set_param('/rover_execution', policy)
        with self.assertRaises(ValueError):
            module._load_docking_config()
        rospy.delete_param('/rover_execution')
        self.assertEqual(module._load_docking_config(), baseline)


if __name__ == '__main__':
    rospy.init_node('rover_execution_ros_test')
    rostest.rosrun('agx_workbench_navigation', 'rover_execution_ros', RoverExecutionRosTest)
