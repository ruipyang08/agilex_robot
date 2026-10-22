#!/usr/bin/env python3
"""Exercise the production constructor's pre-actuation configuration boundary."""
from copy import deepcopy
import subprocess
import unittest

import rospy
import rostest


class SpeedConfigRejection(unittest.TestCase):
    def test_legacy_speeds_resolve_uniformly_before_late_validation(self):
        baseline = rospy.get_param('/tube_grasp')
        grasp = deepcopy(baseline)
        del grasp['motion']['arm_speed']
        del grasp['motion']['gripper_speeds']
        grasp['motion'].update(velocity_scaling=.08, acceleration_scaling=.20,
                               gripper_speed=10., gripper_force=0.)
        # A deliberately invalid force trips final validation after speed loading,
        # so the production loader can be tested without constructing actuators.
        try:
            rospy.set_param('/tube_grasp', grasp)
            result = subprocess.run(['rosrun', 'agx_xarm_pick', 'tube_speed_config_rejection_test'],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    universal_newlines=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertIn('legacy uniform scaling', result.stdout)
            for name in ('preclose', 'final_close', 'recovery'):
                self.assertIn('tube resolved gripper speed: '+name+' speed=10.0', result.stdout)
            for name in baseline['motion']['arm_speed']:
                if name != 'default_coefficient':
                    self.assertIn('tube resolved arm speed: '+name+' velocity=0.080 acceleration=0.200', result.stdout)
        finally:
            rospy.set_param('/tube_grasp', baseline)

    def test_malformed_speeds_are_rejected_before_actuator_setup(self):
        baseline = rospy.get_param('/tube_grasp')
        global_baseline = rospy.get_param('/arm_execution')
        cases = []
        for field in ('velocity_scale', 'acceleration_scale'):
            for value in (True, 0., -0.1, 1.01, float('nan'), float('inf')):
                cases.append(('global_'+field+'_'+str(value), 'global', field, value))
        for field in ('default_coefficient', 'insertion'):
            for value in (True, 0., -0.1, 1.01, float('nan'), float('inf')):
                cases.append(('coefficient_'+field+'_'+str(value), 'coefficient', field, value))
        for field in ('preclose', 'final_close', 'recovery'):
            for value in (True, 0., -1., 101., float('nan'), float('inf')):
                cases.append(('gripper_'+field+'_'+str(value), 'gripper', field, value))
        cases += [('unknown_gripper', 'gripper', 'typo_motion', 10.),
                  ('mixed_gripper', 'motion', 'gripper_speed', 10.),
                  ('missing_gripper', 'missing_gripper', '', None),
                  ('unknown_motion', 'coefficient', 'typo_motion', .5),
                  ('mixed_forms', 'motion', 'velocity_scaling', .08),
                  ('missing_global', 'missing_global', '', None),
                  ('missing_default', 'missing_default', '', None),
                  ('legacy_missing_acceleration', 'legacy_missing', '', None),
                  ('effective_underflow', 'underflow', '', None)]
        for name, kind, field, value in cases:
            with self.subTest(case=name):
                grasp, global_settings = deepcopy(baseline), deepcopy(global_baseline)
                if kind == 'global': global_settings[field] = value
                elif kind == 'coefficient': grasp['motion']['arm_speed'][field] = value
                elif kind == 'gripper': grasp['motion']['gripper_speeds'][field] = value
                elif kind == 'missing_gripper': del grasp['motion']['gripper_speeds']['recovery']
                elif kind == 'motion': grasp['motion'][field] = value
                elif kind == 'missing_default': del grasp['motion']['arm_speed']['default_coefficient']
                elif kind == 'legacy_missing':
                    del grasp['motion']['arm_speed']
                    grasp['motion']['velocity_scaling'] = .08
                elif kind == 'underflow':
                    global_settings['velocity_scale'] = 5e-324
                    grasp['motion']['arm_speed']['insertion'] = .1
                rospy.set_param('/tube_grasp', grasp)
                rospy.set_param('/arm_execution', global_settings)
                if kind == 'missing_global': rospy.delete_param('/arm_execution')
                result = subprocess.run(['rosrun', 'agx_xarm_pick', 'tube_speed_config_rejection_test'],
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        universal_newlines=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertIn('before actuator setup: PASS', result.stdout)
        rospy.set_param('/tube_grasp', baseline)
        rospy.set_param('/arm_execution', global_baseline)


if __name__ == '__main__':
    rospy.init_node('speed_config_rejection')
    rostest.rosrun('agx_xarm_pick', 'speed_config_rejection', SpeedConfigRejection)
