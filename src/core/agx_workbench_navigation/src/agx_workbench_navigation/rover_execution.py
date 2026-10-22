"""Shared ROS1 execution scales; phase reference limits remain upper bounds."""

from dataclasses import dataclass, replace
import math
from numbers import Real
from types import MappingProxyType
from typing import Mapping


TASKS = frozenset(('coarse_navigation', 'visual_docking', 'final_approach', 'rollback'))


def _scale(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('rover execution scales must be numbers in (0, 1]')
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= 1:
        raise ValueError('rover execution scales must be finite and in (0, 1]')
    return value


@dataclass(frozen=True)
class RoverExecutionPolicy:
    velocity_scale: float
    acceleration_scale: float
    default_coefficient: float
    task_coefficients: Mapping[str, float]

    @classmethod
    def from_mapping(cls, values):
        fields = {'velocity_scale', 'acceleration_scale', 'default_coefficient', 'task_coefficients'}
        if not isinstance(values, Mapping) or set(values) != fields:
            raise ValueError('rover execution requires exactly {}'.format(sorted(fields)))
        tasks = values['task_coefficients']
        if not isinstance(tasks, Mapping) or set(tasks) - TASKS:
            raise ValueError('unknown rover execution task or invalid coefficient map')
        policy = cls(_scale(values['velocity_scale']), _scale(values['acceleration_scale']),
                     _scale(values['default_coefficient']),
                     MappingProxyType({name: _scale(value) for name, value in tasks.items()}))
        # Reject underflow even in a currently unused task/default combination.
        for coefficient in [policy.default_coefficient] + list(policy.task_coefficients.values()):
            _scale(policy.velocity_scale * coefficient)
            _scale(policy.acceleration_scale * coefficient)
        return policy

    def scales(self, task):
        if task not in TASKS:
            raise ValueError('unknown rover execution task: {}'.format(task))
        coefficient = self.task_coefficients.get(task, self.default_coefficient)
        return self.velocity_scale * coefficient, self.acceleration_scale * coefficient

    def apply_docking(self, config):
        groups = {
            'visual_docking': (
                ('max_yaw_speed_radps', 'max_lateral_speed_mps',
                 'max_visual_forward_speed_mps'),
                ('max_visual_linear_acceleration_mps2', 'max_visual_angular_acceleration_radps2')),
            'final_approach': (
                ('max_blind_speed_mps',), ('max_blind_acceleration_mps2',)),
            'rollback': (
                ('max_rollback_speed_mps',), ('max_rollback_acceleration_mps2',)),
        }
        changes = {}
        for task, groups_for_task in groups.items():
            for scale, names in zip(self.scales(task), groups_for_task):
                for name in names:
                    changes[name] = getattr(config, name) * scale
        return replace(config, **changes)
