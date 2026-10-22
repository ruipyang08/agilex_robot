"""Final command-ray contraction; measured progress grants only a fixed lead.

This bounds requested motion, not physical acceleration or stopping performance.
The caller owns original command leases, measured-evidence freshness and post-stop
settling. Normal source-time shaping belongs before producer collision checks;
this algebraic contraction makes no final-publication slew guarantee.
"""
from dataclasses import dataclass
import math
import struct
import sys

from .contracts import Velocity2D

ZERO = Velocity2D(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class LimitedCommand:
    command: Velocity2D
    reason: str
    requires_settle: bool = False
    new_inhibition: bool = False


class FinalCommandLimiter:
    def __init__(self, config):
        for name in ('max_linear_speed_lead_mps', 'max_angular_speed_lead_radps',
                     'min_turn_radius_m', 'stop_linear_speed_mps', 'stop_angular_speed_radps'):
            value = getattr(config, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid final command limit: ' + name)
            setattr(self, name, float(value))
        self.reset()

    def reset(self, *, source_now_s=None, binding=None):
        self._previous = ZERO
        self._source_s = source_now_s
        self._binding = tuple(binding) if binding else None
        self._waiting = False
        self._mode = None

    @property
    def waiting_for_settle(self):
        return self._waiting

    def stop(self, *, source_now_s, binding):
        # Same-grant zero resets trajectory credit, not the physical dwell
        # required after moving output. New authority already requires its own
        # fresh measured stop and does not inherit this operation's history.
        same = binding is not None and tuple(binding) == self._binding
        new = same and not self._waiting and self._previous != ZERO
        waiting = same and (self._waiting or new)
        mode = self._mode if same else None
        self.reset(source_now_s=source_now_s, binding=binding)
        self._waiting, self._mode = waiting, mode
        return new

    def _vendor_signature(self, command):
        # Active Ranger Mini V2 TwistCmdCallback/CalculateSteeringAngle rules.
        # The branch key also protects pure-crab and near-epsilon steering from
        # changing when an otherwise common scalar is applied.
        if command.linear_y != 0:
            return ('parallel', abs(command.linear_x) <= sys.float_info.epsilon)
        if abs(command.angular_z) <= sys.float_info.epsilon:
            return ('ackermann', 'straight')
        if abs(command.linear_x/command.angular_z) < self.min_turn_radius_m:
            return ('spin',)
        return ('ackermann', 'arc')

    def _motion_mode(self, command):
        signature = self._vendor_signature(command)
        return signature if signature[0] == 'parallel' else signature[:1]

    def _inhibit(self, now, reason):
        new = not self._waiting
        self._previous = ZERO
        self._source_s = now
        self._waiting = True
        return LimitedCommand(ZERO, reason, True, new)

    def limit(self, desired, measured, *, source_now_s, binding, measured_settled):
        if not isinstance(desired, Velocity2D) or not isinstance(measured, Velocity2D):
            self.reset()
            raise ValueError('final limiter needs finite typed velocities')
        if isinstance(source_now_s, bool) or not isinstance(source_now_s, (int, float)) or not math.isfinite(source_now_s) or source_now_s <= 0:
            self.reset()
            raise ValueError('invalid command source clock')
        # Prevent nonfinite intermediates in projection/scaling.
        numeric_limit = math.sqrt(sys.float_info.max)/4
        if any(abs(value)>numeric_limit for value in
               (desired.linear_x, desired.linear_y, desired.angular_z,
                measured.linear_x, measured.linear_y, measured.angular_z)):
            self.reset()
            raise ValueError('velocity exceeds finite calculation range')
        now = float(source_now_s)
        if self._source_s is not None and now < self._source_s:
            self.reset()
            raise ValueError('command source clock regressed')
        if not binding or not all(isinstance(part, str) and part for part in binding):
            self.reset()
            raise ValueError('missing final command identity')
        if desired == ZERO:
            new = self.stop(source_now_s=now, binding=binding)
            return LimitedCommand(ZERO, 'command_zero', self._waiting, new)
        if tuple(binding) != self._binding or self._source_s is None:
            self.reset()
            self._binding = tuple(binding)
            self._source_s = now
            return LimitedCommand(ZERO, 'command_limiter_startup')
        self._source_s = now
        if self._waiting:
            if not measured_settled:
                return LimitedCommand(ZERO, 'command_limiter_waiting_for_settle', True)
            self._waiting = False
            self._mode = None
        if self._mode is not None and self._motion_mode(desired) != self._mode:
            return self._inhibit(now, 'command_limiter_mode_stop')
        vx, vy, wz = desired.linear_x, desired.linear_y, desired.angular_z
        previous = self._previous
        if (vx*previous.linear_x + vy*previous.linear_y < 0 or
                (self._motion_mode(desired) == ("spin",) and wz*previous.angular_z < 0)):
            return self._inhibit(now, 'command_limiter_direction_stop')
        speed = math.hypot(vx, vy)
        measured_speed = math.hypot(measured.linear_x, measured.linear_y)
        incompatible_linear = measured_speed > self.stop_linear_speed_mps and (
            speed == 0 or
            vx*measured.linear_x + vy*measured.linear_y < 0 or
            abs(vx*measured.linear_y-vy*measured.linear_x)/speed > self.stop_linear_speed_mps)
        # Ackermann curvature can cross zero while retaining translation and
        # its vendor mode. Opposite yaw earns only the fixed lead below. The
        # measured reversal stop is unchanged for spin and parallel commands.
        incompatible_yaw = (self._motion_mode(desired) != ('ackermann',) and
                            abs(measured.angular_z) > self.stop_angular_speed_radps and
                            wz*measured.angular_z < 0)
        if incompatible_linear or incompatible_yaw:
            return self._inhibit(now, 'command_limiter_measured_direction_stop')
        upper = 1.0
        if speed:
            progress = max(0., (vx*measured.linear_x + vy*measured.linear_y)/speed)
            upper = min(upper, (progress+self.max_linear_speed_lead_mps)/speed)
        if wz:
            progress = max(0., math.copysign(1., wz)*measured.angular_z)
            upper = min(upper, (progress+self.max_angular_speed_lead_radps)/abs(wz))
        # A fresh producer command already has its exact curve checked. Only
        # contract that ray here. Intersecting it with another normal ramp has
        # no solution for legitimate deceleration/curvature changes.
        scale = max(0., min(1., upper))
        values = tuple(scale*value for value in (vx, vy, wz))
        if scale > 0 and any(value != 0 and limited == 0 for value, limited in zip((vx, vy, wz), values)):
            return self._inhibit(now, 'command_limiter_component_underflow')
        command = Velocity2D(*values)
        signature = self._vendor_signature(desired)
        if signature == ('ackermann', 'arc') and self._vendor_signature(command) == ('spin',):
            # A supported boundary arc can round into vendor spin after common
            # scaling. Try only adjacent *smaller common scalars*, keeping the
            # checked ray and progress bounds. Never repair an unsupported spin
            # demand into an arc or change components independently. Bit stepping
            # also works on onboard Python 3.8, which lacks math.nextafter.
            for _ in range(8):
                bits = struct.unpack('!Q', struct.pack('!d', scale))[0]
                if bits <= 1:
                    break
                scale = struct.unpack('!d', struct.pack('!Q', bits-1))[0]
                values = tuple(scale*value for value in (vx, vy, wz))
                if any(value != 0 and limited == 0 for value, limited in zip((vx, vy, wz), values)):
                    break
                candidate = Velocity2D(*values)
                if self._vendor_signature(candidate) == signature:
                    command = candidate
                    break
        if command == ZERO and self._previous != ZERO:
            return self._inhibit(now, 'command_limiter_direction_stop')
        if command != ZERO and self._vendor_signature(command) != signature:
            return self._inhibit(now, 'command_limiter_mode_scale_stop')
        self._previous = command
        if command != ZERO:
            self._mode = self._motion_mode(command)
        return LimitedCommand(command, 'command_progress_limited' if scale < 1 else 'command_unlimited')
