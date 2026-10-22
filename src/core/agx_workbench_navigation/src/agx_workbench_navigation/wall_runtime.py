"""Small ROS-independent wall-clock primitives for safety control wrappers."""

from __future__ import annotations

import math
from numbers import Real
import threading
import time


class WallCallTimeout(RuntimeError):
    """A background call did not finish within its wall-clock deadline."""


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{0} must be positive".format(name))
    converted = float(value)
    if not math.isfinite(converted) or converted <= 0.0:
        raise ValueError("{0} must be positive".format(name))
    return converted


def call_with_wall_timeout(operation, timeout_s):
    """Run a bounded call without depending on ROS time.

    The worker is a daemon because Python cannot safely interrupt an arbitrary
    blocking ROS transport call. Callers must use this only for operations
    whose delayed completion is side-effect free or remains safe.
    """

    if not callable(operation):
        raise ValueError("operation must be callable")
    timeout = _positive(timeout_s, "timeout_s")
    completed = threading.Event()
    outcome = {}

    def invoke():
        try:
            outcome["result"] = operation()
        except BaseException as error:
            outcome["error"] = error
        finally:
            completed.set()

    worker = threading.Thread(
        target=invoke,
        name="workbench-wall-call",
        daemon=True,
    )
    worker.start()
    if not completed.wait(timeout):
        raise WallCallTimeout(
            "operation exceeded {0:.3f}s wall timeout".format(timeout)
        )
    if "error" in outcome:
        raise outcome["error"]
    return outcome.get("result")


class WallRate:
    """Fixed-rate sleep driven only by ``time.monotonic``."""

    def __init__(self, rate_hz, *, clock=None, sleeper=None):
        self._period_s = 1.0 / _positive(rate_hz, "rate_hz")
        self._clock = time.monotonic if clock is None else clock
        self._sleep = time.sleep if sleeper is None else sleeper
        if not callable(self._clock) or not callable(self._sleep):
            raise ValueError("clock and sleeper must be callable")
        self._deadline_s = float(self._clock())

    def sleep(self):
        self._deadline_s += self._period_s
        now = float(self._clock())
        remaining = self._deadline_s - now
        if remaining > 0.0:
            self._sleep(remaining)
            return
        self._deadline_s = now
