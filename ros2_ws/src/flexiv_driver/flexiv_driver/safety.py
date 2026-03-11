"""Robot-side safety helpers for ArmCommander.

This layer only monitors robot-native signals:
- commanded workspace bounds
- robot external wrench from RDK state
- robot/RDK fault state
- streaming command heartbeat

CoinFT readiness, CoinFT health, and other sensor faults belong in the
application/sensor layer above ArmCommander.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

from common.config import SafetyConfig
from common.errors import SafetyViolation
from common.types import Pose, RobotState

logger = logging.getLogger(__name__)


class SafetyChecker:
    """Per-command robot safety checks."""

    def __init__(self, config: SafetyConfig):
        self._config = config

    def check_workspace(self, pose: Pose):
        """Raise if the commanded pose is outside the configured workspace."""
        if not self._config.workspace.contains(pose.position):
            raise SafetyViolation(f"Pose {pose} outside workspace bounds")


class SafetyWatchdog:
    """Background robot safety monitor.

    Watches robot-native safety signals only. Heartbeat is only armed while the
    caller is actively streaming Cartesian setpoints.

    Robot fault and force observations are logged for operator visibility. The
    active stop path in this layer is the streaming heartbeat timeout, because
    that is a software/application failure mode Flexiv cannot infer on its own.
    """

    def __init__(
        self,
        config: SafetyConfig,
        get_state: Callable[[], Optional[RobotState]],
        has_fault: Callable[[], bool],
        stop_robot: Callable[[], None],
        poll_period: float = 0.1,
        time_fn: Callable[[], float] = time.monotonic,
    ):
        self._config = config
        self._get_state = get_state
        self._has_fault = has_fault
        self._stop_robot = stop_robot
        self._poll_period = poll_period
        self._time_fn = time_fn

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._streaming_active = False
        self._last_stream_time = 0.0
        self._tripped = False
        self._fault_logged = False
        self._force_logged = False
        self._thread = threading.Thread(
            target=self._watchdog_loop,
            name="arm-safety-watchdog",
            daemon=True,
        )

    def start(self):
        """Start the watchdog thread once."""
        if not self._thread.is_alive():
            self._thread.start()

    def shutdown(self, timeout: float = 1.0):
        """Stop the watchdog thread."""
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=timeout)

    def note_stream_command(self):
        """Arm heartbeat monitoring and refresh the last stream timestamp."""
        with self._lock:
            self._streaming_active = True
            self._last_stream_time = self._time_fn()
            self._tripped = False

    def pause_heartbeat(self):
        """Pause streaming heartbeat checks during blocking primitives."""
        with self._lock:
            self._streaming_active = False

    def disarm(self):
        """Disable heartbeat monitoring after a manual or watchdog stop."""
        with self._lock:
            self._streaming_active = False
            self._tripped = True

    def _trip(self, message: str):
        with self._lock:
            if self._tripped:
                return
            self._tripped = True
            self._streaming_active = False
        logger.error(message)
        self._stop_robot()

    def _watchdog_loop(self):
        while not self._stop_event.wait(self._poll_period):
            try:
                if self._has_fault():
                    if not self._fault_logged:
                        logger.error("Robot fault observed by watchdog")
                        self._fault_logged = True
                    self.pause_heartbeat()
                else:
                    self._fault_logged = False

                state = self._get_state()
                if state is not None and abs(state.wrench.fz) > self._config.max_force_z:
                    if not self._force_logged:
                        logger.warning(
                            "Robot force limit observed by watchdog: "
                            f"|fz|={abs(state.wrench.fz):.2f}N > {self._config.max_force_z:.2f}N"
                        )
                        self._force_logged = True
                else:
                    self._force_logged = False

                with self._lock:
                    streaming_active = self._streaming_active
                    last_stream_time = self._last_stream_time

                if (
                    streaming_active
                    and self._time_fn() - last_stream_time > self._config.heartbeat_timeout
                ):
                    self._trip("Streaming heartbeat timeout detected by watchdog")
            except Exception as exc:
                logger.warning("Safety watchdog loop error: %s", exc)
