from __future__ import annotations

import logging
import time
import unittest

from common.config import SafetyConfig, WorkspaceBounds
from common.errors import SafetyViolation
from common.types import Pose, RobotState, Wrench
from flexiv_driver.safety import SafetyChecker, SafetyWatchdog


def wait_until(predicate, timeout: float = 0.5, interval: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class SafetyCheckerTests(unittest.TestCase):
    def test_workspace_violation_raises(self):
        checker = SafetyChecker(
            SafetyConfig(
                workspace=WorkspaceBounds(
                    x_min=0.0,
                    x_max=1.0,
                    y_min=0.0,
                    y_max=1.0,
                    z_min=0.0,
                    z_max=1.0,
                )
            )
        )

        with self.assertRaises(SafetyViolation):
            checker.check_workspace(Pose(x=2.0, y=0.5, z=0.5))


class SafetyWatchdogTests(unittest.TestCase):
    def test_force_limit_is_logged_but_does_not_stop(self):
        stop_calls = []
        state = RobotState(wrench=Wrench(fz=40.0))
        watchdog = SafetyWatchdog(
            config=SafetyConfig(max_force_z=25.0, heartbeat_timeout=1.0),
            get_state=lambda: state,
            has_fault=lambda: False,
            stop_robot=lambda: stop_calls.append("stop"),
            poll_period=0.01,
        )

        with self.assertLogs("flexiv_driver.safety", level="WARNING") as logs:
            watchdog.start()
            self.assertTrue(
                wait_until(
                    lambda: any(
                        "Robot force limit observed by watchdog" in entry
                        for entry in logs.output
                    )
                )
            )
            watchdog.shutdown()

        self.assertEqual(stop_calls, [])

    def test_robot_fault_is_logged_but_does_not_stop(self):
        stop_calls = []
        watchdog = SafetyWatchdog(
            config=SafetyConfig(max_force_z=25.0, heartbeat_timeout=1.0),
            get_state=lambda: RobotState(wrench=Wrench()),
            has_fault=lambda: True,
            stop_robot=lambda: stop_calls.append("stop"),
            poll_period=0.01,
        )

        with self.assertLogs("flexiv_driver.safety", level="ERROR") as logs:
            watchdog.start()
            self.assertTrue(
                wait_until(
                    lambda: any(
                        "Robot fault observed by watchdog" in entry
                        for entry in logs.output
                    )
                )
            )
            watchdog.shutdown()

        self.assertEqual(stop_calls, [])

    def test_stream_heartbeat_timeout_stops_robot(self):
        stop_calls = []
        watchdog = SafetyWatchdog(
            config=SafetyConfig(max_force_z=25.0, heartbeat_timeout=0.05),
            get_state=lambda: RobotState(wrench=Wrench()),
            has_fault=lambda: False,
            stop_robot=lambda: stop_calls.append("stop"),
            poll_period=0.01,
        )

        watchdog.start()
        watchdog.note_stream_command()

        self.assertTrue(wait_until(lambda: len(stop_calls) == 1, timeout=0.5))

        watchdog.shutdown()
        self.assertEqual(stop_calls, ["stop"])

    def test_no_heartbeat_timeout_when_streaming_is_paused(self):
        stop_calls = []
        watchdog = SafetyWatchdog(
            config=SafetyConfig(max_force_z=25.0, heartbeat_timeout=0.05),
            get_state=lambda: RobotState(wrench=Wrench()),
            has_fault=lambda: False,
            stop_robot=lambda: stop_calls.append("stop"),
            poll_period=0.01,
        )

        watchdog.start()
        time.sleep(0.15)
        watchdog.shutdown()

        self.assertEqual(stop_calls, [])


if __name__ == "__main__":
    unittest.main()
