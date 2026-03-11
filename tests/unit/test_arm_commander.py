from __future__ import annotations

import importlib
import sys
import types
import unittest

from common.config import RobotConfig
from common.errors import SafetyViolation
from common.types import Pose


class FakeRobot:
    def __init__(self):
        self.stop_calls = 0
        self.switch_modes = []
        self.max_contact_wrench_calls = []
        self.send_cartesian_calls = []

    def Stop(self):
        self.stop_calls += 1

    def SwitchMode(self, mode):
        self.switch_modes.append(mode)

    def SendCartesianMotionForce(
        self,
        pose,
        wrench,
        max_linear_vel,
        max_angular_vel,
        max_linear_acc,
        max_angular_acc,
        ):
        self.send_cartesian_calls.append(
            (
                pose,
                wrench,
                max_linear_vel,
                max_angular_vel,
                max_linear_acc,
                max_angular_acc,
            )
        )

    def SetMaxContactWrench(self, wrench):
        self.max_contact_wrench_calls.append(list(wrench))


class FakeTool:
    def __init__(self, robot):
        self.robot = robot


class FakeToolParams:
    pass


def load_arm_commander_module():
    fake_flexivrdk = types.SimpleNamespace(
        Robot=FakeRobot,
        Tool=FakeTool,
        ToolParams=FakeToolParams,
        Coord=lambda *args, **kwargs: ("coord", args, kwargs),
        Mode=types.SimpleNamespace(
            IDLE="IDLE",
            NRT_PLAN_EXECUTION="NRT_PLAN_EXECUTION",
            NRT_PRIMITIVE_EXECUTION="NRT_PRIMITIVE_EXECUTION",
            NRT_CARTESIAN_MOTION_FORCE="NRT_CARTESIAN_MOTION_FORCE",
        ),
    )
    original_module = sys.modules.get("flexivrdk")
    sys.modules["flexivrdk"] = fake_flexivrdk

    import flexiv_driver.arm_commander as arm_module

    arm_module = importlib.reload(arm_module)
    return arm_module, original_module


class ArmCommanderTests(unittest.TestCase):
    def tearDown(self):
        sys.modules.pop("flexiv_driver.arm_commander", None)

    def test_workspace_violation_stops_robot(self):
        arm_module, original_module = load_arm_commander_module()
        try:
            arm = arm_module.ArmCommander(RobotConfig())
            arm._robot = FakeRobot()
            arm._connected = True

            with self.assertRaises(SafetyViolation):
                arm.stream_cartesian(Pose(x=2.0, y=0.0, z=0.2))

            self.assertEqual(arm._robot.stop_calls, 1)
            self.assertTrue(arm._stopped)
        finally:
            if original_module is None:
                sys.modules.pop("flexivrdk", None)
            else:
                sys.modules["flexivrdk"] = original_module

    def test_stream_cartesian_uses_robot_motion_limits(self):
        arm_module, original_module = load_arm_commander_module()
        try:
            config = RobotConfig(
                max_linear_vel=0.12,
                max_angular_vel=0.34,
                max_linear_acc=0.56,
                max_angular_acc=0.78,
            )
            config.safety.max_contact_wrench = [11.0, 12.0, 13.0, 1.0, 2.0, 3.0]
            arm = arm_module.ArmCommander(config)
            arm._robot = FakeRobot()
            arm._connected = True

            arm.stream_cartesian(Pose(x=0.4, y=0.0, z=0.2))

            self.assertEqual(
                arm._robot.switch_modes,
                ["NRT_CARTESIAN_MOTION_FORCE"],
            )
            self.assertEqual(
                arm._robot.max_contact_wrench_calls,
                [[11.0, 12.0, 13.0, 1.0, 2.0, 3.0]],
            )
            self.assertEqual(
                arm._robot.send_cartesian_calls,
                [
                    (
                        [0.4, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        0.12,
                        0.34,
                        0.56,
                        0.78,
                    )
                ],
            )
        finally:
            if original_module is None:
                sys.modules.pop("flexivrdk", None)
            else:
                sys.modules["flexivrdk"] = original_module


if __name__ == "__main__":
    unittest.main()
