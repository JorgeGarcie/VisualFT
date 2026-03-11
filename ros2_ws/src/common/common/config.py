"""Configuration dataclasses for robot and safety parameters.

Loaded from YAML via load_config(). Single source of truth for all limits.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass
class WorkspaceBounds:
    """Axis-aligned bounding box in metres."""

    x_min: float = 0.35
    x_max: float = 0.65
    y_min: float = -0.20
    y_max: float = 0.20
    z_min: float = 0.08
    z_max: float = 0.50

    def contains(self, position: tuple[float, float, float]) -> bool:
        x, y, z = position
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )


@dataclass
class SafetyConfig:
    """Application-level safety limits (firmware handles hard real-time)."""

    workspace: WorkspaceBounds = field(default_factory=WorkspaceBounds)
    max_force_z: float = 25.0  # N
    max_contact_wrench: list[float] = field(
        default_factory=lambda: [50.0, 50.0, 50.0, 10.0, 10.0, 10.0]
    )
    heartbeat_timeout: float = 2.0  # seconds


@dataclass
class RobotConfig:
    """Robot connection and motion parameters."""

    serial_number: str = "Rizon4-062174"
    control_rate_hz: float = 50.0
    max_linear_vel: float = 0.05  # m/s
    max_angular_vel: float = 0.5  # rad/s
    max_linear_acc: float = 0.5  # m/s²
    max_angular_acc: float = 1.0  # rad/s²
    safety: SafetyConfig = field(default_factory=SafetyConfig)


def load_config(path: str | Path) -> RobotConfig:
    """Load RobotConfig from a YAML file.

    Expected format matches config/robot.yaml.
    """
    path = Path(path)
    with open(path) as f:
        data = yaml.safe_load(f)

    robot_data = data.get("robot", {})
    safety_data = data.get("safety", {})
    ws_data = safety_data.get("workspace", {})
    max_contact_wrench = safety_data.get(
        "max_contact_wrench",
        [50.0, 50.0, 50.0, 10.0, 10.0, 10.0],
    )
    if len(max_contact_wrench) != 6:
        raise ValueError("safety.max_contact_wrench must have 6 elements")

    workspace = WorkspaceBounds(
        x_min=ws_data.get("x", [0.35, 0.65])[0],
        x_max=ws_data.get("x", [0.35, 0.65])[1],
        y_min=ws_data.get("y", [-0.20, 0.20])[0],
        y_max=ws_data.get("y", [-0.20, 0.20])[1],
        z_min=ws_data.get("z", [0.08, 0.50])[0],
        z_max=ws_data.get("z", [0.08, 0.50])[1],
    )

    safety = SafetyConfig(
        workspace=workspace,
        max_force_z=safety_data.get("max_force_z", 50.0),
        max_contact_wrench=[float(value) for value in max_contact_wrench],
        heartbeat_timeout=safety_data.get("heartbeat_timeout", 2.0),
    )

    return RobotConfig(
        serial_number=robot_data.get("serial_number", "Rizon4-062174"),
        control_rate_hz=robot_data.get("control_rate_hz", 50.0),
        max_linear_vel=robot_data.get("max_linear_vel", 0.05),
        max_angular_vel=robot_data.get("max_angular_vel", 0.5),
        max_linear_acc=robot_data.get("max_linear_acc", 0.5),
        max_angular_acc=robot_data.get("max_angular_acc", 1.0),
        safety=safety,
    )
