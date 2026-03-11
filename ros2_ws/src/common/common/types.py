"""Common types for the VisualFT system.

Pose, Wrench, and RobotState dataclasses used across all layers.
Pose stores quaternions in ROS2 convention (x, y, z, w) everywhere.
Conversion to/from RDK format happens only via to_rdk() / from_rdk().
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Pose:
    """6-DOF pose: position (metres) + quaternion (ROS2: x, y, z, w)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    @property
    def position(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def orientation(self) -> tuple[float, float, float, float]:
        """Quaternion in ROS2 order: (x, y, z, w)."""
        return (self.qx, self.qy, self.qz, self.qw)

    def to_rdk(self) -> list[float]:
        """Convert to RDK format: [x, y, z, qw, qx, qy, qz]."""
        return [self.x, self.y, self.z, self.qw, self.qx, self.qy, self.qz]

    @classmethod
    def from_rdk(cls, tcp_pose) -> Pose:
        """Create from RDK format: [x, y, z, qw, qx, qy, qz]."""
        return cls(
            x=float(tcp_pose[0]),
            y=float(tcp_pose[1]),
            z=float(tcp_pose[2]),
            qx=float(tcp_pose[4]),
            qy=float(tcp_pose[5]),
            qz=float(tcp_pose[6]),
            qw=float(tcp_pose[3]),
        )

    def distance_to(self, other: Pose) -> float:
        """Euclidean distance between positions (metres)."""
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


@dataclass
class Wrench:
    """Force/torque: forces (N) and torques (Nm)."""

    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0

    @classmethod
    def from_list(cls, values) -> Wrench:
        """Create from [fx, fy, fz, tx, ty, tz] list."""
        return cls(
            fx=float(values[0]),
            fy=float(values[1]),
            fz=float(values[2]),
            tx=float(values[3]),
            ty=float(values[4]),
            tz=float(values[5]),
        )

    def to_list(self) -> list[float]:
        """Convert to [fx, fy, fz, tx, ty, tz]."""
        return [self.fx, self.fy, self.fz, self.tx, self.ty, self.tz]

    @property
    def force_magnitude(self) -> float:
        return math.sqrt(self.fx**2 + self.fy**2 + self.fz**2)


@dataclass
class RobotState:
    """Snapshot of robot state at a point in time."""

    pose: Pose = field(default_factory=Pose)
    wrench: Wrench = field(default_factory=Wrench)
    joint_positions: list[float] = field(default_factory=list)
    timestamp: float = 0.0
    operational: bool = False
    fault: bool = False
