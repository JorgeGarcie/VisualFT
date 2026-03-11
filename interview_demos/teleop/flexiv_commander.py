"""
Flexiv Commander — A3
=====================
Sends Cartesian impedance commands to Flexiv Rizon4 via RDK.
Configurable stiffness/damping per axis.
Safety: workspace bounds, velocity limits, force thresholds → auto-stop.
"""

import numpy as np
import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class SafetyConfig:
    """Safety limits for teleop."""
    # Workspace bounds (meters)
    pos_min: np.ndarray = field(
        default_factory=lambda: np.array([0.3, -0.3, 0.05])
    )
    pos_max: np.ndarray = field(
        default_factory=lambda: np.array([0.7, 0.3, 0.55])
    )
    # Max Cartesian velocity (m/s)
    max_linear_vel: float = 0.15  # conservative for teleop
    max_angular_vel: float = 0.5  # rad/s
    # Force threshold (N) — stop if exceeded
    force_threshold: float = 30.0
    # Max acceleration
    max_linear_acc: float = 1.0
    max_angular_acc: float = 3.0


@dataclass
class ImpedanceConfig:
    """Cartesian impedance parameters."""
    # Stiffness: [Kx, Ky, Kz, Krx, Kry, Krz] (N/m, Nm/rad)
    stiffness: np.ndarray = field(
        default_factory=lambda: np.array([2000, 2000, 1500, 80, 80, 80], dtype=float)
    )
    # Damping ratio per axis [0.3 - 0.8]
    damping: np.ndarray = field(
        default_factory=lambda: np.array([0.7, 0.7, 0.7, 0.7, 0.7, 0.7])
    )


class FlexivCommander:
    """High-level interface to Flexiv RDK for teleop."""

    def __init__(self, robot_sn: str = "Rizon4-062174",
                 safety: Optional[SafetyConfig] = None,
                 impedance: Optional[ImpedanceConfig] = None):
        self.robot_sn = robot_sn
        self.safety = safety or SafetyConfig()
        self.impedance = impedance or ImpedanceConfig()

        self._robot = None
        self._connected = False
        self._last_cmd_pos: Optional[np.ndarray] = None
        self._last_cmd_time: Optional[float] = None
        self._stopped = False
        self._mode = 'full'  # 'full' or 'z_approach'

    def connect(self, go_home: bool = True, zero_ft: bool = True):
        """Connect to robot and configure for Cartesian control.

        Follows proven init sequence from leapft flexiv_direct.py:
        fault clear → Enable → wait operational → Home → Zero FT → SwitchMode.
        """
        import flexivrdk
        self._flexivrdk = flexivrdk

        self._robot = flexivrdk.Robot(self.robot_sn)

        # Clear any existing faults (must happen before Enable)
        if self._robot.fault():
            print("Fault detected — clearing...")
            self._robot.ClearFault()
            time.sleep(2)
            if self._robot.fault():
                raise RuntimeError("Cannot clear robot fault — check E-stop and teach pendant")
            print("Fault cleared.")

        # Enable and wait for operational
        self._robot.Enable()
        print("Waiting for robot to become operational...")
        timeout = 10
        start = time.time()
        while not self._robot.operational():
            if time.time() - start > timeout:
                raise RuntimeError("Timeout waiting for robot to become operational")
            time.sleep(0.1)
        print("Robot operational.")

        # Home via built-in plan (proven pattern from leapft)
        if go_home:
            print("Executing PLAN-Home...")
            self._robot.SwitchMode(flexivrdk.Mode.NRT_PLAN_EXECUTION)
            self._robot.ExecutePlan("PLAN-Home")
            while self._robot.busy():
                time.sleep(0.1)
            print("Home position reached.")

        # Zero F/T sensor (proven pattern from leapft)
        if zero_ft:
            print("Zeroing F/T sensor...")
            self._robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
            self._robot.ExecutePrimitive("ZeroFTSensor", dict())
            while not self._robot.primitive_states()["terminated"]:
                time.sleep(0.1)
            print("F/T sensor zeroed.")

        # Switch to NRT Cartesian motion+force mode
        self._robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
        time.sleep(0.5)

        # Pure motion control on all axes (no force control axes)
        self._robot.SetForceControlAxis([False] * 6)

        # Store initial pose as fallback
        self._init_pose = list(self._robot.states().tcp_pose)

        self._connected = True
        print(f"Flexiv connected: {self.robot_sn}")
        print(f"Initial TCP: {[round(v, 4) for v in self._init_pose[:3]]}")

    def set_z_approach_mode(self, z_stiffness: float = 500.0, z_damping: float = 0.8):
        """Switch to Z-approach: stiff XY, compliant Z.

        Same concept as scan_node SCANNING phase — XY tracks the commanded
        position tightly, Z is soft so contact with the surface is gentle.
        VR drives Z, impedance is the safety net.
        """
        K_xy = self.impedance.stiffness[0]  # keep XY stiff from config
        K_rot = self.impedance.stiffness[3]
        self._robot.SetCartesianImpedance(
            [K_xy, K_xy, z_stiffness, K_rot, K_rot, K_rot],
            [0.7, 0.7, z_damping, 0.7, 0.7, 0.7],
        )
        self._robot.SetForceControlAxis([False] * 6)  # all motion, impedance handles compliance
        self._mode = 'z_approach'
        print(f"Z-approach mode: XY K={K_xy}, Z K={z_stiffness}, Z damping={z_damping}")

    def set_full_mode(self):
        """Switch back to full 6DOF teleop impedance."""
        self._robot.SetCartesianImpedance(
            self.impedance.stiffness.tolist(),
            self.impedance.damping.tolist(),
        )
        self._robot.SetForceControlAxis([False] * 6)
        self._mode = 'full'
        print(f"Full teleop mode: K={self.impedance.stiffness[:3]}")

    def send_pose(self, pos: np.ndarray, quat_xyzw: np.ndarray, t: Optional[float] = None):
        """
        Send target pose to robot with safety checks.

        Args:
            pos: [x, y, z] in meters
            quat_xyzw: [qx, qy, qz, qw] quaternion (scipy convention)
            t: timestamp for velocity estimation
        """
        if self._stopped:
            return

        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")

        # ─── Safety checks ───

        # 1. Workspace bounds
        pos_clamped = np.clip(pos, self.safety.pos_min, self.safety.pos_max)
        if not np.allclose(pos, pos_clamped):
            if not hasattr(self, '_clamp_count'):
                self._clamp_count = 0
            self._clamp_count += 1
            if self._clamp_count <= 3 or self._clamp_count % 120 == 0:
                print(f"WARNING: pose clamped to workspace (x{self._clamp_count})")
            pos = pos_clamped

        # 2. Velocity limit
        if self._last_cmd_pos is not None and t is not None and self._last_cmd_time is not None:
            dt = max(t - self._last_cmd_time, 1e-6)
            vel = np.linalg.norm(pos - self._last_cmd_pos) / dt
            if vel > self.safety.max_linear_vel:
                # Scale down motion to respect velocity limit
                direction = pos - self._last_cmd_pos
                dir_norm = np.linalg.norm(direction)
                if dir_norm > 1e-9:
                    max_step = self.safety.max_linear_vel * dt
                    pos = self._last_cmd_pos + direction / dir_norm * max_step

        # 3. Robot fault check
        if self._robot.fault():
            print("FAULT: Robot fault detected — stopping teleop")
            self._stopped = True
            return

        # 4. Force threshold check (software-level, on top of SetMaxContactWrench)
        robot_states = self._robot.states()
        ext_force = np.array(robot_states.ext_wrench_in_world[:3])
        force_mag = np.linalg.norm(ext_force)
        if force_mag > self.safety.force_threshold:
            print(f"SAFETY STOP: external force {force_mag:.1f}N > threshold {self.safety.force_threshold}N")
            self._stopped = True
            self._robot.Stop()
            return

        # ─── Send command ───
        # RDK tcp_pose format: [x, y, z, qw, qx, qy, qz]
        # Input quat is scipy convention [qx, qy, qz, qw]
        tcp_pose = [
            float(pos[0]), float(pos[1]), float(pos[2]),
            float(quat_xyzw[3]),  # qw
            float(quat_xyzw[0]),  # qx
            float(quat_xyzw[1]),  # qy
            float(quat_xyzw[2]),  # qz
        ]

        try:
            # Match leapft pattern: just send pose, no extra params
            self._robot.SendCartesianMotionForce(tcp_pose)
        except RuntimeError as e:
            print(f"SendCartesianMotionForce error: {e}")
            self._stopped = True
            return

        self._last_cmd_pos = pos.copy()
        self._last_cmd_time = t

    def get_state(self) -> dict:
        """Get current robot state.

        Returns dict with:
          tcp_pos: [x, y, z] metres
          tcp_quat_wfirst: [qw, qx, qy, qz] (RDK convention)
          ext_wrench: [fx, fy, fz, tx, ty, tz] in world frame
          ft_sensor: [fx, fy, fz, tx, ty, tz] in TCP frame
          tcp_vel: [vx, vy, vz, wx, wy, wz]
        """
        if not self._connected:
            return {}
        states = self._robot.states()
        tcp = list(states.tcp_pose)  # [x,y,z, qw,qx,qy,qz]
        return {
            'tcp_pos': np.array(tcp[:3]),
            'tcp_quat_wfirst': np.array(tcp[3:]),  # [qw, qx, qy, qz]
            'ext_wrench': np.array(list(states.ext_wrench_in_world)),
            'ft_sensor': np.array(list(states.ext_wrench_in_tcp)),
            'tcp_vel': np.array(list(states.tcp_vel)),
        }

    def stop(self):
        """Emergency stop."""
        if self._robot:
            self._robot.Stop()
            self._stopped = True
            print("Robot stopped.")

    def reset_stop(self):
        """Clear stopped state (after checking it's safe).

        Will attempt fault recovery if needed.
        """
        if self._robot is None:
            return

        # Clear fault if present
        if self._robot.fault():
            print("Clearing fault...")
            self._robot.ClearFault()
            time.sleep(2)
            if self._robot.fault():
                print("ERROR: Cannot clear fault")
                return

        # Re-enable if needed
        if not self._robot.operational():
            self._robot.Enable()
            while not self._robot.operational():
                time.sleep(1)

        self._robot.SwitchMode(self._flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
        time.sleep(0.5)
        self._robot.SetForceControlAxis([False] * 6)
        self._robot.SetCartesianImpedance(
            self.impedance.stiffness.tolist(),
            self.impedance.damping.tolist()
        )
        self._stopped = False
        print("Stop cleared, re-enabled.")

    def disconnect(self):
        if self._robot:
            self._robot.Stop()
            self._connected = False


# ─── Mock for testing without hardware ──────────────────────────────────────────
class MockFlexivCommander:
    """Simulates robot for testing the teleop pipeline."""

    def __init__(self):
        self._pos = np.array([0.5, 0.0, 0.4])
        self._quat = np.array([0, 0, 0, 1.0])
        self._stopped = False

    def connect(self):
        print("MockFlexiv connected (simulated robot)")

    def send_pose(self, pos, quat_xyzw, t=None):
        if not self._stopped:
            self._pos = pos.copy()
            self._quat = quat_xyzw.copy()

    def get_state(self):
        return {
            'tcp_pos': self._pos.copy(),
            'tcp_quat_wfirst': np.array([self._quat[3], self._quat[0],
                                         self._quat[1], self._quat[2]]),
            'ext_wrench': np.zeros(6),
            'ft_sensor': np.zeros(6),
        }

    def stop(self):
        self._stopped = True
        print("MockRobot stopped.")

    def disconnect(self):
        pass


if __name__ == '__main__':
    # Test with mock
    cmd = MockFlexivCommander()
    cmd.connect()

    pos = np.array([0.5, 0.0, 0.4])
    quat = np.array([0, 0, 0, 1.0])
    cmd.send_pose(pos, quat, time.monotonic())

    state = cmd.get_state()
    print(f"Robot state: pos={state['tcp_pos']}")
    print("Mock test passed.")
