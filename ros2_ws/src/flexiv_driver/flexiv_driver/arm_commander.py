"""ArmCommander — Single owner of the Flexiv RDK connection.

Pure Python, no ROS2 dependency. All RDK access is lock-protected so a
watchdog thread (Step 3) can safely share the connection.

Usage:
    from common.config import load_config
    from flexiv_driver.arm_commander import ArmCommander

    config = load_config("config/robot.yaml")
    arm = ArmCommander(config)
    arm.connect()
    arm.home()
    arm.zero_ft()
    arm.stream_cartesian(some_pose)
    arm.shutdown()
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import flexivrdk

from common.types import Pose, RobotState, Wrench
from common.config import RobotConfig
from common.errors import ConnectionLost, RobotFault, SafetyViolation
from flexiv_driver.safety import SafetyChecker, SafetyWatchdog

logger = logging.getLogger(__name__)


class ArmCommander:
    """Single owner of the Flexiv RDK robot connection.

    All methods are thread-safe via an internal lock. Mode switching is
    handled internally — callers never need to know about RDK modes.
    """

    def __init__(self, config: RobotConfig):
        self._config = config
        self._robot: Optional[flexivrdk.Robot] = None
        self._tool: Optional[flexivrdk.Tool] = None
        self._lock = threading.Lock()
        self._mode: Optional[flexivrdk.Mode] = None
        self._connected = False
        self._stopped = False
        self._safety = SafetyChecker(config.safety)
        self._watchdog: Optional[SafetyWatchdog] = None
        self._configured_max_contact_wrench: Optional[tuple[float, ...]] = None

    # ──────────────────────────────────────────────────────────────────
    # Connection lifecycle
    # ──────────────────────────────────────────────────────────────────

    def connect(self):
        """Connect to robot, clear faults, enable, wait for operational."""
        with self._lock:
            self._robot = flexivrdk.Robot(self._config.serial_number)

            if self._robot.fault():
                logger.warning("Fault detected — clearing...")
                if not self._robot.ClearFault():
                    raise RobotFault("Cannot clear robot fault")
                time.sleep(2)
                if self._robot.fault():
                    raise RobotFault(
                        "Cannot clear robot fault — check E-stop and teach pendant"
                    )
                logger.info("Fault cleared")

            logger.info("Enabling robot...")
            self._robot.Enable()

            timeout = 10.0
            start = time.monotonic()
            while not self._robot.operational():
                if time.monotonic() - start > timeout:
                    raise ConnectionLost("Timeout waiting for robot to become operational")
                time.sleep(0.1)

            self._tool = flexivrdk.Tool(self._robot)
            self._mode = None
            self._connected = True
            self._stopped = False
            self._configured_max_contact_wrench = None
            logger.info("Robot operational: %s", self._config.serial_number)

        self._start_watchdog()

    def shutdown(self):
        """Stop robot and release connection."""
        self._stop_watchdog()
        with self._lock:
            if self._robot is not None:
                try:
                    self._robot.Stop()
                except Exception as e:
                    logger.warning("Error during shutdown Stop(): %s", e)
            self._connected = False
            self._stopped = True
            logger.info("Robot shutdown complete")

    # ──────────────────────────────────────────────────────────────────
    # Blocking primitives
    # ──────────────────────────────────────────────────────────────────

    def home(self):
        """Move to home position via built-in PLAN-Home. Blocking."""
        self._ensure_connected()
        self._pause_stream_heartbeat()
        with self._lock:
            self._switch_mode(flexivrdk.Mode.NRT_PLAN_EXECUTION)
            self._robot.ExecutePlan("PLAN-Home")

        # Poll outside the lock so watchdog can still read state
        while True:
            with self._lock:
                if not self._robot.busy():
                    break
            time.sleep(0.1)
        logger.info("Home position reached")

    def move_to(self, pose: Pose, velocity: float = 0.05):
        """Blocking MoveL to target pose.

        Args:
            pose: Target pose (ROS2 convention internally, converted to RDK).
            velocity: TCP linear velocity in m/s.
        """
        self._ensure_connected()
        self._check_pose_safe(pose)
        self._pause_stream_heartbeat()

        # MoveL expects Coord with [mm,mm,mm] position and [deg,deg,deg] rotation.
        # But actually RDK Coord takes metres and degrees depending on the primitive.
        # From scan_node: it passes metres for position, degrees for rotation.
        from scipy.spatial.transform import Rotation as R

        quat_xyzw = [pose.qx, pose.qy, pose.qz, pose.qw]
        euler_deg = R.from_quat(quat_xyzw).as_euler("xyz", degrees=True).tolist()
        pos_m = [pose.x, pose.y, pose.z]

        with self._lock:
            self._switch_mode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
            self._robot.ExecutePrimitive(
                "MoveL",
                {
                    "target": flexivrdk.Coord(
                        pos_m, euler_deg, ["WORLD", "WORLD_ORIGIN"]
                    ),
                    "vel": velocity,
                },
            )

        # Poll until reached
        while True:
            with self._lock:
                states = self._robot.primitive_states()
            if states.get("reachedTarget"):
                break
            time.sleep(0.1)
        logger.info("MoveL complete")

    def zero_ft(self):
        """Zero the force/torque sensor. Blocking."""
        self._ensure_connected()
        self._pause_stream_heartbeat()
        with self._lock:
            self._switch_mode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
            self._robot.ExecutePrimitive("ZeroFTSensor", dict())

        while True:
            with self._lock:
                states = self._robot.primitive_states()
            if states.get("terminated"):
                break
            time.sleep(0.1)
        logger.info("FT sensor zeroed")

    def contact(
        self,
        direction: list[float] = None,
        velocity: float = 0.02,
        max_force: float = 20.0,
    ):
        """Move in direction until contact. Blocking.

        Uses Flexiv's built-in Contact primitive — firmware-level,
        stops automatically when force exceeds max_force.

        Args:
            direction: [x, y, z] approach direction. Defaults to [0, 0, -1] (down).
            velocity: Approach velocity in m/s.
            max_force: Maximum contact force in N before stopping.
        """
        if direction is None:
            direction = [0.0, 0.0, -1.0]

        self._ensure_connected()
        self._pause_stream_heartbeat()
        with self._lock:
            self._switch_mode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
            self._robot.ExecutePrimitive(
                "Contact",
                {
                    "contactDir": direction,
                    "contactVel": velocity,
                    "maxContactForce": max_force,
                    "enableFineContact": 1,
                },
            )

        while True:
            with self._lock:
                states = self._robot.primitive_states()
            if states.get("terminated"):
                break
            time.sleep(0.1)
        logger.info("Contact detected (max_force=%.1fN)", max_force)

    def float(
        self,
        axes: list[int] = None,
        damping: list[float] = None,
    ):
        """Enter zero-gravity floating mode. Blocking (runs until stopped).

        Args:
            axes: 6-element list, 1=floating allowed, 0=locked. Default all floating.
            damping: 6-element damping level [0-100]. Default all 0 (easiest to move).
        """
        if axes is None:
            axes = [1, 1, 1, 1, 1, 1]
        if damping is None:
            damping = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self._ensure_connected()
        self._pause_stream_heartbeat()
        with self._lock:
            self._switch_mode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
            self._robot.ExecutePrimitive(
                "Float",
                {
                    "floatingAxis": axes,
                    "dampingLevel": damping,
                },
            )
        logger.info("Floating mode active (axes=%s, damping=%s)", axes, damping)

    # ──────────────────────────────────────────────────────────────────
    # Tool management
    # ──────────────────────────────────────────────────────────────────

    def set_tool(
        self,
        name: str,
        mass: float,
        com: list[float],
        inertia: list[float],
        tcp_location: list[float] = None,
    ):
        """Register and activate a tool for gravity compensation.

        Must be called before entering modes that need tool compensation
        (e.g. float). Robot switches to IDLE internally, then back.

        Args:
            name: Tool name (e.g. "CoinFT").
            mass: Tool mass in kg.
            com: Center of mass [x, y, z] in metres.
            inertia: Inertia [Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
            tcp_location: TCP offset [x, y, z, qw, qx, qy, qz]. None = no offset.
        """
        self._ensure_connected()
        self._pause_stream_heartbeat()
        with self._lock:
            self._switch_mode(flexivrdk.Mode.IDLE)

            params = flexivrdk.ToolParams()
            params.mass = mass
            params.CoM = com
            params.inertia = inertia
            if tcp_location is not None:
                params.tcp_location = tcp_location

            if self._tool.exist(name):
                self._tool.Switch("Flange")
                self._tool.Remove(name)

            self._tool.Add(name, params)
            self._tool.Switch(name)
            logger.info("Tool active: %s (mass=%.2fkg)", name, mass)

    # ──────────────────────────────────────────────────────────────────
    # Streaming control (call at control rate)
    # ──────────────────────────────────────────────────────────────────

    def stream_cartesian(
        self,
        pose: Pose,
        wrench: list[float] = None,
        max_linear_vel: float = None,
        max_angular_vel: float = None,
        max_linear_acc: float = None,
        max_angular_acc: float = None,
    ):
        """Send a non-realtime Cartesian motion+force target.

        Call this at the control rate (e.g. 50Hz). The robot holds
        the last target if no new command arrives.

        Args:
            pose: Target pose.
            wrench: Target wrench [fx,fy,fz,tx,ty,tz]. Default all zeros.
            max_linear_vel: Override config max linear velocity (m/s).
            max_angular_vel: Override config max angular velocity (rad/s).
            max_linear_acc: Override config max linear acceleration (m/s²).
            max_angular_acc: Override config max angular acceleration (rad/s²).
        """
        self._ensure_connected()
        self._check_pose_safe(pose)
        if wrench is None:
            wrench = [0.0] * 6

        mlv = max_linear_vel or self._config.max_linear_vel
        mav = max_angular_vel or self._config.max_angular_vel
        mla = max_linear_acc or self._config.max_linear_acc
        maa = max_angular_acc or self._config.max_angular_acc

        with self._lock:
            self._switch_mode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
            self._ensure_cartesian_contact_wrench_locked()
            self._robot.SendCartesianMotionForce(
                pose.to_rdk(), wrench, mlv, mav, mla, maa
            )
        self._note_stream_command()

    def set_impedance(
        self,
        stiffness: list[float],
        damping: list[float],
    ):
        """Set Cartesian impedance parameters.

        Args:
            stiffness: [Kx, Ky, Kz, Krx, Kry, Krz] in N/m and Nm/rad.
            damping: [Zx, Zy, Zz, Zrx, Zry, Zrz] ratio [0.3-0.8].
        """
        self._ensure_connected()
        with self._lock:
            self._robot.SetCartesianImpedance(stiffness, damping)
        logger.info("Impedance set: K=%s, Z=%s", stiffness, damping)

    def set_max_contact_wrench(self, wrench: list[float]):
        """Set maximum contact wrench before robot auto-stops.

        Args:
            wrench: [fx, fy, fz, tx, ty, tz] in N and Nm.
        """
        self._ensure_connected()
        with self._lock:
            self._robot.SetMaxContactWrench(wrench)
            self._configured_max_contact_wrench = tuple(float(value) for value in wrench)

    def set_force_control_axis(self, axes: list[bool]):
        """Set which axes use force control vs motion control.

        Args:
            axes: 6-element list. True=force control, False=motion control.
        """
        self._ensure_connected()
        with self._lock:
            self._robot.SetForceControlAxis(axes)

    # ──────────────────────────────────────────────────────────────────
    # State
    # ──────────────────────────────────────────────────────────────────

    def get_state(self) -> RobotState:
        """Read current robot state (pose, wrench, joints)."""
        self._ensure_connected()
        with self._lock:
            states = self._robot.states()
            operational = self._robot.operational()
            fault = self._robot.fault()
        return RobotState(
            pose=Pose.from_rdk(states.tcp_pose),
            wrench=Wrench.from_list(states.ext_wrench_in_tcp),
            joint_positions=list(states.q),
            timestamp=time.monotonic(),
            operational=operational,
            fault=fault,
        )

    def fault(self) -> bool:
        """True if robot is in fault state."""
        with self._lock:
            if self._robot is None:
                return False
            return self._robot.fault()

    def healthy(self) -> bool:
        """True if connected, operational, no fault, not stopped."""
        if not self._connected or self._robot is None:
            return False
        with self._lock:
            return (
                self._robot.operational()
                and not self._robot.fault()
                and not self._stopped
            )

    def stop(self):
        """Emergency stop. Robot holds position."""
        self._pause_stream_heartbeat()
        with self._lock:
            if self._robot is not None:
                self._robot.Stop()
            self._stopped = True
        logger.warning("Robot stopped")

    # ──────────────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────────────

    def _ensure_connected(self):
        """Raise if not connected."""
        if not self._connected or self._robot is None:
            raise ConnectionLost("Not connected. Call connect() first.")
        if self._stopped:
            raise RobotFault("Robot is stopped. Call connect() to re-initialize.")

    def _check_pose_safe(self, pose: Pose):
        """Validate a commanded pose against synchronous safety checks."""
        try:
            self._safety.check_workspace(pose)
        except SafetyViolation:
            self.stop()
            raise

    def _start_watchdog(self):
        """Start the robot-side safety watchdog."""
        self._stop_watchdog()
        self._watchdog = SafetyWatchdog(
            config=self._config.safety,
            get_state=self._get_state_for_watchdog,
            has_fault=self._fault_for_watchdog,
            stop_robot=self.stop,
        )
        self._watchdog.start()

    def _stop_watchdog(self):
        """Stop the watchdog thread if it is running."""
        if self._watchdog is not None:
            self._watchdog.shutdown()
            self._watchdog = None

    def _pause_stream_heartbeat(self):
        """Pause heartbeat monitoring during non-streaming robot operations."""
        if self._watchdog is not None:
            self._watchdog.pause_heartbeat()

    def _note_stream_command(self):
        """Refresh streaming heartbeat after a successful Cartesian command."""
        if self._watchdog is not None:
            self._watchdog.note_stream_command()

    def _get_state_for_watchdog(self) -> Optional[RobotState]:
        """Internal state read for watchdog use.

        Returns None when the robot is disconnected or manually stopped so the
        watchdog can stay quiet during teardown and stop conditions.
        """
        with self._lock:
            if not self._connected or self._robot is None or self._stopped:
                return None
            states = self._robot.states()
            operational = self._robot.operational()
            fault = self._robot.fault()
        return RobotState(
            pose=Pose.from_rdk(states.tcp_pose),
            wrench=Wrench.from_list(states.ext_wrench_in_tcp),
            joint_positions=list(states.q),
            timestamp=time.monotonic(),
            operational=operational,
            fault=fault,
        )

    def _fault_for_watchdog(self) -> bool:
        """Internal robot fault read for watchdog use."""
        with self._lock:
            if not self._connected or self._robot is None or self._stopped:
                return False
            return self._robot.fault()

    def _switch_mode(self, target_mode: flexivrdk.Mode):
        """Switch RDK mode if not already in it. Must hold self._lock."""
        if self._mode == target_mode:
            return
        self._robot.SwitchMode(target_mode)
        self._mode = target_mode
        if target_mode != flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE:
            self._configured_max_contact_wrench = None

    def _ensure_cartesian_contact_wrench_locked(self):
        """Apply configured contact-wrench regulation in Cartesian streaming mode."""
        target_wrench = tuple(
            float(value) for value in self._config.safety.max_contact_wrench
        )
        if self._configured_max_contact_wrench == target_wrench:
            return
        self._robot.SetMaxContactWrench(list(target_wrench))
        self._configured_max_contact_wrench = target_wrench
