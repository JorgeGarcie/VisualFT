#!/usr/bin/env python3

"""scan_node.py

Session-based automated phantom scanning with force control and MCAP recording.

Reads a session config YAML that defines one or more scans.  For each scan the
node runs the state machine (HOMING → ZEROING_FT → DESCENDING → SCANNING →
RETURNING), manages its own `ros2 bag record` subprocess (one MCAP bag per
scan), then moves on to the next scan.  Exits automatically when all scans are
done.

Without a session config, runs a single scan using ROS parameters (backward
compatible with the original interface).

Publishes:
  /scan/state    (std_msgs/String)              State machine phase + pass info
  /rdk/tcp_pose  (geometry_msgs/PoseStamped)    TCP pose at control rate
  /rdk/wrench    (geometry_msgs/WrenchStamped)  External wrench at control rate
"""

import math
import os
import signal
import subprocess
import time
from datetime import datetime

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, WrenchStamped
import flexivrdk
from visionft.rdk_utils import init_robot, build_pose_msg, build_wrench_msg, euler_to_rdk_quat


RECORD_TOPICS = [
    '/scan/state',
    '/rdk/tcp_pose',
    '/rdk/wrench',
    '/coinft/wrench',
    '/image_raw',
]

SCAN_PARAM_DEFAULTS = {
    'home_pose': [512.0, 260.0, 45.0, 0.05, -179.44, 0.0],
    'y_scan_range': 0.03,
    'scan_speed': 0.04,
    'contact_force': 20.0,
    'search_velocity': 0.02,
    'rz_start': 0.0,
    'rz_end': 175.0,
    'rz_step': 5.0,
    'rx_start': 0.0,
    'rx_end': 0.0,
    'rx_step': 5.0,
    'x_offsets_mm': [0.0, 1.0, 2.0, 1.0, -1.0, 0.0],
}


class ScanNode(Node):
    def __init__(self):
        super().__init__('scan_node')

        # -- ROS parameters (used when no session config is provided) --
        self.declare_parameter('robot_sn', 'Rizon4-062174')
        self.declare_parameter('session_config', '')
        self.declare_parameter('output_dir',
                               os.path.join(os.path.expanduser('~'),
                                            'VisualFT', 'data'))
        self.declare_parameter('record', True)
        self.declare_parameter('control_rate_hz', 50.0)

        # Motion limits (global, not per-scan — these are safety caps)
        self.declare_parameter('movel_vel', 0.05)          # m/s for MoveL primitives
        self.declare_parameter('max_linear_vel', 0.05)     # m/s
        self.declare_parameter('max_angular_vel', 0.3)     # rad/s
        self.declare_parameter('max_linear_acc', 0.3)      # m/s²
        self.declare_parameter('max_angular_acc', 0.5)     # rad/s²

        # Impedance parameters for scanning phase
        # Z: soft spring + heavy damping — robot gently follows surface
        # XY: stiff — stays on the commanded scan line
        self.declare_parameter('z_stiffness', 5000.0)      # N/m
        self.declare_parameter('xy_stiffness', 5000.0)    # N/m
        self.declare_parameter('rot_stiffness', 1500.0)   # Nm/rad
        self.declare_parameter('damping_ratio', 0.8)       # [0.3–0.8], high = no bounce

        # Sinusoidal X oscillation during Y sweep
        self.declare_parameter('x_amplitude', 0.0)     # m  (0 = straight line, disabled by default)
        self.declare_parameter('x_period', 0.008)      # m  (full oscillation every 8mm of Y)

        # Declare scan params so they can be overridden via --ros-args
        for key, default in SCAN_PARAM_DEFAULTS.items():
            self.declare_parameter(key, default)

        self._robot_sn = self.get_parameter('robot_sn').value
        self._output_dir = (self.get_parameter('output_dir').value
                            or os.path.join(os.path.expanduser('~'),
                                            'VisualFT', 'data'))
        self._do_record = self.get_parameter('record').value
        rate_hz = self.get_parameter('control_rate_hz').value
        self._movel_vel = self.get_parameter('movel_vel').value
        self._max_lin_vel = self.get_parameter('max_linear_vel').value
        self._max_ang_vel = self.get_parameter('max_angular_vel').value
        self._max_lin_acc = self.get_parameter('max_linear_acc').value
        self._max_ang_acc = self.get_parameter('max_angular_acc').value
        self._z_stiffness = self.get_parameter('z_stiffness').value
        self._xy_stiffness = self.get_parameter('xy_stiffness').value
        self._rot_stiffness = self.get_parameter('rot_stiffness').value
        self._damping_ratio = self.get_parameter('damping_ratio').value
        self._x_amplitude = self.get_parameter('x_amplitude').value
        self._x_period = self.get_parameter('x_period').value

        # -- Publishers --
        self._state_pub = self.create_publisher(String, '/scan/state', 10)
        self._tcp_pub = self.create_publisher(PoseStamped, '/rdk/tcp_pose', 10)
        self._wrench_pub = self.create_publisher(WrenchStamped, '/rdk/wrench', 10)

        # -- Load session --
        self._session = self._load_session()
        self._scan_idx = 0
        self._bag_process = None

        # -- State machine --
        self._state = 'INIT'
        self._pass_idx = 0
        self._scan_init_pose = None
        self._contact_z = None
        self._contact_settle_until = 0.0
        self._shutdown_requested = False

        # Per-scan params (set by _begin_scan)
        self._home_pos_m = None
        self._home_rot_deg = None
        self._passes = []
        self._scan_target_pose = None
        self._scan_target_wrench = None
        self._scan_target_y = None
        self._scanning_settle_until = 0.0

        # -- Connect robot --
        self._robot = self._init_robot()

        # -- Start control loop --
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"Scan node running at {rate_hz} Hz")

        n = len(self._session['scans'])
        self.get_logger().info(
            f"Session '{self._session['session_name']}': {n} scan(s)")

        # Start first scan
        self._begin_scan()

    # ======================================================================
    # Session loading
    # ======================================================================
    def _load_session(self):
        config_path = self.get_parameter('session_config').value
        if config_path:
            self.get_logger().info(f"Loading session config: {config_path}")
            with open(config_path) as f:
                session = yaml.safe_load(f)
            if 'session_name' not in session:
                session['session_name'] = (
                    f'session_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
            if 'scans' not in session or len(session['scans']) == 0:
                raise ValueError(
                    "session_config must have at least one entry in 'scans'")
            return session

        # No config file — single scan from ROS params
        return {
            'session_name': f'scan_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
            'scans': [{'name': 'scan'}],
        }

    def _get_scan_params(self, scan_config):
        """Merge: hardcoded defaults < session defaults < per-scan overrides.
        When no session config is used, ROS params override hardcoded defaults."""
        params = dict(SCAN_PARAM_DEFAULTS)

        if self.get_parameter('session_config').value:
            # YAML mode: session defaults, then per-scan overrides
            for k, v in self._session.get('defaults', {}).items():
                if k in params:
                    params[k] = v
            for k, v in scan_config.items():
                if k in params:
                    params[k] = v
        else:
            # ROS param mode: override defaults with declared params
            for key in params:
                params[key] = self.get_parameter(key).value

        return params

    # ======================================================================
    # Per-scan lifecycle
    # ======================================================================
    def _begin_scan(self):
        scan_config = self._session['scans'][self._scan_idx]
        scan_name = scan_config.get('name', f'scan_{self._scan_idx:03d}')

        self.get_logger().info(
            f"{'=' * 60}\n"
            f"  Scan {self._scan_idx + 1}/{len(self._session['scans'])}: "
            f"{scan_name}\n"
            f"{'=' * 60}")

        # Load params for this scan
        params = self._get_scan_params(scan_config)

        home_elements = params['home_pose']
        self._home_pos_m = [
            home_elements[0] / 1000.0,
            home_elements[1] / 1000.0,
            home_elements[2] / 1000.0,
        ]
        self._home_rot_deg = [
            home_elements[3],
            home_elements[4],
            home_elements[5],
        ]
        self._y_scan_range = params['y_scan_range']
        self._scan_speed = params['scan_speed']
        self._contact_force = params['contact_force']
        self._search_velocity = params['search_velocity']

        # Build passes: each rz gets an x_offset cycling through x_offsets_mm
        rz_values = self._arange_inclusive(
            params['rz_start'], params['rz_end'], params['rz_step'])
        rx_values = self._arange_inclusive(
            params['rx_start'], params['rx_end'], params['rx_step'])
        x_offsets_mm = params.get('x_offsets_mm', [0.0])
        self._passes = []
        for i, rz in enumerate(rz_values):
            x_m = x_offsets_mm[i % len(x_offsets_mm)] / 1000.0
            for rx in rx_values:
                self._passes.append((rz, rx, True, x_m))   # forward
                self._passes.append((rz, rx, False, x_m))  # backward

        self.get_logger().info(f"  {len(self._passes)} passes")

        # Start bag recording
        self._start_bag(scan_name)

        # Reset state machine for this scan
        self._pass_idx = 0
        self._forward = True
        self._scan_init_pose = None
        self._contact_z = None
        self._contact_settle_until = 0.0
        self._transition('HOMING')

    def _finish_scan(self):
        """Called when RETURNING completes for the current scan."""
        self._stop_bag()
        self._scan_idx += 1

        if self._scan_idx < len(self._session['scans']):
            self._begin_scan()
        else:
            self.get_logger().info("All scans complete!")
            self._robot.Stop()
            self._state = 'DONE'
            self._publish_state('done')

    # ======================================================================
    # Bag recording management
    # ======================================================================
    def _start_bag(self, scan_name):
        if not self._do_record:
            return
        bag_dir = os.path.join(
            self._output_dir,
            self._session['session_name'],
            f'{self._scan_idx:03d}_{scan_name}',
        )
        os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
        cmd = [
            'ros2', 'bag', 'record',
            '--output', bag_dir,
            '--storage', 'mcap',
            '--max-cache-size', '524288000',
        ] + RECORD_TOPICS
        self._bag_process = subprocess.Popen(cmd)
        self.get_logger().info(f"  Recording → {bag_dir}")

    def _stop_bag(self):
        if self._bag_process is None:
            return
        self._bag_process.send_signal(signal.SIGINT)
        try:
            self._bag_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self._bag_process.terminate()
            self._bag_process.wait(timeout=5)
        self._bag_process = None
        self.get_logger().info("  Recording stopped")

    # ======================================================================
    # Robot init
    # ======================================================================
    def _init_robot(self):
        return init_robot(self._robot_sn, logger=self.get_logger())

    # ======================================================================
    # Helpers
    # ======================================================================
    @staticmethod
    def _arange_inclusive(start, end, step):
        if step <= 0:
            return [start]
        vals = []
        v = start
        while v <= end + 1e-9:
            vals.append(round(v, 6))
            v += step
        return vals

    @staticmethod
    def _euler_to_rdk_quat(rx_deg, ry_deg, rz_deg):
        """Euler ZYX degrees → RDK quaternion [qw, qx, qy, qz]."""
        return euler_to_rdk_quat(rx_deg, ry_deg, rz_deg)

    def _publish_state(self, label):
        msg = String()
        msg.data = label
        self._state_pub.publish(msg)

    def _publish_feedback(self):
        states = self._robot.states()
        now = self.get_clock().now().to_msg()
        self._tcp_pub.publish(build_pose_msg(states.tcp_pose, now, 'world'))
        self._wrench_pub.publish(build_wrench_msg(states.ext_wrench_in_world, now, 'world'))

    # ======================================================================
    # State transitions
    # ======================================================================
    def _transition(self, new_state):
        self.get_logger().info(f"State: {self._state} → {new_state}")
        self._state = new_state

        if new_state == 'HOMING':
            self._enter_homing()
        elif new_state == 'ZEROING_FT':
            self._enter_zeroing_ft()
        elif new_state == 'DESCENDING':
            self._enter_descending()
        elif new_state == 'SCANNING':
            self._enter_scanning()
        elif new_state == 'RETURNING':
            self._enter_returning()

    def _enter_homing(self):
        self._publish_state('homing')
        self._robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
        self._robot.ExecutePrimitive(
            "MoveL",
            {
                "target": flexivrdk.Coord(
                    self._home_pos_m,
                    self._home_rot_deg,
                    ["WORLD", "WORLD_ORIGIN"],
                ),
                "vel": self._movel_vel,
            },
        )

    def _enter_zeroing_ft(self):
        self._publish_state('zeroing_ft')
        self._robot.ExecutePrimitive("ZeroFTSensor", dict())

    def _enter_descending(self):
        self._publish_state('descending')
        self._robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
        self._robot.SetForceControlAxis([False] * 6)
        self._robot.SetMaxContactWrench([50.0, 50.0, 50.0, 10.0, 10.0, 10.0])

        target = list(self._robot.states().tcp_pose)
        target[2] -= 1.0
        self._robot.SendCartesianMotionForce(
            target, [0] * 6,
            self._search_velocity,
            self._max_ang_vel,
            self._max_lin_acc,
            self._max_ang_acc)

    def _enter_scanning(self):
        if len(self._passes) == 0:
            self.get_logger().info("No scan passes — skipping to RETURNING")
            self._transition('RETURNING')
            return

        # Already in NRT_CARTESIAN_MOTION_FORCE from DESCENDING — just update impedance
        self._robot.SetForceControlAxis([False] * 6)
        K = self._xy_stiffness
        Kz = self._z_stiffness
        Kr = self._rot_stiffness
        Z = self._damping_ratio
        self._robot.SetCartesianImpedance(
            [K, K, Kz, Kr, Kr, Kr],   # stiffness [Kx, Ky, Kz, Krx, Kry, Krz]
            [Z, Z, Z, Z, Z, Z])       # damping ratio (same for all axes)
        self._robot.SetMaxContactWrench([50.0, 50.0, 50.0, 10.0, 10.0, 10.0])

        self._scan_init_pose = list(self._robot.states().tcp_pose)
        self._pass_idx = 0
        self._scanning_settle_until = time.monotonic() + 0.5
        self._start_pass()

    def _enter_returning(self):
        self._publish_state('returning')
        self._robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
        self._robot.ExecutePrimitive(
            "MoveL",
            {
                "target": flexivrdk.Coord(
                    self._home_pos_m,
                    self._home_rot_deg,
                    ["WORLD", "WORLD_ORIGIN"],
                ),
                "vel": self._movel_vel,
            },
        )

    def _start_pass(self):
        rz_offset, rx_offset, forward, x_offset = self._passes[self._pass_idx]

        rx = self._home_rot_deg[0] + rx_offset
        ry = self._home_rot_deg[1]
        rz = self._home_rot_deg[2] + rz_offset
        q = self._euler_to_rdk_quat(rx, ry, rz)

        home_y = self._home_pos_m[1]
        target_y = (home_y + self._y_scan_range) if forward else home_y
        self._pass_start_y = home_y if forward else (home_y + self._y_scan_range)

        self._scan_target_pose = [
            self._home_pos_m[0] + x_offset,
            target_y,
            self._contact_z,
            q[0], q[1], q[2], q[3],
        ]
        self._scan_target_wrench = [0.0] * 6  # impedance handles force, no explicit wrench
        self._scan_target_y = target_y

        direction = 'fwd' if forward else 'bwd'
        label = f"scanning_rz{rz_offset}_rx{rx_offset}_x{x_offset*1000:.0f}mm_{direction}"
        self._publish_state(label)
        self.get_logger().info(
            f"Pass {self._pass_idx + 1}/{len(self._passes)}: {label}, "
            f"target_y={target_y:.4f}")

    # ======================================================================
    # Main tick
    # ======================================================================
    def _tick(self):
        try:
            if self._state == 'DONE':
                if not self._shutdown_requested:
                    self._shutdown_requested = True
                    self.create_timer(1.0, self._request_shutdown)
                return

            if self._robot.fault():
                self.get_logger().error("Robot fault detected!")
                self._stop_bag()
                self._state = 'DONE'
                self._publish_state('fault')
                return

            self._publish_feedback()

            if self._state == 'HOMING':
                self._tick_homing()
            elif self._state == 'ZEROING_FT':
                self._tick_zeroing_ft()
            elif self._state == 'DESCENDING':
                self._tick_descending()
            elif self._state == 'SCANNING':
                self._tick_scanning()
            elif self._state == 'RETURNING':
                self._tick_returning()

        except Exception as e:
            self.get_logger().error(f"Tick error: {e}")

    def _tick_homing(self):
        self._publish_state('homing')
        if self._robot.primitive_states().get("reachedTarget"):
            self._transition('ZEROING_FT')

    def _tick_zeroing_ft(self):
        self._publish_state('zeroing_ft')
        if self._robot.primitive_states().get("terminated"):
            self.get_logger().info("FT sensor zeroing complete")
            self._transition('DESCENDING')

    def _tick_descending(self):
        self._publish_state('descending')
        ext_force = np.array(self._robot.states().ext_wrench_in_world[:3])
        if np.linalg.norm(ext_force) > self._contact_force:
            self._contact_z = self._robot.states().tcp_pose[2]
            self.get_logger().info(
                f"Contact! force={np.linalg.norm(ext_force):.2f}N, "
                f"contact Z={self._contact_z:.4f}m")
            self._transition('SCANNING')

    def _tick_scanning(self):
        if self._scan_target_pose is None:
            return
        if time.monotonic() < self._scanning_settle_until:
            return  # wait for impedance settings to take effect

        # Update X sinusoidally based on Y progress along this pass
        current_y = self._robot.states().tcp_pose[1]
        y_progress = abs(current_y - self._pass_start_y)
        if self._x_period > 0 and self._x_amplitude > 0:
            x_offset = self._x_amplitude * math.sin(
                2.0 * math.pi * y_progress / self._x_period)
            self._scan_target_pose[0] = self._home_pos_m[0] + x_offset

        self._robot.SendCartesianMotionForce(
            self._scan_target_pose,
            self._scan_target_wrench,
            self._scan_speed,
            self._max_ang_vel,
            self._max_lin_acc,
            self._max_ang_acc)

        if abs(current_y - self._scan_target_y) < 0.002:
            rz_offset, rx_offset, _, x_offset = self._passes[self._pass_idx]
            done_label = f"pass_done_rz{rz_offset}_rx{rx_offset}_x{x_offset*1000:.0f}mm"
            self._publish_state(done_label)
            self.get_logger().info(
                f"Pass {self._pass_idx + 1}/{len(self._passes)} done")

            self._pass_idx += 1

            if self._pass_idx >= len(self._passes):
                self.get_logger().info("All passes complete!")
                self._transition('RETURNING')
            else:
                # Pause 0.5s when orientation or x_offset changes
                next_rz, next_rx, _, next_x = self._passes[self._pass_idx]
                if (next_rz, next_rx, next_x) != (rz_offset, rx_offset, x_offset):
                    self.get_logger().info(
                        f"Config change → rz={next_rz}, rx={next_rx}, "
                        f"x={next_x*1000:.0f}mm (settling 0.5s)")
                    self._scanning_settle_until = time.monotonic() + 0.5
                self._start_pass()

    def _tick_returning(self):
        self._publish_state('returning')
        if self._robot.primitive_states().get("reachedTarget"):
            self.get_logger().info("Returned to home")
            self._finish_scan()

    def _request_shutdown(self):
        self.get_logger().info("Session complete — shutting down")
        raise SystemExit(0)

    # ======================================================================
    # Cleanup
    # ======================================================================
    def destroy_node(self):
        self.get_logger().info("Stopping robot...")
        self._stop_bag()
        try:
            self._robot.Stop()
        except Exception as e:
            self.get_logger().warning(f"Error stopping robot: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # normal shutdown
    except SystemExit:
        pass  # scan_node requests this on completion
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
