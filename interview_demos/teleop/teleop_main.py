"""
VR Teleop Main Loop — A4
=========================
Ties VR reading + retargeting + Flexiv command in real-time loop.
1-Euro filter inline on VR pose.
Logs timestamped trajectories (pose, force) for RL training.
Clean shutdown on Ctrl+C or safety trigger.

Usage:
  python teleop_main.py              # Real hardware (VR + Flexiv)
  python teleop_main.py --mock       # Mock (no hardware)
  python teleop_main.py --mock-vr    # Mock VR only, real robot
  python teleop_main.py --mock-robot # Real VR, mock robot
  python teleop_main.py --input hand # Quest hand tracking via ZMQ
  python teleop_main.py --input hand --mock-robot  # Hand tracking + mock robot
"""

import numpy as np
import time
import signal
import argparse
import json
from pathlib import Path
from datetime import datetime

from vr_reader import VRReader, MockVRReader, HandTrackingReader
from retargeting import TaskSpaceRetargeter, RetargetConfig
from flexiv_commander import FlexivCommander, MockFlexivCommander, SafetyConfig, ImpedanceConfig
from safety_config import (SAFE_BOX_MIN, SAFE_BOX_MAX, MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
                           MAX_LINEAR_ACC, MAX_ANGULAR_ACC, FORCE_THRESHOLD,
                           STIFFNESS, DAMPING, print_safety_summary,
                           Z_APPROACH_STIFFNESS, Z_APPROACH_DAMPING, Z_APPROACH_XY_POS,
                           HAPTIC_ENABLED, HAPTIC_MAX_FORCE)

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
from teleop_filters import DropoutHandler


class TeleopLogger:
    """Logs trajectory data for RL training."""

    def __init__(self, log_dir: str = '/home/li2053/VisualFT/interview_demos/teleop/logs'):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filepath = self.log_dir / f'teleop_{ts}.jsonl'
        self._file = open(self.filepath, 'w')
        self._count = 0

    def log(self, t: float, vr_pos: np.ndarray, robot_pos: np.ndarray,
            robot_quat: np.ndarray, ext_wrench: np.ndarray,
            grip: bool, trigger: float, engaged: bool):
        entry = {
            't': round(t, 4),
            'vr_pos': vr_pos.tolist(),
            'robot_pos': robot_pos.tolist(),
            'robot_quat': robot_quat.tolist(),
            'wrench': ext_wrench.tolist(),
            'grip': grip,
            'trigger': round(trigger, 3),
            'engaged': engaged,
        }
        self._file.write(json.dumps(entry) + '\n')
        self._count += 1

    def close(self):
        self._file.close()
        print(f"Logged {self._count} samples to {self.filepath}")


def run_teleop(use_mock_vr=False, use_mock_robot=False, rate_hz=90, mode='full',
               input_mode='controller', zmq_host='localhost', zmq_keypoint_port=8089,
               zmq_pause_port=8102):
    """Main teleop loop."""

    # ─── Initialize components (all limits from safety_config.py) ───
    if use_mock_vr:
        vr = MockVRReader()
    elif input_mode == 'hand':
        vr = HandTrackingReader(host=zmq_host, keypoint_port=zmq_keypoint_port,
                                pause_port=zmq_pause_port)
    else:
        vr = VRReader('right')
    robot = MockFlexivCommander() if use_mock_robot else FlexivCommander(
        safety=SafetyConfig(
            pos_min=SAFE_BOX_MIN.copy(),
            pos_max=SAFE_BOX_MAX.copy(),
            max_linear_vel=MAX_LINEAR_VEL,
            max_angular_vel=MAX_ANGULAR_VEL,
            max_linear_acc=MAX_LINEAR_ACC,
            max_angular_acc=MAX_ANGULAR_ACC,
            force_threshold=FORCE_THRESHOLD,
        ),
        impedance=ImpedanceConfig(
            stiffness=STIFFNESS.copy(),
            damping=DAMPING.copy(),
        ),
    )
    # Hand tracking: use their proven axis swap [delta_y, -delta_x, delta_z]
    if input_mode == 'hand':
        vr_to_robot = np.array([
            [0, 1, 0],   # robot X ← VR Y
            [-1, 0, 0],  # robot Y ← -VR X
            [0, 0, 1],   # robot Z ← VR Z
        ], dtype=float)
        retarget = TaskSpaceRetargeter(RetargetConfig(
            vr_to_robot_rot=vr_to_robot,
            vr_workspace=1.0,             # hand tracking units
            robot_workspace=np.array([0.7, 0.7, 0.7]),  # 0.7x scale
            orientation_scale=0.3,        # hand orientation is noisy
        ))
    else:
        retarget = TaskSpaceRetargeter()  # pulls box from safety_config automatically
    dropout = DropoutHandler(timeout_ms=50, max_extrapolate_ms=200)
    logger = TeleopLogger()

    # ─── Shutdown handler ───
    running = True
    def on_signal(sig, frame):
        nonlocal running
        running = False
        print("\nShutting down...")
    signal.signal(signal.SIGINT, on_signal)

    # ─── Connect ───
    print("Connecting VR...")
    vr.connect()
    print("Connecting robot...")
    robot.connect()

    # ─── Print safety box + confirm TCP is inside ───
    robot_state = robot.get_state()
    current_tcp = robot_state.get('tcp_pos', None)
    print_safety_summary(current_tcp)

    if current_tcp is not None:
        if np.any(current_tcp < SAFE_BOX_MIN) or np.any(current_tcp > SAFE_BOX_MAX):
            print("WARNING: Current TCP is OUTSIDE the safety box!")
            print("Move the robot inside the box before engaging teleop.")

    # ─── Set mode ───
    if mode == 'z-approach' and not use_mock_robot:
        robot.set_z_approach_mode(Z_APPROACH_STIFFNESS, Z_APPROACH_DAMPING)

    input_label = 'hand tracking (ZMQ)' if input_mode == 'hand' else 'controller (OpenVR)'
    print(f"=== VR Teleop Active ({rate_hz} Hz) — mode: {mode}, input: {input_label} ===")
    if input_mode == 'hand':
        print("  Quest APK button to engage/disengage teleop")
        print("  No haptic feedback (hand tracking)")
    else:
        print("  GRIP to engage/disengage teleop")
        print("  TRIGGER for grasp (logged)")
    if mode == 'z-approach':
        print(f"  Z-approach: XY locked at [{Z_APPROACH_XY_POS[0]:.3f}, {Z_APPROACH_XY_POS[1]:.3f}]")
        print("  VR controls Z only (up/down). Compliant Z on contact.")
    else:
        print("  Full 6DOF: VR maps to all Cartesian axes")
    print("  Ctrl+C to stop\n")

    dt = 1.0 / rate_hz
    loop_count = 0
    t0 = time.monotonic()

    try:
        while running:
            t_loop_start = time.monotonic()
            t = t_loop_start - t0

            # ─── Read VR ───
            vr_state = vr.read()

            # Dropout handling
            vr_pos_in = vr_state.pos if vr_state.valid else None
            vr_pos_safe, is_dropout = dropout.update(vr_pos_in, t)

            if is_dropout and loop_count % 90 == 0:
                print(f"[{t:.1f}s] VR DROPOUT — holding last pose")

            # ─── Clutch ───
            robot_state = robot.get_state()
            current_pos = robot_state.get('tcp_pos', np.array([0.5, 0, 0.4]))
            # Convert robot quat from RDK [qw,qx,qy,qz] to scipy [qx,qy,qz,qw]
            qw_first = robot_state.get('tcp_quat_wfirst', np.array([1, 0, 0, 0]))
            current_quat_xyzw = np.array([qw_first[1], qw_first[2], qw_first[3], qw_first[0]])
            retarget.update_clutch(vr_state.grip, vr_pos_safe, vr_state.rot, current_pos, current_quat_xyzw)

            # ─── Retarget ───
            target_pos, target_quat = retarget.retarget(vr_pos_safe, vr_state.rot, t)

            # Z-approach mode: lock XY to phantom center, only VR Z passes through
            if mode == 'z-approach':
                target_pos[0] = Z_APPROACH_XY_POS[0]
                target_pos[1] = Z_APPROACH_XY_POS[1]

            # ─── Send to robot ───
            # NRT mode requires continuous commands (heartbeat).
            # When engaged: send retargeted pose.
            # When not engaged: resend current pose to hold position.
            if retarget.is_engaged:
                robot.send_pose(target_pos, target_quat, t)
            else:
                robot.send_pose(current_pos, current_quat_xyzw, t)

            # ─── Haptic feedback: force → vibration ───
            ext_wrench = robot_state.get('ext_wrench', np.zeros(6))
            if HAPTIC_ENABLED:
                force_mag = np.linalg.norm(ext_wrench[:3])
                vr.send_haptic(force_mag, HAPTIC_MAX_FORCE)

            # ─── Log ───
            logger.log(t, vr_pos_safe, target_pos, target_quat,
                       ext_wrench, vr_state.grip, vr_state.trigger,
                       retarget.is_engaged)

            # ─── Status (every 2s) ───
            if loop_count % (rate_hz * 2) == 0 and loop_count > 0:
                avg_dt = (time.monotonic() - t0) / loop_count
                print(f"[{t:.1f}s] pos={target_pos}, engaged={retarget.is_engaged}, "
                      f"trigger={vr_state.trigger:.2f}, rate={1/avg_dt:.0f}Hz")

            # ─── Rate limit ───
            loop_count += 1
            elapsed = time.monotonic() - t_loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except Exception as e:
        print(f"ERROR: {e}")
        robot.stop()
        raise

    finally:
        print("\nStopping robot...")
        robot.disconnect()
        vr.shutdown()
        logger.close()
        print("Teleop shutdown complete.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='VR Teleop for Flexiv Rizon4')
    parser.add_argument('--mock', action='store_true', help='Mock everything (no hardware)')
    parser.add_argument('--mock-vr', action='store_true', help='Mock VR only')
    parser.add_argument('--mock-robot', action='store_true', help='Mock robot only')
    parser.add_argument('--rate', type=int, default=90, help='Loop rate Hz')
    parser.add_argument('--mode', choices=['full', 'z-approach'], default='full',
                        help='full = 6DOF VR teleop | z-approach = VR Z only, compliant contact')
    parser.add_argument('--input', choices=['controller', 'hand'], default='controller',
                        dest='input_mode',
                        help='controller = SteamVR OpenVR | hand = Quest hand tracking via ZMQ')
    parser.add_argument('--zmq-host', default='localhost', help='ZMQ host for hand tracking')
    parser.add_argument('--zmq-keypoint-port', type=int, default=8089,
                        help='ZMQ port for transformed hand keypoints')
    parser.add_argument('--zmq-pause-port', type=int, default=8102,
                        help='ZMQ port for teleop pause/resume')
    args = parser.parse_args()

    mock_vr = args.mock or args.mock_vr
    mock_robot = args.mock or args.mock_robot

    run_teleop(use_mock_vr=mock_vr, use_mock_robot=mock_robot,
               rate_hz=args.rate, mode=args.mode, input_mode=args.input_mode,
               zmq_host=args.zmq_host, zmq_keypoint_port=args.zmq_keypoint_port,
               zmq_pause_port=args.zmq_pause_port)
