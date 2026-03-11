"""
VR Pose Streaming — A1
======================
Two input modes:
  1. SteamVR controller: openvr Python — 6DOF rigid body + buttons + haptics
  2. Quest hand tracking: ZMQ subscriber — wrist frame from LeapFT pipeline

Both produce the same VRState interface for the teleop loop.
"""

import sys
import numpy as np
import time
from dataclasses import dataclass
from typing import Optional

# Ensure user site-packages is on path (ROS2 Python strips it)
_user_site = '/home/li2053/.local/lib/python3.10/site-packages'
if _user_site not in sys.path:
    sys.path.append(_user_site)


@dataclass
class VRState:
    """Current state of a VR controller."""
    pos: np.ndarray          # [x, y, z] in meters (SteamVR coordinates)
    rot: np.ndarray          # 3x3 rotation matrix
    grip: bool               # grip button (enable teleop)
    trigger: float           # trigger axis [0, 1]
    valid: bool              # tracking valid
    timestamp: float


class VRReader:
    """Reads VR controller pose from SteamVR via OpenVR."""

    def __init__(self, hand: str = 'right'):
        """
        Args:
            hand: 'left' or 'right' controller
        """
        import openvr as _openvr
        self._openvr = _openvr
        self.hand = hand
        self.vr = None
        self._controller_idx: Optional[int] = None

    def connect(self):
        """Initialize OpenVR and find the controller."""
        self.vr = self._openvr.init(self._openvr.VRApplication_Other)
        self._find_controller()
        print(f"VR connected. Controller ({self.hand}): index {self._controller_idx}")

    def _find_controller(self):
        """Find the target controller device index."""
        ovr = self._openvr
        for i in range(ovr.k_unMaxTrackedDeviceCount):
            device_class = self.vr.getTrackedDeviceClass(i)
            if device_class == ovr.TrackedDeviceClass_Controller:
                role = self.vr.getControllerRoleForTrackedDeviceIndex(i)
                if self.hand == 'right' and role == ovr.TrackedControllerRole_RightHand:
                    self._controller_idx = i
                    return
                elif self.hand == 'left' and role == ovr.TrackedControllerRole_LeftHand:
                    self._controller_idx = i
                    return

        # Fallback: use first controller found
        for i in range(ovr.k_unMaxTrackedDeviceCount):
            if self.vr.getTrackedDeviceClass(i) == ovr.TrackedDeviceClass_Controller:
                self._controller_idx = i
                print(f"Warning: couldn't find {self.hand} hand, using controller {i}")
                return

        raise RuntimeError("No VR controller found")

    def read(self) -> VRState:
        """Read current controller state. Non-blocking."""
        ovr = self._openvr
        if self._controller_idx is None:
            raise RuntimeError("Not connected. Call connect() first.")

        t = time.monotonic()

        # Get tracked device pose
        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            ovr.TrackingUniverseStanding, 0,
            ovr.k_unMaxTrackedDeviceCount
        )
        pose = poses[self._controller_idx]
        valid = pose.bPoseIsValid

        # Extract 3x4 matrix → position + rotation
        mat = pose.mDeviceToAbsoluteTracking
        rot = np.array([
            [mat[0][0], mat[0][1], mat[0][2]],
            [mat[1][0], mat[1][1], mat[1][2]],
            [mat[2][0], mat[2][1], mat[2][2]],
        ])
        pos = np.array([mat[0][3], mat[1][3], mat[2][3]])

        # Get button state
        success, state = self.vr.getControllerState(self._controller_idx)
        grip = bool(state.ulButtonPressed & (1 << ovr.k_EButton_Grip))
        trigger = state.rAxis[0].x if success else 0.0  # axis 0 = trigger

        return VRState(
            pos=pos,
            rot=rot,
            grip=grip,
            trigger=trigger,
            valid=valid,
            timestamp=t,
        )

    def send_haptic(self, force_magnitude: float, max_force: float = 30.0):
        """Send force-reflective haptic pulse to the controller.

        Maps robot external force magnitude to controller vibration intensity.
        Higher force = stronger/longer vibration — operator feels contact.

        Args:
            force_magnitude: current external force (N) from robot F/T sensor
            max_force: force that maps to maximum vibration (N)
        """
        if self._controller_idx is None or self.vr is None:
            return

        # Map force [0, max_force] → duration [0, 3999] microseconds
        # OpenVR haptic pulse: axis 0, max ~3999 µs per call
        intensity = min(force_magnitude / max_force, 1.0)

        if intensity < 0.05:
            return  # deadzone — don't buzz for negligible force

        duration_us = int(intensity * 3500)  # 0–3500 µs range
        self.vr.triggerHapticPulse(self._controller_idx, 0, duration_us)

    def shutdown(self):
        if self.vr:
            self._openvr.shutdown()
            self.vr = None


# ─── Hand Tracking VR reader (Quest → ZMQ) ──────────────────────────────────────

class HandTrackingReader:
    """Reads wrist 6DOF from Quest hand tracking via ZMQ (LeapFT pipeline).

    Subscribes to the transformed_hand_frame topic published by
    TransformHandPositionCoords. Data format: [origin, x_axis, y_axis, z_axis]
    where origin is wrist position in robot frame and axes form the palm orientation.

    Engage/disengage comes from the Quest APK button via the 'pause' topic.
    """

    def __init__(self, host: str = 'localhost', keypoint_port: int = 8089,
                 pause_port: int = 8102):
        self._host = host
        self._keypoint_port = keypoint_port
        self._pause_port = pause_port
        self._frame_sub = None
        self._pause_sub = None
        self._engaged = False
        self._prev_engaged = None  # track transitions, not raw state
        self._last_pos = np.zeros(3)
        self._last_rot = np.eye(3)
        self._has_data = False

    def connect(self):
        """Connect ZMQ subscribers to the LeapFT pipeline."""
        import zmq
        import pickle

        # Subscribe to transformed wrist frame
        ctx = zmq.Context()

        self._frame_sub = ctx.socket(zmq.SUB)
        self._frame_sub.setsockopt(zmq.CONFLATE, 1)  # only keep latest
        self._frame_sub.connect(f'tcp://{self._host}:{self._keypoint_port}')
        self._frame_sub.setsockopt(zmq.SUBSCRIBE, b'transformed_hand_frame')

        self._pause_sub = ctx.socket(zmq.SUB)
        self._pause_sub.setsockopt(zmq.CONFLATE, 1)
        self._pause_sub.connect(f'tcp://{self._host}:{self._pause_port}')
        self._pause_sub.setsockopt(zmq.SUBSCRIBE, b'pause')

        self._pickle = pickle
        self._zmq = zmq
        print(f"HandTracking connected: frames tcp://{self._host}:{self._keypoint_port}, "
              f"pause tcp://{self._host}:{self._pause_port}")

    def read(self) -> VRState:
        """Read wrist frame from ZMQ. Non-blocking (uses latest cached data)."""
        t = time.monotonic()

        # Poll pause/engage state — only engage on explicit STOP→CONT transition
        try:
            raw = self._pause_sub.recv(self._zmq.NOBLOCK)
            topic_prefix = b'pause '
            if raw.startswith(topic_prefix):
                raw = raw[len(topic_prefix):]
            data = self._pickle.loads(raw)
            if isinstance(data, (int, np.integer)):
                new_state = bool(data)
            elif isinstance(data, (list, np.ndarray)):
                new_state = bool(data[0])
            else:
                new_state = bool(data)

            if self._prev_engaged is None:
                # First message — record state but stay disengaged
                self._prev_engaged = new_state
                self._engaged = False
            else:
                self._engaged = new_state
                self._prev_engaged = new_state
        except self._zmq.Again:
            pass  # no new data on ZMQ NOBLOCK — expected, keep last state

        # Poll wrist frame
        valid = False
        try:
            raw = self._frame_sub.recv(self._zmq.NOBLOCK)
            topic_prefix = b'transformed_hand_frame '
            if raw.startswith(topic_prefix):
                raw = raw[len(topic_prefix):]
            frame = self._pickle.loads(raw)
            # frame = [origin, x_axis, y_axis, z_axis] — each is 3D vector
            if frame is not None and len(frame) >= 4:
                origin = np.array(frame[0])
                x_axis = np.array(frame[1])
                y_axis = np.array(frame[2])
                z_axis = np.array(frame[3])

                if not np.allclose(origin, 0, atol=1e-6):
                    self._last_pos = origin
                    rot = np.column_stack([x_axis, y_axis, z_axis])
                    # Ensure valid rotation matrix
                    if np.linalg.det(rot) < 0:
                        rot[:, 0] = -rot[:, 0]
                    self._last_rot = rot
                    self._has_data = True
                    valid = True
        except self._zmq.Again:
            valid = self._has_data  # use cached data if we had data before

        return VRState(
            pos=self._last_pos.copy(),
            rot=self._last_rot.copy(),
            grip=self._engaged,
            trigger=0.0,  # no trigger with hand tracking
            valid=valid,
            timestamp=t,
        )

    def send_haptic(self, force_magnitude: float, max_force: float = 30.0):
        """No haptics with hand tracking — no-op."""
        pass

    def shutdown(self):
        if self._frame_sub is not None:
            self._frame_sub.close()
        if self._pause_sub is not None:
            self._pause_sub.close()


# ─── Mock VR reader for testing without hardware ────────────────────────────────
class MockVRReader:
    """Simulates VR controller for development/testing."""

    def __init__(self):
        self._t0 = time.monotonic()
        self._last_haptic = 0.0

    def connect(self):
        print("MockVR connected (simulated controller)")

    def read(self) -> VRState:
        t = time.monotonic() - self._t0
        # Figure-8 pattern
        pos = np.array([
            0.1 * np.sin(0.5 * t),
            0.3 + 0.08 * np.sin(1.0 * t),
            1.0 + 0.05 * np.sin(0.3 * t),
        ])
        rot = np.eye(3)
        return VRState(
            pos=pos, rot=rot, grip=True,
            trigger=0.5 * (1 + np.sin(0.2 * t)),
            valid=True, timestamp=time.monotonic(),
        )

    def send_haptic(self, force_magnitude: float, max_force: float = 30.0):
        """Mock haptic — prints force level periodically."""
        t = time.monotonic()
        if force_magnitude > 1.0 and t - self._last_haptic > 1.0:
            intensity = min(force_magnitude / max_force, 1.0)
            print(f"  [MockHaptic] force={force_magnitude:.1f}N → intensity={intensity:.0%}")
            self._last_haptic = t

    def shutdown(self):
        pass


if __name__ == '__main__':
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else 'mock'

    if mode == 'hand':
        host = sys.argv[2] if len(sys.argv) > 2 else 'localhost'
        print(f"Testing hand tracking reader (host={host})...")
        vr = HandTrackingReader(host=host)
        vr.connect()
        for i in range(300):  # ~5s at 60Hz
            state = vr.read()
            if i % 30 == 0:
                print(f"pos={state.pos}, grip={state.grip}, valid={state.valid}")
            time.sleep(1 / 60)
        vr.shutdown()
    elif mode == 'controller':
        try:
            vr = VRReader('right')
            vr.connect()
            for _ in range(100):
                state = vr.read()
                print(f"pos={state.pos}, grip={state.grip}, trigger={state.trigger:.2f}, valid={state.valid}")
                time.sleep(1 / 90)
            vr.shutdown()
        except Exception as e:
            print(f"VR controller not available: {e}")
    else:
        print("Running mock VR...")
        vr = MockVRReader()
        vr.connect()
        for _ in range(10):
            state = vr.read()
            print(f"pos={state.pos}, trigger={state.trigger:.2f}")
            time.sleep(0.1)
        vr.shutdown()
