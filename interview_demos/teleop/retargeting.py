"""
Task-Space Retargeting — A2
============================
VR controller frame → robot base frame:
  - Rotation + translation mapping
  - Workspace scaling
  - Orientation remapping
  - Clutching (decouple/re-engage with button)
  - Deadzone near center for jitter reduction
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass, field
from typing import Optional

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
from teleop_filters import OneEuroFilter


@dataclass
class RetargetConfig:
    """Configuration for VR → robot retargeting."""
    # Workspace scaling
    vr_workspace: float = 0.5         # VR controller range (m, each axis)
    robot_workspace: np.ndarray = field(
        default_factory=lambda: np.array([0.3, 0.3, 0.2])  # robot XYZ range (m)
    )

    # Robot home position (center of workspace)
    robot_home: np.ndarray = field(
        default_factory=lambda: np.array([0.5, 0.0, 0.4])  # meters
    )

    # VR → robot frame rotation (SteamVR Y-up → robot Z-up)
    # SteamVR: X-right, Y-up, Z-backward
    # Robot:   X-forward, Y-left, Z-up
    vr_to_robot_rot: np.ndarray = field(
        default_factory=lambda: np.array([
            [0, 0, -1],   # robot X ← -VR Z (forward)
            [-1, 0, 0],   # robot Y ← -VR X (left)
            [0, 1, 0],    # robot Z ← VR Y  (up)
        ], dtype=float)
    )

    # Deadzone (meters) — VR positions within this radius of origin are zeroed
    deadzone_radius: float = 0.005

    # Orientation scaling (reduce wrist rotation magnitude)
    orientation_scale: float = 0.7

    # Workspace safety limits (robot frame, meters)
    # Defaults from safety_config.py — override via constructor if needed
    pos_min: np.ndarray = field(default=None)
    pos_max: np.ndarray = field(default=None)

    def __post_init__(self):
        from safety_config import SAFE_BOX_MIN, SAFE_BOX_MAX
        if self.pos_min is None:
            self.pos_min = SAFE_BOX_MIN.copy()
        if self.pos_max is None:
            self.pos_max = SAFE_BOX_MAX.copy()


class TaskSpaceRetargeter:
    """Maps VR controller pose to robot end-effector pose."""

    def __init__(self, config: Optional[RetargetConfig] = None):
        self.cfg = config or RetargetConfig()

        # Clutch state
        self._clutch_engaged = False
        self._vr_offset: Optional[np.ndarray] = None   # VR origin when clutch engaged
        self._robot_anchor: Optional[np.ndarray] = None  # robot pos when clutch engaged
        self._robot_anchor_quat: Optional[np.ndarray] = None
        self._rot_offset: Optional[R] = None

        # Post-engage hold: freeze for N frames after engage to settle anchor
        self._hold_frames = 0
        self._hold_duration = 10  # ~170ms at 60Hz

        # Filter
        self._pos_filter = OneEuroFilter(freq=90.0, min_cutoff=2.0, beta=0.05)
        self._rot_filter = OneEuroFilter(freq=90.0, min_cutoff=1.5, beta=0.03)

        # Scaling per axis
        self._scale = self.cfg.robot_workspace / self.cfg.vr_workspace

    def update_clutch(self, grip_pressed: bool, vr_pos: np.ndarray, vr_rot: np.ndarray,
                      current_robot_pos: np.ndarray, current_robot_quat: Optional[np.ndarray] = None):
        """Handle clutching — grip pressed = engaged, released = decoupled."""
        if grip_pressed and not self._clutch_engaged:
            # Engage: record offsets, start hold period
            self._clutch_engaged = True
            self._vr_offset = vr_pos.copy()
            self._robot_anchor = current_robot_pos.copy()
            self._robot_anchor_quat = current_robot_quat.copy() if current_robot_quat is not None else None
            self._rot_offset = R.from_matrix(vr_rot)
            self._hold_frames = self._hold_duration
            # Reset filters so they don't carry stale state
            self._pos_filter = OneEuroFilter(freq=90.0, min_cutoff=2.0, beta=0.05)
            self._rot_filter = OneEuroFilter(freq=90.0, min_cutoff=1.5, beta=0.03)

        elif not grip_pressed:
            self._clutch_engaged = False

    def retarget(self, vr_pos: np.ndarray, vr_rot: np.ndarray,
                 t: Optional[float] = None) -> tuple[np.ndarray, np.ndarray]:
        """
        Map VR controller pose → robot EE pose.

        Returns:
            (position [3], quaternion [4] as [x,y,z,w] scipy convention)
        """
        if not self._clutch_engaged or self._vr_offset is None:
            return self.cfg.robot_home.copy(), R.identity().as_quat()

        # Post-engage hold: keep robot at anchor while VR settles
        if self._hold_frames > 0:
            self._hold_frames -= 1
            # Update VR offset to latest position (settle the anchor)
            self._vr_offset = vr_pos.copy()
            self._rot_offset = R.from_matrix(vr_rot)
            anchor_quat = self._robot_anchor_quat if self._robot_anchor_quat is not None else R.identity().as_quat()
            return self._robot_anchor.copy(), anchor_quat

        # ─── Position ───
        # Delta in VR frame
        delta_vr = vr_pos - self._vr_offset

        # Deadzone
        dist = np.linalg.norm(delta_vr)
        if dist < self.cfg.deadzone_radius:
            delta_vr = np.zeros(3)
        else:
            # Smooth deadzone (subtract deadzone radius in direction of motion)
            delta_vr = delta_vr * (1 - self.cfg.deadzone_radius / dist)

        # Transform to robot frame
        delta_robot = self.cfg.vr_to_robot_rot @ delta_vr

        # Scale
        delta_robot *= self._scale

        # Add to anchor
        target_pos = self._robot_anchor + delta_robot

        # Safety clamp
        target_pos = np.clip(target_pos, self.cfg.pos_min, self.cfg.pos_max)

        # Filter position
        target_pos = self._pos_filter(target_pos, t)

        # ─── Orientation ───
        vr_rot_R = R.from_matrix(vr_rot)
        delta_rot = vr_rot_R * self._rot_offset.inv()

        # Scale rotation (reduce magnitude)
        rotvec = delta_rot.as_rotvec()
        rotvec *= self.cfg.orientation_scale

        # Filter orientation (as rotvec)
        rotvec = self._rot_filter(rotvec, t)

        # Apply delta to robot anchor orientation (not just raw delta)
        scaled_delta = R.from_rotvec(rotvec)
        if self._robot_anchor_quat is not None:
            anchor_rot = R.from_quat(self._robot_anchor_quat)
            target_quat = (anchor_rot * scaled_delta).as_quat()  # [x,y,z,w]
        else:
            target_quat = scaled_delta.as_quat()

        return target_pos, target_quat

    @property
    def is_engaged(self) -> bool:
        return self._clutch_engaged


if __name__ == '__main__':
    # Unit test: simulated VR motion
    import time

    retarget = TaskSpaceRetargeter()

    # Simulate grip press + motion
    vr_pos = np.array([0.0, 1.0, -0.5])  # initial VR pos
    vr_rot = np.eye(3)
    robot_pos = np.array([0.5, 0.0, 0.4])

    retarget.update_clutch(True, vr_pos, vr_rot, robot_pos)

    print("Simulating VR → robot retargeting:")
    for i in range(20):
        t = i * 0.011  # ~90Hz
        # Move VR controller 10cm in VR-Z (= robot -X)
        vr_pos_new = vr_pos + np.array([0, 0, -0.1 * (i / 20)])
        pos, quat = retarget.retarget(vr_pos_new, vr_rot, t)
        if i % 5 == 0:
            print(f"  t={t:.3f}: VR delta Z={-0.1*(i/20):.3f} → robot pos={pos}")

    print("Done. Clutch release:")
    retarget.update_clutch(False, vr_pos_new, vr_rot, pos)
    pos, quat = retarget.retarget(vr_pos_new, vr_rot)
    print(f"  Released → pos={pos} (returns to home)")
