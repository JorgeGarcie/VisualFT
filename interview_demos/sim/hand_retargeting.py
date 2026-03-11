"""
Hand Retargeting — Interview Demo B1
=====================================
Maps synthetic human hand keypoints → MuJoCo 5-finger hand (16 DOF).

Two methods compared:
  1. Geometric IK: per-finger chain, closed-form (fast, ~0.1ms)
  2. Optimization IK: scipy L-BFGS-B on fingertip error + joint limits (accurate, ~5ms)

Outputs: error comparison table + rendered GIF of the hand tracking keypoints.
"""

import os
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
import mujoco
import matplotlib.pyplot as plt
from pathlib import Path
import imageio.v2 as imageio


# ─── Hand Model ────────────────────────────────────────────────────────────────

HAND_XML = """
<mujoco model="retarget_hand">
  <option timestep="0.002" gravity="0 0 0"/>

  <default>
    <joint damping="0.05" armature="0.001"/>
    <geom type="capsule" size="0.008" rgba="0.9 0.75 0.65 1" contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <light pos="0.3 0.3 0.5" dir="-0.3 -0.3 -0.5" diffuse="1 1 1"/>
    <light pos="-0.3 0.3 0.5" dir="0.3 -0.3 -0.5" diffuse="0.5 0.5 0.5"/>

    <!-- Palm base -->
    <body name="palm" pos="0 0 0.15">
      <geom type="box" size="0.04 0.05 0.01" rgba="0.9 0.75 0.65 1"/>

      <!-- Thumb (2 DOF: spread + flex) -->
      <body name="thumb_base" pos="-0.035 -0.03 0.005" euler="0 0 -60">
        <joint name="thumb_spread" axis="0 0 1" range="-30 30"/>
        <geom fromto="0 0 0 0.03 0 0"/>
        <body name="thumb_mid" pos="0.03 0 0">
          <joint name="thumb_pip" axis="0 1 0" range="0 90"/>
          <geom fromto="0 0 0 0.025 0 0"/>
          <body name="thumb_tip" pos="0.025 0 0">
            <joint name="thumb_dip" axis="0 1 0" range="0 90"/>
            <geom fromto="0 0 0 0.02 0 0"/>
            <site name="thumb_tip_site" pos="0.02 0 0" size="0.005" rgba="1 0 0 1"/>
          </body>
        </body>
      </body>

      <!-- Index finger (3 DOF: spread + 2 flex) -->
      <body name="index_base" pos="-0.02 0.05 0.005">
        <joint name="index_spread" axis="0 0 1" range="-15 15"/>
        <geom fromto="0 0 0 0 0.035 0"/>
        <body name="index_mid" pos="0 0.035 0">
          <joint name="index_pip" axis="1 0 0" range="0 100"/>
          <geom fromto="0 0 0 0 0.03 0"/>
          <body name="index_tip" pos="0 0.03 0">
            <joint name="index_dip" axis="1 0 0" range="0 80"/>
            <geom fromto="0 0 0 0 0.02 0"/>
            <site name="index_tip_site" pos="0 0.02 0" size="0.005" rgba="1 0 0 1"/>
          </body>
        </body>
      </body>

      <!-- Middle finger -->
      <body name="middle_base" pos="0 0.055 0.005">
        <joint name="middle_spread" axis="0 0 1" range="-15 15"/>
        <geom fromto="0 0 0 0 0.04 0"/>
        <body name="middle_mid" pos="0 0.04 0">
          <joint name="middle_pip" axis="1 0 0" range="0 100"/>
          <geom fromto="0 0 0 0 0.03 0"/>
          <body name="middle_tip" pos="0 0.03 0">
            <joint name="middle_dip" axis="1 0 0" range="0 80"/>
            <geom fromto="0 0 0 0 0.022 0"/>
            <site name="middle_tip_site" pos="0 0.022 0" size="0.005" rgba="1 0 0 1"/>
          </body>
        </body>
      </body>

      <!-- Ring finger -->
      <body name="ring_base" pos="0.02 0.05 0.005">
        <joint name="ring_spread" axis="0 0 1" range="-15 15"/>
        <geom fromto="0 0 0 0 0.035 0"/>
        <body name="ring_mid" pos="0 0.035 0">
          <joint name="ring_pip" axis="1 0 0" range="0 100"/>
          <geom fromto="0 0 0 0 0.028 0"/>
          <body name="ring_tip" pos="0 0.028 0">
            <joint name="ring_dip" axis="1 0 0" range="0 80"/>
            <geom fromto="0 0 0 0 0.018 0"/>
            <site name="ring_tip_site" pos="0 0.018 0" size="0.005" rgba="1 0 0 1"/>
          </body>
        </body>
      </body>

      <!-- Pinky -->
      <body name="pinky_base" pos="0.035 0.04 0.005">
        <joint name="pinky_spread" axis="0 0 1" range="-15 15"/>
        <geom fromto="0 0 0 0 0.028 0"/>
        <body name="pinky_mid" pos="0 0.028 0">
          <joint name="pinky_pip" axis="1 0 0" range="0 100"/>
          <geom fromto="0 0 0 0 0.022 0"/>
          <body name="pinky_tip" pos="0 0.022 0">
            <joint name="pinky_dip" axis="1 0 0" range="0 80"/>
            <geom fromto="0 0 0 0 0.015 0"/>
            <site name="pinky_tip_site" pos="0 0.015 0" size="0.005" rgba="1 0 0 1"/>
          </body>
        </body>
      </body>

    </body>
  </worldbody>

  <actuator>
    <position name="a_thumb_spread" joint="thumb_spread" kp="1"/>
    <position name="a_thumb_pip"    joint="thumb_pip"    kp="1"/>
    <position name="a_thumb_dip"    joint="thumb_dip"    kp="1"/>
    <position name="a_index_spread" joint="index_spread" kp="1"/>
    <position name="a_index_pip"    joint="index_pip"    kp="1"/>
    <position name="a_index_dip"    joint="index_dip"    kp="1"/>
    <position name="a_middle_spread" joint="middle_spread" kp="1"/>
    <position name="a_middle_pip"   joint="middle_pip"   kp="1"/>
    <position name="a_middle_dip"   joint="middle_dip"   kp="1"/>
    <position name="a_ring_spread"  joint="ring_spread"  kp="1"/>
    <position name="a_ring_pip"     joint="ring_pip"     kp="1"/>
    <position name="a_ring_dip"     joint="ring_dip"     kp="1"/>
    <position name="a_pinky_spread" joint="pinky_spread" kp="1"/>
    <position name="a_pinky_pip"    joint="pinky_pip"    kp="1"/>
    <position name="a_pinky_dip"    joint="pinky_dip"    kp="1"/>
  </actuator>
</mujoco>
"""

# Joint indices per finger: [spread, pip, dip]
FINGER_JOINTS = {
    'thumb':  [0, 1, 2],
    'index':  [3, 4, 5],
    'middle': [6, 7, 8],
    'ring':   [9, 10, 11],
    'pinky':  [12, 13, 14],
}

FINGER_SITES = {
    'thumb':  'thumb_tip_site',
    'index':  'index_tip_site',
    'middle': 'middle_tip_site',
    'ring':   'ring_tip_site',
    'pinky':  'pinky_tip_site',
}


def get_site_pos(model, data, site_name):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    return data.site_xpos[site_id].copy()


# ─── Synthetic Human Keypoints ─────────────────────────────────────────────────
def generate_keypoint_trajectory(model, data, n_frames=120):
    """Generate a grasping motion using actual FK open/closed positions."""
    import math
    t_param = np.linspace(0, 2 * np.pi, n_frames)
    flex = 0.5 * (1 - np.cos(t_param))  # 0=open, 1=closed

    # Get open-hand fingertip positions
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    open_pos = {}
    for f in FINGER_SITES:
        open_pos[f] = get_site_pos(model, data, FINGER_SITES[f])

    # Get closed-hand fingertip positions
    for i in range(model.nq):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if 'pip' in name or 'dip' in name:
            data.qpos[i] = math.radians(75)
        else:
            data.qpos[i] = 0
    mujoco.mj_forward(model, data)
    closed_pos = {}
    for f in FINGER_SITES:
        closed_pos[f] = get_site_pos(model, data, FINGER_SITES[f])

    # Interpolate + add small noise to make it a challenge
    targets = {}
    for f in FINGER_SITES:
        traj = np.zeros((n_frames, 3))
        for i in range(n_frames):
            traj[i] = open_pos[f] + flex[i] * (closed_pos[f] - open_pos[f])
            traj[i] += np.random.normal(0, 0.001, 3)  # 1mm noise
        targets[f] = traj

    data.qpos[:] = 0  # reset
    return targets


# ─── Method 1: Geometric IK (fast, per-finger) ────────────────────────────────
def geometric_ik(model, data, target_positions):
    """Simple proportional IK — maps target distance to joint angles."""
    q = np.zeros(model.nq)

    for finger, joint_ids in FINGER_JOINTS.items():
        target = target_positions[finger]
        site_name = FINGER_SITES[finger]

        # Get reference positions (fully open)
        data.qpos[:] = 0
        mujoco.mj_forward(model, data)
        open_pos = get_site_pos(model, data, site_name)

        # Heuristic: flex proportional to how close target is to palm
        palm_pos = np.array([0, 0.15, 0])
        open_dist = np.linalg.norm(open_pos - palm_pos)
        target_dist = np.linalg.norm(target - palm_pos)
        flex_ratio = np.clip(1.0 - target_dist / max(open_dist, 1e-6), 0, 1)

        # Map to joint angles (degrees → radians)
        q[joint_ids[0]] = 0  # spread stays 0 for simplicity
        q[joint_ids[1]] = flex_ratio * np.radians(90)
        q[joint_ids[2]] = flex_ratio * np.radians(70)

    return q


# ─── Method 2: Optimization IK (accurate) ─────────────────────────────────────
def optimization_ik(model, data, target_positions, q_init=None):
    """Scipy L-BFGS-B minimizing fingertip position error."""
    if q_init is None:
        q_init = np.zeros(model.nq)

    jnt_range = model.jnt_range[:model.nq].copy()
    bounds = [(jnt_range[i, 0], jnt_range[i, 1]) for i in range(model.nq)]

    def cost(q):
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        err = 0.0
        for finger in FINGER_JOINTS:
            site_pos = get_site_pos(model, data, FINGER_SITES[finger])
            target = target_positions[finger]
            err += np.sum((site_pos - target) ** 2)
        # Small regularization toward center of range
        mid = 0.5 * (jnt_range[:, 0] + jnt_range[:, 1])
        err += 1e-4 * np.sum((q - mid) ** 2)
        return err

    result = minimize(cost, q_init, method='L-BFGS-B', bounds=bounds,
                      options={'maxiter': 50, 'ftol': 1e-10})
    return result.x


# ─── Run comparison + render ───────────────────────────────────────────────────
def run_demo():
    model = mujoco.MjModel.from_xml_string(HAND_XML)
    data = mujoco.MjData(model)

    targets = generate_keypoint_trajectory(model, data, n_frames=120)
    n_frames = 120

    geo_errors = {f: [] for f in FINGER_JOINTS}
    opt_errors = {f: [] for f in FINGER_JOINTS}

    # Renderer for GIF
    renderer = mujoco.Renderer(model, width=640, height=480)
    frames = []

    q_opt_prev = np.zeros(model.nq)

    for frame in range(n_frames):
        target_pos = {f: targets[f][frame] for f in FINGER_JOINTS}

        # Geometric IK
        q_geo = geometric_ik(model, data, target_pos)
        data.qpos[:] = q_geo
        mujoco.mj_forward(model, data)
        for f in FINGER_JOINTS:
            pos = get_site_pos(model, data, FINGER_SITES[f])
            geo_errors[f].append(np.linalg.norm(pos - target_pos[f]) * 1000)

        # Optimization IK (warm-started from previous solution)
        q_opt = optimization_ik(model, data, target_pos, q_init=q_opt_prev)
        q_opt_prev = q_opt.copy()
        data.qpos[:] = q_opt
        mujoco.mj_forward(model, data)
        for f in FINGER_JOINTS:
            pos = get_site_pos(model, data, FINGER_SITES[f])
            opt_errors[f].append(np.linalg.norm(pos - target_pos[f]) * 1000)

        # Render optimization result
        data.ctrl[:] = q_opt[:model.nu]
        mujoco.mj_forward(model, data)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0.2, 0.15]
        cam.distance = 0.5
        cam.azimuth = 150
        cam.elevation = -25
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render().copy())

    renderer.close()

    # Save GIF
    gif_path = '/home/li2053/VisualFT/interview_demos/sim/hand_retargeting.gif'
    imageio.mimsave(gif_path, frames[::2], fps=20)  # Skip every other frame for file size
    print(f"Saved GIF: {gif_path}")

    # Error comparison table
    print("\n┌─────────────┬────────────────────┬────────────────────┐")
    print("│   Finger    │ Geometric IK (mm)  │ Optimization (mm)  │")
    print("├─────────────┼────────────────────┼────────────────────┤")
    for f in FINGER_JOINTS:
        geo_mean = np.mean(geo_errors[f])
        opt_mean = np.mean(opt_errors[f])
        print(f"│ {f:11s} │ {geo_mean:16.2f}   │ {opt_mean:16.2f}   │")
    print("├─────────────┼────────────────────┼────────────────────┤")
    all_geo = np.mean([np.mean(v) for v in geo_errors.values()])
    all_opt = np.mean([np.mean(v) for v in opt_errors.values()])
    print(f"│ {'AVERAGE':11s} │ {all_geo:16.2f}   │ {all_opt:16.2f}   │")
    print("└─────────────┴────────────────────┴────────────────────┘")
    print(f"\nImprovement: {(1 - all_opt/all_geo)*100:.1f}%")


if __name__ == '__main__':
    run_demo()
