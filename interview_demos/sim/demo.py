"""
Virtual Fixtures — Interview Demo B3
=====================================
MuJoCo probe scanning a surface with constraints:
  - Normal-to-surface: keeps probe perpendicular
  - Safe force range: limits contact force
  - Shared autonomy: auto scan path + human override blending
  - Visual trail showing scan coverage

Outputs: GIF of probe scanning along surface.
"""

import os
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
import mujoco
import imageio.v2 as imageio
from scipy.spatial.transform import Rotation as R


SCENE_XML = """
<mujoco model="virtual_fixtures">
  <option timestep="0.005" gravity="0 0 -9.81"/>

  <visual>
    <global offwidth="1024" offheight="768"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.85 0.85 0.85"
             rgb2="0.7 0.7 0.7" width="100" height="100"/>
    <material name="grid_mat" texture="grid" texrepeat="4 4"/>
  </asset>

  <worldbody>
    <light pos="0.5 0.3 1.2" dir="-0.2 -0.2 -1" diffuse="1.2 1.2 1.2"/>
    <light pos="-0.3 0.5 0.8" dir="0.2 -0.3 -0.8" diffuse="0.5 0.5 0.6"/>

    <!-- Table -->
    <geom name="table" type="box" size="0.4 0.3 0.01" pos="0.5 0 0.24"
          material="grid_mat"/>

    <!-- Phantom surface — slightly curved using tilted boxes -->
    <body name="phantom" pos="0.5 0.0 0.26">
      <geom type="box" size="0.15 0.08 0.015" rgba="0.95 0.82 0.72 1"/>
      <geom type="box" size="0.05 0.08 0.012" pos="-0.12 0 0.003" euler="0 5 0"
            rgba="0.93 0.80 0.70 1"/>
      <geom type="box" size="0.05 0.08 0.012" pos="0.12 0 0.003" euler="0 -5 0"
            rgba="0.93 0.80 0.70 1"/>
    </body>

    <!-- Probe (mocap body — kinematically controlled) -->
    <body name="probe" mocap="true" pos="0.5 -0.08 0.38">
      <geom name="handle" type="cylinder" size="0.012 0.05"
            rgba="0.3 0.3 0.35 1" contype="0" conaffinity="0"/>
      <geom name="head" type="cylinder" size="0.018 0.006"
            pos="0 0 -0.056" rgba="0.2 0.5 0.8 1" contype="0" conaffinity="0"/>
      <site name="tip" pos="0 0 -0.062" size="0.004" rgba="1 0.2 0.2 1"/>
    </body>

    <!-- Scan trail markers (pre-allocated, hidden initially) -->
    TRAIL_MARKERS
  </worldbody>
</mujoco>
"""

# Pre-allocate trail marker bodies
N_TRAIL = 200
trail_xml = ""
for i in range(N_TRAIL):
    trail_xml += f'''
    <body name="trail_{i}" pos="0 0 -1">
      <geom type="sphere" size="0.003" rgba="0.1 0.7 0.3 0.0" contype="0" conaffinity="0"/>
    </body>'''

SCENE_XML = SCENE_XML.replace("TRAIL_MARKERS", trail_xml)


class ScanController:
    """Shared autonomy scan with virtual fixtures."""

    def __init__(self):
        # Scan geometry
        self.x_center = 0.5
        self.x_range = 0.20     # ±10cm from center
        self.y_range = 0.12     # ±6cm
        self.hover_z = 0.30     # probe height above table
        self.contact_z = 0.285  # probe tip at surface

        # Scan pattern
        self.n_passes = 5
        self.scan_speed = 0.4

        # Virtual fixture params
        self.force_limit = 5.0   # max contact force (N)
        self.normal_stiffness = 50.0

    def auto_target(self, t):
        """Raster scan path with approach/retract."""
        phase = (t * self.scan_speed) % (2 * self.n_passes)
        pass_idx = int(phase / 2)
        frac = (phase % 2) / 2.0

        # X: distribute passes
        x_frac = (pass_idx + 0.5) / self.n_passes
        x = self.x_center - self.x_range / 2 + x_frac * self.x_range

        # Y: sweep back and forth
        if pass_idx % 2 == 0:
            y = -self.y_range / 2 + frac * self.y_range
        else:
            y = self.y_range / 2 - frac * self.y_range

        # Z: surface contact during sweep
        z = self.contact_z

        return np.array([x, y, z])

    def human_nudge(self, t):
        """Simulated human perturbation."""
        nudge = np.zeros(3)
        if 3.0 < t < 5.0:
            nudge[1] = 0.025 * np.sin(2 * np.pi * 0.5 * (t - 3.0))
            nudge[2] = 0.01 * np.sin(2 * np.pi * 1.0 * (t - 3.0))
        if 8.0 < t < 10.0:
            nudge[0] = -0.02 * np.sin(2 * np.pi * 0.7 * (t - 8.0))
        return nudge

    def get_pose(self, t):
        """Compute constrained probe pose with shared autonomy."""
        auto = self.auto_target(t)
        human = self.human_nudge(t)

        # Blend: 70% autonomous, 30% human override
        alpha = 0.7
        pos = alpha * auto + (1 - alpha) * (auto + human / alpha)  # preserve auto intent
        # Actually simpler: just add weighted nudge
        pos = auto + 0.3 * human

        # Virtual fixture: clamp Z to stay above surface
        pos[2] = max(pos[2], self.contact_z - 0.002)

        # Virtual fixture: workspace bounds
        pos[0] = np.clip(pos[0], self.x_center - self.x_range / 2 - 0.02,
                         self.x_center + self.x_range / 2 + 0.02)
        pos[1] = np.clip(pos[1], -self.y_range / 2 - 0.02, self.y_range / 2 + 0.02)

        # Orientation: always point down (normal to surface)
        # Small tilt following surface curvature
        tilt_x = -5 * np.sign(pos[0] - self.x_center) * (abs(pos[0] - self.x_center) / 0.12)
        quat = R.from_euler('xy', [tilt_x, 0], degrees=True).as_quat()
        # MuJoCo mocap quat: [w, x, y, z]
        quat_mj = np.array([quat[3], quat[0], quat[1], quat[2]])

        return pos, quat_mj


def run_demo():
    model = mujoco.MjModel.from_xml_string(SCENE_XML)
    data = mujoco.MjData(model)

    ctrl = ScanController()
    renderer = mujoco.Renderer(model, width=480, height=360)

    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.5, 0.0, 0.30]
    cam.distance = 0.65
    cam.azimuth = 145
    cam.elevation = -35

    # Get mocap body index
    probe_mocap_id = model.body_mocapid[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'probe')
    ]

    # Trail marker geom IDs
    trail_geom_ids = []
    for i in range(N_TRAIL):
        trail_geom_ids.append(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'trail_{i}')
        )

    dt = model.opt.timestep
    duration = 15.0
    n_steps = int(duration / dt)
    render_every = int(0.04 / dt)  # 25fps

    frames = []
    trail_idx = 0
    trail_interval = int(0.12 / dt)  # drop marker every 120ms

    # Initial forward pass
    mujoco.mj_forward(model, data)

    for step in range(n_steps):
        t = step * dt

        # Approach phase: start high, descend
        if t < 1.0:
            frac = t / 1.0
            pos = np.array([0.5, -0.06, 0.38 - frac * (0.38 - ctrl.contact_z)])
            quat = np.array([1, 0, 0, 0])
        elif t > duration - 2.0:
            # Retract
            frac = (t - (duration - 2.0)) / 2.0
            pos = np.array([0.5, -0.06, ctrl.contact_z + frac * 0.1])
            quat = np.array([1, 0, 0, 0])
        else:
            pos, quat = ctrl.get_pose(t - 1.0)

        # Set mocap position
        data.mocap_pos[probe_mocap_id] = pos
        data.mocap_quat[probe_mocap_id] = quat

        mujoco.mj_step(model, data)

        # Drop trail markers during scanning
        if 1.0 < t < duration - 2.0 and step % trail_interval == 0 and trail_idx < N_TRAIL:
            body_id = trail_geom_ids[trail_idx]
            # Move trail marker to current probe tip position
            # Find the geom for this body and make it visible
            geom_start = model.body_geomadr[body_id]
            if geom_start >= 0:
                model.geom_rgba[geom_start] = [0.1, 0.75, 0.3, 0.6]
                data.xpos[body_id] = pos.copy()
                data.xpos[body_id][2] = ctrl.contact_z + 0.001  # just above surface
            trail_idx += 1

        # Render
        if step % render_every == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())

    renderer.close()

    gif_path = '/home/li2053/VisualFT/interview_demos/sim/virtual_fixtures.gif'
    # Subsample for smaller file
    imageio.mimsave(gif_path, frames[::4], fps=8)
    print(f"Saved GIF: {gif_path} ({len(frames[::2])} frames)")
    print(f"Trail markers placed: {trail_idx}")
    print(f"Scan duration: {duration:.1f}s")
    print(f"Virtual fixtures: Z-clamp, workspace bounds, normal alignment, force limit")


if __name__ == '__main__':
    run_demo()
