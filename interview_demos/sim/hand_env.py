"""
Hand Retargeting Gym Environment
=================================
Wraps the MuJoCo hand model as a gymnasium.Env for RL training.

Observation: target fingertip positions (15D) + current joint angles (15D) = 30D
Action: joint angle deltas (15D)
Reward: negative sum of fingertip position errors

This allows swapping manual teleop (optimization IK) with a learned policy.
"""

import os
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces

from hand_retargeting import HAND_XML, FINGER_JOINTS, FINGER_SITES, get_site_pos


class HandRetargetEnv(gym.Env):
    """Gymnasium env: retarget target fingertip positions to joint angles."""

    metadata = {'render_modes': ['rgb_array'], 'render_fps': 30}

    def __init__(self, render_mode=None):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_string(HAND_XML)
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode

        self.n_joints = self.model.nq  # 15
        self.n_fingers = len(FINGER_JOINTS)

        # Action: joint angle deltas (radians), clipped to [-0.1, 0.1] per step
        self.action_space = spaces.Box(
            low=-0.1, high=0.1, shape=(self.n_joints,), dtype=np.float32
        )

        # Observation: [target_fingertip_positions (15D), current_joint_angles (15D)]
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.n_joints + self.n_fingers * 3,), dtype=np.float32
        )

        # Joint limits
        self._jnt_min = self.model.jnt_range[:self.n_joints, 0].copy()
        self._jnt_max = self.model.jnt_range[:self.n_joints, 1].copy()

        # Episode state
        self._target_positions = {}
        self._step_count = 0
        self._max_steps = 100

        # Renderer
        self._renderer = None
        if render_mode == 'rgb_array':
            self._renderer = mujoco.Renderer(self.model, width=480, height=360)

    def _get_fingertip_positions(self):
        """Get current fingertip positions as flat array."""
        positions = []
        for finger in FINGER_JOINTS:
            pos = get_site_pos(self.model, self.data, FINGER_SITES[finger])
            positions.extend(pos)
        return np.array(positions, dtype=np.float32)

    def _get_obs(self):
        target_flat = []
        for finger in FINGER_JOINTS:
            target_flat.extend(self._target_positions[finger])
        target_flat = np.array(target_flat, dtype=np.float32)
        joints = self.data.qpos[:self.n_joints].astype(np.float32)
        return np.concatenate([target_flat, joints])

    def _compute_reward(self):
        """Negative total fingertip error (mm). Higher = better."""
        total_error = 0.0
        for finger in FINGER_JOINTS:
            pos = get_site_pos(self.model, self.data, FINGER_SITES[finger])
            target = self._target_positions[finger]
            total_error += np.linalg.norm(pos - target)
        return -total_error * 1000  # in mm, negative

    def _sample_target(self):
        """Random reachable target via FK at random joint angles."""
        import math
        # Random flex [0, 1]
        flex = np.random.uniform(0.2, 0.9)
        for i in range(self.n_joints):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if 'pip' in name or 'dip' in name:
                self.data.qpos[i] = math.radians(flex * 80)
            else:
                self.data.qpos[i] = math.radians(np.random.uniform(-10, 10))
        mujoco.mj_forward(self.model, self.data)

        targets = {}
        for finger in FINGER_JOINTS:
            pos = get_site_pos(self.model, self.data, FINGER_SITES[finger])
            # Add small noise so it's not trivially solvable
            targets[finger] = pos + np.random.normal(0, 0.001, 3)
        return targets

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # Sample a random target
        self._target_positions = self._sample_target()
        # Reset joints to zero
        self.data.qpos[:] = 0
        mujoco.mj_forward(self.model, self.data)
        self._step_count = 0
        return self._get_obs(), {}

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)

        # Apply action as joint angle delta
        self.data.qpos[:self.n_joints] += action
        # Clamp to joint limits
        self.data.qpos[:self.n_joints] = np.clip(
            self.data.qpos[:self.n_joints], self._jnt_min, self._jnt_max
        )
        mujoco.mj_forward(self.model, self.data)

        self._step_count += 1
        reward = self._compute_reward()
        terminated = self._step_count >= self._max_steps
        truncated = False

        return self._get_obs(), reward, terminated, truncated, {
            'error_mm': -reward,
            'step': self._step_count,
        }

    def render(self):
        if self._renderer is None:
            return None
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0.2, 0.15]
        cam.distance = 0.5
        cam.azimuth = 150
        cam.elevation = -25
        self._renderer.update_scene(self.data, camera=cam)
        return self._renderer.render()

    def close(self):
        if self._renderer:
            self._renderer.close()
            self._renderer = None


# Quick sanity check
if __name__ == '__main__':
    env = HandRetargetEnv()
    obs, _ = env.reset()
    print(f"Obs shape: {obs.shape}")
    print(f"Action shape: {env.action_space.shape}")

    total_reward = 0
    for i in range(50):
        action = env.action_space.sample()
        obs, reward, done, trunc, info = env.step(action)
        total_reward += reward
        if i % 10 == 0:
            print(f"  step {i}: error={info['error_mm']:.1f}mm, reward={reward:.1f}")

    print(f"Total reward (random policy, 50 steps): {total_reward:.1f}")
    env.close()
