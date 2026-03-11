"""
Behavior Cloning — Train a learned retargeting policy
=====================================================
1. Generate expert demonstrations using optimization IK (from hand_retargeting.py)
2. Train MLP policy via supervised learning (behavior cloning)
3. Evaluate: compare expert IK vs learned policy vs random

This demonstrates the RL scaffolding: teleop data → learned policy → swap in.
"""

import os
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import mujoco
import time

from hand_retargeting import (HAND_XML, FINGER_JOINTS, FINGER_SITES,
                              get_site_pos, optimization_ik)
from hand_env import HandRetargetEnv


# ─── Policy Network ────────────────────────────────────────────────────────────

class RetargetPolicy(nn.Module):
    """MLP that maps (target_positions, current_joints) → joint deltas."""

    def __init__(self, obs_dim=30, act_dim=15, hidden=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, act_dim),
            nn.Tanh(),  # output in [-1, 1], scaled to action range
        )
        self.action_scale = 0.1  # match env action space

    def forward(self, obs):
        return self.net(obs) * self.action_scale


# ─── Generate Expert Demonstrations ───────────────────────────────────────────

def generate_demonstrations(n_episodes=200, steps_per_episode=50):
    """Use optimization IK to generate expert (observation, action) pairs."""
    print(f"Generating {n_episodes} expert demonstrations...")

    model = mujoco.MjModel.from_xml_string(HAND_XML)
    data = mujoco.MjData(model)
    env = HandRetargetEnv()

    all_obs = []
    all_actions = []

    for ep in range(n_episodes):
        obs, _ = env.reset()
        targets = env._target_positions

        # Expert: compute optimal joint angles via optimization IK
        q_expert = optimization_ik(model, data, targets, q_init=np.zeros(model.nq))

        # Generate trajectory: step from zero toward expert solution
        current_q = np.zeros(model.nq)
        for step in range(steps_per_episode):
            # Expert action: move toward optimal in small steps
            delta = q_expert - current_q
            # Clip to action range
            action = np.clip(delta * 0.3, -0.1, 0.1).astype(np.float32)

            all_obs.append(obs.copy())
            all_actions.append(action.copy())

            obs, _, done, _, _ = env.step(action)
            current_q = env.data.qpos[:model.nq].copy()

            if done:
                break

    env.close()

    obs_array = np.array(all_obs, dtype=np.float32)
    act_array = np.array(all_actions, dtype=np.float32)
    print(f"Generated {len(obs_array)} expert samples")
    return obs_array, act_array


# ─── Train ─────────────────────────────────────────────────────────────────────

def train_policy(obs_data, act_data, epochs=30, batch_size=256, lr=3e-4):
    """Train MLP via behavior cloning (supervised learning on expert data)."""
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Training on {device}...")

    dataset = TensorDataset(
        torch.from_numpy(obs_data),
        torch.from_numpy(act_data),
    )
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    policy = RetargetPolicy(obs_dim=obs_data.shape[1], act_dim=act_data.shape[1]).to(device)
    optimizer = torch.optim.Adam(policy.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    for epoch in range(epochs):
        total_loss = 0
        n_batches = 0
        for obs_batch, act_batch in loader:
            obs_batch = obs_batch.to(device)
            act_batch = act_batch.to(device)

            pred = policy(obs_batch)
            loss = loss_fn(pred, act_batch)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()
            n_batches += 1

        if (epoch + 1) % 5 == 0 or epoch == 0:
            print(f"  Epoch {epoch+1}/{epochs}: loss={total_loss/n_batches:.6f}")

    return policy


# ─── Evaluate ──────────────────────────────────────────────────────────────────

def evaluate(policy, n_episodes=50, max_steps=100):
    """Compare learned policy vs random baseline."""
    device = next(policy.parameters()).device
    env = HandRetargetEnv()

    results = {'learned': [], 'random': []}

    for ep in range(n_episodes):
        # Learned policy
        obs, _ = env.reset()
        for _ in range(max_steps):
            with torch.no_grad():
                obs_t = torch.from_numpy(obs).unsqueeze(0).to(device)
                action = policy(obs_t).cpu().numpy()[0]
            obs, reward, done, _, info = env.step(action)
            if done:
                break
        results['learned'].append(info['error_mm'])

        # Random policy (same target)
        env._target_positions = env._target_positions  # keep same target
        env.data.qpos[:] = 0
        mujoco.mj_forward(env.model, env.data)
        env._step_count = 0
        obs = env._get_obs()
        for _ in range(max_steps):
            action = env.action_space.sample()
            obs, reward, done, _, info = env.step(action)
            if done:
                break
        results['random'].append(info['error_mm'])

    env.close()
    return results


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    t0 = time.time()

    # 1. Generate expert demonstrations
    obs_data, act_data = generate_demonstrations(n_episodes=200, steps_per_episode=50)

    # 2. Train policy
    policy = train_policy(obs_data, act_data, epochs=30)

    # 3. Evaluate
    results = evaluate(policy, n_episodes=50)

    learned_mean = np.mean(results['learned'])
    learned_std = np.std(results['learned'])
    random_mean = np.mean(results['random'])
    random_std = np.std(results['random'])

    print("\n┌──────────────────┬────────────────────┐")
    print("│     Method       │ Final Error (mm)   │")
    print("├──────────────────┼────────────────────┤")
    print(f"│ Random policy    │ {random_mean:8.1f} ± {random_std:5.1f}    │")
    print(f"│ Learned (BC)     │ {learned_mean:8.1f} ± {learned_std:5.1f}    │")
    print("└──────────────────┴────────────────────┘")
    print(f"\nImprovement over random: {(1 - learned_mean/random_mean)*100:.1f}%")
    print(f"Total time: {time.time() - t0:.1f}s")

    # Save policy
    save_path = '/home/li2053/VisualFT/interview_demos/sim/bc_policy.pt'
    torch.save(policy.state_dict(), save_path)
    print(f"Policy saved: {save_path}")


if __name__ == '__main__':
    main()
