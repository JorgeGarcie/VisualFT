"""
Teleop Signal Processing — Interview Demo B2
=============================================
1-Euro filter (adaptive jitter), dropout detection + safe fallback,
velocity extrapolation during gaps.

Run standalone to generate comparison plot PNG.
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import Optional
import time


# ─── 1-Euro Filter ─────────────────────────────────────────────────────────────
class OneEuroFilter:
    """Adaptive low-pass filter for noisy real-time signals.

    At low velocity → heavy smoothing (kills jitter).
    At high velocity → light smoothing (preserves responsiveness).
    """

    def __init__(self, freq: float = 90.0, min_cutoff: float = 1.0,
                 beta: float = 0.007, d_cutoff: float = 1.0):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._x_prev: Optional[np.ndarray] = None
        self._dx_prev: Optional[np.ndarray] = None
        self._t_prev: Optional[float] = None

    def _alpha(self, cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x: np.ndarray, t: Optional[float] = None) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if t is None:
            t = time.monotonic()

        if self._x_prev is None:
            self._x_prev = x.copy()
            self._dx_prev = np.zeros_like(x)
            self._t_prev = t
            return x.copy()

        dt = max(t - self._t_prev, 1e-6)
        self._t_prev = t

        # Derivative estimate (low-pass filtered)
        dx = (x - self._x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1 - a_d) * self._dx_prev
        self._dx_prev = dx_hat

        # Adaptive cutoff
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)

        # Per-element filtering with per-element cutoff
        a = np.array([self._alpha(c, dt) for c in cutoff])
        x_hat = a * x + (1 - a) * self._x_prev
        self._x_prev = x_hat
        return x_hat


# ─── Dropout Detector + Fallback ───────────────────────────────────────────────
@dataclass
class DropoutHandler:
    """Detects VR tracking dropouts and provides safe fallback.

    - If no new reading for > timeout_ms → dropout detected
    - During dropout: hold last pose, then decay velocity to zero
    - Extrapolates briefly using last known velocity
    """

    timeout_ms: float = 50.0          # dropout threshold
    max_extrapolate_ms: float = 200.0  # max extrapolation before hold
    decay_rate: float = 5.0            # velocity decay (1/s)

    _last_valid_pose: Optional[np.ndarray] = field(default=None, init=False)
    _last_velocity: Optional[np.ndarray] = field(default=None, init=False)
    _last_valid_t: Optional[float] = field(default=None, init=False)
    _prev_pose: Optional[np.ndarray] = field(default=None, init=False)
    _prev_t: Optional[float] = field(default=None, init=False)
    _in_dropout: bool = field(default=False, init=False)

    def update(self, pose: Optional[np.ndarray], t: float) -> tuple[np.ndarray, bool]:
        """Returns (output_pose, is_dropout)."""
        if pose is not None:
            # Valid reading
            if self._prev_pose is not None and self._prev_t is not None:
                dt = max(t - self._prev_t, 1e-6)
                self._last_velocity = (pose - self._prev_pose) / dt
            self._prev_pose = pose.copy()
            self._prev_t = t
            self._last_valid_pose = pose.copy()
            self._last_valid_t = t
            self._in_dropout = False
            return pose.copy(), False

        # No reading — check if dropout
        if self._last_valid_pose is None:
            return np.zeros(3), True

        dt_since_valid = (t - self._last_valid_t) * 1000  # ms
        self._in_dropout = dt_since_valid > self.timeout_ms

        if not self._in_dropout:
            return self._last_valid_pose.copy(), False

        # Dropout — extrapolate with decaying velocity
        if self._last_velocity is not None and dt_since_valid < self.max_extrapolate_ms:
            dt_s = (t - self._last_valid_t)
            decay = np.exp(-self.decay_rate * dt_s)
            extrapolated = self._last_valid_pose + self._last_velocity * dt_s * decay
            return extrapolated, True

        # Beyond extrapolation window — hold last pose
        return self._last_valid_pose.copy(), True


# ─── Demo: Generate comparison plot ────────────────────────────────────────────
def generate_demo_signal(duration: float = 3.0, rate: float = 90.0):
    """Synthetic VR controller trajectory with noise and dropouts."""
    n = int(duration * rate)
    t = np.linspace(0, duration, n)

    # Smooth ground truth: figure-8 in XY, gentle Z wave
    gt = np.column_stack([
        0.15 * np.sin(2 * np.pi * 0.5 * t),
        0.10 * np.sin(2 * np.pi * 1.0 * t),
        0.05 * np.sin(2 * np.pi * 0.3 * t),
    ])

    # Add realistic VR tracking noise (2mm std)
    noisy = gt + np.random.normal(0, 0.002, gt.shape)

    # Inject dropouts: two regions
    dropout_mask = np.ones(n, dtype=bool)  # True = valid
    drop1 = (t > 0.8) & (t < 0.95)   # 150ms dropout
    drop2 = (t > 1.9) & (t < 2.15)   # 250ms dropout
    dropout_mask[drop1 | drop2] = False

    return t, gt, noisy, dropout_mask


def run_demo():
    t, gt, noisy, valid_mask = generate_demo_signal()

    euro = OneEuroFilter(freq=90.0, min_cutoff=1.0, beta=0.1)
    dropout = DropoutHandler(timeout_ms=50, max_extrapolate_ms=200)

    filtered = np.zeros_like(noisy)
    handled = np.zeros_like(noisy)
    dropout_flags = np.zeros(len(t), dtype=bool)

    for i in range(len(t)):
        # Filter the noisy signal
        filtered[i] = euro(noisy[i], t[i])

        # Dropout handling
        pose_in = noisy[i] if valid_mask[i] else None
        handled[i], dropout_flags[i] = dropout.update(
            filtered[i] if valid_mask[i] else None, t[i]
        )

    # ─── Plot ───
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    labels = ['X (m)', 'Y (m)', 'Z (m)']

    for ax_i, (ax, label) in enumerate(zip(axes, labels)):
        ax.plot(t, gt[:, ax_i], 'k-', lw=1.5, alpha=0.4, label='Ground Truth')
        ax.plot(t, noisy[:, ax_i], '.', color='#aaa', ms=1, label='Raw VR (noisy)')
        ax.plot(t, filtered[:, ax_i], '-', color='#2196F3', lw=1.2, label='1-Euro Filtered')
        ax.plot(t, handled[:, ax_i], '-', color='#4CAF50', lw=1.5, label='+ Dropout Handling')

        # Shade dropout regions
        for start_idx in np.where(np.diff(dropout_flags.astype(int)) == 1)[0]:
            end_idx = start_idx + np.argmax(~dropout_flags[start_idx + 1:]) + 1
            if end_idx <= start_idx:
                end_idx = len(t) - 1
            ax.axvspan(t[start_idx], t[min(end_idx, len(t) - 1)],
                       alpha=0.15, color='red', label='Dropout' if ax_i == 0 and start_idx < 100 else '')

        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)

    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].set_title('VR Teleop Signal Processing — 1-Euro Filter + Dropout Handling')
    axes[-1].set_xlabel('Time (s)')

    plt.tight_layout()
    out_path = '/home/li2053/VisualFT/interview_demos/sim/signal_processing_demo.png'
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Saved: {out_path}")

    # Stats — measure jitter as std of frame-to-frame velocity difference
    raw_jitter = np.std(np.diff(noisy, axis=0), axis=0).mean() * 1000
    filt_jitter = np.std(np.diff(filtered, axis=0), axis=0).mean() * 1000
    print(f"Raw  jitter (vel std): {raw_jitter:.3f} mm/frame")
    print(f"Filt jitter (vel std): {filt_jitter:.3f} mm/frame")
    print(f"Jitter reduction: {(1 - filt_jitter/raw_jitter)*100:.1f}%")
    print(f"Dropout regions handled: {np.sum(dropout_flags)} samples")


if __name__ == '__main__':
    run_demo()
