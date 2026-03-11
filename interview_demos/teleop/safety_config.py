"""
Safety Configuration — Single source of truth for workspace limits.
=====================================================================
Edit the BOX below before running teleop. All values in METRES.

The box is enforced at two layers:
  1. Retargeting (soft clamp — target is clamped before sending)
  2. Commander  (hard clamp — double-check before RDK call + force threshold)
"""

import numpy as np

# ┌──────────────────────────────────────────────────────────┐
# │              EDIT THESE BEFORE RUNNING                   │
# └──────────────────────────────────────────────────────────┘

# Cartesian safety box — robot TCP must stay inside this box.
# Coordinates in robot base frame (metres).
SAFE_BOX_MIN = np.array([0.35, -0.20, 0.08])   # [x_min, y_min, z_min]
SAFE_BOX_MAX = np.array([0.65,  0.20, 0.50])   # [x_max, y_max, z_max]

# Max Cartesian velocity (m/s) — commands exceeding this get scaled down
MAX_LINEAR_VEL = 0.10    # 10 cm/s — conservative for first test
MAX_ANGULAR_VEL = 0.3    # rad/s

# Max acceleration
MAX_LINEAR_ACC = 0.5     # m/s²
MAX_ANGULAR_ACC = 1.0    # rad/s²

# Force safety threshold (N) — auto-stop if external force exceeds this
FORCE_THRESHOLD = 25.0

# Impedance parameters
STIFFNESS = np.array([2000, 2000, 1500, 80, 80, 80], dtype=float)  # N/m, Nm/rad
DAMPING = np.array([0.7, 0.7, 0.7, 0.7, 0.7, 0.7])                # ratio [0.3–0.8]


# ─── Haptic feedback ───
HAPTIC_ENABLED = True
HAPTIC_MAX_FORCE = 25.0     # N — force that maps to max vibration (matches FORCE_THRESHOLD)

# ─── Z-Approach mode ───
Z_APPROACH_STIFFNESS = 500.0    # N/m — soft in Z for gentle contact
Z_APPROACH_DAMPING = 0.8        # high damping — no bounce
Z_APPROACH_XY_POS = np.array([0.512, 0.260])  # phantom center XY (metres)


def print_safety_summary(current_tcp=None):
    """Print safety config for visual confirmation before starting."""
    box_size = SAFE_BOX_MAX - SAFE_BOX_MIN
    center = (SAFE_BOX_MAX + SAFE_BOX_MIN) / 2

    print("\n╔══════════════════════════════════════════════╗")
    print("║         TELEOP SAFETY CONFIGURATION          ║")
    print("╠══════════════════════════════════════════════╣")
    print(f"║  Safe box min:  [{SAFE_BOX_MIN[0]:.3f}, {SAFE_BOX_MIN[1]:.3f}, {SAFE_BOX_MIN[2]:.3f}] m")
    print(f"║  Safe box max:  [{SAFE_BOX_MAX[0]:.3f}, {SAFE_BOX_MAX[1]:.3f}, {SAFE_BOX_MAX[2]:.3f}] m")
    print(f"║  Box size:      [{box_size[0]:.3f}, {box_size[1]:.3f}, {box_size[2]:.3f}] m")
    print(f"║  Box center:    [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}] m")
    print(f"║  Max lin vel:   {MAX_LINEAR_VEL:.3f} m/s")
    print(f"║  Max ang vel:   {MAX_ANGULAR_VEL:.3f} rad/s")
    print(f"║  Force limit:   {FORCE_THRESHOLD:.1f} N")
    if current_tcp is not None:
        inside = np.all(current_tcp >= SAFE_BOX_MIN) and np.all(current_tcp <= SAFE_BOX_MAX)
        status = "INSIDE" if inside else "!! OUTSIDE !!"
        print(f"║  Current TCP:   [{current_tcp[0]:.3f}, {current_tcp[1]:.3f}, {current_tcp[2]:.3f}] — {status}")
    print("╚══════════════════════════════════════════════╝\n")


if __name__ == '__main__':
    print_safety_summary()
