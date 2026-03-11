# Apptronik Dexterity Engineer — Interview Cheat Sheet

## Narrative

"I built a VR teleoperation system with task-space retargeting, impedance control,
and signal processing — the same architecture scales to dexterous hand control for Apollo."

## Evidence Mapping

| Requirement | Evidence |
|---|---|
| **Retargeting** | A2 (task-space, real HW, clutching, workspace scaling) + B1 (joint-space IK, 80% improvement with optimization) |
| **Signal integrity** | B2 (1-Euro filter 42% jitter reduction, dropout detection + velocity extrapolation fallback) |
| **Sim & testing** | B1/B3 in MuJoCo (hand retargeting + virtual fixture scanning) |
| **Haptics & tactile** | TendonClassifier (vision+force fusion, Sparsh embeddings, 75.6% OOD accuracy) |
| **RL scaffolding** | A4 logs training data (JSONL: poses, forces, timestamps); TWIST teacher-student discussion |
| **Shared autonomy** | B3 virtual fixtures (auto scan + 70/30 human blend, workspace clamp, normal constraint) |
| **Impedance control** | A3 (configurable per-axis stiffness/damping, Flexiv Cartesian impedance mode) |

## TWIST Talking Points

1. **Two-stage retargeting**: optimization offline (L-BFGS-B, accurate), geometric IK online (fast)
2. **PPO + behavior cloning**: teacher sees future (privileged info), student deployed with partial obs
3. **Domain randomization**: mass ±3kg, friction 0.1-2.0, actuator delay ±5ms for sim-to-real
4. **Teleop data**: our JSONL logs are exactly the format needed for BC pretraining before RL

## Technical Details to Drop

### 1-Euro Filter
- Adaptive: `cutoff = min_cutoff + beta * |velocity|`
- Low velocity → heavy smoothing (kills jitter)
- High velocity → light smoothing (preserves intent)
- O(1) per sample, no lookahead — real-time compatible

### Clutching
- Grip press = record VR↔robot offset, start tracking deltas
- Grip release = freeze robot, user can reposition hand
- Re-engage = new offset from current positions
- Prevents workspace limits from halting operation

### Impedance Control
- Stiffness K_x = [2000, 2000, 1500, 80, 80, 80] (N/m, Nm/rad)
- Damping ratio = 0.7 (critically damped range)
- K_x → 0 = free-floating (for manual guidance / teaching)
- Force threshold = 30N auto-stop (safety)

### Virtual Fixtures
- Normal constraint: torque = K × (probe_z × desired_z) — keeps probe perpendicular
- Force clamp: cap downward force to prevent tissue damage
- Workspace bounds: hard limits prevent leaving scan area
- Shared autonomy blend: α=0.7 auto + 0.3 human override

## Questions to Ask Them

1. What end-effector is Apollo using? COTS or custom fingers?
2. Current teleop hardware? (VR? Exoskeleton? Haptic gloves?)
3. Timeline for teleop → learned autonomous policies?
4. Sim-to-real transfer strategy for contact-rich manipulation?
5. What's the fingertip sensing modality? (Force/torque? Tactile arrays? BioTac?)
6. How many DOF per hand, and coupling between joints?

## File Quick Reference

```
interview_demos/
├── sim/
│   ├── teleop_filters.py        # B2: 1-Euro filter + dropout handling
│   ├── signal_processing_demo.png  # B2 output
│   ├── hand_retargeting.py      # B1: geometric + optimization IK
│   ├── hand_retargeting.gif     # B1 output
│   ├── demo.py                  # B3: virtual fixtures scanning
│   └── virtual_fixtures.gif     # B3 output
├── teleop/
│   ├── vr_reader.py             # A1: SteamVR pose streaming
│   ├── retargeting.py           # A2: VR→robot retargeting
│   ├── flexiv_commander.py      # A3: Flexiv impedance commands
│   ├── teleop_main.py           # A4: main loop + logging
│   └── logs/                    # JSONL trajectory logs
└── cheatsheet.md                # This file
```

## Run Commands

```bash
# Sim demos (no hardware needed)
python sim/teleop_filters.py          # generates signal_processing_demo.png
python sim/hand_retargeting.py        # generates hand_retargeting.gif
python sim/demo.py                    # generates virtual_fixtures.gif

# Real teleop (needs VR + Flexiv)
python teleop/teleop_main.py          # full hardware
python teleop/teleop_main.py --mock   # test without hardware
```
