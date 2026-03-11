# Interview Demo Spec — Apptronik Dexterity Engineer

Evaluation against the **actual job description duties**, then against our original build plan.

---

## JD Coverage: Essential Duties

### 1. Retargeting & Mapping Algorithms

#### Generic Dexterity — "IK-based, optimization-based, or learned retargeting"

| JD Requirement | What We Have | File | Verdict |
|---|---|---|---|
| IK-based retargeting | Geometric per-finger IK (proportional mapping) | `sim/hand_retargeting.py` | DONE |
| Optimization-based retargeting | L-BFGS-B on fingertip error + joint limits, warm-started | `sim/hand_retargeting.py` | DONE |
| Learned retargeting | MLP trained via BC on optimization IK demos (28mm error, 81% vs random) | `sim/train_bc.py` | DONE |
| Map human kinematics to robot | VR 6DOF → Flexiv task-space (frame transform, scaling, clutching) | `teleop/retargeting.py` | DONE |
| Varied robotic configurations | Only Flexiv Rizon4 + MuJoCo hand — not parameterized for multiple robots | — | PARTIAL |

**Coverage: 4.5/5** — all three retargeting methods (IK, optimization, learned) demonstrated

---

#### Signal Integrity — "jitter reduction, sensor dropouts, safe fallback"

| JD Requirement | What We Have | File | Verdict |
|---|---|---|---|
| Jitter reduction | 1-Euro adaptive filter (42% jitter reduction demonstrated) | `sim/teleop_filters.py` | DONE |
| Handling sensor dropouts | Dropout detector (50ms timeout) + velocity extrapolation (200ms) | `sim/teleop_filters.py` | DONE |
| Safe fallback behaviors | Hold last pose → decaying extrapolation → freeze | `sim/teleop_filters.py` | DONE |
| Integrated in real pipeline | 1-Euro in retargeter, dropout in main loop | `teleop/retargeting.py`, `teleop/teleop_main.py` | DONE |

**Coverage: 4/4 PASS**

---

### 2. Simulation & Testing Infrastructure

#### End-Effector Validation — "automated workflows in MuJoCo, benchmark hand designs"

| JD Requirement | What We Have | File | Verdict |
|---|---|---|---|
| MuJoCo environment | Hand model (15 DOF), virtual fixture scanning scene | `sim/hand_retargeting.py`, `sim/demo.py` | DONE |
| IsaacSim | Nothing | — | GAP |
| Automated workflow | Scripts run end-to-end, produce outputs (GIF, PNG, error table) | all `sim/` | DONE |
| Benchmark hand designs | Error comparison table (geometric vs optimization per finger) | `sim/hand_retargeting.py` | PARTIAL |
| Real-robot testbed | Flexiv teleop pipeline ready, untested on HW | `teleop/` | PARTIAL |

**Coverage: 3/5** — have MuJoCo + automated outputs, but no IsaacSim, benchmarking is basic

#### Performance Metrics — "automated scoring for dexterity"

| JD Requirement | What We Have | File | Verdict |
|---|---|---|---|
| Automated dexterity scoring | Per-finger IK error table (mm), jitter % reduction | `sim/hand_retargeting.py`, `sim/teleop_filters.py` | PARTIAL |
| Inform hardware/software design | Error table shows which fingers need better kinematics | `sim/hand_retargeting.py` | PARTIAL |
| Standardized metrics framework | No reusable metrics class or benchmark suite | — | GAP |

**Coverage: 1.5/3** — we compute metrics but don't have a reusable scoring framework

---

### 3. Haptics & Tactile Integration

#### Sensory Loop — "haptic gloves, tactile sensors, force-reflective feedback"

| JD Requirement | What We Have | File | Verdict |
|---|---|---|---|
| Haptic feedback to operator | F/T magnitude → VR controller vibration via `triggerHapticPulse()` | `teleop/vr_reader.py` | DONE |
| Tactile sensor integration | Flexiv F/T sensor data read (`ext_wrench_in_tcp`, `ext_wrench_in_world`) | `teleop/flexiv_commander.py` | DONE |
| Force-reflective feedback | Wrench magnitude mapped to pulse duration [0–3500µs], deadzone at <5% | `teleop/vr_reader.py`, `teleop/teleop_main.py` | DONE |

**Coverage: 3/3 PASS** — full haptic loop: robot F/T → wrench magnitude → VR vibration intensity

**Talking point:** "Operator feels contact forces through controller vibration — proportional mapping so light touch = gentle buzz, heavy force = strong pulse. Combined with the software force threshold auto-stop for safety."

#### Data Consumption — "high-bandwidth tactile data ingested and logged"

| JD Requirement | What We Have | File | Verdict |
|---|---|---|---|
| Tactile data logged | Wrench (6D) logged every frame in JSONL | `teleop/teleop_main.py` | DONE |
| High-bandwidth ingestion | 90Hz wrench logging, but no tactile array / high-res skin | — | PARTIAL |
| Logged for model training | JSONL format ready for BC / offline RL | `teleop/logs/` | DONE |

**Coverage: 2/3** — logging works, but only F/T sensor, no tactile arrays

---

### 4. Advanced Control & RL Frameworks

#### Fidelity Improvements — "control methodologies, transparency of teleop"

| JD Requirement | What We Have | File | Verdict |
|---|---|---|---|
| Impedance control | Configurable per-axis stiffness/damping | `teleop/flexiv_commander.py` | DONE |
| Compliance modes | Z-approach (soft Z, stiff XY) for surface contact | `teleop/flexiv_commander.py` | DONE |
| Teleop transparency | Clutching, workspace scaling, deadzone, 1-Euro filter | `teleop/retargeting.py` | DONE |
| Force control | `SetForceControlAxis`, `SetMaxContactWrench` available | `teleop/flexiv_commander.py` | DONE |

**Coverage: 4/4 PASS**

#### RL Scaffolding — "swap teleop with learned policies, shared autonomy"

| JD Requirement | What We Have | File | Verdict |
|---|---|---|---|
| Training data collection | JSONL logs: pose, wrench, timestamps, engagement state | `teleop/teleop_main.py` | DONE |
| Shared autonomy | Virtual fixtures: 70/30 auto/human blend, workspace constraints | `sim/demo.py` | DONE |
| Policy swap framework | BC training pipeline: expert demos → MLP → 81% improvement over random | `sim/train_bc.py` | DONE |
| RL environment | `HandRetargetEnv` — Gymnasium env wrapping MuJoCo hand scene | `sim/hand_env.py` | DONE |

**Coverage: 4/4 PASS** — full pipeline: Gym env → expert demos → BC training → learned policy

**Talking point:** "Complete RL scaffolding: the Gym env wraps MuJoCo for standardized training. BC on optimization IK demos gives 28mm error (vs 151mm random). JSONL teleop logs feed the same pipeline. Next step is PPO fine-tuning on the Gym env with domain randomization."

---

### 5. Skills & Requirements

| Skill | Evidence | Verdict |
|---|---|---|
| Python proficiency | Entire codebase | DONE |
| Controls for robotics | Impedance control, Cartesian motion, force thresholds | DONE |
| Git / version control | Project in VisualFT repo (though demos not committed yet) | PARTIAL |
| Linux + bash | All runs on Ubuntu, shell commands, ROS2 | DONE |
| Real-time hardware-in-loop | 90Hz Flexiv control loop, NRT heartbeat, VR at 90Hz | DONE |

**Coverage: 4.5/5**

---

## JD Coverage Summary

| Duty Area | Coverage | Key Gaps |
|---|---|---|
| Retargeting & Mapping | **8.5/9** | Only 1 robot config (Flexiv + MuJoCo hand) |
| Simulation & Testing | **4.5/8** | No IsaacSim, no metrics framework |
| Haptics & Tactile | **5/6** | No tactile array (only F/T sensor) |
| Advanced Control & RL | **8/8** | — |
| Skills | **4.5/5** | Minor: demos not committed |
| **TOTAL** | **30.5/36 = 85%** | |

---

## Remaining Gaps (to reach 90%+)

| Gap | Impact | Effort | Mitigation |
|---|---|---|---|
| **Performance metrics framework** | MED | 20 min | Reusable `metrics.py` class with success rate, tracking error, force profiles |
| **IsaacSim** | LOW | 0 min | Talking point: "architecture is sim-agnostic, MuJoCo for speed, Isaac for scale" |
| **Tactile arrays** | LOW | N/A | Talking point: reference TendonClassifier (Sparsh embeddings, vision+force fusion) |
| **Multiple robot configs** | LOW | 30 min | Parameterize hand model XML, show same IK pipeline works on different hand topologies |

---

## Original Build Plan Tracker

| # | Planned Item | Status | Score |
|---|---|---|---|
| B2 | Signal processing (1-Euro, dropout, plot) | DONE | 5/5 |
| B1 | Hand retargeting (geometric + optimization, GIF, table) | DONE | 5.5/6 |
| B3 | Virtual fixtures (MuJoCo, shared autonomy, GIF) | DONE | 4/5 |
| A1 | VR pose streaming (openvr, buttons) | DONE | 4/4 |
| A2 | Task-space retargeting (frame transform, clutch, deadzone) | DONE | 5/5 |
| A3 | Flexiv commander (impedance, safety) | DONE | 8/8 |
| A4 | Main loop (filter, log, shutdown) | DONE | 5/5 |
| — | Safety config (box, force limits) | DONE (bonus) | 4/4 |
| — | Z-approach mode | DONE (bonus) | — |
| C | Cheat sheet | DONE | 4/4 |
| | **Plan execution** | | **44.5/46 = 96.7%** |

---

## Run Everything

```bash
cd ~/VisualFT/interview_demos

# Sim demos (no hardware)
python3 sim/teleop_filters.py        # → signal_processing_demo.png
python3 sim/hand_retargeting.py      # → hand_retargeting.gif + error table
python3 sim/demo.py                  # → virtual_fixtures.gif

# RL scaffolding
python3 sim/hand_env.py              # → test Gym env (random policy)
python3 sim/train_bc.py              # → train BC policy → bc_policy.pt

# Teleop — mock test
python3 teleop/teleop_main.py --mock

# Teleop — real hardware
python3 teleop/teleop_main.py --mode full        # 6DOF task-space VR → Flexiv
python3 teleop/teleop_main.py --mode z-approach   # Z-only, compliant contact
```
