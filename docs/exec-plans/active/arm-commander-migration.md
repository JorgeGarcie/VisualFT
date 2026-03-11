# ExecPlan: ArmCommander Migration

**Status**: In Progress
**Created**: 2026-03-10
**Author**: li2053
**Motivated by**: Floating mode (admittance control) is the next feature. Building it
against the current architecture would create a 4th separate RDK connection path.
This is the turning point to consolidate.

## Goal

Build `ArmCommander` as the single robot interface, use it for floating mode first,
then migrate existing code incrementally with tests as the safety net.

## Context

See `docs/exec-plans/active/target-architecture.md` for the full architecture reference
(layer diagram, abstractions, file structure, testing strategy).

This plan is the **concrete implementation steps** — what to build, in what order.

## Architecture Summary

```
Script (scan.py / teleop.py / floating.py)
  │
  │ behavior logic
  ▼
ArmCommander
  ├── SafetyChecker (per-command, synchronous)
  ├── SafetyWatchdog (own thread, continuous)
  └── FlexivRDK (single connection, lock-protected)
```

Key design decisions:
- ArmCommander owns the single RDK connection
- threading.Lock protects all RDK access (watchdog + main thread)
- SafetyChecker validates every command before sending
- SafetyWatchdog runs independently, catches faults/force/heartbeat
- Pure Python class, NOT a ROS2 node (testable without ROS2)
- Scripts are behavior logic only — no RDK, no quaternion reorder

## Step 1: Common types and config

Create shared dataclasses that ArmCommander and all scripts use.

- [ ] `common/types.py`
  - `Pose` (position xyz + quaternion xyzw, ROS2 convention)
  - `RobotState` (pose, wrench, joint_positions, timestamp)
  - `Wrench` (fx, fy, fz, tx, ty, tz)
  - Conversion methods: `Pose.to_rdk()` → `[x,y,z,qw,qx,qy,qz]`
  - Conversion methods: `Pose.from_rdk(list)` → `Pose`

- [ ] `common/config.py`
  - `RobotConfig` (serial_number, control_rate_hz, velocity/acc limits)
  - `SafetyConfig` (workspace bounds, max forces, max velocity, heartbeat timeout)
  - Load from YAML: `config/robot.yaml`

- [ ] `config/robot.yaml`
  ```yaml
  robot:
    serial_number: Rizon4-062174
    control_rate_hz: 50.0
    max_linear_vel: 0.05
    max_angular_vel: 0.5
  safety:
    workspace:  # metres
      x: [0.2, 0.8]
      y: [0.0, 0.5]
      z: [0.0, 0.6]
    max_force_z: 50.0
    max_velocity: 0.15
    heartbeat_timeout: 2.0
  ```

## Step 2: ArmCommander core

- [ ] `flexiv_driver/arm_commander.py`
  ```python
  class ArmCommander:
      def __init__(self, config: RobotConfig, safety_config: SafetyConfig)

      # Motion
      def move_to(self, pose: Pose, velocity: float)     # blocking MoveL
      def stream_cartesian(self, pose: Pose)              # NRT target
      def set_impedance(self, stiffness, damping)
      def zero_ft(self)                                   # blocking
      def home(self)                                       # blocking

      # State
      def get_state(self) -> RobotState
      def fault(self) -> bool
      def healthy(self) -> bool

      # Lifecycle
      def stop(self)
      def shutdown(self)
  ```

- [ ] Thread safety: `threading.Lock` around ALL `self._robot` calls
  - Watchdog thread reads `states()` at 50Hz
  - Main thread calls `SendCartesianMotionForce` at control rate
  - Lock prevents concurrent RDK access

- [ ] Mode switching internal to ArmCommander
  - `move_to()` switches to NRT_PRIMITIVE internally, switches back after
  - `stream_cartesian()` ensures NRT_CARTESIAN_MOTION_FORCE
  - Scripts never call SwitchMode

## Step 3: Safety (two mechanisms)

- [ ] `flexiv_driver/safety.py`

  **SafetyChecker** (synchronous, per-command):
  - `check_workspace(pose)` → raises `SafetyViolation`
  - `check_velocity(new_pose, old_pose, dt)` → raises `SafetyViolation`
  - Called inside `stream_cartesian()` and `move_to()`

  **SafetyWatchdog** (own thread, continuous):
  - Monitors force limits (reads `get_state().wrench`)
  - Monitors robot fault
  - Monitors heartbeat (time since last command)
  - On violation: calls `robot.Stop()`, sets `_stopped` flag
  - Uses same `threading.Lock` for RDK access

- [ ] `common/errors.py`
  - `VisionFTError` (base)
  - `RobotFault`
  - `SafetyViolation`
  - `ConnectionLost`
  - `SensorError`

## Step 4: MockArmCommander + tests

- [ ] `tests/mock_arm.py`
  - Records all calls
  - Configurable state returns
  - Can simulate faults, force spikes

- [ ] `tests/unit/test_safety.py`
  - Workspace violation → stop + raise
  - Force limit → watchdog stops robot
  - Heartbeat timeout → watchdog stops robot
  - Velocity limit → raise

- [ ] `tests/unit/test_arm_commander.py`
  - Mode switching: move_to then stream_cartesian
  - State reading
  - Shutdown cleanup

## Step 5: Floating mode (the actual feature)

- [ ] `visionft/floating_mode.py`
  - Uses ArmCommander (first real consumer)
  - `set_impedance(K_x≈0, Z_x=configurable)` for backdriveable robot
  - Service `/rdk/set_mode` to switch hold ↔ float
  - Reads F/T for feedback display

This is the validation — if floating mode works cleanly against ArmCommander,
the abstraction is correct.

## Step 6: Migrate scan_node (when ready)

- [ ] Write unit tests for scan state machine using MockArmCommander
- [ ] Create `scan_controller.py` using ArmCommander instead of raw RDK
- [ ] Verify tests pass on mock, then on real robot
- [ ] Retire old `scan_node.py`

## Step 7: Migrate teleop (when ready)

- [ ] Move from `interview_demos/teleop/` to `visionft/teleop/`
- [ ] Replace `flexiv_commander.py` with ArmCommander
- [ ] Keep VR reader, retargeting unchanged

## Progress

| Date | Update |
|------|--------|
| 2026-03-10 | Architecture designed, plan written |

## Decision Log

| Decision | Rationale | Date |
|----------|-----------|------|
| ArmCommander is pure Python, not ROS2 node | Testable without ROS2, thin ROS2 wrapper added separately | 2026-03-10 |
| threading.Lock on all RDK access | Watchdog + main thread share connection, RDK thread safety unknown | 2026-03-10 |
| Safety built into ArmCommander, not separate wrapper | Simpler API, can't bypass, watchdog needs same robot reference | 2026-03-10 |
| Build floating mode first against ArmCommander | Real feature validates the abstraction, not a dry refactor | 2026-03-10 |
| Scripts are behavior logic only | No RDK imports, no quaternion handling, just Pose in/out | 2026-03-10 |
| Common types with conversion methods | Pose.to_rdk() / Pose.from_rdk() — conversion in one place | 2026-03-10 |

## Surprises

(none yet)

## Outcomes

**Result**: (filled when floating mode works on ArmCommander)
**Follow-ups**: scan migration, teleop migration, legacy cleanup
