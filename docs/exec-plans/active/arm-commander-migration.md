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
- Force/fault watchdog is intentionally redundant with lower-level protections
- Pure Python class, NOT a ROS2 node (testable without ROS2)
- ArmCommander is decoupled from CoinFT and other non-robot sensors
- Sensor readiness is handled by the application layer before starting behavior
- Scripts are behavior logic only — no RDK, no quaternion reorder

## Step 1: Common types and config

Create shared dataclasses that ArmCommander and all scripts use.

- [x] `common/types.py`
  - `Pose` (position xyz + quaternion xyzw, ROS2 convention)
  - `RobotState` (pose, wrench, joint_positions, timestamp)
  - `Wrench` (fx, fy, fz, tx, ty, tz)
  - Conversion methods: `Pose.to_rdk()` → `[x,y,z,qw,qx,qy,qz]`
  - Conversion methods: `Pose.from_rdk(list)` → `Pose`

- [x] `common/config.py`
  - `RobotConfig` (serial_number, control_rate_hz, velocity/acc limits)
  - `SafetyConfig` (workspace bounds, log thresholds, SDK contact-wrench limit, heartbeat timeout)
  - Load from YAML: `config/robot.yaml`

- [x] `config/robot.yaml`
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
    max_contact_wrench: [50.0, 50.0, 50.0, 10.0, 10.0, 10.0]
    heartbeat_timeout: 2.0
  ```

## Step 2: ArmCommander core

- [x] `flexiv_driver/arm_commander.py`
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

- [x] Thread safety: `threading.Lock` around ALL `self._robot` calls
  - Watchdog thread reads `states()` at 50Hz
  - Main thread calls `SendCartesianMotionForce` at control rate
  - Lock prevents concurrent RDK access

- [x] Mode switching internal to ArmCommander
  - `move_to()` switches to NRT_PRIMITIVE internally, switches back after
  - `stream_cartesian()` ensures NRT_CARTESIAN_MOTION_FORCE
  - Scripts never call SwitchMode

## Step 3: Safety (two mechanisms)

- [ ] `flexiv_driver/safety.py`

  **SafetyChecker** (synchronous, per-command):
  - `check_workspace(pose)` → raises `SafetyViolation`
  - Called inside `stream_cartesian()` and `move_to()`
  - Stops the robot before raising if a command violates configured limits
  - Does not estimate velocity from consecutive setpoints

  **SafetyWatchdog** (own thread, continuous):
  - Monitors force limits (reads `get_state().wrench`) for logging/observability
  - Monitors robot fault for logging/observability
  - Monitors heartbeat for streaming Cartesian control only
  - Heartbeat timeout calls `robot.Stop()` and sets `_stopped`
  - Uses same `threading.Lock` for RDK access
  - Force/fault checks remain active while connected, even during blocking primitives

  **Heartbeat scope**:
  - `stream_cartesian()` arms and refreshes the heartbeat
  - `home()`, `move_to()`, `zero_ft()`, `contact()`, `float()`, and tool setup pause heartbeat monitoring
  - Reason: long blocking primitives are legitimate robot work and must not false-trigger a timeout

  **Motion limit source of truth**:
  - Velocity and acceleration limits belong to `RobotConfig`, not `SafetyConfig`
  - `stream_cartesian()` passes those limits directly to `SendCartesianMotionForce()`
  - `stream_cartesian()` also programs `SetMaxContactWrench()` from config when entering Cartesian streaming mode
  - No differential `dt`-based software velocity check inside ArmCommander

  **Force threshold scope**:
  - `max_force_z` in SafetyConfig is a watchdog logging threshold, not a stop trigger
  - The *response* to force depends on application context (scan backs off, teleop vibrates, float ignores)
  - Application-level behaviors should read `get_state().wrench` and react accordingly
  - The real safety floor is firmware `SetMaxContactWrench` — that stops unconditionally
  - `max_force_z` may move to per-behavior config in the future

  **Sensor boundary**:
  - CoinFT readiness and freshness are NOT handled inside ArmCommander
  - Before behavior that depends on CoinFT, the application waits for sensor readiness
  - After start, sensor freshness/health is tracked by the higher-level sensor/application layer
  - If a required sensor goes stale, the higher-level layer pauses the robot/behavior and logs the fault

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

- [x] `tests/unit/test_safety.py`
  - Workspace violation → stop + raise
  - Force limit → watchdog logs observation
  - Robot fault → watchdog logs observation
  - Heartbeat timeout during streaming → watchdog stops robot
  - No heartbeat timeout during blocking primitive

- [x] `tests/unit/test_arm_commander.py`
  - Mode switching: move_to then stream_cartesian
  - State reading
  - Shutdown cleanup
  - Cartesian streaming applies configured SDK contact-wrench limits

  **Execution note**:
  - These minimal unit tests were intentionally pulled forward during Step 3
    because watchdog and safety-boundary changes are high-risk and cheap to
    validate in pure Python
  - `tests/mock_arm.py` and broader state-machine tests still remain as the
    later migration test harness

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
| 2026-03-11 | Step 1 implemented; ArmCommander core implemented; safety boundary refined: heartbeat applies to streaming control only, CoinFT readiness stays above ArmCommander |
| 2026-03-11 | Minimal `test_safety.py` and `test_arm_commander.py` pulled forward during Step 3 to validate watchdog semantics and motion-limit wiring before further migration |
| 2026-03-11 | Added configured SDK contact-wrench regulation for Cartesian streaming; clarified that stale required sensors should pause behavior at the higher layer |

## Decision Log

| Decision | Rationale | Date |
|----------|-----------|------|
| ArmCommander is pure Python, not ROS2 node | Testable without ROS2, thin ROS2 wrapper added separately | 2026-03-10 |
| threading.Lock on all RDK access | Watchdog + main thread share connection, RDK thread safety unknown | 2026-03-10 |
| Safety built into ArmCommander, not separate wrapper | Simpler API, can't bypass, watchdog needs same robot reference | 2026-03-10 |
| Build floating mode first against ArmCommander | Real feature validates the abstraction, not a dry refactor | 2026-03-10 |
| Scripts are behavior logic only | No RDK imports, no quaternion handling, just Pose in/out | 2026-03-10 |
| Common types with conversion methods | Pose.to_rdk() / Pose.from_rdk() — conversion in one place | 2026-03-10 |
| Heartbeat timeout applies only to streaming Cartesian control | Blocking primitives like Home and MoveL are expected to run for seconds and must not false-stop | 2026-03-11 |
| No software `dt`-based velocity checker in ArmCommander | Consecutive-setpoint velocity estimates are noisy for differential streaming; robot-side limits already exist in the RDK call | 2026-03-11 |
| Force/fault watchdog is intentionally redundant | Independent monitoring provides logging and observability on top of Flexiv's built-in stop behavior | 2026-03-11 |
| Program SDK contact-wrench limits in Cartesian streaming mode | Flexiv regulation should be configured proactively rather than inferred from later fault/force observations | 2026-03-11 |
| CoinFT readiness stays outside ArmCommander | ArmCommander is the robot driver boundary; sensor gating belongs to the application/sensor layer | 2026-03-11 |
| Stale required sensors pause behavior above ArmCommander | Sensor freshness is an application concern; the higher layer should log and pause when required data disappears | 2026-03-11 |
| `common/` and `flexiv_driver/` stay pure Python for now | ROS packaging is unnecessary until these modules need to be colcon-installed or wrapped by ROS nodes | 2026-03-11 |
| Pull minimal unit tests forward when changing safety-critical code | The plan order stays directional, but watchdog behavior should be validated immediately when implemented | 2026-03-11 |

## Surprises

(none yet)

## Outcomes

**Result**: (filled when floating mode works on ArmCommander)
**Follow-ups**: scan migration, teleop migration, legacy cleanup
