# ExecPlan: Target Architecture

**Status**: Draft
**Created**: 2026-03-10
**Author**: li2053

## Goal

Define the layered architecture that new code should be written against.
Existing working code migrates incrementally — never a stop-and-rewrite.

## Context

The system grew organically. scan_node is 600 lines spanning all layers. Teleop
works but lives under interview_demos/. Three separate RDK connection paths exist.
The robot, sensors, and applications are not abstracted from each other.

The key observation: **teleop and scanning are separate applications that share
the same hardware.** They should compose from the same building blocks, not each
reinvent the robot connection.

## Target Layer Diagram

```
┌───────────────────────────────────────────────────────┐
│                    Applications                        │
│  scan_controller    teleop_controller    (future...)   │
│  "what sequence     "map VR input to                   │
│   of actions"        robot motion"                     │
├───────────────────────────────────────────────────────┤
│                    Abstractions                        │
│  ArmCommander          SensorManager                   │
│  - move_to(pose)       - get_wrench() → Wrench         │
│  - set_impedance(K,Z)  - get_image() → Image           │
│  - zero_ft()           - healthy() → bool              │
│  - get_state() → State - wait_for_ready(timeout)       │
│  - fault() → bool     - track freshness()              │
│  - home()             RecordingManager                 │
│  - stop()             - start_bag(name, topics)        │
│  - built-in safety    - stop_bag()                     │
│    workspace heartbeat (streaming only)                │
│    fault/force logging + SDK contact wrench            │
│                         CoinFT readiness/staleness gate│
├───────────────────────────────────────────────────────┤
│                    Drivers                             │
│  FlexivRDK binding    CoinFTDriver     CameraDriver   │
│  (single conn, owned  (serial 360Hz)   (gscam2/UDP)   │
│   by ArmCommander)    offset, ONNX     H264 decode    │
│                       calibration                      │
├───────────────────────────────────────────────────────┤
│                    Common                              │
│  config.py   errors.py      types.py                   │
│  robot SN    exceptions     RobotState, Wrench,        │
│  limits                     ScanConfig dataclasses      │
└───────────────────────────────────────────────────────┘
```

## The Abstractions

### ArmCommander

The key abstraction. Applications never touch RDK directly. They call methods
on ArmCommander, which handles mode switching, connection, and robot-side safety.

```python
class ArmCommander:
    """Single owner of the RDK connection."""

    def __init__(self, config: RobotConfig):
        self._robot = init_robot(config.serial_number)
        self._watchdog_thread = ...  # monitor robot fault/force logs + stream heartbeat

    # --- Motion ---
    def move_to(self, pose: Pose, velocity: float = 0.05):
        """Blocking MoveL to target pose. Handles mode switching internally."""

    def stream_cartesian(self, pose: Pose):
        """Non-blocking NRT cartesian target. Call at control rate."""

    def set_impedance(self, stiffness: list[float], damping: list[float]):
        """Set Cartesian impedance parameters."""

    def zero_ft(self):
        """Execute ZeroFTSensor primitive. Blocking."""

    def home(self):
        """Execute PLAN-Home or MoveL to home pose. Blocking."""

    # --- State ---
    def get_state(self) -> RobotState:
        """Current TCP pose, wrench, joint positions."""

    def fault(self) -> bool:
        """True if robot is in fault state."""

    def healthy(self) -> bool:
        """True if connected, operational, no fault."""

    def stop(self):
        """Emergency stop."""
```

Why this matters:
- scan_controller calls `robot.move_to(home)`, not `robot.SwitchMode(NRT_PRIMITIVE); robot.ExecutePrimitive("MoveL", ...)`
- teleop_controller calls `robot.stream_cartesian(pose)`, not `robot.SendCartesianMotionForce(target, [0]*6, ...)`
- Mode switching is internal — the application doesn't care which RDK mode it's in
- Robot safety is built in, not bolted on later
- Velocity and acceleration limits live with the robot motion command, not a separate differential checker
- Cartesian streaming should proactively configure SDK contact-wrench regulation
- CoinFT readiness is not part of ArmCommander and remains a separate concern

### SensorManager

```python
class SensorManager:
    """Aggregates sensor health and data access."""

    def __init__(self, robot: ArmCommander, coinft_topic: str, camera_topic: str):
        ...

    def get_wrench(self, source='robot') -> Wrench:
        """Latest wrench from robot or coinft."""

    def get_image(self) -> Optional[Image]:
        """Latest camera frame, or None if unavailable."""

    def healthy(self) -> dict[str, bool]:
        """Per-sensor health: {'robot': True, 'coinft': True, 'camera': False}"""

    def wait_for_ready(self, sensors: list[str], timeout: float = 10.0):
        """Block until listed sensors are publishing. Raise on timeout."""
```

`SensorManager.wait_for_ready()` is where application preflight lives. For CoinFT,
"ready" should mean the serial stream is open, samples are arriving, initialization
or bias-zeroing has completed, and the latest wrench is finite and recent. That
gate happens before starting behavior. Ongoing sensor freshness checks stay here
too; they are not folded into ArmCommander. If a required sensor goes stale after
startup, this higher layer should log the fault and pause robot behavior.

### What applications look like with these abstractions

**scan_controller.py** (~150 lines instead of 600):
```python
class ScanController:
    def __init__(self, robot: ArmCommander, sensors: SensorManager,
                 recorder: RecordingManager, config: ScanConfig):
        self.robot = robot
        self.sensors = sensors
        ...

    def run_scan(self):
        self.robot.home()
        self.robot.zero_ft()
        self.sensors.wait_for_ready(['robot', 'coinft', 'camera'])
        self.recorder.start_bag(self.config.name)

        # Descend until contact
        while self.sensors.get_wrench().fz < self.config.contact_force:
            self.robot.stream_cartesian(descend_target)

        # Scan passes
        self.robot.set_impedance(self.config.stiffness, self.config.damping)
        for pass_config in self.config.passes:
            self._run_pass(pass_config)

        self.robot.home()
        self.recorder.stop_bag()
```

**teleop_controller.py** (~100 lines):
```python
class TeleopController:
    def __init__(self, robot: ArmCommander, vr: VRReader,
                 retargeter: TaskSpaceRetargeter):
        self.robot = robot
        ...

    def run(self):
        self.robot.home()
        self.robot.zero_ft()
        self.robot.set_impedance(TELEOP_STIFFNESS, TELEOP_DAMPING)

        while running:
            vr_state = self.vr.read()
            if vr_state.engaged:
                target = self.retargeter.compute(vr_state, self.robot.get_state())
                self.robot.stream_cartesian(target)
```

Notice: neither application knows about `flexivrdk`, `SwitchMode`, quaternion
ordering, or message building. They express intent, not mechanism.

### Safety Inside ArmCommander

Robot safety is built into ArmCommander itself rather than added as a separate
wrapper. That keeps the robot boundary explicit: ArmCommander owns robot safety,
while SensorManager/application preflight owns CoinFT readiness and other
non-robot sensor health.

```python
class SafetyChecker:
    """Per-command safety checks before sending robot motion."""

    def __init__(self, config: SafetyConfig):
        self._config = config

    def check_workspace(self, pose: Pose):
        if not self._config.workspace.contains(pose.position):
            raise SafetyViolation(f"Pose {pose} outside workspace bounds")


class ArmCommander:
    def __init__(self, config: RobotConfig):
        self._checker = SafetyChecker(config.safety)
        self._streaming_active = False
        self._last_stream_time = 0.0
        self._watchdog = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog.start()

    def stream_cartesian(self, pose: Pose):
        self._checker.check_workspace(pose)
        self._streaming_active = True
        self._last_stream_time = time.monotonic()
        self._send_cartesian_to_rdk(
            pose,
            max_linear_vel=self._config.max_linear_vel,
            max_angular_vel=self._config.max_angular_vel,
            max_linear_acc=self._config.max_linear_acc,
            max_angular_acc=self._config.max_angular_acc,
        )

    def move_to(self, pose: Pose, velocity: float = 0.05):
        self._checker.check_workspace(pose)
        self._streaming_active = False
        self._execute_movel_primitive(pose, velocity)

    def _watchdog_loop(self):
        while True:
            time.sleep(0.1)  # 10 Hz check
            state = self.get_state()
            if abs(state.wrench.fz) > self._config.max_force_z:
                logging.warning("Force limit observed by watchdog")
            if self.fault():
                logging.error("Robot fault observed by watchdog")
            if self._streaming_active and (
                time.monotonic() - self._last_stream_time
                > self._config.heartbeat_timeout
            ):
                self.stop()
                logging.error("Streaming heartbeat timeout — robot stopped")
```

```python
@dataclass
class SafetyConfig:
    workspace: BoundingBox       # min/max xyz
    max_force_z: float = 50.0   # N
    max_contact_wrench: list[float] = [50, 50, 50, 10, 10, 10]
    heartbeat_timeout: float = 1.0  # seconds without command → stop
```

Key design: applications use ArmCommander directly for robot control.
Safety cannot be bypassed because it is part of ArmCommander itself. The watchdog
thread runs independently. Robot fault and robot force observations are logged
there for visibility, while streaming heartbeat timeout remains the active stop
path for application-side hangs.

Velocity and acceleration limits are part of the robot motion command itself.
ArmCommander passes `max_linear_vel`, `max_angular_vel`, `max_linear_acc`, and
`max_angular_acc` directly to `SendCartesianMotionForce()`. It does not compute
a separate software "velocity" from consecutive setpoints and `dt`, because that
is only a noisy proxy for differential command streams.

For the contact-force edge case, ArmCommander should also configure
`SetMaxContactWrench()` when entering Cartesian streaming mode. The RDK documents
that as output regulation for the motion-controlled part of Cartesian
motion-force control, which is stronger than merely logging a later force spike.

Heartbeat is intentionally scoped to continuous streaming control only. It is not
a global "every robot operation must issue a command every N seconds" rule,
because blocking primitives like Home, MoveL, ZeroFTSensor, Contact, Float, or
tool setup can legitimately take seconds to complete. During those operations,
force and fault monitoring remain active while the heartbeat timeout is paused.

### Testing Strategy

Tests are the safety net for migration. Write tests against current behavior
first, then refactor with confidence.

**Layer 1 — Unit tests (no hardware, no ROS2):**
```python
class MockArmCommander:
    """Records all calls, returns configurable state."""
    def __init__(self):
        self.calls = []
        self._state = RobotState(...)  # configurable

    def move_to(self, pose, velocity=0.05):
        self.calls.append(('move_to', pose, velocity))

    def get_state(self):
        return self._state

# Test scan state machine logic
def test_scan_skips_to_returning_when_no_passes():
    robot = MockArmCommander()
    config = ScanConfig(passes=[])
    controller = ScanController(robot, MockSensorManager(), MockRecorder(), config)
    controller.run_scan()
    assert ('home',) in robot.calls  # went home, didn't scan

def test_safety_stops_on_workspace_violation():
    robot = MockArmCommander()
    with pytest.raises(SafetyViolation):
        robot.stream_cartesian(Pose(x=99.0, y=0, z=0))  # way out of bounds
    assert ('stop',) in robot.calls
```

**Layer 2 — Integration tests (ROS2, no hardware):**
- Launch nodes with mock drivers
- Verify topic connections, message formats, service responses
- Test launch files actually start the right nodes

**Layer 3 — Hardware smoke tests (on robot):**
- `test_cartesian_rotation.py` (already exists) — small rotation round-trip
- Home → zero FT → read wrench → verify near-zero
- Workspace boundary check — command pose at edge, verify safety stops

**Test file structure:**
```
tests/
├── unit/
│   ├── test_scan_controller.py
│   ├── test_teleop_controller.py
│   ├── test_safety.py
│   └── test_arm_commander.py
├── integration/
│   ├── test_driver_node.py
│   └── test_launch_files.py
└── hardware/
    ├── test_rotation.py          ← existing
    └── test_smoke.py             ← basic connectivity
```

**Migration workflow:**
1. Write unit tests for scan_controller logic using MockArmCommander
2. Verify tests pass against the mock
3. Build ArmCommander wrapping real RDK
4. Run same tests — if they pass, the abstraction is correct
5. Swap scan_node for scan_controller — tests catch regressions

## File Structure Target

```
ros2_ws/src/
├── flexiv_driver/                  ← driver layer
│   ├── flexiv_driver/
│   │   ├── arm_commander.py        ← pure Python robot interface
│   │   └── safety.py               ← SafetyChecker + SafetyWatchdog
│
├── coinft_driver/                  ← already mostly clean, extract from visionft
│   └── coinft_driver/
│       └── coinft_node.py
│
├── visionft/                       ← application layer
│   ├── visionft/
│   │   ├── scan_controller.py      ← pure state machine
│   │   ├── teleop_controller.py    ← moved from interview_demos
│   │   ├── sensor_manager.py       ← aggregated sensor access
│   │   ├── recording_manager.py    ← MCAP bag lifecycle
│   │   ├── grid_visualizer.py      ← visualization (unchanged)
│   │   └── wrench_plotter.py       ← visualization (unchanged)
│   ├── config/
│   │   ├── robot.yaml              ← shared robot config (SN, limits, safety bounds)
│   │   └── example_session.yaml    ← scan session template
│   └── launch/
│       ├── scan.launch.py
│       ├── teleop.launch.py
│       └── sensors.launch.py
│
├── inference/                      ← perception (unchanged)
│
├── common/                         ← shared types and utilities
│   ├── common/
│   │   ├── types.py                ← RobotState, Wrench, Pose dataclasses
│   │   ├── errors.py               ← shared exception hierarchy
│   │   └── config.py               ← shared config loader
│   └── config/
│       └── robot.yaml              ← shared robot/safety config
│
└── tests/                          ← test suite
    ├── unit/
    │   ├── test_scan_controller.py
    │   ├── test_teleop_controller.py
    │   ├── test_safety.py
    │   └── test_arm_commander.py
    ├── integration/
    │   ├── test_driver_node.py
    │   └── test_launch_files.py
    └── hardware/
        ├── test_rotation.py        ← existing
        └── test_smoke.py           ← basic connectivity
```

## Migration Strategy

**Rule: never rewrite, always migrate one piece at a time.**

### Phase 0: Now (done)
- [x] Created `rdk_utils.py` with shared utilities
- [x] Documented target architecture (this file)
- [x] Existing code continues to work unchanged

### Phase 1: Foundation (next new feature)
When building floating mode or next feature:
- [ ] Create `ArmCommander` as the single RDK owner
- [ ] Build `SafetyChecker` + `SafetyWatchdog` inside ArmCommander
- [ ] Create `MockArmCommander` for testing
- [ ] Write unit tests for safety (workspace bounds, force limits, heartbeat timeout)
- [ ] Build the new feature against ArmCommander
- [ ] Existing scan_node and teleop continue working as-is
- [ ] Keep `common/` and `flexiv_driver/` as pure Python modules for now

### Phase 2: Migration (tests enable this)
Write tests first, then migrate with confidence:
- [ ] Write unit tests for scan state machine logic using MockArmCommander
- [ ] Migrate scan_node to use ArmCommander → becomes scan_controller
- [ ] Verify tests pass against both mock and real robot
- [ ] Move teleop from interview_demos/ into visionft/teleop/
- [ ] Extract coinft into its own package
- [ ] Create shared config/robot.yaml
- [ ] Add sensor preflight that waits for CoinFT readiness before behavior starts

### Phase 3: Cleanup
- [ ] Remove legacy files (data_logger, flexiv.py, robot_publisher)
- [ ] Remove old scan_node once scan_controller passes all tests
- [ ] Remove rdk_cartesian_bridge (functionality absorbed by ArmCommander)
- [ ] Integration tests for launch files

## Decision Log

| Decision | Rationale | Date |
|----------|-----------|------|
| Migrate incrementally, not rewrite | Working system, solo dev, risk of breaking things | 2026-03-10 |
| ArmCommander hides mode switching | Applications shouldn't care about NRT_PLAN vs NRT_CARTESIAN | 2026-03-10 |
| Keep scan and teleop as separate applications | Different concerns, different control loops, share hardware | 2026-03-10 |
| Safety lives inside ArmCommander | Cannot be bypassed and stays at the robot boundary | 2026-03-11 |
| CoinFT readiness belongs to SensorManager/application preflight | Sensor initialization and freshness are not robot-driver concerns | 2026-03-11 |
| Stale required sensors should pause behavior in the higher layer | Loss of required data is a behavior/application issue, not a robot-driver issue | 2026-03-11 |
| Heartbeat timeout only applies to streaming control | Blocking primitives are legitimate long-running operations and should not false-stop | 2026-03-11 |
| No differential `dt`-based velocity checker in ArmCommander | Differential setpoint deltas are a poor proxy for robot velocity; RDK motion limits are the source of truth | 2026-03-11 |
| Keep a robot fault/force watchdog for observability | Flexiv already handles the low-level stop path; our layer should still log and surface those events | 2026-03-11 |
| Configure SDK contact-wrench limits proactively in streaming mode | `SetMaxContactWrench()` is regulation, not just post-hoc observation, so it should be set before contact issues appear | 2026-03-11 |
| Keep new layers pure Python for now | ROS packaging/wrappers can be added later if the migration needs them | 2026-03-11 |
| Tests enable migration, not the other way around | Write tests for current behavior first, then refactor safely | 2026-03-10 |
| MockArmCommander for unit tests | Test state machine logic without hardware, fast CI | 2026-03-11 |

## Surprises

(none yet)

## Outcomes

**Result**: (filled when migration is complete)
**Follow-ups**: (any new plans spawned)
