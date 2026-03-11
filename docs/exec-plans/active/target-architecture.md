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
│  RobotInterface         SensorManager                  │
│  - move_to(pose)        - get_wrench() → Wrench        │
│  - set_impedance(K,Z)   - get_image() → Image          │
│  - zero_ft()            - healthy() → bool              │
│  - get_state() → State  - wait_for_ready(timeout)       │
│  - fault() → bool                                      │
│  - home()               RecordingManager                │
│  - stop()               - start_bag(name, topics)       │
│                         - stop_bag()                    │
│  SafetyMonitor (cross-cutting — wraps RobotInterface)  │
│  - workspace bounds     - force/torque limits           │
│  - velocity limits      - heartbeat timeout → stop      │
├───────────────────────────────────────────────────────┤
│                    Drivers                             │
│  FlexivRDKDriver      CoinFTDriver     CameraDriver   │
│  (owns RDK conn)      (serial 360Hz)   (gscam2/UDP)   │
│  heartbeat, fault     offset, ONNX     H264 decode     │
│  monitoring           calibration                      │
├───────────────────────────────────────────────────────┤
│                    Common                              │
│  config.py   rdk_utils.py   types.py                   │
│  robot SN    quat convert   RobotState, Wrench,        │
│  limits      msg builders   ScanConfig dataclasses      │
└───────────────────────────────────────────────────────┘
```

## The Abstractions

### RobotInterface

The key abstraction. Applications never touch RDK directly. They call methods
on RobotInterface, which handles mode switching, connection, heartbeat.

```python
class RobotInterface:
    """Single owner of the RDK connection."""

    def __init__(self, config: RobotConfig):
        self._robot = init_robot(config.serial_number)
        self._heartbeat_thread = ...  # monitor connection health

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
- Heartbeat monitoring is built in, not bolted on later

### SensorManager

```python
class SensorManager:
    """Aggregates sensor health and data access."""

    def __init__(self, robot: RobotInterface, coinft_topic: str, camera_topic: str):
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

### What applications look like with these abstractions

**scan_controller.py** (~150 lines instead of 600):
```python
class ScanController:
    def __init__(self, robot: RobotInterface, sensors: SensorManager,
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
    def __init__(self, robot: RobotInterface, vr: VRReader,
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

### SafetyMonitor

Cross-cutting concern — wraps RobotInterface so every command is bounds-checked
before reaching RDK. Independent watchdog thread catches failures even if the
application hangs.

```python
class SafetyMonitor:
    """Wraps RobotInterface. Every command passes through safety checks."""

    def __init__(self, robot: RobotInterface, config: SafetyConfig):
        self._robot = robot
        self._config = config
        self._watchdog = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._last_command_time = time.monotonic()
        self._watchdog.start()

    def stream_cartesian(self, pose: Pose):
        """Check bounds, then forward to robot."""
        self._check_workspace(pose)
        self._check_velocity(pose)
        self._last_command_time = time.monotonic()
        self._robot.stream_cartesian(pose)

    def _check_workspace(self, pose: Pose):
        """Raise SafetyViolation if pose is outside configured bounds."""
        if not self._config.workspace.contains(pose.position):
            self._robot.stop()
            raise SafetyViolation(f"Pose {pose} outside workspace bounds")

    def _check_force(self):
        """Check current wrench against limits."""
        wrench = self._robot.get_state().wrench
        if abs(wrench.fz) > self._config.max_force_z:
            self._robot.stop()
            raise SafetyViolation(f"Force {wrench.fz}N exceeds limit")

    def _watchdog_loop(self):
        """Independent thread — catches hangs and faults."""
        while True:
            time.sleep(0.1)  # 10 Hz check
            # Heartbeat timeout
            if time.monotonic() - self._last_command_time > self._config.heartbeat_timeout:
                self._robot.stop()
                logging.error("Heartbeat timeout — robot stopped")
            # Force check (always, regardless of application state)
            self._check_force()
            # Fault check
            if self._robot.fault():
                logging.error("Robot fault detected by watchdog")
```

```python
@dataclass
class SafetyConfig:
    workspace: BoundingBox       # min/max xyz
    max_force_z: float = 50.0   # N
    max_velocity: float = 0.1   # m/s
    heartbeat_timeout: float = 1.0  # seconds without command → stop
```

Key design: applications use `SafetyMonitor` instead of `RobotInterface` directly.
Safety cannot be bypassed — it's not a check the application opts into, it's
the interface the application uses. The watchdog thread runs independently, so
even if the application hangs or crashes, the robot gets stopped.

### Testing Strategy

Tests are the safety net for migration. Write tests against current behavior
first, then refactor with confidence.

**Layer 1 — Unit tests (no hardware, no ROS2):**
```python
class MockRobotInterface(RobotInterface):
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
    robot = MockRobotInterface()
    config = ScanConfig(passes=[])
    controller = ScanController(robot, MockSensorManager(), MockRecorder(), config)
    controller.run_scan()
    assert ('home',) in robot.calls  # went home, didn't scan

def test_safety_stops_on_workspace_violation():
    robot = MockRobotInterface()
    safety = SafetyMonitor(robot, SafetyConfig(workspace=BoundingBox(...)))
    with pytest.raises(SafetyViolation):
        safety.stream_cartesian(Pose(x=99.0, y=0, z=0))  # way out of bounds
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
│   ├── test_safety_monitor.py
│   └── test_rdk_utils.py
├── integration/
│   ├── test_driver_node.py
│   └── test_launch_files.py
└── hardware/
    ├── test_rotation.py          ← existing
    └── test_smoke.py             ← basic connectivity
```

**Migration workflow:**
1. Write unit tests for scan_controller logic using MockRobotInterface
2. Verify tests pass against the mock
3. Build RobotInterface wrapping real RDK
4. Run same tests — if they pass, the abstraction is correct
5. Swap scan_node for scan_controller — tests catch regressions

## File Structure Target

```
ros2_ws/src/
├── flexiv_driver/                  ← driver layer
│   ├── flexiv_driver/
│   │   ├── robot_interface.py      ← RobotInterface class
│   │   ├── safety_monitor.py       ← SafetyMonitor wrapping RobotInterface
│   │   ├── driver_node.py          ← ROS2 node wrapping RobotInterface
│   │   └── config.py               ← RobotConfig, SafetyConfig dataclasses
│   └── package.xml
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
│   │   ├── rdk_utils.py            ← quat conversion, msg builders
│   │   └── config.py               ← shared config loader
│   └── package.xml
│
└── tests/                          ← test suite
    ├── unit/
    │   ├── test_scan_controller.py
    │   ├── test_teleop_controller.py
    │   ├── test_safety_monitor.py
    │   └── test_rdk_utils.py
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
- [ ] Create `RobotInterface` class wrapping RDK
- [ ] Create `SafetyMonitor` wrapping RobotInterface
- [ ] Create `MockRobotInterface` for testing
- [ ] Write unit tests for safety (workspace bounds, force limits, heartbeat timeout)
- [ ] Build the new feature against SafetyMonitor
- [ ] Existing scan_node and teleop continue working as-is

### Phase 2: Migration (tests enable this)
Write tests first, then migrate with confidence:
- [ ] Write unit tests for scan state machine logic using MockRobotInterface
- [ ] Migrate scan_node to use RobotInterface → becomes scan_controller
- [ ] Verify tests pass against both mock and real robot
- [ ] Move teleop from interview_demos/ into visionft/teleop/
- [ ] Extract coinft into its own package
- [ ] Create shared config/robot.yaml

### Phase 3: Cleanup
- [ ] Remove legacy files (data_logger, flexiv.py, robot_publisher)
- [ ] Remove old scan_node once scan_controller passes all tests
- [ ] Remove rdk_cartesian_bridge (functionality absorbed by RobotInterface)
- [ ] Integration tests for launch files

## Decision Log

| Decision | Rationale | Date |
|----------|-----------|------|
| Migrate incrementally, not rewrite | Working system, solo dev, risk of breaking things | 2026-03-10 |
| RobotInterface hides mode switching | Applications shouldn't care about NRT_PLAN vs NRT_CARTESIAN | 2026-03-10 |
| Keep scan and teleop as separate applications | Different concerns, different control loops, share hardware | 2026-03-10 |
| Safety wraps RobotInterface, not alongside it | Cannot be bypassed — it IS the interface apps use | 2026-03-10 |
| Tests enable migration, not the other way around | Write tests for current behavior first, then refactor safely | 2026-03-10 |
| MockRobotInterface for unit tests | Test state machine logic without hardware, fast CI | 2026-03-10 |

## Surprises

(none yet)

## Outcomes

**Result**: (filled when migration is complete)
**Follow-ups**: (any new plans spawned)
