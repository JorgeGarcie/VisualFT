# flexiv_test_nodes — Project Plan

## Phase 1: Cartesian Trajectory Primitives [DONE]

Extensible Cartesian trajectory generation executed through the joint
impedance controller with low gains. Uses MoveIt2 GetCartesianPath for
Cartesian-to-joint conversion.

### Files created

- `trajectory_generators/base.py` — abstract base class
  (`generate_waypoints`, `configure`, `get_parameter_declarations`)
- `trajectory_generators/zigzag.py` — boustrophedon X-Y scan
- `trajectory_generators/straight_line.py` — relative (dx, dy, dz) offset
- `trajectory_generators/__init__.py` — `TRAJECTORY_REGISTRY`
- `cartesian_impedance_node.py` — subscribes to joint_states + robot_states,
  calls compute_cartesian_path, resamples at 1kHz, publishes JointPosVel
- `rizon_controllers_low_impedance.yaml` — reduced k_p/k_d gains
- `cartesian_impedance_config.yaml` — node parameters
- `cartesian_impedance.launch.py` — launch file

### Files modified

- `setup.py` — added trajectory_generators subpackage + entry point
- `package.xml` — added flexiv_msgs, moveit_msgs, geometry_msgs, sensor_msgs
- `rizon.launch.py` — added `controllers_config` arg

### Usage

```bash
# Terminal 1: Robot bringup with low impedance
ros2 launch flexiv_bringup rizon.launch.py \
    robot_sn:=Rizon4-062174 \
    robot_controller:=joint_impedance_controller \
    controllers_config:=rizon_controllers_low_impedance.yaml

# Terminal 2: MoveIt2 (for IK)
ros2 launch flexiv_bringup rizon_moveit.launch.py robot_sn:=Rizon4-062174

# Terminal 3: Run trajectory
ros2 launch flexiv_bringup cartesian_impedance.launch.py \
    robot_sn:=Rizon4-062174 trajectory_type:=zigzag
```

### Adding new trajectory primitives

1. Create `trajectory_generators/my_new.py` (subclass `TrajectoryGenerator`)
2. Add to `TRAJECTORY_REGISTRY` in `__init__.py`
3. No other changes needed.

---

## Phase 2: BT Routine Runner [DONE]

Compose trajectory primitives into reactive multi-step routines using
behavior trees (py_trees). Routines defined in YAML, parsed into a
py_trees tree, ticked at a configurable rate.

### Architecture

```
routine_runner_node (MultiThreadedExecutor, 2 threads)
  ├── Thread 1: 1kHz JointPosVel publish timer (MutuallyExclusiveCallbackGroup)
  └── Thread 2: BT tick timer + subscriptions + MoveIt service calls
          │
          ├── RobotController (shared state on blackboard)
          │   ├── joint_states subscription (continuously updated)
          │   ├── robot_states subscription (tcp_pose, ext_wrench)
          │   ├── MoveIt compute_cartesian_path async client
          │   ├── trajectory resampling + playback (thread-safe via Lock)
          │   └── hold-position between trajectories
          │
          └── py_trees BehaviourTree (from YAML)
              ├── Composites: Sequence, ReactiveSequence, Selector,
              │               ReactiveSelector, Parallel
              ├── Decorators: Repeat, Retry, Timeout, Inverter, etc.
              ├── Actions:    MoveTo, Wait, Trajectory, SetSpeed, Log
              └── Conditions: ForceBelow, ForceAbove, TCPInBounds,
                              ParameterCheck
```

### Files created

- `robot_controller.py` — shared infrastructure extracted from
  cartesian_impedance_node. Async MoveIt planning API for BT integration.
  Continuously updates joint_positions, tcp_pose, ext_wrench. Accepts
  `robot_sn` to derive joint name prefix (for `/joint_states` matching)
  and robot states topic name.
- `bt_nodes/__init__.py` — combined `NODE_REGISTRY`
- `bt_nodes/actions.py` — `MoveToAction` (absolute/relative Cartesian),
  `WaitAction`, `TrajectoryAction` (runs registered generators),
  `SetSpeedAction`, `LogAction`. All non-blocking (return RUNNING).
- `bt_nodes/conditions.py` — `ForceBelowCondition`, `ForceAboveCondition`,
  `TCPInBoundsCondition`, `ParameterCheckCondition`. Instant SUCCESS/FAILURE.
- `bt_parser.py` — YAML → py_trees tree builder. Recursive parser mapping
  composites, decorators, and leaf nodes via registries.
- `routine_runner_node.py` — ROS 2 node. Loads YAML, builds BT, ticks until
  root returns SUCCESS/FAILURE. MultiThreadedExecutor keeps 1kHz timer
  running during MoveIt planning. Accepts `robot_sn` parameter to
  auto-derive joint name prefix and robot states topic.
- `configs/routines/example_force_limited_scan.yaml` — example routine with
  force-guarded approach + zigzag scan + recovery fallback.
- `configs/routines/go_home.yaml` — move to saved home position.
- `configs/routines/test_zigzag.yaml` — run zigzag from current position
  (for testing individual motions).
- `launch/routine_runner.launch.py` — launch file with routine_file,
  controller_name, linear_speed args.

### Files modified

- `setup.py` — added `bt_nodes` subpackage, `configs/routines` data files,
  `routine_runner` entry point
- `package.xml` — added `py-trees` exec dependency

### YAML routine schema

```yaml
name: "my_routine"
tick_rate_hz: 25.0

tree:
  type: Sequence           # or ReactiveSequence, Selector, Parallel, etc.
  children:
    - type: SetSpeed
      linear_speed: 0.01

    - type: MoveTo           # absolute Cartesian
      position: {x: 0.4, y: -0.1, z: 0.3}
      orientation: {x: 0, y: 1, z: 0, w: 0}   # optional

    - type: MoveTo           # relative offset
      relative: true
      position: {dx: 0.0, dy: 0.02, dz: 0.0}

    - type: Wait
      duration: 1.0

    - type: Trajectory       # run a registered generator
      generator: zigzag
      params:
        zigzag_x_amplitude: 0.04
        zigzag_num_sweeps: 6

    - type: Log
      message: "Done"

    # Reactivity pattern: guard an action with a force condition
    - type: ReactiveSequence
      children:
        - type: ForceBelow
          threshold: 10.0      # re-checked every tick
        - type: Trajectory     # preempted if force exceeds threshold
          generator: zigzag

    # Decorators
    - type: Repeat
      count: 3               # 0 or -1 = infinite
      child:
        type: MoveTo
        relative: true
        position: {dx: 0.01, dy: 0.0, dz: 0.0}

    # Selector (fallback)
    - type: Selector
      children:
        - type: ReactiveSequence
          children:
            - type: ForceBelow
              threshold: 15.0
            - type: MoveTo
              position: {x: 0.4, y: 0.0, z: 0.15}
        - type: Sequence       # fallback: retract
          children:
            - type: Log
              message: "Force exceeded, retracting"
              level: warn
            - type: MoveTo
              relative: true
              position: {dx: 0.0, dy: 0.0, dz: 0.05}
```

### Available BT node types

**Actions** (return RUNNING while executing):
| Type | Parameters |
|------|-----------|
| `MoveTo` | `position: {x,y,z}` or `{dx,dy,dz}`, `orientation` (opt), `relative` (opt) |
| `Wait` | `duration` (seconds) |
| `Trajectory` | `generator` (registry key), `params` (dict) |
| `SetSpeed` | `linear_speed` (m/s) |
| `Log` | `message`, `level` (info/warn/error) |

**Conditions** (instant SUCCESS/FAILURE each tick):
| Type | Parameters |
|------|-----------|
| `ForceBelow` | `threshold` (N), `axis` (x/y/z/norm) |
| `ForceAbove` | `threshold` (N), `axis` (x/y/z/norm) |
| `TCPInBounds` | `min: {x,y,z}`, `max: {x,y,z}` |
| `ParameterCheck` | `param_name`, `expected` |

**Composites**: `Sequence`, `ReactiveSequence`, `Selector`, `ReactiveSelector`, `Parallel`
**Decorators**: `Repeat`, `Retry`, `Timeout`, `Inverter`, `SuccessIsFailure`, `FailureIsSuccess`

### Usage

```bash
ros2 run flexiv_test_nodes routine_runner --ros-args \
  -p routine_file:=/path/to/routine.yaml \
  -p robot_sn:=Rizon4-062174

# Or via launch file
ros2 launch flexiv_test_nodes routine_runner.launch.py \
    routine_file:=/path/to/routine.yaml
```

The `robot_sn` parameter auto-derives:
- Joint name prefix for `/joint_states` matching (e.g. `Rizon4-062174_joint1`)
- Robot states topic (e.g. `/Rizon4_062174/flexiv_robot_states`)

The `joints` parameter defaults to `['joint1',...,'joint7']` (URDF names)
and no longer needs to be specified explicitly.

### Adding new BT node types

1. Add action class to `bt_nodes/actions.py` and register in `ACTION_REGISTRY`
2. Or add condition class to `bt_nodes/conditions.py` and register in `CONDITION_REGISTRY`
3. The parser picks them up automatically via `NODE_REGISTRY`.

---

## Phase 3: Phantom Scanning Routine + Automated Recording [DONE]

Complete scanning workflow with BT-controlled data recording. The
DataLogger (visionft package) is kept alive between recordings and
toggled via a `std_srvs/SetBool` service from a new BT action node.

### Workflow

1. Move above phantom (absolute Cartesian)
2. Start recording (DataLogger `~/set_recording` service)
3. Slow approach until contact (ReactiveSequence + ForceBelow guard,
   wrapped in FailureIsSuccess — contact = expected failure)
4. Settle (0.5 s)
5. Zigzag scan at contact height (Z locked by trajectory generator)
6. Stop recording
7. Retract (relative +Z) and return home

### Contact detection pattern

```
FailureIsSuccess
  └── ReactiveSequence          (re-evaluates every tick @ 25 Hz)
        ├── ForceBelow(2.0 N)   SUCCESS while no contact
        └── MoveTo(dz: -0.15)   keeps descending (RUNNING)
```

When force >= 2 N: ForceBelow → FAILURE → ReactiveSequence → FAILURE →
MoveTo preempted (robot holds position) → FailureIsSuccess flips to
SUCCESS → Sequence continues to scan.

### Files created (flexiv_test_nodes)

- `configs/routines/phantom_scan.yaml` — full scan routine YAML
- `launch/phantom_scan.launch.py` — launches `routine_runner` (with
  phantom_scan.yaml) + `data_logger` (from visionft, idle by default)

### Files modified (flexiv_test_nodes)

- `bt_nodes/actions.py` — added `SetRecordingAction` (calls
  `/data_logger/set_recording` SetBool service async; polls future in
  `update()`). Registered in `ACTION_REGISTRY` as `"SetRecording"`.
- `package.xml` — added `std_srvs` dependency

### Files modified (visionft)

- `visionft/data_logger.py` — added `~/set_recording` service
  (`std_srvs/SetBool`). Node starts idle (`_recording = False`). All
  callbacks early-return when not recording. `data=True` creates new
  timestamped trial directory + opens CSV/video files. `data=False`
  closes files. Node stays alive between recordings.
- `package.xml` — added `std_srvs` dependency

### Available BT node types (updated)

**Actions** (return RUNNING while executing):
| Type | Parameters |
|------|-----------|
| `MoveTo` | `position: {x,y,z}` or `{dx,dy,dz}`, `orientation` (opt), `relative` (opt) |
| `Wait` | `duration` (seconds) |
| `Trajectory` | `generator` (registry key), `params` (dict) |
| `SetSpeed` | `linear_speed` (m/s) |
| `Log` | `message`, `level` (info/warn/error) |
| `SetRecording` | `enabled` (bool) — start/stop visionft DataLogger |

### Saved positions

| Name | Position (x, y, z) | Orientation (x, y, z, w) | Description |
|------|--------------------|--------------------------| ----------- |
| Home | 0.6804, -0.1078, 0.2362 | -0.0054, 0.9999, 0.0027, -0.0074 | Resting position |
| AbovePhantom | 0.4771, 0.1882, 0.0916 | -0.0063, 0.9999, 0.0013, -0.0070 | Above tendon |

### Usage

```bash
# Terminal 1: Robot bringup with low impedance
ros2 launch flexiv_bringup rizon.launch.py \
    robot_sn:=Rizon4-062174 \
    robot_controller:=joint_impedance_controller \
    controllers_config:=rizon_controllers_low_impedance.yaml

# Terminal 2: MoveIt2
ros2 launch flexiv_bringup rizon_moveit.launch.py robot_sn:=Rizon4-062174

# Terminal 3: VisionFT sensors (camera + coinft + flexiv wrench)
ros2 launch visionft visionft.launch.py

# Terminal 4: Phantom scan routine + recording
ros2 launch flexiv_test_nodes phantom_scan.launch.py

# Testing individual motions (no launch file needed)
ros2 run flexiv_test_nodes routine_runner --ros-args \
  -p routine_file:=$(ros2 pkg prefix flexiv_test_nodes)/share/flexiv_test_nodes/configs/routines/test_zigzag.yaml \
  -p robot_sn:=Rizon4-062174

# Go home
ros2 run flexiv_test_nodes routine_runner --ros-args \
  -p routine_file:=$(ros2 pkg prefix flexiv_test_nodes)/share/flexiv_test_nodes/configs/routines/go_home.yaml \
  -p robot_sn:=Rizon4-062174

# Read current TCP pose (for saving new positions)
ros2 topic echo /robot_states flexiv_msgs/msg/RobotStates --field tcp_pose --once
```

### Safety limits

- **Table Z limit**: `z = 0.002` — robot must never go below this or it
  will hit the table. When setting `dz` for approach descent, ensure
  `AbovePhantom.z + dz > 0.002`.

### Tuning

- `threshold` in `ForceBelow` (contact detection): adjust for phantom
  stiffness (default 2.0 N)
- `position.z` in `AbovePhantom` / `Home`: adjust for your setup
- `dz` in `ApproachSurface`: max descent distance. Must satisfy
  `AbovePhantom.z + dz > 0.002` (table limit)
- `zigzag_*` params: scan area and density

---

## Future work

- **Native RDK force control**: Extend hardware interface with Cartesian
  command interfaces to support `StreamCartesianMotionForce` /
  `SetCartesianImpedance` through ros2_control. Requires C++ changes to
  `flexiv_hardware_interface.cpp`.
- **Cartesian impedance at EE**: Either via RDK native mode (bypass
  ros2_control) or custom ros2 controller computing τ = Jᵀ · K_cart · e.
- **Gripper actions**: Add `GripperMove` / `GripperGrasp` BT action nodes
  wrapping the existing flexiv_gripper action servers.
