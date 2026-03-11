# RDK Cartesian Bridge — Description & Test Protocol

## What this is

A ROS2 node (`rdk_cartesian_bridge`) that owns the **exclusive RDK connection**
to the Flexiv Rizon4 robot and exposes Cartesian motion control via standard
ROS2 topics.  All other nodes that need to command or read the robot go through
this bridge — no direct RDK access elsewhere.

## Architecture

```
Scan path publisher / vision node / test scripts
        │
        │  /rdk/cartesian_target  (geometry_msgs/PoseStamped)
        │  metres + quaternion (x, y, z, w)
        ▼
  rdk_cartesian_bridge  ←── owns RDK connection
  50 Hz control loop         NRT_CARTESIAN_MOTION_FORCE
        │
        ├──▶  /rdk/tcp_pose   (geometry_msgs/PoseStamped)
        └──▶  /rdk/wrench     (geometry_msgs/WrenchStamped)
        │
        ▼
  Flexiv Rizon4-062174
```

## Pose formats

| Format | Convention |
|--------|-----------|
| **RDK** `tcp_pose` | `[x, y, z, qw, qx, qy, qz]` — metres, quaternion |
| **Flexiv Elements** | `[x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]` — mm, Euler ZYX degrees |
| **ROS2** `PoseStamped` | position metres, orientation quaternion `(x, y, z, w)` |

Elements → ROS2 conversion (use when hardcoding known poses):
```python
from scipy.spatial.transform import Rotation as R
def elements_to_pose_stamped(pose):
    # pose: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    q = R.from_euler('xyz', pose[3:6], degrees=True).as_quat()  # [x,y,z,w]
    # fill PoseStamped: position = pose[:3]/1000, orientation = q
```

## Key parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_sn` | `Rizon4-062174` | Robot serial number |
| `control_rate_hz` | `50.0` | Heartbeat rate to RDK |
| `max_linear_vel` | `0.05` m/s | Max TCP linear velocity |
| `max_angular_vel` | `0.5` rad/s | Max TCP angular velocity |
| `max_linear_acc` | `0.5` m/s² | Max linear acceleration |
| `max_angular_acc` | `1.0` rad/s² | Max angular acceleration |

## ⚠️ MUST TEST BEFORE USE

### Pre-conditions
- Kill any other RDK process first: `pkill -f rizon.launch; pkill -f robot_publisher`
- Clear space around robot, E-stop in hand
- Build: `colcon build --packages-select visionft && source install/setup.bash`

### Test 1 — Bridge startup (hold mode)
```bash
# Terminal 1
ros2 run visionft rdk_cartesian_bridge \
  --ros-args -p max_linear_vel:=0.005 -p max_angular_vel:=0.05

# Verify: robot holds position, topics appear
ros2 topic list | grep rdk
ros2 topic echo /rdk/tcp_pose --once
```
Expected: bridge starts, robot holds, `/rdk/tcp_pose` and `/rdk/wrench` publishing.

### Test 2 — Rotation verification (`test_cartesian_rotation`)
```bash
# Terminal 2 (bridge already running)
ros2 run visionft test_cartesian_rotation

# Optional: watch pose live in terminal 3
ros2 topic echo /rdk/tcp_pose
```
Expected:
- Robot wrist rotates ~2 degrees around Z (slight twist, no translation)
- Holds 8 seconds
- Returns to original orientation
- No fault, no unexpected motion

### Test 3 — Verify `/rdk/tcp_pose` orientation is correct
Compare `tcp_pose` quaternion output against Flexiv Elements display.
The previous `robot_publisher.py` had a bug (Euler treated as quaternion) — this is now fixed.

## Known reference pose (above phantom)

Elements format: `[512, 260, 45, 0.05, -179.44, 0]`
→ TCP pointing straight down, directly above phantom

## Files

| File | Purpose |
|------|---------|
| `visionft/rdk_cartesian_bridge.py` | Main bridge node |
| `visionft/robot_publisher.py` | Legacy read-only publisher (fixed) |
| `visionft/test_cartesian_rotation.py` | Rotation verification test |
| `rdk/example_py/test_move_y.py` | Standalone RDK test (no ROS2) |

## Next: Floating Cartesian Mode

See memory notes. Plan: extend bridge with a `/rdk/set_mode` service that
switches between `hold` (current) and `float` (zero stiffness, backdriveable)
with configurable per-axis damping ratio via `SetCartesianImpedance()`.
