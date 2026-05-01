# Getting Started — VR Teleop

VR hand-tracking teleop using Quest 3S to control the Flexiv Rizon4.

## Prerequisites

- Quest 3S running FrankaBot APK (This comes as an APK from OpenTeach)
- Quest and this machine on the same WiFi network
- Workspace built (`colcon build --symlink-install`)

## Setup

### 1. Build (if not already)

```bash
cd ~/VisualFT/ros2_ws
colcon build --packages-select arm_commander robot_behaviors visionft --symlink-install
source install/setup.bash
```

You can skip number 2 and 3 after initial setup.

### 2. Find your machine's IP

```bash
hostname -I
```

Use the IP on the same subnet as the Quest (e.g. `192.168.0.x`). Configure the FrankaBot APK to connect to this IP.

### 3. Verify Quest connectivity

```bash
ping <quest-ip>
```

## Running

Run each in a separate terminal, in order.

### Terminal 1 — Sensor stack (no sudo)

```bash
ros2 launch visionft visionft.launch.py
```

### Terminal 2 — Teleop launch (requires sudo)

Launches VR server, camera streams (tactile + scene), and teleop together. Sudo is required because `RT_JOINT_TORQUE` mode needs realtime thread scheduling.

```bash
sudo bash -c "source /opt/ros/humble/setup.bash && \
  source ~/VisualFT/ros2_ws/install/setup.bash && \
  ros2 launch visionft teleop.launch.py \
    teleop_config:=~/VisualFT/ros2_ws/install/robot_behaviors/share/robot_behaviors/config/teleop.yaml \
    robot_config:=~/VisualFT/ros2_ws/install/robot_behaviors/share/robot_behaviors/config/robot.yaml \
    stream:=true"
```

This starts:
- **vr_server** — receives Quest hand tracking, publishes ZMQ keypoints
- **tactile_stream** — ROS2 `/image_raw` with force overlay to Quest (port 15001)
- **scene_stream** — USB webcam to Quest (port 10505)
- **teleop** — ZMQ keypoints to robot cartesian commands

The robot will home, zero the F/T sensor, then wait for VR input. Use the Quest APK button to engage/disengage teleop. Ctrl+C to stop.

**Note:** Running under sudo means root needs `zmq` and `cv2`. Install with:
```bash
sudo pip install pyzmq opencv-python
```

If you don't want to install packages for root, run streams and vr_server separately as your user (see Fallback below).


| Stream | Port | Direction |
|--------|------|-----------|
| Quest hand tracking (PUSH) | 8087 | Quest -> vr_server |
| Transformed keypoints (PUB) | 8089 | vr_server -> teleop |
| Pause state (PUB) | 8102 | vr_server -> teleop |
| Tactile camera (PUB) | 15001 | this machine -> Quest |
| Scene camera (PUB) | 10505 | this machine -> Quest |

## Troubleshooting

- **Robot won't move**: teleop needs `sudo` for realtime scheduling (`RT_JOINT_TORQUE`)
- **Scene camera not visible in VR**: confirm the stream is on port `10505` (not `10005`)
- **TCP outside safety box**: check the warning in teleop output — adjust `safety.pos_max` in `config/teleop.yaml`
- **`No module named 'zmq'` under sudo**: run streams as your user in separate terminals (don't use the launch file with sudo)
- **VR DROPOUT warning**: Quest APK is not sending data — check that vr_server is running and the Quest app is active

## Headset configuration

Each Quest's APK has its scene-camera port hardcoded:

| Headset | Scene camera port |
|---------|-------------------|
| Lukas's Quest | 10505 (default) |
| VISIONFT Quest | 10005 |

If using the VISIONFT Quest, run `scene_stream` with `--port 10005`:

```bash
ros2 run visionft scene_stream --port 10005
```

Tactile stream port (15001) is the same for both headsets.

## Config

Teleop parameters are in `ros2_ws/src/robot_behaviors/config/teleop.yaml`:

- `safety.pos_min` / `pos_max` — workspace safety box (metres)
- `retargeting.robot_workspace` — VR-to-robot scaling per axis
- `impedance.stiffness` / `damping` — cartesian impedance control
- `zmq.host` / `keypoint_port` / `pause_port` — ZMQ connection to vr_server

## Known limitations

- **Workspace bounds clip the reachable area.** The `safety.workspace` box in
  `ros2_ws/src/robot_behaviors/config/robot.yaml` (currently x∈[0.30, 0.735],
  y∈[-0.25, 0.430], z∈[-0.02, 0.55] in robot base frame) is intentionally
  conservative — the robot can physically reach further, but the bounds are set
  to keep the TCP away from table edges, the operator, and the phantom mount
  during early teleop sessions. If you need more reach, widen the box in
  `robot.yaml` after confirming the wider region is collision-free. This is by
  design, not a bug.
