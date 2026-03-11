# Getting Started

## Prerequisites

- Ubuntu 22.04 with ROS2 Humble
- Python 3.10 (system — `flexivrdk` not available on conda)
- Flexiv Rizon4 robot (SN: Rizon4-062174) on Ethernet
- CoinFT sensor on USB serial
- Raspberry Pi camera streaming H264 over UDP

## Build

```bash
cd /home/li2053/VisualFT/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Common Workflows

### 1. Sensor Stack (continuous monitoring)

```bash
ros2 launch visionft visionft.launch.py
```

Starts: gscam2 (camera), flexiv_wrench_publisher (robot F/T), coinft (tactile sensor).

### 2. Automated Phantom Scan

```bash
# Default single scan
ros2 launch visionft scan.launch.py

# Custom parameters
ros2 launch visionft scan.launch.py y_scan_range:=0.05 scan_speed:=0.03

# Multi-scan session from YAML
ros2 launch visionft scan.launch.py session_config:=/path/to/session.yaml
```

Auto-records MCAP bag per scan. Exits when all scans complete.

### 3. Bridge + Manual Control

```bash
ros2 run visionft rdk_cartesian_bridge
# Then publish PoseStamped to /rdk/cartesian_target from another node
```

### 4. Tendon Inference

```bash
ros2 run inference tendon_inference --ros-args -p model_dir:=/path/to/model
```

### 5. VR Teleop (interview demo)

```bash
cd interview_demos/teleop
python3 teleop_main.py
```

Requires Quest 3S on same WiFi network.

## Important Notes

- **Never run bridge and scan_node simultaneously** — both need exclusive RDK access
- **Use `python3` not `python`** — flexivrdk is on system Python 3.10, not conda
- **Zero F/T sensor after homing** — prevents force bias drift
- **E-stop**: If triggered, must be physically released before robot can re-enable
