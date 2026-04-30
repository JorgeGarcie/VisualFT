# Getting Started

## Prerequisites

- Ubuntu 22.04 with ROS2 Humble
- Python 3.10 (system — `flexivrdk` not available on conda)
- Flexiv Rizon4 robot (SN: Rizon4-062174) on Ethernet
- CoinFT sensor on USB serial
- USB tactile camera (CoinFT sensor view) on `/dev/video2`
- USB scene camera (3rd person view) on `/dev/video0`

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

Starts: usb_camera (tactile camera on `/dev/video2` → `/image_raw`), coinft (CoinFT sensor).

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

### 3. Tendon Inference

```bash
ros2 run tendon_classifier tendon_inference --ros-args -p model_dir:=/path/to/model
```

### 4. VR Teleop

See [getting-started-teleop.md](getting-started-teleop.md) for full setup instructions.

### 5. LED + Vision Dashboard

PyQt5 dashboard for LED control (Arduino NeoPixels), live CoinFT force plots, and 5x2 camera vision analytics. Requires the sensor stack running.

```bash
# Terminal 1: sensor stack
ros2 launch visionft visionft.launch.py

# Terminal 2: dashboard
ros2 run visionft led_dashboard --led-port /dev/ttyACM0
```

Subscribes to `/image_raw` and `/coinft/wrench` from ROS2. LED panel runs in simulation mode if Arduino is not connected.

## Important Notes

- **Only one robot_behaviors executable at a time** — floating_scan, scan_controller, and teleop each open the single RDK connection
- **Zero F/T sensor after homing** — prevents force bias drift
- **E-stop**: If triggered, must be physically released before robot can re-enable
- **RT scheduler needs sudo** — teleop uses `RT_JOINT_TORQUE`; run with `sudo` or `setcap cap_sys_nice+ep` on the binary
