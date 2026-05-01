# VisionFT-TRACE

**Tactile Robot Arm Contact Experiments** — a research platform for contact-rich robotics on a Flexiv Rizon4. VisionFT is the sensing stack (vision + force/torque + tactile); TRACE is the experimentation layer built on top of it.

## What it does

- **VR hand-tracking teleoperation** — Quest 3S headset drives the robot via ZMQ keypoints, with live tactile and scene video streamed back to the headset.
- **Admittance / force-controlled contact** — constant-force pressing, hand-guided floating, and Cartesian motion-force control.
- **Automated scanning** — state-machine-driven phantom scans with auto-recording.
- **Tendon classification** — runs on the scanning camera and publishes class labels in real time.

## Hardware

- Flexiv Rizon4 
- PhysioVisionFT (CoinFT + Visual Based Tactile Sensor)
- Meta Quest 3S for Teleop

## Quickstart

```bash
# Build
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# Sensor stack (camera + force sensors)
ros2 launch visionft visionft.launch.py

# Hand-guided floating
ros2 run robot_behaviors floating_scan <robot_config>

# Automated scan (auto-records, auto-exits)
ros2 launch visionft scan.launch.py \
  scan_config:=/path/to/scan.yaml \
  robot_config:=/path/to/robot.yaml

# VR teleop (Quest 3S)
ros2 launch visionft teleop.launch.py \
  teleop_config:=/path/to/teleop.yaml \
  robot_config:=/path/to/robot.yaml \
  record:=true

# Tendon inference
ros2 run tendon_classifier tendon_inference
```


