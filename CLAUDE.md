# VisionFT-TRACE

> Entry point. Repo map, conventions, run commands. For module diagrams, integration protocols, and dependency rules see `docs/architecture.md`.

**Tactile Robot Arm Contact Experiments** — research platform for contact-rich robotics on a
Flexiv Rizon4. Covers VR teleoperation, admittance / force-controlled contact, automated
scanning, and tactile sensing. ROS2 is the control and data layer; one exclusive RDK bridge
owns the robot connection.

## Repository Map

```
visionft-trace/
├── ros2_ws/src/
│   ├── arm_commander/          — C++ ArmCommander library (reusable robot interface)
│   │   ├── include/arm_commander/  — ArmCommander class, config (safety logic lives in arm_commander.cpp)
│   │   └── src/                    — Library implementation
│   ├── robot_behaviors/        — C++ robot behavior executables
│   │   ├── include/robot_behaviors/ — App-specific configs (scan, teleop)
│   │   ├── src/
│   │   │   ├── floating_scan.cpp   — Hand-guided joint floating
│   │   │   ├── scan_controller.cpp — Automated phantom scanning state machine
│   │   │   ├── teleop.cpp          — VR hand-tracking teleop (Quest 3S → ZMQ)
│   │   │   └── massage.cpp         — Constant-force Z press (force ctrl Z, motion ctrl XY)
│   │   └── config/                 — YAML configs (robot, scan, teleop)
│   ├── visionft/              — Python sensors, VR streaming, dashboards
│   │   ├── visionft/
│   │   │   ├── sensors/                 — coinft, usb_camera, wait_for_coinft
│   │   │   ├── streams/                 — scene_stream, tactile_stream (cameras → ZMQ for VR)
│   │   │   └── viz/                     — led_dashboard, grid_visualizer, plot_csv
│   │   ├── launch/                      — visionft, scan, teleop, record
│   │   └── config/
│   │       └── example_session.yaml     — Multi-scan session template
│   ├── tendon_classifier/     — Tendon classification CNN
│   │   └── tendon_classifier/
│   │       ├── inference_node.py        — ROS2 node: /image_raw → /tendon_class
│   │       ├── config.py               — Dataclass configs + YAML loader
│   │       ├── models_v2.py            — Model architectures (ResNet/DINOv2/CLIP)
│   │       ├── encoders.py             — Vision encoder wrappers
│   │       └── attention.py            — Fusion + temporal aggregation modules
│   └── flexiv_ros2/           — Flexiv RDK bindings, examples, utility functions
├── references/                — Reference code (leapft, VR teleop prototype)
├── classifier/                — TendonClassifier training/labeling pipeline
├── scripts/
│   └── extract_mcap.py        — MCAP bag → CSV/images extraction
└── data/                      — Recorded session data (.mcap, .csv)
```

## Quick References

- **Pose Formats**: RDK=[x,y,z,qw,qx,qy,qz], ROS2=[x,y,z] + quat(x,y,z,w), Elements=[mm,mm,mm,deg,deg,deg]
- **Scan State Machine**: HOMING → ZEROING_FT → DESCENDING → SCANNING → RETURNING → DONE

## Conventions

- Python 3.10 (system), ROS2 Humble
- MCAP bags for data recording (not CSV loggers)
- Quaternion order: always document which convention (w-first vs w-last) at every boundary
- YAML configs for scan/robot/teleop (loaded by C++ via yaml-cpp; not ROS2 params)

## Golden Principles

See `docs/golden-principles.md`.
GP1 one fact one place · GP2 centralize don't duplicate · GP3 validate at boundaries · GP4 fail loud not silent · GP5 record with MCAP

## How to Run

```bash
# Sensor stack (camera + force sensors)
ros2 launch visionft visionft.launch.py

# Hand-guided floating (manual control)
ros2 run robot_behaviors floating_scan <robot_config>

# Automated scan (auto-records, auto-exits)
ros2 run robot_behaviors scan_controller <scan_config> <robot_config>
# Or via launch file:
ros2 launch visionft scan.launch.py scan_config:=/path/to/scan.yaml robot_config:=/path/to/robot.yaml

# VR teleop (Quest 3S hand tracking)
ros2 run robot_behaviors teleop <teleop_config> <robot_config>
# Or via launch file (includes ZMQ bridge + optional MCAP recording):
ros2 launch visionft teleop.launch.py teleop_config:=/path/to/teleop.yaml robot_config:=/path/to/robot.yaml record:=true

# Inference
ros2 run tendon_classifier tendon_inference
```

## Deeper Context

| Topic | File |
|-------|------|
| Architecture & data flow | `docs/architecture.md` |
| Golden principles | `docs/golden-principles.md` |
| Lessons learned & architecture decisions | `docs/lessons-learned.md` |
| Getting started | `docs/getting-started.md` |
| VR teleop setup | `docs/getting-started-teleop.md` |
