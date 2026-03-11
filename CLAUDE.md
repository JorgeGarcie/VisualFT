# VisualFT

Robotic ultrasound phantom scanning system using a Flexiv Rizon4 robot arm with force-controlled
contact, a CoinFT tactile sensor, and a vision-based tendon classifier. ROS2 is the control and
data layer; one exclusive RDK bridge owns the robot connection.

## Repository Map

```
VisualFT/
├── ros2_ws/src/
│   ├── floating_scan/         — C++ ArmCommander library + robot executables
│   │   ├── include/arm_commander/  — ArmCommander class, config, safety
│   │   ├── src/
│   │   │   ├── floating_scan.cpp   — Hand-guided joint floating
│   │   │   ├── scan_controller.cpp — Automated phantom scanning state machine
│   │   │   └── teleop.cpp          — VR hand-tracking teleop (Quest 3S → ZMQ)
│   │   └── config/                 — YAML configs (robot, scan, teleop)
│   ├── visionft/              — Python sensors, visualization, utilities
│   │   ├── visionft/
│   │   │   ├── coinft.py               — CoinFT serial reader + ONNX calibration (360Hz)
│   │   │   ├── grid_visualizer.py       — Live 2D workspace classification grid
│   │   │   ├── wrench_plotter.py        — Live wrench plotting + CSV export
│   │   │   └── plot_csv.py             — Offline CSV plotting
│   │   ├── launch/
│   │   │   ├── visionft.launch.py       — Sensor stack (camera + wrench + CoinFT)
│   │   │   ├── scan.launch.py           — Launches C++ scan_controller + MCAP recording
│   │   │   └── record.launch.py         — Manual MCAP recording
│   │   └── config/
│   │       └── example_session.yaml     — Multi-scan session template
│   ├── inference/             — Tendon classification (spatial image-only CNN)
│   │   └── inference/
│   │       ├── inference_node.py        — ROS2 node: /image_raw → /tendon_class
│   │       ├── config.py               — Dataclass configs + YAML loader
│   │       ├── models_v2.py            — Model architectures (ResNet/DINOv2/CLIP)
│   │       ├── encoders.py             — Vision encoder wrappers
│   │       └── attention.py            — Fusion + temporal aggregation modules
│   ├── flexiv_ros2/           — Flexiv RDK bindings, examples, utility functions
│   ├── gscam2/                — GStreamer camera bridge (H264 UDP → /image_raw)
│   └── ptrmu/                 — ROS2 shared utilities
├── interview_demos/           — VR teleop demo (Quest 3S → ZMQ → Flexiv)
├── classifier/                — TendonClassifier training/labeling pipeline
├── scripts/
│   └── extract_mcap.py        — MCAP bag → CSV/images extraction
└── data/                      — Recorded session data (.mcap, .csv)
```

## Key Concepts

- **ArmCommander**: C++ class that owns the single RDK connection. All robot executables (floating_scan, scan_controller, teleop) use ArmCommander -- no direct RDK calls elsewhere.
- **Pose Formats**: RDK=[x,y,z,qw,qx,qy,qz], ROS2=[x,y,z] + quat(x,y,z,w), Elements=[mm,mm,mm,deg,deg,deg]
- **Scan State Machine**: HOMING → ZEROING_FT → DESCENDING → SCANNING → RETURNING → DONE
- **CoinFT**: Serial 360Hz → 1500-sample offset → ONNX calibration → bias-zeroed wrench
- **Inference**: Camera frame → crop/resize 224px → ImageNet normalize → spatial CNN → class (0-3)

## Conventions

- Python 3.10 (system), ROS2 Humble
- All robot commands go through ROS2 topics, never direct RDK from non-bridge nodes
- MCAP bags for data recording (not CSV loggers)
- Quaternion order: always document which convention (w-first vs w-last)
- Config: scan parameters via ROS2 params or YAML session files

## Golden Principles

1. **One fact, one place** — constants, config, shared logic have a single source of truth
2. **Centralize, don't duplicate** — extract repeated patterns into shared utilities
3. **Validate at boundaries** — assert shapes/types/ranges where data enters a module
4. **Fail loud, not silent** — log warnings, don't swallow errors
5. **Single RDK owner** — exactly one node owns the robot connection at any time
6. **Record with MCAP** — all data recording via ROS2 bag, not custom loggers

## How to Run

```bash
# Sensor stack (camera + force sensors)
ros2 launch visionft visionft.launch.py

# Hand-guided floating (manual control)
ros2 run floating_scan floating_scan <robot_config>

# Automated scan (auto-records, auto-exits)
ros2 run floating_scan scan_controller <scan_config> <robot_config>
# Or via launch file:
ros2 launch visionft scan.launch.py scan_config:=/path/to/scan.yaml robot_config:=/path/to/robot.yaml

# VR teleop (Quest 3S hand tracking)
ros2 run floating_scan teleop <teleop_config> <robot_config>

# Inference
ros2 run inference tendon_inference
```

## Deeper Context

| Topic | File |
|-------|------|
| Architecture & data flow | `ARCHITECTURE.md` |
| Design decisions | `docs/design-docs/design.md` |
| Golden principles (detailed) | `docs/design-docs/golden-principles.md` |
| Lessons learned | `docs/design-docs/lessons-learned.md` |
| ExecPlan template | `docs/exec-plans/plans.md` |
| Tech debt tracker | `docs/exec-plans/tech-debt-tracker.md` |
| Getting started | `docs/getting-started.md` |
| Quality scores | `docs/quality-score.md` |
