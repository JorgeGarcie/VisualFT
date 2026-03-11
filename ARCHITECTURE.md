# Architecture

## Module Dependency Graph

```
                    ┌─────────────┐
                    │  Flexiv RDK  │  (C++ library, one connection only)
                    └──────┬───────┘
                           │ exclusive
                    ┌──────▼───────────┐
                    │  ArmCommander    │  (C++ class, owns RDK connection)
                    │  config, safety, │
                    │  all primitives  │
                    └──────┬───────────┘
                           │ used by
              ┌────────────┼────────────┐
              │            │            │
    ┌─────────▼──────┐ ┌──▼──────────┐ ┌▼────────────┐
    │ floating_scan  │ │scan_controller│ │   teleop    │
    │ (hand-guided   │ │(auto scan    │ │(VR hand     │
    │  joint float)  │ │ state machine│ │ tracking)   │
    └────────────────┘ │ + MCAP rec)  │ └─────────────┘
                       └──────┬───────┘
                              │ publishes
                              ▼
                       /rdk/tcp_pose
                       /rdk/wrench
                       /scan/state
```

## Data Flow

```
 ┌──────────┐   H264/UDP    ┌────────┐  /image_raw   ┌──────────────┐  /tendon_class
 │ RPi Cam  │ ──────────► │ gscam2 │ ──────────► │inference_node│ ──────────►
 └──────────┘              └────────┘              └──────────────┘

 ┌──────────┐   Serial 1Mbps ┌─────────┐  /coinft/wrench
 │ CoinFT   │ ────────────► │coinft.py│ ──────────────►
 └──────────┘                └─────────┘

 ┌──────────┐   RDK C++ API  ┌─────────────────┐  /rdk/wrench
 │Flexiv Arm│ ◄────────────►│  ArmCommander   │ ──────────────►
 └──────────┘                │  (C++ class)    │  /rdk/tcp_pose
                             └────────┬────────┘
                                      │ used by
                    ┌─────────────────┼──────────────────┐
                    │                 │                   │
              floating_scan    scan_controller        teleop
              (manual)         (automated)      (VR, Quest 3S → ZMQ)

                             ┌─────────────┐
                    All ──► │  MCAP Bag    │  (ros2 bag record or scan_controller)
                             └─────────────┘
```

## Language Boundary

- **C++**: All robot control (ArmCommander, floating_scan, scan_controller, teleop)
- **Python**: Sensors and ML only (CoinFT, gscam2, inference, grid_visualizer, wrench_plotter)
- **Bridge**: ROS2 topics connect C++ and Python worlds

## Integration Points

| Boundary | Protocol | Rate | Format |
|----------|----------|------|--------|
| RPi Camera → gscam2 | UDP H264 | ~30 Hz | GStreamer pipeline |
| CoinFT → coinft.py | Serial | 360 Hz | Raw bytes → ONNX calibration |
| Flexiv RDK ↔ ArmCommander | C++ API (Ethernet) | 1kHz RT / 50Hz NRT | [x,y,z,qw,qx,qy,qz] + [fx,fy,fz,tx,ty,tz] |
| ROS2 inter-node | DDS | varies | PoseStamped, WrenchStamped, Image, String, Int32 |
| VR Teleop → teleop | ZMQ (WiFi) | ~60 Hz | Hand poses via PUB/SUB |

## Dependency Rules

- **ArmCommander** is the single RDK owner — no direct RDK calls outside this class
- **floating_scan**, **scan_controller**, **teleop** are C++ executables that use ArmCommander
- **inference** depends only on `/image_raw` (no force data)
- **coinft** is standalone — no dependency on robot state
- **gscam2** is standalone — no dependency on robot state
- No Python node imports `flexivrdk` — all robot control is C++
