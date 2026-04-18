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
           └──── robot_behaviors package ─────┘
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

- **C++**: All robot control (`arm_commander` library + `robot_behaviors` executables)
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

## ZMQ Serialization (Python → C++)

The VR teleop pipeline crosses the language boundary via ZMQ with **raw bytes** — no
pickle, no protobuf, just `numpy.tobytes()` on the Python side and `std::memcpy()` on C++.

**Wire format:** each message is an ASCII topic prefix (space-terminated) followed by a
fixed-size binary payload.

| Topic | Payload | Size |
|-------|---------|------|
| `transformed_hand_frame ` | 12 × `float64` (origin, x/y/z axes) | 96 bytes |
| `pause ` | 1 × `uint8` (0 = STOP, 1 = CONT) | 1 byte |

**Python (publish):**
```python
prefix = f'{topic} '.encode('utf-8')
buffer = np.asarray(data, dtype=np.float64).ravel().tobytes()   # or struct.pack('B', val)
sock.send(prefix + buffer)
```

**C++ (subscribe):**
```cpp
double data[12];
std::memcpy(data, static_cast<const char*>(msg.data()) + prefix_len, 12 * sizeof(double));
```

**Why raw bytes instead of pickle/protobuf:**
- Zero-copy compatible — `memcpy` straight into Eigen vectors on the C++ side
- No serialization library needed across the language boundary
- Fixed-size payloads make validation trivial (prefix check + size check)
- ZMQ `CONFLATE` drops stale frames automatically, so only the newest hand pose arrives

**Pipeline:**
```
Quest 3S → vr_server.py (PUB raw bytes, ports 8089/8102)
         → teleop.cpp   (SUB, memcpy → Eigen, Cartesian impedance commands)
```

## Dependency Rules

- **arm_commander** package is the single RDK owner — no direct RDK calls outside this library
- **robot_behaviors** package contains C++ executables (floating_scan, scan_controller, teleop) that link against arm_commander
- **inference** depends only on `/image_raw` (no force data)
- **coinft** is standalone — no dependency on robot state
- **gscam2** is standalone — no dependency on robot state
- No Python node imports `flexivrdk` — all robot control is C++
