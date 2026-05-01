# Architecture

> Technical reference for how modules connect. Diagrams, integration protocols, dependency rules. For repo map, conventions, and run commands see `CLAUDE.md`.

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
         ┌────────────┬─────────────┬─────────────┐
         │            │             │             │
   ┌─────▼──────┐ ┌──▼──────────┐ ┌▼──────────┐ ┌▼─────────┐
   │floating_scan│ │scan_controller│ │  teleop   │ │  massage │
   │(hand-guided │ │(auto scan    │ │(VR hand   │ │(const Z  │
   │ joint float)│ │ state machine│ │ tracking) │ │ force)   │
   └─────────────┘ │ + MCAP rec)  │ └───────────┘ └──────────┘
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
 ┌──────────┐    V4L2       ┌─────────────┐  /image_raw   ┌──────────────┐  /tendon_class
 │ USB Cam  │ ──────────► │ usb_camera  │ ──────────► │tendon_classif│ ──────────►
 └──────────┘              └─────────────┘              └──────────────┘

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
         floating_scan    scan_controller       teleop          massage
         (manual)         (automated)     (VR, Quest 3S→ZMQ)  (const Z force)

                             ┌─────────────┐
                    All ──► │  MCAP Bag    │  (ros2 bag record or scan_controller)
                             └─────────────┘
```

## Integration Points

| Boundary | Protocol | Rate | Format |
|----------|----------|------|--------|
| USB Camera → usb_camera | V4L2 (/dev/video2) | ~30 Hz | OpenCV → sensor_msgs/Image |
| CoinFT → coinft.py | Serial | 300 Hz | Raw bytes → ONNX calibration |
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

- **C++ for robot control, Python for sensors/ML.** ROS2 topics bridge the two worlds.
- **arm_commander owns the RDK connection.** No `flexivrdk` imports outside this library — all robot commands flow through it. The Flexiv SDK only allows one client connection, so this is enforced both architecturally and by the SDK itself.
- **robot_behaviors** executables (floating_scan, scan_controller, teleop, massage) link against arm_commander.
- **tendon_classifier** depends only on `/image_raw` — no force data.
- **coinft** and **usb_camera** are standalone — no dependency on robot state.
