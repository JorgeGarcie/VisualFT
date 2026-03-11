# Architecture

## Module Dependency Graph

```
                    ┌─────────────┐
                    │  Flexiv RDK  │  (C++ library, one connection only)
                    └──────┬───────┘
                           │ exclusive
              ┌────────────┴────────────┐
              │                         │
    ┌─────────▼──────────┐   ┌─────────▼──────────┐
    │ rdk_cartesian_bridge│   │     scan_node       │  ← mutually exclusive
    │  (50Hz hold/cmd)   │   │ (state machine +    │
    │                    │   │  MCAP recording)     │
    └─────────┬──────────┘   └─────────┬───────────┘
              │ publishes               │ publishes
              ▼                         ▼
     /rdk/tcp_pose              /rdk/tcp_pose
     /rdk/wrench                /rdk/wrench
     /rdk/cartesian_target ◄─── (subscribes)   /scan/state
```

## Data Flow

```
 ┌──────────┐   H264/UDP    ┌────────┐  /image_raw   ┌──────────────┐  /tendon_class
 │ RPi Cam  │ ──────────► │ gscam2 │ ──────────► │inference_node│ ──────────►
 └──────────┘              └────────┘              └──────────────┘

 ┌──────────┐   Serial 1Mbps ┌─────────┐  /coinft/wrench
 │ CoinFT   │ ────────────► │coinft.py│ ──────────────►
 └──────────┘                └─────────┘

 ┌──────────┐   RDK C++ API  ┌─────────────────────┐  /rdk/wrench
 │Flexiv Arm│ ◄────────────►│rdk_cartesian_bridge │ ──────────────►
 └──────────┘                │  OR scan_node       │  /rdk/tcp_pose
                             └─────────────────────┘
                                    ▲
                                    │ /rdk/cartesian_target
                             ┌──────┴──────┐
                             │ User / Teleop│
                             └─────────────┘

                             ┌─────────────┐
                    All ──► │  MCAP Bag    │  (ros2 bag record or scan_node internal)
                             └─────────────┘
```

## Integration Points

| Boundary | Protocol | Rate | Format |
|----------|----------|------|--------|
| RPi Camera → gscam2 | UDP H264 | ~30 Hz | GStreamer pipeline |
| CoinFT → coinft.py | Serial | 360 Hz | Raw bytes → ONNX calibration |
| Flexiv RDK ↔ Bridge | C++ API (Ethernet) | 50 Hz | [x,y,z,qw,qx,qy,qz] + [fx,fy,fz,tx,ty,tz] |
| ROS2 inter-node | DDS | varies | PoseStamped, WrenchStamped, Image, String, Int32 |
| VR Teleop → Bridge | ZMQ (WiFi) | ~60 Hz | JSON poses via PUB/SUB |

## Dependency Rules

- **rdk_cartesian_bridge** and **scan_node** are mutually exclusive — never run both
- **robot_publisher** is legacy — do not run alongside bridge or scan_node
- **inference** depends only on `/image_raw` (no force data)
- **coinft** is standalone — no dependency on robot state
- **gscam2** is standalone — no dependency on robot state
- No ROS2 node may import `flexivrdk` directly except bridge and scan_node
