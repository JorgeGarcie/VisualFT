# Design Decisions

## Why ROS2 as the control layer (not direct RDK everywhere)

**Decision**: All robot commands flow through a single RDK bridge node. Other nodes publish
to `/rdk/cartesian_target`.

**Why**: Flexiv RDK allows only one connection at a time. Running multiple scripts that each
try to connect causes crashes. A single bridge node owns the connection and exposes it as
ROS2 topics. This also enables MCAP bag recording of all data streams in sync.

## Why MCAP bags over CSV loggers

**Decision**: `scan_node` manages MCAP bags internally. `data_logger.py` is legacy.

**Why**: data_logger.py had frame drops under load — it tried to write CSV + video in
callbacks. MCAP bags capture all topics at full rate with zero application-level overhead.
Per-scan bags enable clean per-pass post-processing.

## Why scan_node owns RDK directly (instead of using bridge)

**Decision**: `scan_node` connects to RDK itself rather than publishing to the bridge.

**Why**: The scan state machine needs primitives (MoveL, ZeroFTSensor) that require mode
switching (NRT_PLAN_EXECUTION → NRT_PRIMITIVE_EXECUTION → NRT_CARTESIAN_MOTION_FORCE).
The bridge only supports NRT_CARTESIAN_MOTION_FORCE. Extending the bridge to handle all
modes would add complexity for a single use case.

## Why CoinFT uses ONNX (not raw voltages)

**Decision**: CoinFT node loads a PFT5-1 ONNX model for voltage→force calibration.

**Why**: The sensor outputs raw ADC values. The manufacturer provides a calibration model
(trained neural net) that maps raw readings to calibrated forces/torques in SI units.

## Why spatial image-only model (no force fusion)

**Decision**: Current inference uses only camera frames, no force input.

**Why**: Ablation studies showed the spatial image-only model matched or exceeded force-fused
models for tendon classification. Simpler architecture, fewer failure modes, no dependency
on force sensor timing.

## Why ZMQ for teleop (not ROS2)

**Decision**: VR teleop uses ZMQ PUB/SUB for pose streaming.

**Why**: Quest 3S runs Android — no native ROS2. ZMQ is cross-platform, lightweight, and
adds minimal latency (~1ms serialization). ROS2 DDS overhead is unnecessary for point-to-point
streaming. A sensor bridge (ROS2 → ZMQ) sends camera/force data back to VR.
