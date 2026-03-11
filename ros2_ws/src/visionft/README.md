# VisualFT

ROS2 package for vision-based force/torque estimation using a camera, CoinFT sensor, and Flexiv Rizon4 robot.

## Overview

Combines tactile sensing (CoinFT), robot wrench/pose (Flexiv RDK), and a GStreamer camera stream to collect synchronized multimodal data for training and running force estimation models.

## Nodes

| Executable | Module | Description |
|---|---|---|
| `flexiv_wrench_publisher` | `robot_publisher.py` | Reads Flexiv robot states, publishes ext wrench and TCP pose |
| `visualft_coinft` | `coinft.py` | Reads CoinFT sensor over serial, runs ONNX model, publishes calibrated wrench |
| `tendon_inference` | `inference_node.py` | Runs the spatial tendon-classification model at camera framerate; publishes predicted class (0=none, 1=single, 2=crossed, 3=double) |
| `grid_visualizer` | `grid_visualizer.py` | Live 2D grid (100×28 cells, 2 mm resolution) coloured by tendon class as the TCP moves through the workspace |
| `wrench_plotter` | `wrench_plotter.py` | Live wrench plotting |
| `plot_csv` | `plot_csv.py` | Plots logged CSV data |
| `scan_node` | `scan_node.py` | Automated phantom scanning with Cartesian impedance control. Descends until contact, sweeps Y at multiple (rz, rx) rotations (round-trip zigzag per orientation), publishes scan state. |
| `rdk_cartesian_bridge` | `rdk_cartesian_bridge.py` | RDK bridge for NRT Cartesian motion-force control via ROS2 topics |
| `data_logger` | `data_logger.py` | (Legacy) Logs wrench, TCP pose, and camera frames to CSV/video. Prefer MCAP bags via `scan.launch.py`. |

## Topics

| Topic | Type | Publisher | Rate |
|---|---|---|---|
| `/coinft/wrench` | `WrenchStamped` | `visualft_coinft` | ~360 Hz |
| `/flexiv/wrench` | `WrenchStamped` | `flexiv_wrench_publisher` | 100 Hz |
| `/flexiv/tcp_pose` | `PoseStamped` | `flexiv_wrench_publisher` | 100 Hz |
| `/image_raw` | `Image` | `gscam2` (H264 via UDP) | ~30 Hz |
| `/tendon_class` | `Int32` | `tendon_inference` | ~30 Hz |
| `/tendon_class_name` | `String` | `tendon_inference` | ~30 Hz |
| `/scan/state` | `String` | `scan_node` | on transition |
| `/rdk/tcp_pose` | `PoseStamped` | `scan_node` / `rdk_cartesian_bridge` | 50 Hz |
| `/rdk/wrench` | `WrenchStamped` | `scan_node` / `rdk_cartesian_bridge` | 50 Hz |

## Launch Files

### `visionft.launch.py` — Start all sensor nodes

```bash
ros2 launch visionft visionft.launch.py
```

Starts: `gscam2` (camera), `flexiv_wrench_publisher`, `visualft_coinft`

### `record.launch.py` — Record a trial to MCAP bag

```bash
# Auto-named (timestamp)
ros2 launch visionft record.launch.py

# Custom trial name
ros2 launch visionft record.launch.py trial_name:=insertion_test_1

# Custom output directory
ros2 launch visionft record.launch.py output_dir:=/data/trials trial_name:=run_01
```

Saves to `~/VisualFT/data/<trial_name>/`. Ctrl+C to stop.

Recorded topics: `/coinft/wrench`, `/flexiv/wrench`, `/flexiv/tcp_pose`, `/image_raw`

### `scan.launch.py` — Automated scan + MCAP recording

```bash
ros2 launch visionft scan.launch.py
ros2 launch visionft scan.launch.py trial_name:=phantom_run_01
ros2 launch visionft scan.launch.py y_scan_range:=0.05 rz_end:=0.0 rx_end:=0.0
```

Starts `scan_node` which manages its own MCAP bag recording internally (one bag per scan). Exits automatically when all scans are done. See **Automated Phantom Scanning** below for full details.

## Typical Workflow

```bash
# Terminal 1 — sensors
source install/setup.bash
ros2 launch visionft visionft.launch.py

# Terminal 2 — recording
source install/setup.bash
ros2 launch visionft record.launch.py trial_name:=my_trial
```

## Real-Time Tendon Classification

Run the inference node and grid visualizer alongside the sensor stack to classify the tendon type under the camera in real time and paint a 2D workspace map.

> Inference node subscribes to `/image_raw` (published by gscam2).

```bash
# Terminal 3 — inference (needs /camera/image_raw + /coinft/wrench)
source install/setup.bash
ros2 run visionft tendon_inference --ros-args -p model_dir:=/home/li2053/tendon_classifier_inference

# Terminal 4 — grid visualizer (matplotlib window)
source install/setup.bash
ros2 run visionft grid_visualizer
```

### Inference node parameters

| Parameter | Default | Description |
|---|---|---|
| `model_dir` | `/home/li2053/tendon_classifier_inference` | Path to the directory containing `spatial_combined.yaml`, `best.pth`, `config.py`, and `models_v2.py` |
| `use_cuda` | `true` | Use GPU if available |

### Grid spec

| Axis | Range | Span | Cells |
|---|---|---|---|
| X | 0.370 – 0.570 m | 200 mm | 100 |
| Y | 0.245 – 0.300 m | 55 mm | 28 |

Cell size: 2 mm. Color legend: grey = unvisited, white = none, blue = single, orange = crossed, red = double.

## Automated Phantom Scanning

The `scan_node` performs fully automated scans: for each scan it homes the
robot, zeros FT, descends until contact, then sweeps Y at every (rz, rx)
orientation combination. Each orientation does a **full round-trip zigzag**:
forward sweep (+Y) then backward sweep (−Y) before rotating to the next
orientation. A 0.5 s settle pause is inserted at each orientation change.
MCAP bag recording is managed internally — one bag per scan, auto-named,
auto-start/stop.

A **session config** (YAML) defines one or more scans to run back-to-back.
Without a YAML, a single scan runs using ROS parameters.

### Quick start

```bash
# Terminal 1 — sensors (keep running across all scans)
ros2 launch visionft visionft.launch.py

# Terminal 2 — single scan with defaults (auto-records, auto-exits)
ros2 launch visionft scan.launch.py

# Terminal 2 — or session with multiple scans
ros2 launch visionft scan.launch.py session_config:=/path/to/session.yaml
```

Bags are saved to `~/VisualFT/data/<session_name>/<scan_idx>_<scan_name>/`.
The node exits automatically when all scans are done.

### Session config (YAML)

Define multiple scans with shared defaults and per-scan overrides.
See `config/example_session.yaml` for a full example.

```yaml
session_name: phantom_dataset_01

defaults:
  home_pose: [512, 260, 45, 0.05, -179.44, 0]
  y_scan_range: 0.10
  scan_speed: 0.01
  rz_end: 175.0
  rz_step: 5.0
  rx_end: 15.0
  rx_step: 5.0

scans:
  - name: full_sweep            # uses all defaults
  - name: fine_rotation         # override specific params
    rz_end: 45.0
    rz_step: 2.5
  - name: slow_highres
    scan_speed: 0.005
    rz_step: 2.0
```

This produces:
```
~/VisualFT/data/phantom_dataset_01/
  000_full_sweep/          ← MCAP bag
  001_fine_rotation/       ← MCAP bag
  002_slow_highres/        ← MCAP bag
```

### Single-scan mode (no YAML)

```bash
# Override params directly
ros2 launch visionft scan.launch.py y_scan_range:=0.05 rz_end:=0.0 rx_end:=0.0

# Disable recording for dry-run testing
ros2 run visionft scan_node --ros-args -p record:=false -p rz_end:=0.0 -p rx_end:=0.0
```

### Scan parameters

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `robot_sn` | `Rizon4-062174` | — | Robot serial number |
| `home_pose` | `[512,260,45,0.05,-179.44,0]` | mm, deg | Elements-format home pose |
| `y_scan_range` | `0.10` | m | Sweep distance along +Y |
| `scan_speed` | `0.01` | m/s | TCP speed during sweep |
| `contact_force` | `5.0` | N | Force threshold for contact detection |
| `search_velocity` | `0.02` | m/s | Descent speed for contact search |
| `rz_start` / `rz_end` / `rz_step` | `0 / 175 / 5` | deg | TCP rotation around Z |
| `rx_start` / `rx_end` / `rx_step` | `0 / 15 / 5` | deg | TCP rotation around X |
| `xy_stiffness` | `5000.0` | N/m | Cartesian impedance stiffness XY (holds scan line) |
| `z_stiffness` | `2000.0` | N/m | Cartesian impedance stiffness Z (surface compliance) |
| `rot_stiffness` | `800.0` | Nm/rad | Cartesian impedance rotational stiffness |
| `damping_ratio` | `0.8` | — | Impedance damping ratio (high = no bounce) |
| `movel_vel` | `0.05` | m/s | Speed for MoveL primitives (homing/returning) |
| `max_linear_vel` | `0.05` | m/s | Max linear velocity during scanning |
| `max_angular_vel` | `0.3` | rad/s | Max angular velocity during scanning |
| `max_linear_acc` | `0.3` | m/s² | Max linear acceleration |
| `max_angular_acc` | `0.5` | rad/s² | Max angular acceleration |
| `control_rate_hz` | `50.0` | Hz | Control loop frequency |
| `session_config` | `""` | path | YAML config for multi-scan sessions |
| `output_dir` | `~/VisualFT/data` | path | Where bags are saved |
| `record` | `true` | — | Enable/disable MCAP recording |

### State machine (per scan)

```
HOMING → ZEROING_FT → DESCENDING → SCANNING → RETURNING → (next scan or DONE)
```

**SCANNING pass structure** (zigzag Y, round-trip per orientation):
```
for each (rz, rx) orientation:
    forward sweep  → home_y to home_y + y_scan_range
    backward sweep → home_y + y_scan_range to home_y
    [0.5 s settle pause before next orientation]
```

`/scan/state` publishes:
- `homing`, `zeroing_ft`, `descending`, `returning`, `done`
- `scanning_rz{N}_rx{N}_fwd` — forward pass in progress
- `scanning_rz{N}_rx{N}_bwd` — backward pass in progress
- `pass_done_rz{N}_rx{N}` — one direction of a pass completed

### MCAP bag contents

| Topic | Type | Source |
|---|---|---|
| `/scan/state` | `String` | scan_node — state + pass info |
| `/rdk/tcp_pose` | `PoseStamped` | scan_node — 50 Hz |
| `/rdk/wrench` | `WrenchStamped` | scan_node — 50 Hz |
| `/coinft/wrench` | `WrenchStamped` | coinft — ~360 Hz |
| `/image_raw` | `Image` | gscam2 — ~30 Hz |

```bash
ros2 bag info ~/VisualFT/data/phantom_dataset_01/000_full_sweep/
ros2 bag play ~/VisualFT/data/phantom_dataset_01/000_full_sweep/
```

## Hardware

- **Robot**: Flexiv Rizon4 (`Rizon4-062174`)
- **FT Sensor**: CoinFT on `/dev/ttyACM1` at 1000000 baud
- **Camera**: H264 stream via UDP port 5000, decoded by gscam2
