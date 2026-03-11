# Workspace Change Log

## 2026-02-24 — Automated Phantom Scan Node + Session-Based MCAP Recording

### What was added

**`visionft/scan_node.py`** (new)
Session-based automated phantom scanning with Cartesian impedance control.
Reads a YAML session config that defines one or more scans. For each scan:
HOMING → ZEROING_FT → DESCENDING → SCANNING → RETURNING.

**Scanning pass structure**: each (rz, rx) orientation does a full round-trip
— forward sweep then backward sweep — before rotating to the next orientation.
A 0.5 s settle pause is inserted at each orientation change (robot holds
position via impedance while rotating in place).

**Impedance control**: DESCENDING uses pure motion control (contact_force=5N
threshold). On contact, Z position is recorded and the node switches to
Cartesian impedance (no explicit force command). Stiffness: XY=5000 N/m,
Z=2000 N/m, rotation=800 Nm/rad, damping=0.8. The robot follows the surface
compliantly in Z while holding the scan line stiffly in XY.

Manages its own `ros2 bag record` subprocess per scan — one MCAP bag per scan,
auto-named, auto-start/stop. Exits when all scans are done.
Without a YAML, runs a single scan from ROS parameters (backward compat).

**`launch/scan.launch.py`** (new)
Launch file for scan_node.  Passes through scan params for single-scan mode
or a `session_config` path for multi-scan sessions.

**`config/example_session.yaml`** (new)
Sample session config demonstrating defaults + per-scan overrides.

### What was modified

**`setup.py`**
- Added `scan_node` entry point
- Added `scan.launch.py` to launch files
- Added `config/` directory to installed data files

**`README.md`**
- Added scan_node, rdk_cartesian_bridge to Nodes table
- Added /scan/state, /rdk/tcp_pose, /rdk/wrench to Topics table
- Added scan.launch.py to Launch Files section
- Added full "Automated Phantom Scanning" section: session config format,
  single-scan mode, parameters, state machine, MCAP bag contents

### What was NOT changed

**`data_logger.py`** — left as-is (legacy). Use MCAP bags via scan_node for
reliable data collection.

### How to run

```bash
# Terminal 1 — sensors (keep running)
ros2 launch visionft visionft.launch.py

# Terminal 2 — single scan (auto-records, auto-exits)
ros2 launch visionft scan.launch.py

# Terminal 2 — session (multiple scans back-to-back)
ros2 launch visionft scan.launch.py session_config:=/path/to/session.yaml
```

### Output structure

```
~/VisualFT/data/
  phantom_dataset_01/            ← session_name from YAML
    000_full_sweep/              ← MCAP bag (scan 1)
    001_fine_rotation/           ← MCAP bag (scan 2)
    002_slow_highres/            ← MCAP bag (scan 3)
```

### Design decisions

- **MCAP over CSV**: `data_logger.py` had data drops. `ros2 bag record` with
  MCAP handles I/O natively with a 500 MB write cache.
- **Node-managed recording**: scan_node starts/stops `ros2 bag record` as a
  subprocess per scan. No separate launch process needed.
- **Session YAML**: Defines multiple scans with shared defaults + per-scan
  overrides. One command → all scans → all bags. No manual naming.
- **Auto-exit**: Node exits after all scans complete. No Ctrl+C needed.
- **scan_node owns the RDK connection**: Kill other RDK nodes first.
- **Cartesian impedance (not force control)**: Descend to contact, then scan
  at contact Z with impedance. No explicit pressing force — spring compliance
  follows surface naturally. Stiff XY holds scan line. High damping (0.8)
  prevents oscillation on silicone surfaces.
- **Round-trip per orientation**: Each (rz, rx) pair does forward + backward
  before rotating. Orientation changes pause 0.5 s for impedance to settle.
- **No SwitchMode between DESCENDING and SCANNING**: Both use
  NRT_CARTESIAN_MOTION_FORCE. Scanning updates SetCartesianImpedance in-place,
  then waits 0.5 s before sending motion targets.
