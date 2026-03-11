# ExecPlan: ArmCommander Migration

**Status**: In Progress (pivoted to C++)
**Created**: 2026-03-10
**Author**: li2053
**Motivated by**: Floating mode (admittance control) is the next feature. Building it
against the current architecture would create a 4th separate RDK connection path.
This is the turning point to consolidate.

## Goal

Build `ArmCommander` as the single robot interface in C++, use it for floating mode first,
then migrate existing behavior scripts incrementally.

## Context

See `docs/exec-plans/active/target-architecture.md` for the full architecture reference
(layer diagram, abstractions, file structure, testing strategy).

This plan is the **concrete implementation steps** — what to build, in what order.

**2026-03-11 pivot**: Original plan was pure Python. Hardware testing revealed that
the Float primitive does not exist in RDK v1.7. Floating requires `RT_JOINT_TORQUE`
mode, which is only available through the C++ real-time scheduler at 1kHz. There is
no reason to use Python for robot control — all behavior scripts move to C++.
Python stays only for inference (PyTorch/ONNX), CoinFT (ONNX calibration), and camera.

## Architecture Summary

```
Script (floating_scan / scan_controller / teleop) — C++ executables
  │
  │ behavior logic
  ▼
ArmCommander (C++ class, header + source in floating_scan or own library)
  ├── SafetyChecker (per-command workspace check)
  ├── SafetyWatchdog (own thread, heartbeat + force/fault logging)
  └── flexiv::rdk::Robot (single connection, mutex-protected)
```

Key design decisions:
- ArmCommander owns the single RDK connection
- `std::mutex` protects all RDK access (watchdog + main thread)
- SafetyChecker validates every command before sending
- SafetyWatchdog runs independently, catches faults/force/heartbeat
- Force/fault watchdog is intentionally redundant with lower-level protections
- C++ class, NOT a ROS2 node (testable without ROS2)
- ArmCommander is decoupled from CoinFT and other non-robot sensors
- Sensor readiness is handled by the application layer before starting behavior
- Config loaded from `robot.yaml` via yaml-cpp

**Language boundary**:
- C++: all robot control (ArmCommander, behavior scripts, safety)
- Python: inference node (PyTorch/ONNX), CoinFT node (ONNX calibration), camera node (gscam2)
- ROS2 topics bridge the two worlds (wrench, pose, image, classification)

## Step 1: C++ ArmCommander class ✓

Build the ArmCommander as a C++ class in `ros2_ws/src/floating_scan/`.
May later be extracted to its own library package if shared across multiple executables.

- [x] `include/arm_commander/arm_commander.hpp` + `src/arm_commander.cpp`
  - Full API: connect, shutdown, home, zero_ft, contact, move_to, set_tool,
    stream_cartesian, set_impedance, set_max_contact_wrench, set_force_control_axis,
    float_joints (RT_JOINT_TORQUE + 1kHz scheduler), get_state, stop
  - `RobotState` struct defined inline (no separate types header needed)
  - Workspace check inline (`check_workspace()`) — stops robot on violation
  - `request_stop()` / `stop_was_requested()` for signal handler integration

- [x] `include/arm_commander/config.hpp` + `src/config.cpp`
  - `RobotConfig`, `SafetyConfig`, `WorkspaceBounds` structs
  - `load_config()` from YAML via yaml-cpp

- [ ] SafetyWatchdog (background thread)
  - Not yet ported to C++ — workspace check works but no background monitoring
  - Needed for: heartbeat timeout during streaming, force/fault logging
  - Lower priority than getting floating working on hardware

- [ ] Move `config/robot.yaml` into `floating_scan/config/` or shared location
  - Currently at `ros2_ws/src/common/config/robot.yaml` (inside dead Python package)
  - Values are correct (from tested teleop safety_config.py)

## Step 2: floating_scan executable

Built at `ros2_ws/src/floating_scan/src/floating_scan.cpp`. Compiles and installs.
Uses ArmCommander for the full sequence. Reads serial number from config (not CLI arg).

- [x] Connect, home, zero FT via ArmCommander
- [x] Enter float: `RT_JOINT_TORQUE`, gravity comp, velocity damping
- [x] Ctrl+C → SIGINT → `request_stop()` → scheduler stops → shutdown
- [x] Damping: joints 1-3 low (10,10,5), joints 4-7 high (20,20,20,20) — tune on hardware
- [x] **Hardware test** — float mode validated on hardware (requires sudo for RT scheduler)
- [ ] set_tool for CoinFT (when params available)
- [ ] MCAP recording (separate launch or wrapper script)
- [ ] Damping tuning (functional, values TBD)

## Step 3: scan_controller in C++ ✓

- [x] Port scan state machine (HOMING → ZEROING_FT → DESCENDING → SCANNING → RETURNING → DONE) to C++
- [x] Uses ArmCommander for all robot interaction
- [x] Descent uses Contact primitive (not manual stream+poll)
- [x] Scan session config via yaml-cpp (`scan_config.hpp` + `scan_config.cpp`)
- [x] MCAP bag recording subprocess management (fork/exec, SIGINT to stop)
- [x] Multi-scan session support (same as Python scan_node)
- [x] Extended ArmCommander: `wrench_in_world` in RobotState, velocity overrides in `stream_cartesian`
- [x] Example config at `floating_scan/config/scan.yaml`
- [x] Builds and installs as `scan_controller` executable
- [x] **Hardware test** — validated, matches Python scan_node behavior
- [ ] Retire old `scan_node.py` (keeping until confidence builds)

## Step 4: teleop in C++

- [ ] Port teleop from `interview_demos/teleop/` to C++ using ArmCommander
- [ ] **Known risk**: Quest hand tracking currently uses Python pickle over ZMQ
  - Option A: Change serialization to msgpack or protobuf (both have C++ and Python libs)
  - Option B: Keep a thin Python ZMQ→ROS2 bridge that republishes hand poses as ROS2 messages, C++ teleop subscribes
  - Option B is simpler, Option A is cleaner

## Step 5: Remove dead Python code

- [ ] Delete `ros2_ws/src/common/` (Python types, config, errors)
- [ ] Delete `ros2_ws/src/flexiv_driver/` (Python arm_commander, safety)
- [ ] Delete `visionft/visionft/floating_scan.py` (superseded by C++ executable)
- [ ] Delete `tests/unit/test_safety.py`, `tests/unit/test_arm_commander.py`
- [ ] Clean up `setup.py` / `package.xml` references

## Superseded Python steps (for reference)

The original plan had Steps 1-4 in Python. These are now dead code:

- ~~Step 1: `common/types.py`, `common/config.py`~~ → replaced by C++ `types.hpp`, `config.hpp`
- ~~Step 2: `flexiv_driver/arm_commander.py`~~ → replaced by C++ `ArmCommander`
- ~~Step 3: `flexiv_driver/safety.py`~~ → replaced by C++ `safety.hpp`
- ~~Step 4: `tests/mock_arm.py`, `test_safety.py`, `test_arm_commander.py`~~ → dead code
- ~~Step 5: `visionft/floating_scan.py`~~ → replaced by C++ `floating_scan.cpp`

## Progress

| Date | Update |
|------|--------|
| 2026-03-10 | Architecture designed, plan written (Python) |
| 2026-03-11 | Step 1 implemented in Python; ArmCommander core, safety, minimal tests |
| 2026-03-11 | **PIVOT**: Float primitive not in RDK v1.7. Floating requires RT_JOINT_TORQUE (C++ only, 1kHz). All robot control moves to C++. Python steps become dead code. |
| 2026-03-11 | C++ ArmCommander class complete. floating_scan builds. First hardware test: ZeroFTSensor hung because busy() doesn't work for primitives. Fixed with wait_primitive() polling primitive_states(). Also fixed move_to() quaternion→Euler bug. Audit confirmed all RDK calls needed for Steps 2-4 are covered. |
| 2026-03-11 | **Steps 1-2 validated on hardware.** Float mode works: home → zero FT → RT_JOINT_TORQUE with gravity comp + velocity damping. Requires `sudo` for RT scheduler thread priority. Damping values still need tuning. |
| 2026-03-11 | **Step 3 validated on hardware.** scan_controller runs full state machine. Fixed: workspace bounds for scan region, mode switching before impedance config, replaced manual descent with Contact primitive. Matches Python scan_node behavior. |

## Decision Log

| Decision | Rationale | Date |
|----------|-----------|------|
| ~~ArmCommander is pure Python, not ROS2 node~~ | ~~Testable without ROS2~~ | ~~2026-03-10~~ |
| **ArmCommander is C++** | RT_JOINT_TORQUE requires C++ 1kHz scheduler. No reason to keep Python for robot control. | 2026-03-11 |
| threading/mutex on all RDK access | Watchdog + main thread share connection, RDK thread safety unknown | 2026-03-10 |
| Safety built into ArmCommander, not separate wrapper | Simpler API, can't bypass, watchdog needs same robot reference | 2026-03-10 |
| Build floating mode first against ArmCommander | Real feature validates the abstraction, not a dry refactor | 2026-03-10 |
| Heartbeat timeout applies only to streaming Cartesian control | Blocking primitives like Home and MoveL run for seconds; must not false-stop | 2026-03-11 |
| Force/fault watchdog is intentionally redundant | Independent monitoring provides logging and observability on top of Flexiv's built-in stop | 2026-03-11 |
| Program SDK contact-wrench limits in Cartesian streaming mode | Flexiv regulation should be configured proactively | 2026-03-11 |
| CoinFT readiness stays outside ArmCommander | ArmCommander is the robot driver boundary; sensor gating belongs to the application layer | 2026-03-11 |
| Config via yaml-cpp, not ROS2 params | ArmCommander is not a ROS2 node; yaml-cpp keeps it standalone and testable | 2026-03-11 |
| Python stays for inference, CoinFT, camera only | These nodes use PyTorch/ONNX and have no RT requirements | 2026-03-11 |
| All behavior scripts (float, scan, teleop) are C++ | No advantage to Python when robot control is C++; single language for control stack | 2026-03-11 |
| `common/` and `flexiv_driver/` Python packages are dead code | Superseded by C++ ArmCommander; will be removed in Step 5 | 2026-03-11 |
| Wrench frame inconsistency (TCP vs world) | `rdk_cartesian_bridge` publishes TCP frame, `scan_node` publishes world frame on similar topics. Fix when building C++ ROS2 publisher layer: publish both on separate topics (`/rdk/wrench_tcp`, `/rdk/wrench_world`) with frame_id in header. | 2026-03-11 |

## Surprises

| Surprise | Impact | Date |
|----------|--------|------|
| Float primitive does not exist in RDK v1.7 | Floating requires `RT_JOINT_TORQUE` mode (C++, 1kHz real-time scheduler). This forced the entire pivot from Python to C++ for robot control. The Python ArmCommander, safety, types, config, and tests are all dead code now. | 2026-03-11 |
| `busy()` doesn't work for primitives | SDK says "most primitives won't exit by themselves" — `busy()` stays true forever. Must poll `primitive_states()["terminated"]` or `["reachedTarget"]`. Caused ZeroFTSensor to hang on first hardware test. Fixed with `wait_primitive()` helper. | 2026-03-11 |
| RT scheduler requires sudo | `SCHED_FIFO` for 1kHz loop needs root. Fix: either `setcap cap_sys_nice=ep` on binary (re-run after each build) or add `rtprio 99` + `memlock unlimited` to `/etc/security/limits.conf` for user. | 2026-03-11 |

## Outcomes

**Result**: (filled when floating mode works on C++ ArmCommander)
**Follow-ups**: scan migration (C++), teleop migration (C++ + pickle risk), dead Python cleanup
