# TODOs

Open follow-ups across the project. Inline `// TODO:` comments in code remain the source of truth — this doc is the index.

## Robot control (arm_commander / robot_behaviors)

- **Register CoinFT as a tool** — `arm_commander/src/arm_commander.cpp:157`. Add mass / CoM / inertia in `robot.yaml` so gravity compensation works in floating mode and Cartesian impedance control.
- **Tune floating_scan damping on hardware** — `robot_behaviors/src/floating_scan.cpp:69`. Current `{10,10,10,2,2,2,2}` (J1–J3 shoulder/elbow, J4–J7 wrist) was best of the values tried in sim; needs hardware validation.

## Sensors (visionft)

- **CoinFT stale-detection limit untested on hardware** — `visionft/sensors/coinft.py:32`. `STALE_IDENTICAL_LIMIT = 50` (~167 ms at 300 Hz) is a guess; verify against real sensor noise floor before trusting the "frozen" signal.

## Teleop / VR streams

- **Restore force-tier colors in tactile_stream** — currently overrides Fz overlay text to plain white; original tiers were green <5 N / yellow <15 N / red ≥15 N.
- **Widen teleop force threshold** — `teleop.yaml` uses 25 N. leapft uses 40 N pause / 80 N estop. Decide and align after a few teleop sessions.
- **Revisit teleop workspace bounds** — `robot.yaml` `safety.workspace` is intentionally conservative (clips reachable area). Once early sessions confirm a safe envelope, widen the box so operators aren't fighting the bounds. Documented under "Known limitations" in `docs/getting-started-teleop.md`.

## Per-package metadata

- Replace `TODO: License declaration` and `TODO: Package description` placeholders in `package.xml` / `setup.py` for `arm_commander`, `robot_behaviors`, `visionft`, `tendon_classifier`.

## Documentation

- **Golden-principles audit** — sweep all source files against `docs/golden-principles.md` and record violations.
- **Handoff summary for next agent** — capture the final state of this cleanup pass once everything else is settled.

## Audit findings (2026-04-30)

### GP1: One fact, one place

- **`arm_commander/include/arm_commander/config.hpp:15-17` and `robot_behaviors/config/robot.yaml:11-13`** — Workspace bounds are hardcoded as defaults in `WorkspaceBounds` struct AND live in `robot.yaml`. The struct defaults are the fallback when the YAML key is absent, so both places must be kept in sync. Should remove hardcoded defaults entirely and require the YAML to be explicit. **Severity: med**

- **`arm_commander/include/arm_commander/config.hpp:33` and `robot_behaviors/config/robot.yaml:2`** — Robot serial number `Rizon4-062174` appears as a C++ struct default AND in `robot.yaml`. The default creates a silent no-op if the YAML key is accidentally omitted. Remove the default; require serial_number to be present and non-empty in the YAML. **Severity: med**

- **`robot_behaviors/include/robot_behaviors/teleop_config.hpp:21-22` (defaults 8089/8102) and `robot_behaviors/scripts/vr_server.py:54,58` (constants `TRANSFORMED_PORT`/`RESET_PUB_PORT`) and `robot_behaviors/config/teleop.yaml:8-9`** — ZMQ ports 8089 and 8102 are defined three times: as C++ struct defaults, as Python module constants, and in `teleop.yaml`. A port change requires three edits. Extract to a single shared config or at minimum remove the C++ struct defaults and require the YAML. **Severity: med**

- **`robot_behaviors/src/massage.cpp:32-33`** — `PRESSING_FORCE = 5.0 N` and `LOOP_RATE_HZ = 50.0` are compile-time constants not exposed in any config file. Users cannot change them without recompiling. Move both to `robot.yaml` under a `massage:` section. **Severity: low**

### GP2: Centralize, don't duplicate

- **`robot_behaviors/src/scan_controller.cpp:53-62`, `teleop.cpp:45-54`, `floating_scan.cpp:27-38`, `massage.cpp:35-46`** — All four executables contain an identical `g_commander` atomic pointer + `SignalHandler` + `std::signal(SIGINT/SIGTERM)` pattern. Extract to a shared utility (e.g., `robot_behaviors/src/signal_utils.hpp`) or to `ArmCommander` itself. **Severity: low**

### GP3: Validate at boundaries

- **`robot_behaviors/src/teleop_config.cpp:103-104`** — `pos_min` and `pos_max` are loaded but never validated that `pos_min < pos_max` per axis. An inverted safety box silently permits the robot to move anywhere (workspace check always fails). Add element-wise comparison and throw. **Severity: high**

- **`visionft/sensors/usb_camera.py:27-28`** — If `cv2.VideoCapture` fails to open, the node logs an error and `return`s from `__init__`, leaving the node in a half-initialized state with no publisher. The timer is never created, so no frames are published, but the node starts and `rclpy.spin` runs indefinitely without error. Should raise an exception so the launch system sees the failure. **Severity: med**

- **`visionft/viz/wrench_plotter.py:66-68`** — CSV file is opened unconditionally in `__init__` with no error handling. If the file path is not writable the exception propagates unlogged and kills the node. Wrap in `try/except` and log clearly. **Severity: low**

### GP4: Fail loud, not silent

- **`visionft/sensors/coinft.py:94`** — Watchdog timer is disabled: `# self.create_timer(0.2, self._watchdog_check)`. The `_watchdog_check` method and `WATCHDOG_TIMEOUT_S` constant exist and work, but are never scheduled. A serial stall or sensor freeze will not produce an ERROR state without the stale-data check hitting the `STALE_IDENTICAL_LIMIT` (which is already tracked as untested). Re-enable the watchdog timer. **Severity: high**

- **`robot_behaviors/scripts/vr_server.py:197-203`** — In `run_detector()`, `raw_sock.recv()`, `button_sock.recv()`, and `reset_sock.recv()` are blocking PULL calls with no `RCVTIMEO` set. If the Quest APK stops sending (e.g., headset goes to sleep), the detector process hangs indefinitely and never unblocks, so `KeyboardInterrupt` is not handled and the watchdog `p.is_alive()` loop in `main()` never detects a stall. Set `zmq.RCVTIMEO` on PULL sockets or use `zmq.Poller`. **Severity: med**

### GP5: Record with MCAP

- **`visionft/viz/led_dashboard.py:257-264` (VideoWriter) and `visionft/viz/led_dashboard.py:577-609` (csv.writer)** — The dashboard records video as XVID AVI files and force data as CSV files under `recordings/` in the current working directory, bypassing MCAP entirely. This is a debug/capture tool for Teo's original workflow and the files land in an untracked CWD path. If these recordings are ever used as ground truth, they will not be replayable with `ros2 bag play`. Either remove the recording feature (the dashboard is a viz tool, not a recorder) or gate it with a clear "not for data collection" warning. **Severity: med**

- **`visionft/viz/wrench_plotter.py:64-71` and `127-189`** — `WrenchPlotter` always opens a CSV file at startup and writes every received wrench and TCP pose sample to it in real time. This is the primary logging mechanism, not a side feature — the startup log says "Writing data in real-time to: ...". Use `ros2 bag record` instead; the `record.launch.py` already covers this use case. **Severity: high**

### Documentation gap (not a GP violation but should be fixed)

- **`CLAUDE.md` repo map and `ARCHITECTURE.md`** — `massage.cpp` is compiled and installed (listed in `CMakeLists.txt:52,74`) but is absent from the repo map in `CLAUDE.md` and from `ARCHITECTURE.md`. Any operator reading the docs would not know this behavior exists. Add `massage.cpp` to the repo map under `robot_behaviors/src/`. **Severity: low**

## Recording strategy (decided 2026-04-30, blocked on robot access)

Decision: **MCAP is the only recording. CSV and MP4 are post-hoc exports via `scripts/extract_mcap.py`.** Parallel writers (CSV/AVI alongside MCAP) are explicitly rejected — two sources of truth drift, and the MCAP is lossless while the CSV/AVI is a fixed view.

Action items below need a working robot session to verify nothing downstream breaks. Do them when back on the lab machine.

- **Delete `visionft/viz/wrench_plotter.py`** entirely. Superseded by `led_dashboard` (UI) + `record.launch.py` (data). Removes the GP5 high-severity finding above.
- **Strip recording code from `led_dashboard.py`** — keep the UI (LED control, force plots, vision analytics) but remove the XVID `VideoWriter` (lines 257-264) and the CSV `csv.writer` (lines 577-609). Dashboard becomes pure viz; recording goes through `record.launch.py`.
- **Lower the bar for downstream users (PhD analysis)** — wrap `scripts/extract_mcap.py` so the typical case is one command. Options: (a) `scripts/bag_to_csv.sh <bag.mcap>` wrapper with sane defaults, (b) auto-extract on session end via a launch hook in `record.launch.py`, (c) one-page README in `data/`. Pick one; (a) is cheapest.
