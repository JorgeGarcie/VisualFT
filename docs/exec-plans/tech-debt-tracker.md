# Tech Debt Tracker

**Last scan**: 2026-03-10 (post-cleanup)
**Total lint violations**: 484 (down from 504 — fixed duplication, imports, error swallowing)
**Total doc issues**: 100 (mostly legacy/reference files)

Run: `python3 scripts/lint_principles.py` and `python3 scripts/check_docs_freshness.py`

---

## ~~Priority 1: Duplicated RDK init + quaternion conversion (GP2)~~ DONE

Created `visionft/visionft/rdk_utils.py` with shared `init_robot()`, `build_pose_msg()`,
`build_wrench_msg()`, `euler_to_rdk_quat()`. All three RDK nodes now use it.

## Priority 1: Three separate RDK connection paths (GP5)

Three files each own their own RDK connection:
- `scan_node.py` — **actively used** for scanning workflows
- `rdk_cartesian_bridge.py` — built for teleop/manual control, **possibly unused** (teleop uses its own `flexiv_commander.py`)
- `robot_publisher.py` — legacy read-only publisher, **redundant** if bridge runs

All mutually exclusive. Only scan_node is confirmed in active use.

**Decision needed**: Consolidate into one unified bridge with services (MoveL, ZeroFTSensor,
set_mode) so scan_node becomes a pure state machine? Or keep current split?

**FIX (if consolidating)**: See ExecPlan proposal — unified RDK bridge with service interface.
**FIX (if keeping)**: At minimum, document which path is used when and deprecate unused ones.

## Priority 2: No health monitoring (GP4)

No watchdog, heartbeat, or fault alerting. If scan_node faults mid-scan with probe on
phantom, nothing alerts the operator. Discovery is post-hoc from incomplete bags.

- [ ] Add heartbeat topic or health check service
- [ ] Alert (log + optional audio/visual) on robot fault during scan
- [ ] Consider timeout on state transitions (e.g. DESCENDING stuck = no contact)

## Priority 3: Legacy code still present (GP6)

- [ ] `ros2_ws/src/visionft/visionft/data_logger.py` — CSV/video logger replaced by MCAP
- [ ] `ros2_ws/src/visionft/visionft/flexiv.py` — UDP receiver marked "DO NOT NEED"
- [ ] `ros2_ws/src/visionft/visionft/robot_publisher.py` — possibly unused (bridge publishes same data)

**FIX**: Remove or move to a `legacy/` directory. Remove from `setup.py` entry points.

## Priority 4: No shared robot config (GP1)

Robot SN `Rizon4-062174` declared as default in 3+ nodes. Control rates, velocity limits
also duplicated across nodes. Should be one `config/robot.yaml` loaded at launch.

- [ ] Create `config/robot.yaml` with shared robot parameters
- [ ] Load via launch file and pass to all nodes

## Priority 5: Stale documentation (docs)

- [ ] `ros2_ws/.claude/plan.md` — references 35+ files that don't exist (abandoned plan)
- [ ] `ros2_ws/convo.md` — old conversation notes with broken references

**FIX**: Archive or delete.

## Action needed: Register CoinFT as robot tool for gravity compensation

Flexiv SDK supports custom tool registration via `flexivrdk.Tool` + `ToolParams`
(see `flexiv_hardware/rdk/example_py/basics8_update_robot_tool.py`). Setting
mass, CoM, and inertia enables automatic gravity compensation for Float and
all other modes.

- [ ] Measure CoinFT mass, CoM, inertia (or get from datasheet)
- [ ] Add tool config to `config/robot.yaml` (mass, CoM, inertia, TCP offset)
- [ ] ArmCommander calls `tool.Add()` + `tool.Switch()` during init
- [ ] Test floating mode with tool compensation active

## Deferred (interview_demos/)

~350 magic number violations in `interview_demos/sim/demo.py` — simulation config,
not production code. Acceptable for a demo script. No action needed.
