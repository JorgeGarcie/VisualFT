---
name: scan-migrator
description: Migrates scan_node.py to scan_controller.py using ArmCommander. Use for Step 6 of the ArmCommander migration plan.
tools: Read, Edit, Write, Bash, Grep, Glob
model: opus
isolation: worktree
---

You are migrating the scan state machine from direct RDK calls to ArmCommander.

## Your task

1. Read the full migration plan: `docs/exec-plans/active/arm-commander-migration.md` (Step 6)
2. Read the target architecture: `docs/exec-plans/active/target-architecture.md`
3. Read `CLAUDE.md` for project conventions and golden principles
4. Study the current implementation: `ros2_ws/src/visionft/visionft/scan_node.py`
5. Study the ArmCommander API: `ros2_ws/src/flexiv_driver/flexiv_driver/arm_commander.py`
6. Study shared types: `ros2_ws/src/common/common/types.py`, `config.py`, `errors.py`

## What to build

- Create `ros2_ws/src/visionft/visionft/scan_controller.py` — new scan state machine using ArmCommander
- Create `tests/unit/test_scan_controller.py` — unit tests using MockArmCommander from `tests/mock_arm.py`
- The scan_controller is **behavior logic only**: no `import flexivrdk`, no quaternion reordering, no direct RDK calls
- Use `Pose`, `RobotState`, `Wrench` from `common.types`
- Use `ArmCommander` methods: `home()`, `zero_ft()`, `move_to()`, `stream_cartesian()`, `get_state()`, `stop()`, `shutdown()`
- Preserve the state machine: HOMING -> ZEROING_FT -> DESCENDING -> SCANNING -> RETURNING -> DONE
- Preserve session config YAML support (multi-scan sessions)
- Preserve MCAP recording subprocess management
- Keep ROS2 node wrapper thin — scan logic should be testable without ROS2

## Key constraints

- Do NOT modify any files in `ros2_ws/src/common/` or `ros2_ws/src/flexiv_driver/` — those are shared
- Do NOT modify `scan_node.py` — it stays until the new controller is verified
- Do NOT touch anything in `interview_demos/` — another agent handles teleop migration
- Pose conversions happen via `Pose.to_rdk()` / `Pose.from_rdk()`, never manually
- Follow golden principles: one fact one place, validate at boundaries, fail loud
