# Golden Principles

Timeless philosophies for the VisualFT codebase. Specific violations go in
`docs/exec-plans/tech-debt-tracker.md`, not here.

New candidates go through `docs/design-docs/lessons-learned.md` first.
Promotion criteria: validated across 3+ tasks.

---

## GP1: One fact, one place

Constants, config values, and shared logic have a single source of truth.
If a value appears in 2+ files, extract it to a config or shared module.

## GP2: Centralize, don't duplicate

Extract repeated patterns into shared utilities. Same quaternion conversion
in 3 files → one utility function. Same RDK init in 2 nodes → shared helper.

## GP3: Validate at boundaries

Assert shapes, types, and ranges where data enters a module. Quaternions
must be unit-length. Poses must be in expected units. Serial bytes must
match protocol framing.

## GP4: Fail loud, not silent

Log warnings for unexpected states. Never swallow exceptions. If a sensor
disconnects or RDK faults, the node should report it clearly, not silently
produce zeros.

## GP5: Single RDK owner

Exactly one node owns the Flexiv RDK connection at any time. All other
nodes command the robot through ROS2 topics. This is a hardware constraint
(single connection limit) elevated to a design principle.

## GP6: Record with MCAP

All data recording uses ROS2 MCAP bags, not custom CSV/video loggers.
MCAP captures all topics at full rate with zero application overhead and
enables per-pass post-processing.
