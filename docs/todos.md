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
