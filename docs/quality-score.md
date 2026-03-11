# Quality Score

**Last updated**: 2026-03-10

## Module Grades

| Module | Grade | Notes |
|--------|-------|-------|
| `visionft/scan_node.py` | **B** | Well-structured state machine, but duplicates RDK init/quat patterns |
| `visionft/rdk_cartesian_bridge.py` | **B** | Clean 50Hz loop, duplicates init/quat patterns |
| `visionft/coinft.py` | **B+** | Solid serial reader, good offset collection |
| `inference/inference_node.py` | **B** | Clean inference pipeline, some unused imports |
| `inference/models_v2.py` | **B** | Good architecture, some magic numbers in model dims |
| `visionft/grid_visualizer.py` | **B-** | Hardcoded workspace bounds |
| `visionft/robot_publisher.py` | **C** | Legacy, quat bug fixed but still duplicates patterns |
| `visionft/data_logger.py` | **D** | Legacy, replaced by MCAP. Should be removed |
| `visionft/flexiv.py` | **D** | Unused UDP receiver. Should be removed |
| `interview_demos/teleop/` | **B-** | Working demo, lots of inline constants (acceptable for demo) |
| `scripts/extract_mcap.py` | **B** | Utility, works fine |
| Launch files | **A** | Clean, parameterized |
| Config/YAML | **A** | Well-structured session templates |

## Top 5 Fixes by Leverage

1. **Extract shared RDK utilities** → eliminates 3 instances of duplicated init/quat/wrench code
2. **Remove legacy files** (data_logger.py, flexiv.py) → reduces confusion, smaller surface
3. **Archive stale docs** (ros2_ws/.claude/plan.md, convo.md) → cleaner doc tree
4. **Add `# noqa` to acceptable magic numbers** in scan defaults → reduces lint noise
5. **Clean unused imports** across inference/ → minor hygiene
