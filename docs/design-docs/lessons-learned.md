# Lessons Learned

Pipeline for new golden principles. Status: observation → candidate → promoted to GP#N.

---

## Observations

### F/T sensor bias causes phantom forces
- **Context**: Teleop mode — robot crept forward on engage
- **Root cause**: F/T sensor had accumulated bias; `ZeroFTSensor` primitive fixes it
- **Lesson**: Always zero F/T sensor after homing, before any force-dependent operation
- **Status**: observation (validated in teleop; scan_node already does this)

### Quaternion convention mismatches cause subtle bugs
- **Context**: robot_publisher.py treated `[qw,qx,qy]` as Euler angles
- **Root cause**: RDK uses w-first, ROS2 uses w-last, scipy returns [x,y,z,w]
- **Lesson**: Every quaternion variable should document its convention in a comment
- **Status**: observation (validated in 2 bugs: robot_publisher, teleop retargeting)

### NRT mode requires continuous heartbeat
- **Context**: SendCartesianMotionForce is not a queued command
- **Root cause**: Must be called at control rate (50Hz+) or robot stops
- **Lesson**: NRT control loops need timers, not one-shot publishers
- **Status**: observation
