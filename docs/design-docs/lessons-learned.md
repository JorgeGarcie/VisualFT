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

### Flexiv primitives don't terminate via busy()
- **Context**: ArmCommander used `robot.busy()` to wait for ZeroFTSensor — hung forever
- **Root cause**: SDK docs say "Most primitives won't exit by themselves". `busy()` stays true. Must poll `primitive_states()["terminated"]` or `["reachedTarget"]` instead.
- **Lesson**: Use `busy()` for plans (PLAN-Home), use `primitive_states()` for primitives (MoveL, ZeroFTSensor, Contact)
- **Status**: observation (validated on hardware 2026-03-11)

### Float primitive does not exist on RDK v1.7
- **Context**: Flexiv website documents a Float primitive with axes/damping params. Does not exist on our firmware.
- **Root cause**: Float is a newer firmware feature not available on RDK v1.7
- **Lesson**: Don't trust Flexiv's public docs for available primitives — test on hardware. Floating requires RT_JOINT_TORQUE mode (C++, 1kHz scheduler with gravity compensation).
- **Status**: observation (forced Python→C++ pivot 2026-03-11)

### Python is unnecessary for robot control
- **Context**: Originally planned ArmCommander in Python. Pivoted to C++ after Float discovery.
- **Root cause**: RT_JOINT_TORQUE needs 1kHz C++ scheduler. All other primitives (home, zero_ft, MoveL, Contact, streaming) are equally simple in C++. Python adds no value for sequential robot commands.
- **Lesson**: For Flexiv RDK, default to C++ for all robot control. Python only for ML inference and sensor nodes that use Python-only libraries (PyTorch, ONNX).
- **Status**: observation (validated 2026-03-11)

### Mode switching is implicit and order-dependent
- **Context**: `set_impedance()` and `set_force_control_axis()` failed silently when called before streaming
- **Root cause**: These calls require NRT_CARTESIAN_MOTION_FORCE mode already active. Must call `stream_cartesian()` first to enter the mode.
- **Lesson**: Always start the cartesian stream before configuring impedance or force control axes. Hit this twice — document the required call order.
- **Status**: observation (validated on hardware 2026-03-11)

### Use RDK primitives when they exist
- **Context**: Considered implementing manual descent with force polling for surface contact
- **Root cause**: Contact primitive already does exactly this — descend along axis until force threshold
- **Lesson**: Check available RDK primitives before implementing manual control logic. Don't reimplement what the SDK provides.
- **Status**: observation

### Workspace bounds are a safety feature, not a tuning knob
- **Context**: Kept widening workspace bounds to accommodate scan trajectories that exceeded limits
- **Root cause**: Treating bounds as a tuning parameter rather than measuring the real safe envelope first
- **Lesson**: Measure the physical safe envelope from the actual setup, then set bounds once. Don't keep widening to fit the task — that defeats the safety purpose.
- **Status**: observation

### RT scheduler needs root privileges
- **Context**: 1kHz RT_JOINT_TORQUE loop failed to achieve timing without elevated privileges
- **Root cause**: SCHED_FIFO requires root or capability grants
- **Lesson**: Fix with `setcap cap_sys_nice+ep` on the executable or configure `/etc/security/limits.conf`. Don't run the whole node as root.
- **Status**: observation
