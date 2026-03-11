# VisualFT Simplification Conversation Summary

## Date: 2026-02-09

## User Goal
- **Ultra simplify the project** - it's not working as it should
- **Core problem: Flexiv arm control doesn't work**
- Data recording (by hand) works fine - that's not the issue
- No git repo in ros2_ws (only git repo is in `classifier/TendonClassifier/` - separate ML project, no uncommitted changes)

---

## Current Project Structure

### Custom Package: `visionft` (Python, works fine)
- `robot_publisher.py` (143 lines) - Publishes robot F/T + TCP pose via flexivrdk SDK directly
  - Topics: `/flexiv/wrench`, `/flexiv/tcp_pose` at 100Hz
  - Robot SN: `Rizon4-062174`
- `coinft.py` (188 lines) - CoinFT sensor via serial + ONNX model
  - Topic: `/coinft/wrench` at ~360Hz
  - Serial: `/dev/ttyACM1` at 1M baud
- `data_logger.py` (349 lines) - Records all streams to CSV/video
- `wrench_plotter.py` (342 lines) - Real-time matplotlib plots
- `plot_csv.py` (129 lines) - Post-processing plots
- `flexiv.py` (101 lines) - **DEPRECATED** UDP receiver (dead code)
- Launch: `visionft.launch.py` - launches gscam + robot_publisher + coinft

### Vendor Package: `flexiv_ros2` (C++, **THIS IS THE PROBLEM**)
7 sub-packages, very complex:
1. `flexiv_hardware` - ros2_control hardware interface (C++, links flexiv_rdk static lib)
2. `flexiv_bringup` - Launch files + controller configs
3. `flexiv_msgs` - Custom messages (JointPosVel, RobotStates, etc.)
4. `flexiv_description` - URDF/xacro models
5. `flexiv_moveit_config` - Full MoveIt setup
6. `flexiv_gripper` - Gripper action server
7. `flexiv_test_nodes` - Test/demo nodes (Python):
   - `robot_controller.py` - Cartesian trajectory execution via MoveIt + 1kHz JointPosVel publishing
   - `cartesian_impedance_node.py` - Cartesian impedance control
   - `routine_runner_node.py` - Behavior tree routine executor
   - `phantom_scan.launch.py` - Phantom scanning
   - `routine_runner.launch.py` - BT routines
   - Trajectory generators: straight_line.py, zigzag.py

Controllers: `flexiv_robot_states_broadcaster`, `gpio_controller`, `joint_impedance_controller`

### Third-Party: `gscam2` + `ptrmu/ros2_shared`
- GStreamer camera driver for ROS2 (C++)
- UDP H.264 video from IP camera on port 5000

---

## Key Architecture Insight: TWO SEPARATE ROBOT INTERFACES

1. **visionft/robot_publisher.py** - Uses `flexivrdk` Python SDK directly to READ robot state (F/T, TCP pose). Simple, works.
2. **flexiv_ros2/flexiv_hardware** - Uses `flexiv_rdk` C++ SDK via ros2_control for CONTROLLING the robot (joint position/velocity/torque commands). Complex, broken.

The `flexiv_test_nodes/robot_controller.py` does:
- Subscribes to joint_states and robot_states from ros2_control
- Uses MoveIt `compute_cartesian_path` service for planning
- Resamples trajectories at 1kHz
- Publishes `JointPosVel` commands to impedance controller
- Topic names depend on robot_sn (e.g., `/Rizon4_062174/flexiv_robot_states`)

---

## Launch Chain for Robot Control (rizon.launch.py)
1. `ros2_control_node` (controller_manager) - loads hardware interface
2. `robot_state_publisher` - publishes TF from URDF
3. `joint_state_publisher` - merges joint states
4. `joint_state_broadcaster` spawner
5. `flexiv_robot_states_broadcaster` spawner
6. `rizon_arm_controller` spawner (joint trajectory controller, default)
7. `gpio_controller` spawner
8. RViz (optional)
9. Gripper launch (optional)

---

## Simplification Opportunities

### Remove/Skip (not needed for core robot control):
- `flexiv_moveit_config` - Only needed if using MoveIt planning (could use direct SDK instead)
- `flexiv_gripper` - Only if no gripper needed
- `flexiv_description` rviz configs
- `visionft/flexiv.py` - Already deprecated dead code
- `visionft/wrench_plotter.py` - Nice to have, not essential
- `visionft/plot_csv.py` - Post-processing, not essential

### Core Question to Answer Next Session:
**What exactly does the user need the robot to DO?**
- Just move to positions? → Could use flexivrdk Python SDK directly (like robot_publisher.py does)
- Cartesian impedance control? → Needs ros2_control + impedance controller
- MoveIt planning? → Needs full flexiv_ros2 stack
- Behavior tree routines? → Needs flexiv_test_nodes

### Simplest Possible Approach:
If the goal is just "move robot to positions and record data", you could:
1. Skip the entire ros2_control/flexiv_ros2 stack
2. Use `flexivrdk` Python SDK directly (like robot_publisher.py already does)
3. Add move commands directly in Python
4. Keep the visionft data recording pipeline as-is

---

## Build State
- Last build: 2026-02-04 12:55
- All packages built: flexiv_bringup, flexiv_description, flexiv_gripper, flexiv_hardware, flexiv_moveit_config, flexiv_msgs, flexiv_robot_states_broadcaster, flexiv_test_nodes, gpio_controller, gscam2, joint_impedance_controller, ros2_shared, visionft

## Files Read This Session
- `/home/li2053/VisualFT/ros2_ws/src/flexiv_ros2/flexiv_bringup/launch/rizon.launch.py`
- `/home/li2053/VisualFT/ros2_ws/src/visionft/launch/visionft.launch.py`
- `/home/li2053/VisualFT/ros2_ws/src/visionft/visionft/robot_publisher.py`
- `/home/li2053/VisualFT/ros2_ws/src/flexiv_ros2/flexiv_test_nodes/flexiv_test_nodes/robot_controller.py`
