# Apptronik Dexterity Engineer — Interview Prep

## Your Demo System (what you built and can talk about)

**VR Teleop Pipeline: Quest 3S → ZMQ → Flexiv Rizon4**
- Hand tracking retargeting (no controllers — pure hand tracking)
- Task-space retargeting with configurable axis mapping, workspace scaling
- Post-engage hold period to prevent initial jumps (matched lab's proven pattern)
- 1-Euro filtering for jitter reduction, dropout handling with extrapolation
- Clutch engage/disengage via FrankaBot APK border toggle
- Safety: workspace clamping, velocity limiting, force threshold estop
- NRT Cartesian impedance control via Flexiv RDK
- Sensor bridge: ROS2 camera + force overlay streamed to VR headset

**Automated Scanning Pipeline (ROS2)**
- State machine: HOMING → ZEROING_FT → DESCENDING → SCANNING → RETURNING
- Force-controlled contact detection for surface finding
- Impedance control: stiff XY tracking, compliant Z for gentle surface contact
- Multi-pass rotation sweeps with MCAP data recording
- Session-based config for reproducible scan sequences

---

## JD Checklist — What We Have vs What We'd Add

| JD Requirement | We Have | Gap / Next Steps |
|---|---|---|
| **Retargeting algorithms** | Task-space wrist retargeting (arm), geometric angle-extraction finger retargeting (lab's LEAP Hand system), axis mapping, workspace scaling, orientation retargeting | Lab's finger retargeting is geometric (angle-to-angle) — works for kinematically similar hands. Next: optimization-based fingertip retargeting for hands with different kinematics (like Apollo) |
| **Jitter reduction** | 1-Euro adaptive filter, dropout extrapolation, post-engage hold | Solid. Could add median pre-filter for outlier rejection |
| **Sensor dropouts / safe fallback** | Dropout handler (extrapolate → hold), force estop, workspace clamp | Solid. Five-layer defense in depth |
| **Sim/testing infrastructure** | Lab's MuJoCo sim runs same operator code. Session-based scan configs for parameter sweeps | No IsaacSim. Next: port to IsaacSim for GPU-parallel evaluation |
| **Performance metrics / benchmarking** | MCAP bags with state labels per pass, timestamped trajectories | No automated scoring yet. Next: define dexterity metrics (task completion rate, force consistency, tracking error) |
| **Haptic gloves / tactile integration** | CoinFT sensor → ROS2 → ZMQ. Force viz overlay to VR headset. F/T safety threshold | Visual feedback only, no haptic gloves. Architecture supports it (send_haptic method exists). Next: integrate haptic glove, implement impedance shaping for transparency |
| **Tactile data logging** | CoinFT wrench logged in MCAP bags, synchronized with pose/state | Solid. Would add raw taxel logging at native rate for training data |
| **RL scaffolding** | Retargeting decoupled from control — policy swap is a config change. Training-ready data pipeline | No learned policies yet. Architecture is ready for it |
| **Shared autonomy** | Z-approach mode (robot handles compliance, human handles intent). Virtual fixture concept in impedance | Basic. Next: force barriers, grasp stability corridors |
| **Bimanual** | Single arm only | Not implemented. Know the architecture: leader-follower, collision capsules, force-regulated grasping |
| **C++/Python** | Python primary, C++ via RDK bindings and ROS2 nodes | Strong Python, working C++ knowledge |
| **ROS2** | Full pipeline: nodes, launch files, topics, MCAP recording | Strong |
| **VR integration** | Quest 3S hand tracking, ZMQ, FrankaBot APK, SteamVR/OpenVR support | Strong |
| **Linux/bash** | Ubuntu, process management, scripting | Strong |
| **Git** | Version control throughout | Strong |
| **Real-time HIL** | 60Hz control loop on real Flexiv Rizon4, impedance control, force sensing | Strong |
| **MuJoCo** | Lab's sim environment (same code runs sim and real) | Familiar, not primary developer |
| **Docker** | Not used in this project | Gap — mention familiarity, not expertise |
| **Unity/OpenXR** | Not used (ZMQ approach instead) | Know when you'd use it, can articulate why we didn't |

---

## Mapping JD Requirements → Your Experience

### 1. Retargeting & Mapping Algorithms
**What they want:** IK-based, optimization-based, or learned retargeting. Jitter reduction, sensor dropouts, safe fallbacks.

**What you did:**
- **Arm retargeting (task-space):** VR wrist frame → robot EE with axis swap `[delta_y, -delta_x, delta_z]`, workspace scaling (0.7x), orientation retargeting with scaled delta rotations applied to robot anchor
- **Finger retargeting (geometric, lab system):** 24 VR hand keypoints → 16 LEAP Hand joint angles via direct angle extraction. Per-finger: 3 flexion angles computed from bone-triplet vectors + 1 abduction angle from lateral projection. Thumb uses arctan2 for opposition + 3 flexion. Per-joint scaling and velocity clamping for smoothness.
- 1-Euro filter (adaptive cutoff) for jitter — better than fixed EMA because it's responsive to fast moves but smooth during slow moves
- Dropout handler: detects missing VR frames, extrapolates for up to 200ms, then holds
- Post-engage hold (10 frames) — prevents jump on clutch engage by continuously updating VR anchor while robot stays frozen
- Workspace safety clamp (hard box limits) as fallback

**How the finger retargeting works (be ready to explain):**
- Quest gives 24 keypoints per hand (wrist, palm, 4 joints per finger)
- For each finger: build the chain `[wrist, knuckle, PIP, DIP, tip]`
- MCP/PIP/DIP flexion = angle between consecutive bone vectors (dot product → arccos)
- Abduction = `arctan2(lateral, forward)` of finger direction projected to palm plane
- Thumb opposition = `arctan2(y, x)` of CMC-base direction (how much thumb faces palm)
- Per-joint scaling + offset calibration maps human ROM to robot ROM
- Velocity clamp (`clip(delta, -bounds, bounds)`) instead of filter — simpler, prevents sudden jumps
- This works because LEAP Hand has similar kinematics (1 abduction + 3 flexion per finger). For Apollo's hand with different kinematics → need optimization-based fingertip matching (see Q2 answer)

**Talking points:**
- "I implemented task-space retargeting from Quest hand tracking to a 7-DOF Flexiv arm. The key challenges were axis mapping between VR and robot frames, jitter from hand tracking noise, and preventing discontinuities on clutch transitions."
- "Our lab also has finger-level retargeting for a LEAP dexterous hand. It uses geometric angle extraction — computing flexion and abduction angles directly from VR keypoint geometry, with per-joint scaling to map human range of motion to the robot's. This works well for kinematically similar hands, but for a hand like Apollo's with different joint structure, I'd move to optimization-based fingertip position matching."
- "For jitter, I used 1-Euro filtering which adapts its cutoff frequency based on signal speed — smooth during slow movements, responsive during fast ones. I also handle VR tracking dropouts with extrapolation and graceful fallback to hold."
- "One tricky problem was the initial jump on engage — the first retargeted frame has stale filter state and a large delta. I solved this with a post-engage hold period where the VR anchor continuously updates while the robot stays frozen, so the first real command has zero delta."

### 2. Simulation & Testing Infrastructure
**What they want:** MuJoCo/IsaacSim workflows, automated benchmarking.

**What you can say:**
- Lab's system includes MuJoCo sim environment with the same operator code — sim and real share the same retargeting pipeline
- Your scan node generates reproducible, tagged datasets (MCAP bags with state labels per pass) — this is exactly the kind of automated data collection pipeline they need
- Session configs allow parameter sweeps across orientations and positions

**Talking points:**
- "Our lab's teleop stack runs identically in MuJoCo sim and on real hardware — same retargeting, same control loop. The sim viewer uses MuJoCo's viewer with the operator callback, so you can validate behavior before going to hardware."
- "For data collection, I built a session-based scanning pipeline that runs parameterized sweeps and records synchronized ROS2 bags with per-pass state labels. This makes it straightforward to slice data for training."

### 3. Haptics & Tactile Integration
**What they want:** Haptic gloves, tactile sensors in control loop, force-reflective feedback, high-bandwidth tactile data logging.

**What you did:**
- CoinFT tactile sensor integration (serial → ROS2 → ZMQ)
- Force monitoring with real-time visualization (bar + sparkline streamed to VR headset)
- Robot F/T sensor reading for safety (force threshold estop)
- Sensor bridge: ROS2 wrench data → ZMQ for VR overlay
- Scan node uses F/T for contact detection (surface finding)

**Talking points:**
- "I integrated a CoinFT tactile sensor into our ROS2 pipeline and built a sensor bridge that streams force visualization to the VR headset in real-time. The operator sees a force bar and sparkline overlaid on the camera feed, giving them force-reflective feedback during teleop."
- "The force data is also used for safety — if external forces exceed a threshold, the system triggers an automatic stop. In our scanning pipeline, force sensing drives the contact detection state machine."
- "For haptic feedback specifically, the architecture supports it — the VR reader has a `send_haptic` method that maps force magnitude to vibration intensity. With controllers this works directly; with hand tracking, the visual force overlay serves as the feedback channel."

### 4. Advanced Control & RL Frameworks
**What they want:** Control transparency, RL scaffolding, shared autonomy.

**What you can say:**
- Impedance control for compliant interaction (configurable stiffness/damping per axis)
- Z-approach mode: stiff XY tracking + compliant Z for contact tasks — this IS shared autonomy (robot handles compliance, human handles intent)
- Data pipeline produces training-ready trajectories (timestamped pose, force, state labels)
- Architecture separates retargeting from control — easy to swap in a learned policy

**Talking points:**
- "Our impedance control framework lets you configure stiffness per axis. For contact tasks, I use stiff XY to track the scan line and compliant Z so the robot gently follows the surface. This is a form of shared autonomy — the human provides the high-level intent, the impedance controller handles the contact dynamics."
- "The data pipeline records synchronized trajectories with state labels, camera frames, and force/torque data in MCAP format. This is directly usable for imitation learning or RL — you can segment episodes by state label, extract observation-action pairs, and train policies."
- "The retargeting layer is cleanly separated from the control layer, so swapping human input for a learned policy is a config change, not an architecture change."

### 5. Technical Stack (matching their requirements)
- **Python** — primary language for all teleop, control, data pipeline
- **C++** — ROS2 nodes, Flexiv RDK is C++ with Python bindings
- **ROS2** — control layer, topic-based communication, launch files, MCAP recording
- **Linux** — Ubuntu, bash scripting, process management
- **Git** — version control for all code
- **VR integration** — Quest 3S hand tracking, ZMQ networking, FrankaBot APK
- **Real-time hardware-in-the-loop** — 60Hz control loop, NRT Cartesian mode, impedance control
- **MuJoCo** — lab's sim environment (same operator code runs in sim and real)

---

## Expected Questions & Answers

### "Why teleoperation?"
Teleoperation is the fastest path to getting high-quality manipulation data. You can't train RL policies without demonstrations, and you can't write analytical controllers for every contact-rich task. Teleop lets a human provide the intent — grasp strategy, force modulation, recovery from failures — while the robot handles the low-level dynamics. It's also the bridge to shared autonomy: you start with full teleop, collect data, train policies on that data, then progressively hand off subtasks to the learned policy. Apptronik's JD says exactly this — "swap manual teleoperation with learned policies." Teleop is step one of that pipeline.

### "Why track the wrist? Why not individual fingers?"
The wrist gives you the 6-DOF pose of the end-effector — position and orientation. For arm control, that's what matters. Individual finger tracking is for hand/gripper retargeting, which is a separate layer. Our lab's full system does both: wrist frame drives the arm via Cartesian control, finger keypoints drive a dexterous hand (LEAP Hand) via joint-angle retargeting. I focused on the arm control layer because that's where the hard problems are — latency, safety, force control, impedance tuning. Finger retargeting is comparatively straightforward once you have the kinematic mapping.

### "Do you guarantee precise control?"
No system guarantees precision — you design for it and measure it. Our system has several layers that affect precision:
- **Impedance control** means the robot is compliant, not rigidly position-tracking. This is intentional — for contact tasks you want compliance, not stiffness. The tradeoff is that under external load, the EE deviates from the commanded pose. You tune stiffness per task.
- **Filtering** smooths jitter but adds ~1-2 frames of latency. The 1-Euro filter is adaptive — high cutoff for fast moves (responsive but noisier), low cutoff for slow moves (smooth but laggier).
- **Workspace scaling** at 0.7x means small hand tremors map to even smaller robot motions — this improves effective precision.
- **Repeatability** depends on the robot (Flexiv Rizon4 has +/-0.1mm repeatability) and the control mode. In our impedance mode, tracking accuracy depends on stiffness settings and external forces.

The honest answer: for contact-rich tasks, you don't want or need micron precision. You want compliance and force awareness. Precision matters more for free-space positioning, where our system tracks within a few mm.

### "What are the rates?"
- **VR hand tracking**: 60 Hz (Quest native tracking rate)
- **Teleop control loop**: 60 Hz (matched to VR input rate)
- **Flexiv NRT Cartesian mode**: accepts commands up to 100 Hz, we send at 60 Hz
- **Force/torque sensing**: sampled every control tick (60 Hz for safety checks)
- **Camera stream**: 30 Hz JPEG over ZMQ to headset
- **ROS2 scan node**: 50 Hz control rate

The bottleneck is VR tracking at 60 Hz. The robot can accept faster commands but there's no benefit sending faster than the sensor input.

### "How do you handle latency?"
Three sources of latency, handled differently:
1. **VR to PC (WiFi/ZMQ)**: ~5-10ms. ZMQ CONFLATE option drops stale messages — we always read the latest frame, never queue up old ones.
2. **Filtering**: 1-Euro filter adds 1-2 frames (~16-33ms) of effective latency, but it's adaptive. Fast movements get high cutoff (minimal lag), slow movements get low cutoff (smooth). This is better than a fixed EMA which has constant latency regardless of speed.
3. **PC to Robot (Ethernet/RDK)**: <1ms. Direct RDK connection over local ethernet, negligible.

Total end-to-end: ~20-40ms wrist-to-EE. For reference, human reaction time is ~200ms, so sub-50ms is perceptually real-time.

We also handle **dropouts** — if VR frames stop arriving, the dropout handler extrapolates the last known velocity for up to 200ms, then falls back to holding position. This prevents sudden stops during brief tracking losses.

### "Is there any safety implementation?"
Yes, defense in depth — five independent layers:
1. **Workspace hard clamp**: Target positions outside a bounding box are clamped before sending to the robot. The robot physically cannot be commanded outside this volume.
2. **Velocity limiting**: If a commanded position would require velocity above 0.15 m/s, the motion is scaled down to respect the limit. Prevents sudden jumps.
3. **Force threshold estop**: If external forces exceed 25N (configurable), the system triggers an automatic stop. Software-level, checked every control tick.
4. **Impedance control**: The robot is inherently compliant — it's a spring-damper system, not rigid position tracking. If something unexpected is in the way, the robot yields rather than forcing through.
5. **Post-engage hold**: On clutch engage, the robot freezes for 10 frames while the VR anchor settles. Prevents the initial jump that would otherwise occur from stale filter state or large initial deltas.

Plus the hardware E-stop on the teach pendant as the ultimate fallback.

### "Did you make this in Unity?"
No. The VR side uses a standalone Android APK called FrankaBot that runs natively on the Quest. It captures hand tracking keypoints and sends them over WiFi via ZMQ — no Unity, no game engine. The PC side is pure Python: ZMQ for networking, NumPy/SciPy for math, OpenCV for camera streaming, ROS2 for the sensor pipeline.

I didn't need Unity because we're doing hand tracking, not controller input. The Quest's native hand tracking API gives us the keypoints directly. If we needed custom VR environments, haptic rendering, or complex UI in the headset, then Unity/Unreal with OpenXR would be the right call. But for teleoperation, the lightweight ZMQ approach has lower latency and fewer dependencies than going through a game engine.

That said, I'm familiar with the SteamVR/OpenXR stack — our codebase supports both controller input via OpenVR and hand tracking via ZMQ. The controller path uses SteamVR bindings and would naturally extend to Unity/OpenXR if needed.

### "Why not use the Meta controller instead of hand tracking?"
We actually intended to — controller input via SteamVR was the original plan. The controllers were lost in the lab, so we pivoted to hand tracking. But honestly, hand tracking turned out to have advantages:
- **More natural**: You just move your hand, no device to hold. Lower cognitive load for the operator.
- **No controller drift**: Controllers accumulate IMU drift; hand tracking is vision-based and resets every frame.
- **Closer to the end goal**: For dexterous manipulation with a humanoid hand, you want finger-level tracking anyway. Controllers give you 6-DOF + buttons, hand tracking gives you full articulated hand pose.

The tradeoff is precision — controllers have physical buttons for discrete actions (engage/disengage) and thumbsticks for fine adjustment. Hand tracking relies on gesture recognition for state changes, which is less reliable. Our system uses the FrankaBot app's screen border toggle (blue=engaged, green=paused) as the engage mechanism, which works but isn't as crisp as a button press.

For Apollo specifically, I'd recommend supporting both: controllers for operators who need maximum precision, hand tracking for natural dexterous tasks where finger pose matters.

---

## Deep Technical Questions (from mock interview)

### Q2: Hand Retargeting for 20-DOF Dexterous Hand

**Solver choice:** Levenberg-Marquardt with box constraints. LM is natural for sum-of-squared-residuals cost (fingertip position error). Warm-start from previous frame gives 2-3 iteration convergence. ~50-100us for 20 DOF.

**Residual vector:** 5 fingertip positions (15 residuals, high weight) + joint velocity penalty (20 residuals) + joint limit proximity (20 residuals) + collision pair distances (~10 residuals) + synergy deviation (20 residuals, low weight). ~80 residuals, 20 variables.

**Optimize over:** Joint angles directly (not synergies or fingertip deltas). Synergies used as regularization prior, not as the optimization variable. This preserves full DOF access while biasing toward natural poses.

**Learned model role:** Warm-start initializer for the optimizer, not a replacement. Train on optimizer output data. Use as first phase, then 2-3 LM iterations to refine and guarantee feasibility.

**Three failure modes:**
1. **Keypoint swap/tracking jump** — detect via per-fingertip velocity threshold (>15mm/frame = glitch). Fallback: reject frame, hold previous command, blend to neutral if persistent.
2. **Wrong local minimum** — detect via joint-space velocity spike even when fingertip error is low. Fallback: re-solve with 10x smoothness weight, or interpolate over 5 frames.
3. **Outside robot capability** — detect via high residual norm or >30% joints at limits. Fallback: drop to synergy subspace (6-8 DOF), accept lower fingertip accuracy for natural-looking pose. Visual indicator in VR.

**Meta-principle:** Never show the operator a discontinuity. Smooth degradation > correct but jerky.

### Q3: Haptics & Tactile Integration

**Three-rate architecture:**
- Inner loop (1kHz): robot impedance controller + wrist F/T. Tactile does NOT enter here.
- Mid loop (60Hz): teleop retargeting. Tactile enters as safety monitor (reactive modifier on impedance/velocity).
- Outer loop (1kHz, separate thread): haptic rendering → glove. Output-only, does not feed back into robot control.

**Tactile filtering:** Haptic path: minimal (preserve high-freq content). Safety path: max-over-window downsampled to 60Hz (never miss a peak).

**Time alignment for training data:** Everything PC-stamped on arrival. Log at native rates, resample offline to common rate. Don't try to synchronize clocks in real-time.

**Control for transparency:** Impedance shaping + virtual fixtures.
- Contact detected → stiffen finger, render force to glove
- Free space → soften, light tracking
- Force scaling: nonlinear (log) so operator feels 1-5N differences but isn't fatigued by 30-35N
- Virtual fixtures: force ceiling (haptic wall at force limit), grasp stability corridor (friction cone enforcement), surface following (constrain normal direction)

**Tactile dropout fallback:** Haptic glove goes to constant gentle buzz (NOT zero — zero feels like "no contact"). Robot switches to conservative impedance on affected fingers. Hold current grasp — don't release. Visual indicator in VR.

### Q4: Bimanual Manipulation

**Four concrete challenges:**
1. **Shared workspace collision** — operator's hands can overlap, robot's arms can't
2. **Relative vs absolute frame** — bimanual tasks care about inter-hand pose, not world-frame position. Independent retargeting accumulates relative error
3. **Asymmetric roles** — holder vs manipulator need different impedance. Same settings for both fails under reaction forces
4. **Coordinated timing** — independent latency per arm causes asymmetric forces during synchronized actions (grasping, releasing)

**Collision prevention — three layers:**
1. Predictive: capsule-capsule distance for all inter-arm link pairs every tick (<0.5ms)
2. Repulsive field: velocity bias pushing arms apart when distance <5cm, ramping to strong at 1.5cm
3. Hard freeze: both arms hold if distance hits 1.5cm. Resume when VR targets are safe

**Box holding — leader-follower with force regulation:**
- Leader arm: position-controlled, tracks operator's dominant hand. Defines object pose.
- Follower arm: force-controlled on squeeze axis. Target position = leader pose + grasp offset. PI controller regulates grip force using wrist F/T sensor.
- Operator's non-dominant hand sets target grip force (inward = squeeze harder)
- Why not symmetric force control: you lose position tracking, object drifts
- Role switching: whichever hand has lower velocity variance = fixture = leader

---

## Architecture Walkthrough (if asked)

**"Walk me through your teleop architecture."**
Quest 3S runs FrankaBot APK → sends hand keypoints over WiFi via ZMQ PUSH → VR server (OculusVRHandDetector + TransformHandPositionCoords) processes and republishes → teleop_main subscribes to transformed hand frame and pause state → retargeting maps VR wrist to robot EE with axis swap, scaling, filtering → FlexivCommander sends NRT Cartesian commands at 60Hz → robot executes with impedance control.

**"How would you add a learned policy?"**
The retargeting layer outputs (target_pos, target_quat) at 60Hz. A learned policy would do the same — take observations (camera, force, joint state) and output the same format. The FlexivCommander doesn't care where the target comes from. You'd add a mode flag: `--input policy` alongside `--input hand` and `--input controller`.

**"Tell me about a hard bug you solved."**
The robot would jump immediately when the operator engaged teleop. Three separate issues: (1) First pause message from VR was "engaged" (CONT=1), so the system auto-engaged before the operator was ready — fixed by ignoring the first state message. (2) Stale 1-Euro filter state caused a large delta on the first real frame — fixed by resetting filters on engage. (3) No settling period meant the first retarget had a nonzero delta — fixed by adding a 10-frame hold where the VR anchor continuously updates while the robot stays frozen. Each fix alone wasn't enough; all three together solved it.

---

## What to Emphasize

1. **You work on real hardware** — not just sim. E-stops, fault recovery, impedance tuning, force sensing — all real.
2. **You solve integration problems** — VR headset + ZMQ + ROS2 + Flexiv RDK + tactile sensors + camera streaming, all working together.
3. **You think about safety first** — multiple layers, defense in depth, never trust a single check.
4. **Your data pipeline produces training-ready data** — this is what they need for RL/autonomy.
5. **You can demo it** — working system, not slides.
6. **You know what you haven't built yet** — be honest about gaps (no finger retargeting, no haptic gloves, no bimanual) but show you know the architecture for each.

## What NOT to Say
- Don't pretend you built the FrankaBot APK or the leapft library — those are lab member's work. You integrated and extended them.
- Don't oversell MuJoCo experience — say "familiar through lab's sim infrastructure" not "I built the sim."
- Don't hide that controllers were lost — the pivot to hand tracking is a strength, not a weakness. It shows adaptability.
- Don't say "I would" when you can say "I did." Lead with what's built, extend to what you'd add.
