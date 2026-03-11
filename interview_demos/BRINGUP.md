# Hardware Bring-Up Protocol

Step-by-step validation from software-only → full VR teleop on Flexiv.
Each step has a PASS criteria. Do NOT proceed until the current step passes.

---

## Prerequisites

- [ ] Kill any running ROS2 nodes that use the RDK (`rizon.launch`, `robot_publisher`, `scan_node`)
- [ ] Robot E-stop released, teach pendant in AUTO mode
- [ ] Quest 3 connected to PC via Link cable, SteamVR running (for VR steps)

---

## Phase 1: Software-Only (no hardware)

### Step 1.0 — Mock teleop pipeline
```bash
cd ~/VisualFT/interview_demos
timeout 5 python3 -u teleop/teleop_main.py --mock --rate 30
```
**PASS:** Prints safety box, shows `engaged=True`, position changes over time, exits cleanly.

### Step 1.1 — Mock z-approach mode
```bash
timeout 5 python3 -u teleop/teleop_main.py --mock --rate 30 --mode z-approach
```
**PASS:** XY stays locked at `[0.512, 0.260]`, only Z changes.

---

## Phase 2: Robot Only (no VR)

### Step 2.0 — RDK connection test
```bash
python3 -c "
import flexivrdk, time
robot = flexivrdk.Robot('Rizon4-062174')
if robot.fault():
    robot.ClearFault()
    time.sleep(2)
robot.Enable()
while not robot.operational():
    time.sleep(1)
print('OK — operational')
tcp = robot.states().tcp_pose
print(f'TCP: [{tcp[0]:.4f}, {tcp[1]:.4f}, {tcp[2]:.4f}]')
robot.Stop()
"
```
**PASS:** Prints `OK — operational` and a valid TCP position. No fault.

### Step 2.1 — Verify TCP is inside safety box
Check the TCP from Step 2.0 against `safety_config.py`:
```
SAFE_BOX_MIN = [0.350, -0.200, 0.080]
SAFE_BOX_MAX = [0.650,  0.200, 0.500]
```
**PASS:** All three axes within bounds. If NOT, either:
- Jog the robot inside the box via teach pendant, OR
- Edit `safety_config.py` to fit the current workspace.

### Step 2.2 — Robot teleop with mock VR
```bash
timeout 10 python3 -u teleop/teleop_main.py --mock-vr --rate 50
```
**WATCH THE ROBOT.** It will make small motions following the mock figure-8 pattern.

**PASS criteria:**
- [ ] Safety box prints, TCP shows INSIDE
- [ ] Robot moves gently (small motions, ~few cm)
- [ ] No faults, no jerky motion
- [ ] Exits cleanly on timeout (robot holds position)

**ABORT:** If robot moves unexpectedly, hit Ctrl+C immediately. The signal handler calls `robot.Stop()`.

### Step 2.3 — Robot z-approach with mock VR
```bash
timeout 10 python3 -u teleop/teleop_main.py --mock-vr --rate 50 --mode z-approach
```
**PASS:** Robot only moves in Z (up/down). XY stays locked. Gentle motion.

---

## Phase 3: VR Only (no robot)

### Step 3.0 — SteamVR running
- Open SteamVR on PC
- Quest 3 connected via Link
- Both controllers visible in SteamVR dashboard (green icons)

### Step 3.1 — VR reader test
```bash
python3 teleop/vr_reader.py
```
**PASS:** Prints position + grip + trigger values that change as you move the controller.
If fails with "No VR controller found" — check SteamVR connection.

### Step 3.2 — Teleop with mock robot
```bash
timeout 15 python3 -u teleop/teleop_main.py --mock-robot --rate 90
```
Move the right controller around. Press grip to engage.

**PASS criteria:**
- [ ] `engaged=False` when grip released, `engaged=True` when pressed
- [ ] Printed position changes when you move while engaged
- [ ] Position freezes when you release grip (clutching works)
- [ ] Re-engaging from a different hand position doesn't jump (offset reset works)

---

## Phase 4: Full Integration

### Step 4.0 — Edit safety config for first test
Before connecting VR to robot, tighten the box:
```python
# In safety_config.py — conservative for first test:
SAFE_BOX_MIN = np.array([0.40, -0.10, 0.15])
SAFE_BOX_MAX = np.array([0.60,  0.10, 0.45])
MAX_LINEAR_VEL = 0.05    # 5 cm/s — very slow
```

### Step 4.1 — Full teleop (SLOW, tight box)
```bash
python3 -u teleop/teleop_main.py --mode full --rate 50
```
- Hand on E-stop
- Press grip to engage
- Move controller slowly
- Release grip to disengage

**PASS criteria:**
- [ ] Robot follows VR controller motion
- [ ] Motion is smooth (no jitter, no jumps)
- [ ] Releasing grip freezes robot
- [ ] Re-engaging doesn't cause jumps
- [ ] Status prints show reasonable positions
- [ ] Ctrl+C stops cleanly

### Step 4.2 — Haptic feedback test
While running Step 4.1:
- Gently push on the robot end-effector with your hand
- You should feel vibration in the VR controller proportional to the push force

**PASS:** Controller buzzes when you push the robot. Stronger push = stronger buzz. No buzz when no contact.

### Step 4.3 — Z-approach on phantom
Place phantom at known position. Update `safety_config.py`:
```python
Z_APPROACH_XY_POS = np.array([0.512, 0.260])  # your phantom center
```
```bash
python3 -u teleop/teleop_main.py --mode z-approach --rate 50
```
- Engage grip
- Lower your hand → probe descends toward phantom
- Feel contact through haptic buzz
- Raise hand → probe retracts

**PASS criteria:**
- [ ] XY stays locked over phantom
- [ ] Z follows VR controller height
- [ ] Soft contact on surface (no slam, impedance absorbs it)
- [ ] Haptic buzz on contact
- [ ] Raising hand lifts probe cleanly

### Step 4.4 — Widen box and increase speed
If Step 4.1–4.3 all pass, open up the limits:
```python
SAFE_BOX_MIN = np.array([0.35, -0.20, 0.08])
SAFE_BOX_MAX = np.array([0.65,  0.20, 0.50])
MAX_LINEAR_VEL = 0.10    # 10 cm/s
```
Re-run Step 4.1. Confirm still smooth at wider range.

---

## Phase 5: Data Validation

### Step 5.0 — Check logged data
After any real teleop run:
```bash
ls -la teleop/logs/
# Pick latest file:
python3 -c "
import json
with open('teleop/logs/teleop_XXXXXXXX_XXXXXX.jsonl') as f:
    lines = f.readlines()
print(f'Samples: {len(lines)}')
first = json.loads(lines[0])
last = json.loads(lines[-1])
print(f'Duration: {last[\"t\"] - first[\"t\"]:.1f}s')
print(f'First pos: {first[\"robot_pos\"]}')
print(f'Last pos:  {last[\"robot_pos\"]}')
print(f'Has wrench: {any(abs(v) > 0.1 for v in last[\"wrench\"])}')
"
```
**PASS:** Correct sample count (~rate × duration), valid positions, wrench data present.

---

## Sign-Off

| Phase | Step | Pass? | Notes |
|---|---|---|---|
| 1 Software | 1.0 Mock full | | |
| 1 Software | 1.1 Mock z-approach | | |
| 2 Robot | 2.0 RDK connect | | |
| 2 Robot | 2.1 TCP in box | | |
| 2 Robot | 2.2 Mock VR + real robot | | |
| 2 Robot | 2.3 Mock VR + z-approach | | |
| 3 VR | 3.0 SteamVR up | | |
| 3 VR | 3.1 VR reader test | | |
| 3 VR | 3.2 Mock robot + real VR | | |
| 4 Full | 4.0 Tighten safety box | | |
| 4 Full | 4.1 Full teleop | | |
| 4 Full | 4.2 Haptic test | | |
| 4 Full | 4.3 Z-approach phantom | | |
| 4 Full | 4.4 Widen limits | | |
| 5 Data | 5.0 Check logs | | |
