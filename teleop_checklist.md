# VR Teleoperation System — Development Checklist
*Bimanual Franka Panda · HTC Vive Pro Eye · MuJoCo Digital Twin · Intercontinental Teleoperation*

---

## PHASE 1 — Stabilise (prerequisite for meaningful sim testing)

### 1.1 MuJoCo Simulation Fidelity
- [ ] Tune gravity compensation so arm behaves like real hardware under load (not zero-g)
- [ ] Re-introduce realistic friction values without causing instability
- [ ] Validate that twin feels representative enough to develop operator intuitions on

### 1.2 Command Plausibility & Safety Guards
- [ ] Add plausibility check on incoming head commands (delta cap, NaN/degenerate quaternion rejection)
- [ ] Add plausibility check on incoming arm commands (velocity cap, position delta limit, NaN guard)
- [ ] Decide on cap-vs-reject policy and make it consistent across devices

### 1.3 Arm-to-Arm Collision Safety
- [ ] Define bounding volume representation for each arm (simple boxes or spheres)
- [ ] Implement minimum-distance check in the control loop
- [ ] Define response behaviour: velocity scaling, soft stop, or hard freeze
- [ ] Validate in simulation before any hardware use

### 1.4 Per-Device Reset to Home
- [ ] Implement home-pose reset command per device (arm left, arm right, head)
- [ ] Use joint position controller path to avoid Jacobian singularity issues
- [ ] Expose reset trigger from operator interface (tray menu or dedicated button)
- [ ] Test smooth transition: ENGAGED → homing → AWAITING

### 1.5 Tray Menu — Button Selection
- [ ] Resolve stereo layer UMG raycasting incompatibility (choose: scene-rendered or custom hit-test)
- [ ] Implement dwell-based or gaze-confirm button selection
- [ ] Populate with initial set: view toggles, device reset triggers, control mode switches
- [ ] Live-tune ExpandedWidth in Details panel

---

## PHASE 2 — Harden (safety and reliability for real hardware)

### 2.1 Avatar-Side Error Handler
- [ ] Handle Franka reflex errors (torque, velocity, joint position limit violations)
- [ ] Handle self-collision and arm-to-arm collision faults
- [ ] Handle per-device communication loss with configurable timeout
- [ ] Handle increased latency / stale command detection
- [ ] Distinguish self-clearing faults (comm loss → auto-recover) from intervention-required faults (joint limit → must move away first)
- [ ] Log all fault events with timestamps

### 2.2 Graceful Shutdown & Safe Park
- [ ] Define avatar behaviour on operator machine crash or clean disconnect
- [ ] Implement: hold position → go compliant → attempt home → halt (choose level)
- [ ] Ensure this triggers regardless of which network path drops
- [ ] Test: kill operator process mid-operation in simulation

### 2.3 Interface-Side System State Visualisation
- [ ] Display per-device status (OFFLINE / IDLE / ENGAGED / FAULTED) for: arm left, arm right, head, video
- [ ] Visualise communication health per device (not just video pipeline)
- [ ] Communicate device loss clearly to operator during operation
- [ ] Distinguish degraded-but-running from hard-lost

### 2.4 Input Device Loss Detection
- [ ] Detect controller disconnect during session
- [ ] Define and implement response: pause, notify operator, safe hold
- [ ] Battery level query — assess if queryable at runtime via UE5/OpenXR; deprioritise if not

### 2.5 libfranka Integration & Real Hardware Compilation
- [ ] Add libfranka linkage to CMake build (conditional, flag-gated)
- [ ] Verify real hardware backend compiles cleanly against libfranka
- [ ] Ensure YAML config correctly switches between MuJoCo backend and real hardware
- [ ] Smoke test: connect single arm in low-speed joint impedance mode

### 2.6 Bring-Up Protocol Document
- [ ] Write session start checklist: network (ZeroTier), NTP sync check, avatar launch order, interface launch order
- [ ] Write shutdown / emergency procedure
- [ ] Define minimum viable system state before operator engages (all devices IDLE or better)
- [ ] Keep this as a living document, update after each hardware session

---

## PHASE 3 — Instrument (data collection readiness)

### 3.1 Interface-Side Logging (Restore & Extend)
- [ ] Restore logging module from older project version into current codebase
- [ ] Align timestamps with NTP/chrony clock (same reference as avatar-side logs)
- [ ] Log: gaze direction + confidence, controller pose, button events, state transitions, tray interactions
- [ ] Log: per-device health status over time

### 3.2 Latency Characterisation
- [ ] Measure full round-trip: command sent → robot moves → video frame captured → decoded → displayed
- [ ] Separate contributions: control latency vs video encode/transmit/decode latency
- [ ] Document baseline over ZeroTier at local range before intercontinental test
- [ ] Store latency logs in a format compatible with avatar-side arm logs

### 3.3 Video Logging with Gaze Overlay
- [ ] Map stereo layer render targets onto canvas (as in older project)
- [ ] Implement low-FPS buffered capture thread (JPEG/PNG per frame)
- [ ] Log video frame timestamps aligned with gaze data timestamps
- [ ] Post-processing: stitch frames to video, fuse with gaze data to produce attention map video
- [ ] Define storage format and path convention

### 3.4 Gaze-to-World Coordinate Mapping
- [ ] Collect/confirm camera intrinsics for RealSense D455i
- [ ] Implement pixel → world ray projection using intrinsics + known geometry
- [ ] Log gaze-in-world estimates per frame with timestamps
- [ ] Validate projection visually before relying on it for intention recognition

### 3.5 Haptic Feedback — Contact Forces
- [ ] Confirm F_ext data reaches interface side (already logged on avatar side)
- [ ] Define force threshold and mapping to vibration intensity
- [ ] Implement controller rumble via UE5 haptic feedback API
- [ ] Tune threshold in simulation before hardware (avoid constant buzz)

### 3.6 Audio Feedback — Grasp Confirmation
- [ ] Confirm grasp success signal exists in data stream (gripper closed + F_ext threshold?)
- [ ] Implement audio cue on successful grasp (spatial or head-locked)
- [ ] Consider: leverage audio from handheld device if latency permits

---

## PHASE 4 — Future / Research (not on critical path to first hardware test)

- [ ] Parallel twin + real hardware arbitration and coordination
- [ ] Model-mediated teleoperation under high latency
- [ ] Shared autonomy layer (core thesis contribution)
- [ ] EE-frame vs world-frame rotation — evaluate empirically once sim fidelity is solid; consider making it a runtime config parameter
- [ ] Gaze-to-scene reprojection for 3D scene interaction
- [ ] LLM + gaze context for voice intent resolution
- [ ] Voice service integration into main system
- [ ] Intercontinental latency testing and characterisation

---

## System Overview (reference)

| Subsystem | Status |
|---|---|
| Gaze-activated tray expand/collapse | ✅ Working |
| Tray button selection | ❌ Blocked (stereo layer raycasting) |
| State machine (OFFLINE→IDLE→HOMING→AWAITING→ENGAGED→PAUSED) | ✅ Working |
| Head + arm teleoperation (world-frame rotation deltas) | ✅ Working |
| Quaternion wire convention [W, X, -Y, Z] | ✅ Fixed |
| Clutch / ratcheting | ✅ Working |
| Emergency stop (both controllers) | ✅ Working |
| UDP send-and-forget stream (device commands) | ✅ Working |
| Reliable UDP channel (state + infrequent commands) | ✅ Working |
| Video GStreamer pipeline (encode/transmit/decode) | ✅ Working |
| Video health monitor + adaptive degradation | ✅ Working |
| Live 2D video stats plot (latency, jitter, FEC loss) | ✅ Working |
| Stereo layer video display | ✅ Working |
| RealSense D455i source + shared memory ring buffer | ✅ Working |
| MuJoCo backend (bimanual Panda + pan-tilt, gravity comp) | ✅ Running (fidelity TODO) |
| Pinocchio forward kinematics | ✅ Working |
| YAML config-driven component selection | ✅ Working |
| Avatar-side arm logging (q, q_cmd, dq, tau, F_ext, EE pose) | ✅ Working |
| Avatar-side head logging (q, q_cmd, dq, tau) | ✅ Working |
| NTP/chrony time sync across machines | ✅ Running |
| ZeroTier overlay network | ✅ Running |
| Voice service (STT/TTS/VAD/intent) | 🔵 Exists, deferred |
| Real hardware (libfranka) compilation | ❌ Not yet linked |
| Interface-side logging | ❌ Needs restoration |

---

*Last updated: 2026-03-27*
