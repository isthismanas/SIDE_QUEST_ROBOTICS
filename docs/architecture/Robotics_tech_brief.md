Robotics Technical Brief (v6)

Project: Side Quest: The Leaning Tower of Regolith (ARC 2026) Target
Sub-Team: Robotics Last Updated: 20 Feb 2026 (Dev 7 – RS485 Integration
Complete)

================================================================

1.  Objective

Develop a modular, deterministic robotic control pipeline for a Dobot
Magician E6 performing semi-autonomous lunar block stacking under a
Hover–Commit supervisory control paradigm.

The system must support: - Autonomous pick and approach - Hover pause
for human evaluation - Human authorization to Drop or Fix - Limited
teleoperation (NUDGE mode) - Autonomous continuation to next block -
Combo-based speed boost mechanics - Controlled perceptual uncertainty
via drift injection

Architecture separation: - Vision (pose estimation + drift) - Control
(state machine + sequencing) - Motion (robot + gripper execution) - XR
(UI + visualization only)

Unity never controls raw robot motion.

================================================================

2.  System Architecture Overview

Hardware Stack: - Robot: Dobot Magician E6 (TCP/IP – Port 29999) -
Gripper: DH Robotics PGE Series (RS485 Modbus RTU) - Control Node:
Raspberry Pi 5 - Vision: 2× OAK-D Pro PoE - XR Interface: Unity (Meta
Quest 3 via Link)

Network Segmentation:

Robot Subnet (192.168.5.x) - Robot: 192.168.5.1 - Pi Alias: 192.168.5.10

XR / Vision Subnet (169.254.1.x) - Pi Primary: 169.254.1.10 - Laptop:
169.254.1.5

Unity sends high-level intent via TCP (Port 8088). Video streams on
Ports 8085 / 8086. Pi executes all motion primitives.

================================================================

3.  Control Architecture (Task Controller)

Current Implemented States (Dev 7):

IDLE
HOVER_WAIT
NUDGE
FAULT

The full stacking state machine (MOVING_TO_PICK → GRIPPING → MOVING_TO_HOVER → PLACING → RETRACTING) is architected but not yet fully implemented.

State Machine Extraction (Dev 7 Refactor)

The state machine logic has been separated into a dedicated module (state_machine.py).

task_controller.py is now orchestration-only.
All motion side-effects are handled in actions.py.

Core Loop: 1. Move to Pick pose 2. Close gripper (RS485 deterministic
command) 3. Move to Hover pose 4. WAITING_FOR_DECISION

At decision state: - DROP → PLACING - FIX → NUDGE_MODE

NUDGE_MODE: - Discrete button-based - XY translation only - Optional
yaw - Z locked - Small increments (3 mm XY, 2° yaw) - Reduced speed

================================================================

4.  Motion Driver Layer

Handles: - ClearError() - EnableRobot() - SpeedFactor() - MovJ - MovL -
RelMovLUser (nudges) - Safe Home pose - RS485 gripper control

Robot is armed once per VR TCP session. robot_armed gate blocks motion
when VR disconnects.

Motion side-effects are now isolated in actions.py.
task_controller.py does not directly execute robot primitives.

================================================================

4.1 Gripper Control Architecture (RS485 – Dev 7)

Hardware: - DH Robotics PGE Series - USB–RS485 (FT232) - Interface:
/dev/ttyUSB0

Protocol: - Modbus RTU - pymodbus - pyserial

Base DO pulse control has been fully removed.

Validated Registers: 0x0200 – Initialization state 0x0201 – Grip command
0x0202 – Current position

Verified Positions: Open = 900 Closed = 50

Actuation Model: Open → write position 900 Close → write position 50

Gripper is position-controlled, deterministic, and repeatable. No toggle
behavior. No Continue() dependency.

Planned: Auto-initialization routine at Task Controller startup.

================================================================

5.  Placement Evaluation (Planned – Not Yet Active)

delta = target_pose - measured_pose

Verdicts: GREEN – tight tolerance YELLOW – moderate deviation RED –
large deviation

Computed on Pi. Displayed in Unity.

================================================================

6.  Combo Mode

If 3 consecutive GREEN placements: - Increase SpeedFactor by 10–20%

Applies only to travel states. Resets on YELLOW or RED.

================================================================

7.  Safety Architecture

LED states: Green – Enabled Yellow – Collision Red – Alarm

Controller must: - Enforce Z floor - Enforce XY envelope - Enter FAULT
on alarm - Allow SAFE_HOME anytime

================================================================

8.  Current Development Status (Dev 7.5 – State Machine Extraction)

Completed:
- State machine split out (state_machine.py)
- Actions module (actions.py) driving internal progression events
- Unity START button wired (sends START)
- Blocking motion achieved via RobotMode() polling (not Sync())
- Pick sequence now uses real Dobot Studio poses (L/R/T/Neutral)
- Working behavior: START picks L4 and holds at T1 hover; DROP places.
- Motion driver refactored (DO-based gripper removed)
- State machine extracted into dedicated module
- Action side-effects separated from orchestration
- VR-session-based arming model stabilized
- TCP command routing hardened against thread crashes
- Unity disconnects can happen when headset removed / camera view switched → robot disarms (by design).

Partially Implemented:
- Minimal pick and drop primitives
- NUDGE mode (XY only)

Not Yet Implemented:
- Full autonomous stacking loop
- Vision-derived pose integration
- Placement verdict computation (G/Y/R)
- Combo mode
- Fault escalation logic

================================================================

9.  Roadmap

Phase 1 – Deterministic Motion Phase 2 – State Machine Integration Phase
3 – Vision Pose Integration Phase 4 – Evaluation + Combo Phase 5 –
Safety Hardening

================================================================

10. Software Layer Separation (Dev 7+)

The robotics control stack is now divided into:

- task_controller.py (TCP + orchestration)
- state_machine.py (transition rules and gating)
- actions.py (robot + gripper side-effects)
- dobot_driver.py (Dobot TCP driver)
- dh_gripper.py (RS485 Modbus gripper driver)
- robot_config.py (single source of truth)

This separation reduces cross-module coupling and improves robustness for ARC deployment.