Robotics Technical Brief (v9)

Project: Side Quest: The Leaning Tower of Regolith (ARC 2026 Target)
Sub-Team: Robotics
Last Updated: 21 Mar 2026 (Dev 13 – Placement Softening)

Dev 24 Note (UI-side): No functional robotics pipeline changes; only XR operator UI interaction-layer updates.


================================================================

1. Objective

Develop a modular, deterministic robotic control pipeline for a Dobot Magician E6 performing semi-autonomous lunar block stacking under a Hover–Commit supervisory control paradigm.

The system supports:

Autonomous pick and approach

Hover pause for human evaluation

Human authorization to DROP or FIX

Limited teleoperation (NUDGE mode)

Autonomous continuation to next block

Vision-driven performance classification (GREEN/YELLOW/RED) enabling combo-based speed boost mechanics

Deterministic retreat geometry

Automatic fault detection and recovery

Controlled perceptual uncertainty via drift injection (Dev 10)

Architecture separation:

Vision (pose estimation + drift injection layer)

Control (state machine + sequencing)

Motion (robot + gripper execution)

XR (UI + visualization only)

Unity never controls raw robot motion.

================================================================

2. System Architecture Overview
Hardware Stack

Robot: Dobot Magician E6 (TCP/IP – Port 29999)

Gripper: DH Robotics PGE Series (RS485 Modbus RTU)

Control Node: Raspberry Pi 5

Vision: 2× OAK-D Pro PoE

XR Interface: Unity (Meta Quest 3 via Link)

Network Segmentation

Robot Subnet (192.168.5.x)

Robot: 192.168.5.1

Pi Alias: 192.168.5.10

XR / Vision Subnet (169.254.1.x)

Pi Primary: 169.254.1.10

Laptop: 169.254.1.5

Ports:

8088 – Unity supervisory control

8089 – Admin recovery port (localhost only)

8085 – Inspector video stream

8086 – Site Manager video stream

Unity sends high-level intent only.
Pi executes all motion primitives.

================================================================

3. Control Architecture (Task Controller)
Implemented States (Dev 9)

IDLE
MOVING_TO_PICK
MOVING_TO_TOWER_HOVER
WAITING_FOR_DECISION
PLACING
FAULT

Core Loop (Deterministic – Dev 8 Stable)

START →
Pick sequence (MovJ hover → MovL descend → grip → MovL retract → MovJ exit) →
Move to tower hover →
WAITING_FOR_DECISION →
DROP → complete_place_sequence() →
Auto-continue →
Repeat until 7 blocks complete

Auto-continue logic is internal to task_controller.py and does not require Unity to re-trigger START.

Fault Handling (Dev 9)

RobotMode polling continuously monitors for 9 / 11.

On detection:

STATE → FAULT

Motion blocked

START/DROP gated

Only allowed exits: AUTO_RECOVER, HOME, CLEAR_FAULT

AUTO_RECOVER performs:

ClearError()

EnableRobot()

Safe home move

Gripper open

STATE reset to IDLE (on success)

No Dobot Studio required for routine alarm clearing.

State transitions are gated BEFORE motion execution to prevent inconsistent state advancement.

================================================================

4. Motion Driver Layer

Handles:

ClearError()

EnableRobot()

SpeedFactor()

MovJ

MovL

RelMovLUser (nudges)

Safe Home pose

RS485 gripper control

Continue() hardened (non-fatal on failure)

Robot is armed once per VR TCP session.

robot_armed gate blocks motion when VR disconnects.

Motion side-effects are isolated in actions.py.
task_controller.py does not directly execute robot primitives.

================================================================

4.1 Gripper Control Architecture (RS485 – Stable)

Hardware:

DH Robotics PGE Series

USB–RS485 (FT232)

Interface: /dev/ttyUSB0

Protocol:

Modbus RTU

pymodbus + pyserial

Validated Registers:

0x0200 – Initialization state

0x0201 – Grip command

0x0202 – Current position

Calibrated Positions:

Open = 900

Close = 50

Characteristics:

Position-based deterministic control

No toggle semantics

No Continue() dependency

Holds position without drift

================================================================

4.2 Retreat Geometry & IK Constraints (Validated – Dev 8)

Observed constraint:

IK infeasible above ≈430 mm tower vertical axis

Final stabilized retreat strategy:

MovL vertical retract to tower hover

For levels ≥5: linear +Y sidestep to Y ≈ -10 mm

MovJ to NEUTRAL_3 safe pose

Ensures:

No diagonal sweep through tower envelope

No IK overshoot above ceiling

Deterministic escape at full height

Validated for full 7-block autonomous stacking.

================================================================

5. Drift Injection Architecture (Dev 10–12 Stable)
Properties:

• Z-axis excluded
• Orientation excluded
• Bounded magnitude (configurable)
• Adjustable via DRIFT_SCALE
• Does not modify retreat geometry
• Does not bypass safety envelopes


### Dev Update – 6 Mar 2026

- Added per-run drift randomization (`runtime_run_seed`) while preserving deterministic replay via forced seed override.
- Effective seed now combines baseline `DRIFT_RUN_SEED` + runtime run seed + participant name + stack level.
- Drift is injected during the tower hover move (no post-hover correction move), reducing visible entry jerk into decision state.
- OFFICIAL session logs now include drift metadata: participant, runtime seed, baseline seed, drift scale at run start, and per-level drift `dx/dy`.
- Runtime console verbosity now supports `DEBUG ON/OFF`; DEBUG OFF suppresses non-critical diagnostics while fault/collision warnings remain visible.
- Quiet mode suppresses repeated READY/camera/VR informational chatter; critical fault outputs remain intentionally visible.
 
• RED placements increase DRIFT_SCALE incrementally

Drift modifies only the proposed placement pose.
Motion execution remains deterministic and safety-gated.

================================================================

6. Tolerance Engine (Dev 12 – Implemented)

Classification occurs deterministically on the Raspberry Pi.

The proposed placement pose (after drift + nudges) is evaluated against
the tower base reference using radial XY distance:

radial_error_mm = sqrt((x - x0)^2 + (y - y0)^2)

Thresholds (configurable in robot_config.py):

TOL_GREEN_MM
TOL_YELLOW_MM
TOLERANCE_SCALE

Final ARC configuration (Dev 12):

GREEN:
radial_error_mm <= TOL_GREEN_MM * TOLERANCE_SCALE

YELLOW:
radial_error_mm <= TOL_YELLOW_MM * TOLERANCE_SCALE

RED:
exceeds YELLOW threshold

Characteristics:

• Radial (axis-independent) classification
• Evaluated:
    - At WAITING_FOR_DECISION
    - After every NUDGE
    - Immediately after successful DROP
• Computed exclusively on Pi
• Unity receives verdict only (ZONE GREEN/YELLOW/RED)

Classification does not influence motion sequencing.
It influences combo logic and visual feedback only.

================================================================

7. Vision Pose Integration (Upcoming Phase)

Planned pipeline:

Aruco Detection → Pose Estimation → Target Comparison → Delta Computation → Verdict Classification

Delta:

delta = target_pose – measured_pose

Verdicts:

GREEN – tight tolerance
YELLOW – moderate deviation
RED – large deviation

Classification Logic (Deterministic on Pi):

Measured pose is compared against the proposed placement pose
(after drift and nudge adjustments).

GREEN:
radial_error_xy <= X mm AND |dyaw| <= Y deg

YELLOW:
radial_error_xy within secondary bound

RED:
exceeds tolerance envelope

Classification occurs immediately after placement completion
before auto-continue is triggered.

Computed on Pi.
Rendered in Unity.

Vision does not sequence motion.

================================================================

8. Combo Mode (Dev 12)

Activation Rule:

3 consecutive successful GREEN placements
trigger Combo Mode.

Behavior:

• Speed boost activates immediately after the triggering DROP.
• Boost applies only to MoveJ travel segments.
• Boost persists until streak breaks.
• MovL, vertical descents, gripper motions, and nudges remain at precision speed.

Speed Model:

MOVEJ_SPEED_NORMAL
MOVEJ_SPEED_COMBO

MoveJ speed is explicitly set before every MoveJ call.
No percentage-bonus arithmetic is used.

Combo resets on:

• YELLOW placement
• RED placement
• FIX invocation #nope
• NUDGE invocation #nope
• FAULT state
• TUMBLE event
• Session reset

Logging:

[COMBO] <participant> combo achieved: 3x GREEN placements
[COMBO] combo ended
[COMBO] MoveJ speed set to X%

Combo affects travel tempo only.
It does not alter precision or safety constraints.

================================================================

9. Safety Architecture

LED states:

Green – Enabled

Yellow – Collision

Red – Alarm

Controller enforces:

Z floor

XY envelope

FAULT state on RobotMode 9 / 11

Recovery-only exits from FAULT

Disarm on Unity disconnect

Motion gating before state transition

Continue() failures no longer crash controller threads.

================================================================

10. Run Completion & Metrics (Dev 12)

A run is considered COMPLETE when 7 successful placements occur.

On completion:

System waits 5 seconds (stabilization delay)
Console prints:

<participant> successfully placed 7 blocks in <seconds> seconds (COMPLETE)

If tumble occurs before completion:

<participant> successfully placed <n> blocks in <seconds> seconds (TUMBLE)

Characteristics:

• Elapsed time excludes post-completion delay
• Computed entirely on Pi
• Logged in COMP mode
• No Unity dependency required

(Planned next phase)
Structured experiment logging to CSV
will replace console-only logging for research reproducibility.

================================================================

11. Current Development Status (Dev 12 Stable)

Completed:

• Deterministic 7-block stacking
• Stabilized retreat geometry
• Fault detection & AUTO_RECOVER
• Drift injection (deterministic)
• Tolerance engine (radial classification)
• World-space ZONE visualization in Unity
• Combo Mode Flow B (MoveJ-only speed boost)
• Immediate combo activation timing
• Run completion metrics (COMP / TUMBLE)
• Nudge UX stabilized (non-ACK mode)
• Continue() hardened
• VR-session arming model stable

Next Phase:

• Structured CSV experiment logging
• Re-enable Y-axis drift (after bidirectional nudge UI)
• Vision-based pose estimation integration
• Full drift + vision stress testing
• ARC reliability hardening & operator UX polish

================================================================

10. Software Layer Separation

task_controller.py        – TCP + orchestration
state_machine.py          – transition rules + gating
actions.py                – robot + gripper side-effects
dobot_driver.py           – Dobot TCP driver
dh_gripper.py             – RS485 Modbus driver
drift_engine.py           – deterministic pose perturbation
tolerance_engine.py       – radial classification logic
robot_config.py           – configuration + speed profiles

(Upcoming)
vision_engine.py          – camera-based pose estimation
experiment_logger.py      – structured CSV run logging

================================================================

Dev Note: Added `PLACE_Z_SOFTEN_MM` (default +1.5 mm) in `robot_config.py`. Applied only in explicit build-point mode inside `build_target_pose()`. Raises the final place Z by this offset to reduce downward compression on contact. Hover Z remains derived from the adjusted place pose (+40 mm clearance), so approach geometry is self-consistent.