Robotics Technical Brief (v7)

Project: Side Quest: The Leaning Tower of Regolith (ARC 2026 Target)
Sub-Team: Robotics
Last Updated: 24 Feb 2026 (Dev 9 – Fault Recovery Stabilized; Drift Integration Next)

================================================================

1. Objective

Develop a modular, deterministic robotic control pipeline for a Dobot Magician E6 performing semi-autonomous lunar block stacking under a Hover–Commit supervisory control paradigm.

The system supports:

Autonomous pick and approach

Hover pause for human evaluation

Human authorization to DROP or FIX

Limited teleoperation (NUDGE mode)

Autonomous continuation to next block

Combo-based speed boost mechanics

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

5. Drift Injection Architecture (Dev 10 – In Progress)

Drift is injected at pose proposal stage:

Detection → Base Transform → Drift Injection → Proposed Pose → Control Layer

Properties:

Z-axis excluded

Bounded magnitude

Deterministic per attempt (seedable)

Configurable via robot_config.py

Does not alter retreat geometry

Does not bypass safety envelopes

Drift modifies only proposed placement pose.
Motion execution remains deterministic.

================================================================

6. Vision Pose Integration (Upcoming Phase)

Planned pipeline:

Aruco Detection → Pose Estimation → Target Comparison → Delta Computation → Verdict Classification

Delta:

delta = target_pose – measured_pose

Verdicts:

GREEN – tight tolerance
YELLOW – moderate deviation
RED – large deviation

Computed on Pi.
Rendered in Unity.

Vision does not sequence motion.

================================================================

7. Combo Mode

If 3 consecutive GREEN placements:

Increase SpeedFactor by 10–20%

Applies only to travel states.
Precision states (NUDGE, PLACING) remain capped.
Resets on YELLOW or RED.

================================================================

8. Safety Architecture

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

9. Current Development Status (Dev 9 Complete)

Completed:

Deterministic 7-block stacking

Stabilized retreat geometry

State machine modularization

Admin recovery port (8089)

AUTO_RECOVER routine

FAULT state enforcement

Continue() hardened

Graceful shutdown (no DepthAI abort)

VR-session arming model stable

Next Phase (Dev 10):

Drift Engine integration

Vision pose estimation integration

Tolerance engine activation

XR overlay of proposed vs measured pose

Full-cycle Drift + Vision stress testing

================================================================

10. Software Layer Separation

The robotics control stack is divided into:

task_controller.py (TCP + orchestration)

state_machine.py (transition rules + gating)

actions.py (robot + gripper side-effects)

dobot_driver.py (Dobot TCP driver)

dh_gripper.py (RS485 Modbus driver)

robot_config.py (configuration + pose definitions)

(Upcoming) drift_engine.py

(Upcoming) vision_engine.py

This separation reduces cross-module coupling and improves robustness for ARC deployment.

================================================================