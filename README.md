## SIDE_QUEST

Software stack for the Side Quest XR-mediated human–robot construction system at the Australian Rover Challenge.

This repository contains the XR interface, robotic control logic, and integration layers required to coordinate a Dobot Magician E6 under supervised autonomy. The system enables human-in-the-loop decision authority over robot motion through XR visualization and graded tolerance logic.

### System Architecture
The project is structured into three primary layers:
SIDE_QUEST_ROBOTICS/
  XR/            # Unity project, XR interface, visualization logic
  Robotics/      # Motion control, perception, drift injection, safety
  Integration/   # XR ↔ Robotics communication (ROS / middleware)
  docs/          # Architecture, setup guides, technical notes

XR
- Stereo video ingestion
- Tolerance visualization (G/Y/R logic)
- View orchestration
- User command interface (DROP / FIX / NUDGE)

Robotics
- Motion primitives (Dobot E6)
- Drift injection logic
- Safety gating
- Perception and coordinate transformation

Integration
- Communication layer between Unity and robotics stack
- State synchronization
- Authority handshake logic

### Folder Structure
This mirrors your system architecture (XR / Robotics / Integration / Docs):

SIDE_QUEST_ROBOTICS/
  XR/
    unity/                 # Unity project (or submodule)
    tools/                 # calibration scripts, utilities

  Robotics/
    motion/                # dobot motion primitives, scripts
    perception/            # camera, depth, aruco, detection
    control/               # drift injection, safety gating
    configs/               # robot config, calibration files
    scripts/               # run scripts

  Integration/
    rosbridge/             # Unity <-> ROS interface
    messages/              # message definitions, schemas
    state_sync/            # digital twin sync glue

  docs/
    architecture/          # diagrams, interfaces, decisions
    setup/                 # "how to run" guides
    meetings/              # notes, decisions (brief)

  .gitignore
  README.md

### Tech Stack
- Python 3.10+
- OpenCV / NumPy
- PyTorch (vision models)
- Dobot API
- Unity (XR runtime)
- ROS / rosbridge (integration layer)

### Workflow & Safety Rules (Mandatory)
As we are working with a physical 6-axis robot, safety and code integrity are non-negotiable.

1. The Branching Strategy. Never push directly to main.
2. Pull Request (PR) Policy. All PRs require at least 1 approval before merging. No Self-Approvals: You cannot approve your own PR. Code Review: The Lead Developer (@isthismanas) must be tagged for final review to ensure motor safety and coordinate limits.

### Physical Safety Requirements
This system controls a physical 6-axis robot.
- Initial motion tests must run at SpeedFactor(10)
- Emergency Stop must be accessible during deployment
- Ensure 450mm workspace clearance

Safety rules are mandatory.

### Naming Conventions
Python: lowercase_with_underscores
Unity C#: PascalCase
Separate modules clearly under XR/, Robotics/, Integration/

### Maintainer
Lead Developer: Manas Darekar
Project Lead: Albert Rajkumar

Contact:
- isthismanas@gmail.com/ a1990782@student.adelaide.edu.au
- albertrk@gmail.com / albert.rajkumar@adelaide.edu.au
