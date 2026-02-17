# SIDE_QUEST_ROBOTICS
Robotics Team for the SIDE QUEST experience, a project by Albert Rajkumar,  at the Australian Rover Challenge
## Project: Spatially Aware Robot 
Autonomous 6-Axis Spatial Awareness using Stereo Vision

This repository contains the software stack for the Dobot Magician E6 to achieve autonomous spatial awareness. Using two synchronized camera feeds (Binocular Vision), the system identifies objects in 3D space, calculates their coordinates, and executes precision manipulation.

### Project Overview
Timeline: 3 Weeks (Sprints: Vision, Kinematics, Integration)

Robot: Dobot Magician E6 (6-Axis Cobot)

Vision: Stereo RGB Camera setup

AI/ML: Object Detection (YOLO) + Coordinate Transformation

### Tech Stack
Language: Python 3.10+

Libraries: opencv-python, numpy, dobot-api, torch

Environment: Best run on Windows/Linux (Recommended: NVIDIA GPU for vision processing)

Hardware: Dobot E6, 2x USB Webcams, Ethernet connection



### Workflow & Safety Rules (Mandatory)
As we are working with a physical 6-axis robot, safety and code integrity are non-negotiable.

1. The Branching Strategy
Never push directly to main.

Create a feature branch for every task: git checkout -b feature/your-task-name.

2. Pull Request (PR) Policy
All PRs require at least 1 approval before merging.

No Self-Approvals: You cannot approve your own PR.

Code Review: The Lead Developer (@YourGitHubUsername) must be tagged for final review to ensure motor safety and coordinate limits.

3. Physical Safety
Speed Factor: Initial tests must always run at SpeedFactor(10) (10% speed).

E-Stop: The physical Emergency Stop button must be held during all new code deployments.

Workspace: Ensure the E6's 450mm radius is clear of obstacles.

###  Quick Start
Clone the repo:


Install Dependencies:



Milestones


[ ] Week 1: Stereo Camera Calibration & Depth Map generation.

[ ] Week 2: Hand-Eye Calibration (Mapping pixels to robot mm).

[ ] Week 3: Full Autonomy Loop (See -> Calculate -> Pick).

Lead Developer: Manas Darekar

Contact:isthismanas@gmail.com/ a1990782@student.adelaide.edu.au
