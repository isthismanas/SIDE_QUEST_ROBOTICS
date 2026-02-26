XR Technical Brief (v5)

Project: Side Quest: The Leaning Tower of Regolith (ARC 2026) Target
Sub-Team: XR Last Updated: 23 Feb 2026 (Aligned with Robotics Dev 7)

================================================================

1.  Executive Summary

The XR system provides a low-latency stereoscopic “Robot’s Eye View”
combined with a dedicated Command Highway for robotic actuation.

Architecture: Asymmetric Hybrid System

-   High-speed video handled via raw TCP sockets
-   Robotic control handled via Raspberry Pi Task Controller
-   Unity acts only as supervisory interface

XR does NOT sequence robot motion. XR does NOT compute placement
evaluation. XR sends high-level intent only.

================================================================

2.  Hardware Stack

Sensors: - 2x Luxonis OAK-D Pro PoE Camera A (.222) – “Inspector” (High
Oblique) Camera B (.223) – “Site Manager” (Side Elevation)

Mediator: - Raspberry Pi 5 (8GB, active cooling)

Actuator: - Dobot Magician E6

Operator Station: - Windows 11 Laptop - Unity 6 (URP) - Meta Quest 3
(Quest Link)

Networking: - 1Gbps wired PoE backbone - HP USB-C Dock G5

================================================================

3.  Network Configuration

Robot Subnet: 192.168.5.x - Robot: 192.168.5.1 - Pi Alias: 192.168.5.10

XR / Vision Subnet: 169.254.1.x - Pi Primary: 169.254.1.10 - Laptop:
169.254.1.5

Video Streams: Port 8085 – Inspector feed Port 8086 – Site Manager feed

Control Channel: Port 8088 – Unity ↔ Pi command channel

================================================================

4.  Video Pipeline

4.1 Hardware Encoding

Left and Right mono streams are JPEG-encoded on the OAK-D Myriad X VPU.

This results in: - Near 0% Pi CPU load for video compression - Thermal
stability - Long-duration reliability

4.2 Temporal Synchronization

OAK-D Sync node ensures: - Left/Right frames captured at same
millisecond - No stereo flicker - Stable 3D perception

4.3 Multithreaded Video Bridge

Pi runs threaded Python TCP servers:

-   Port 8085 → Inspector
-   Port 8086 → Site Manager

Each stream handled independently to avoid blocking and latency spikes.

================================================================

5.  Command Highway (Supervisory Control)

Dedicated TCP control channel (Port 8088).

Protocol: - UTF-8 string packets - Stateless command triggers

Example Commands:

PICK DROP FIX NUDGE dx dy NUDGE_YAW dtheta CANCEL HOME

Flow: Unity sends command → Pi receives → Task Controller transitions
state → Motion Driver executes

Video stream is fully isolated from control channel.

================================================================

6.  Operator Interface Design

6.1 Diegetic Control Room

Video projected onto curved industrial monitor.

Design intent: XR acts as “window into site” not floating HUD.

6.2 Stereo Rendering

Custom Shader Graph:

-   Dual texture inputs
-   Eye Index node branches per eye
-   Each eye receives correct camera feed

6.3 World-Space UI

Buttons mounted physically on monitor bezel:

-   View switching
-   FIX
-   DROP
-   NUDGE controls
-   HOME
-   START

Buttons trigger TCP commands only.

================================================================

7.  Current Development Status

Completed:

-   Stable dual-camera stereo stream
-   Hardware-encoded JPEG pipeline
-   Threaded TCP video bridge
-   Working Unity shader graph
-   Functional Unity ↔ Pi command link
-   New command: START (begins autonomous pick → tower hover → WAITING_FOR_DECISION)

In Progress:

-   Full robot motion integration (Dev 7 alignment)
-   VR button → state machine mapping
-   Overlay projection for tolerance visualization

================================================================

8.  Future Roadmap

Phase 1 – Motion Primitives Integration - Complete PICK / DROP loop via
XR

Phase 2 – Tolerance Engine - Aruco pose tracking on Pi - Alignment
overlays in Unity

Phase 3 – Drift Visualization - Visualize proposed vs corrected pose -
Ghost brick projection

Phase 4 – Robustness - Lost connection handling - Reconnect logic -
Frame-drop mitigation

================================================================

9.  Safety & Control Boundaries

Unity: - Cannot directly send joint commands - Cannot change speed
factors - Cannot override safety constraints

All safety logic resides on the Pi Task Controller.

XR is supervisory layer only.

================================================================

cd ~/SIDE_QUEST_ROBOTICS/Robotics/motion
./run_controller.sh
