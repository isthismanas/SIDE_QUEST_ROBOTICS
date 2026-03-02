XR Technical Brief (v5)

Project: Side Quest: The Leaning Tower of Regolith (ARC 2026) Target
Sub-Team: XR 
Last Updated: 26 Feb 2026 (Aligned with Robotics Dev 12 – Tolerance + Combo Active)

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

5. Command Highway (Supervisory Control)

Dedicated TCP control channel (Port 8088).

Protocol:
• UTF-8 newline-delimited string packets
• Stateless high-level intent commands
• Pi remains single source of truth

Example Outbound Commands (Unity → Pi):

START
DROP
FIX
NUDGE dx dy
NUDGE_YAW dtheta
HOME
CANCEL

Inbound Status Messages (Pi → Unity):

ZONE GREEN
ZONE YELLOW
ZONE RED

Unity never evaluates placement correctness.
Zone classification is computed on the Pi and broadcast to XR.

Flow:

Unity sends high-level intent →
Pi Task Controller evaluates state →
Motion Driver executes →
Pi broadcasts updated ZONE →
Unity updates visual indicator

Video stream remains fully isolated from control channel.

================================================================

6. Operator Interface Design (Dev 12)

6.1 Diegetic Control Room

Video feeds are projected onto a curved industrial monitor surface.

Design principle:
XR acts as a "window into the site" rather than a floating HUD.

6.2 Stereo Rendering

Custom Shader Graph:
• Dual texture inputs
• Eye Index node branches per eye
• Each eye receives correct camera feed
• Stable stereoscopic perception

6.3 World-Space Physical Controls

Buttons are mounted physically on the monitor bezel:

• START
• DROP
• FIX
• NUDGE (Left / Right currently active)
• HOME

Buttons send high-level TCP commands only.

6.4 Placement Feedback Indicator (Dev 12)

A world-space ZoneIndicator object (simple sphere mesh) provides
immediate placement classification feedback.

Color Mapping:

GREEN  – Within tight tolerance
YELLOW – Moderate deviation
RED    – Outside tolerance

Characteristics:

• Pure 3D object (no Canvas dependency)
• Updated via RobotCommandPipe.OnZoneChanged event
• Visible in Game view and VR
• No authority over robot motion

XR renders classification only.
XR does not compute classification.

================================================================

7. Current Development Status (Dev 12)

Completed:

• Stable dual-camera stereo stream
• Hardware-encoded JPEG pipeline
• Threaded TCP video bridge
• Stable Unity ↔ Pi command channel
• START command integration (autonomous loop begins from XR)
• Real-time ZONE feedback (GREEN/YELLOW/RED)
• World-space ZoneIndicator visualization
• Stable NUDGE controls (X-axis active)
• Reconnect logic functional

XR now fully aligned with Robotics Dev 12 control loop.

In Progress:

• Bidirectional (XY) nudge UI
• Drift visualization (ghost brick concept)
• Structured run feedback overlays

================================================================

8. Future Roadmap

Phase 1 – Bidirectional Nudge Controls
• Add Y-axis controls
• Re-enable full XY drift challenge

Phase 2 – Drift Visualization
• Ghost brick projection
• Visual comparison of proposed vs corrected pose

Phase 3 – Vision Overlay Integration
• Aruco pose overlays
• Real-time deviation vector display

Phase 4 – Robustness & ARC Hardening
• Lost connection handling UX
• Operator status panel
• Frame-drop mitigation
• Minimalist competition mode UI

================================================================

9.  Safety & Control Boundaries

Unity:

• Cannot send joint-level commands
• Cannot modify speed factors
• Cannot override safety constraints
• Cannot compute placement verdicts

All safety logic resides on the Pi Task Controller.

XR is supervisory and visualization layer only.

Authority remains centralized on the robotics control node.

================================================================

10. Leaderboard & Competition Infrastructure (Dev18)

http://192.168.5.10:8090/

### A) Architecture Decision

- Leaderboard authority is on the Raspberry Pi Task Controller, not Unity.
- Unity remains visualization/supervisory only and does not compute rankings.
- Run finalization happens in controller flow via `finalize_run()` at terminal outcomes.

### B) Data Model

Each finalized leaderboard record includes:

- `run_id`
- `session_id`
- `participant_name`
- `final_height` (bounded by tower target, max 7 in current configuration)
- `completion_time_s`
- `end_state` (`COMPLETE`, `TUMBLE`, `FAULT`, `CANCEL`)
- `mode` (`DEV` or `OFFICIAL`)
- `event_id`

### C) Persistence

- Leaderboard mode/event configuration is persisted at:
  - `logs/leaderboard_mode.json`
- Leaderboard records are persisted to mode-scoped JSONL:
  - `logs/dev/leaderboard.jsonl`
  - `logs/official/leaderboard.jsonl`
- Session event JSONL is OFFICIAL-only and written under:
  - `logs/official/session_<id>.jsonl`

### D) Mode System

Live control commands:

- `MODE DEV`
- `MODE OFFICIAL`
- `MODE SHOW`
- `EVENT <event_id>`

Operational behavior:

- Mode and event ID persist across restarts through `leaderboard_mode.json`.
- Mode switch changes record tagging and default leaderboard view; it does not rewrite historical JSONL.

### E) HTTP API (Port 8090)

- `GET /leaderboard`
  - Query params:
    - `mode`
    - `event_id`
    - `limit`
- `GET /` serves kiosk HTML from the controller process.
- `GET /assets/<file>` serves static kiosk assets (e.g., logos).

================================================================

cd ~/SIDE_QUEST_ROBOTICS/Robotics/motion
./run_controller.sh

================================================================