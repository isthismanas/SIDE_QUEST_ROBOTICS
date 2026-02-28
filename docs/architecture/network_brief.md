1. Topology
- Pi 5 = central authority node
- Robot, cameras, and Unity all communicate through the Pi.
- No direct Unity ↔ Robot link.

2. Physical Layer

Wired (1 Gbps switch):
- Dobot E6
- Raspberry Pi 5
- 2× OAK-D Pro PoE

Wireless (same LAN):
- Unity laptop
- Quest 3
Pi is Ethernet-connected to LAN.

3. Subnets
Robot subnet – 192.168.5.x
- Robot: 192.168.5.1
- Pi alias: 192.168.5.10
  Used only for robot TCP control (29999).

XR / Vision subnet – 169.254.1.x
- Pi primary: 169.254.1.10
- Laptop: 169.254.1.5
  Used for video + supervisory control.

4. Ports
- 29999 → Pi ↔ Dobot motion control
- 8085 → Inspector video stream
- 8086 → Site Manager video stream
- 8088 → Unity ↔ Pi supervisory channel
- 8089 → Local admin (localhost only)
Video and control run on separate sockets.

5. Authority Model
- Unity sends high-level commands only (START, DROP, FIX, NUDGE, VISION_RETRY).
- Pi owns state machine, safety, classification, and motion execution.
- Robot never talks to Unity directly.
- Vision results processed on Pi.