\# Side Quest – Architectural Decisions



\## 1. Human-as-Gatekeeper Model

Robot never self-commits.

DROP requires explicit human authorization.

Unity does not trigger raw motion primitives.



---



\## 2. Drift Injection Location

Drift is injected:



Detection → Base Transform → Drift Injection → Proposed Pose → Control Layer



Drift modifies the proposed placement pose only.

Motion execution remains deterministic.



Z is excluded from drift injection for safety.



---



\## 3. Verdict Computation

GREEN / YELLOW / RED classification is computed on the Pi.



Delta = target\_pose – measured\_pose



Unity renders the result but does not compute it.



---



\## 4. Gripper Control Architecture (RS485 Deterministic)



Date Updated: 2026-02-20



Previous DO pulse control is deprecated.



Current Gripper Control:



Interface: RS485 (FT232 USB adapter)

Device: /dev/ttyUSB0

Protocol: Modbus RTU

Library: pymodbus + pyserial



Verified Registers:

0x0200 – Init state

0x0201 – Grip state

0x0202 – Current position



Calibrated Positions:

Open = 900

Close = 50



Characteristics:

\- Position-based deterministic control

\- No pulse toggling

\- No background interference

\- Holds position without drift

\- True open/close semantics



Implication:

Gripper state is now state-driven, not edge-triggered.

Software no longer assumes toggle behavior.



---



\## 5. Network Separation

Video and control remain separate highways.



Video:

\- Port 8085 (Inspector)

\- Port 8086 (Site Manager)



Control:

\- Port 8088 (Unity → Pi command channel)



RS485 communication is internal to the Pi.

Unity does not interface directly with gripper hardware.



---



\## 6. Motion Safety Constraints

Z floor enforced.

XY envelope enforced.

Robot disabled before facilitator entry.

Gripper position validated before pick execution.



---



\## 7. Combo Logic

3 consecutive GREEN placements:

→ SpeedFactor +10–20%



Boost applies only to travel states.

Precision states (NUDGE, PLACING) remain reduced speed.

Reset on YELLOW or RED.



---



\## 8. Authority Commitment Handshake

Robot may:

\- PICK

\- MOVE\_TO\_HOVER



Robot may not:

\- RELEASE



Until human sends DROP command.

