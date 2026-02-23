import time, socket, struct, os, threading
import depthai as dai
from datetime import timedelta
import hashlib
from state_machine import State, Event, step, parse_event

import actions
from actions import SystemHandles
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE  # NEW: RS485 Modbus gripper driver
import robot_config as cfg

print("USING ACTIONS FROM:", actions.__file__)


# --- 0. STABILITY & PORT CONFIG ---
os.environ["DEPTHAI_WATCHDOG_TIMEOUT"] = "5000"

UNITY_PORT_INSPECTOR = 8085
UNITY_PORT_MANAGER   = 8086
UNITY_PORT_COMMANDS  = 8088

MXID_INSPECTOR = "19443010B14C872F00"
MXID_MANAGER   = "194430108183F12E00"

# --- Gripper ---
LAST_GRIP_TS = 0.0
GRIP_DEBOUNCE_S = 0.40

# Deterministic gripper positions (calibrated by you)
GRIP_OPEN_POS  = 900
GRIP_CLOSE_POS = 50

# --- Robot driver instance ---
robot = DobotDriver()

# --- Gripper driver instance (RS485 Modbus) ---
gripper = DHGripperPGE(
    port=cfg.GRIPPER_PORT,
    baudrate=cfg.GRIPPER_BAUDRATE,
    device_id=cfg.GRIPPER_SLAVE_ID,
)

# --- System handles (dependency injection) ---
handles = SystemHandles(robot=robot, gripper=gripper)

gripper_connected = False

# --- Simple control mode/state (via State Machine) ---
STATE = State.IDLE

# --- NEW: Arm state tied to VR connection lifecycle ---
robot_armed = False


# --- SIGNIFIER: prove which file is running ---
def _startup_banner():
    try:
        path = os.path.abspath(__file__)
        with open(path, "rb") as f:
            h = hashlib.sha1(f.read()).hexdigest()[:10]
        print("\n" + "=" * 72)
        print("SIDE QUEST TASK CONTROLLER — ARM-ON-CONNECT BUILD (NO PER-COMMAND ARM)")
        print(f"RUNNING FILE: {path}")
        print(f"FILE SHA1 (first10): {h}")
        print("=" * 72 + "\n")
    except Exception as e:
        print(f"[WARN] Could not print startup banner: {e}")

_startup_banner()


# --- 2. CAMERA PIPELINE ---
def create_pipeline():
    pipeline = dai.Pipeline()

    monoL = pipeline.create(dai.node.MonoCamera)
    monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoL.setFps(20)

    monoR = pipeline.create(dai.node.MonoCamera)
    monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoR.setFps(20)

    encL = pipeline.create(dai.node.VideoEncoder)
    encL.setDefaultProfilePreset(20, dai.VideoEncoderProperties.Profile.MJPEG)
    encL.setQuality(40)

    encR = pipeline.create(dai.node.VideoEncoder)
    encR.setDefaultProfilePreset(20, dai.VideoEncoderProperties.Profile.MJPEG)
    encR.setQuality(40)

    monoL.out.link(encL.input)
    monoR.out.link(encR.input)

    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(timedelta(milliseconds=50))
    encL.bitstream.link(sync.inputs["left"])
    encR.bitstream.link(sync.inputs["right"])

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("out")
    sync.out.link(xout.input)

    return pipeline


# --- 3. HIGHWAY 1: VIDEO SERVER ---
def camera_server(mxid, port, label):
    pipeline = create_pipeline()
    try:
        with dai.Device(pipeline, dai.DeviceInfo(mxid)) as device:
            print(f"[{label}] Camera Connected.")
            q = device.getOutputQueue("out", maxSize=4, blocking=False)

            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', port))
            server.listen(1)
            print(f"[{label}] Streaming on port {port}")

            while True:
                conn, addr = server.accept()
                print(f"[{label}] Unity connected from {addr}")
                try:
                    while True:
                        group = q.get()
                        dL = group["left"].getData().tobytes()
                        dR = group["right"].getData().tobytes()
                        conn.sendall(b'L' + struct.pack('>I', len(dL)) + dL)
                        conn.sendall(b'R' + struct.pack('>I', len(dR)) + dR)
                except Exception as e:
                    # Typical when Unity stops play mode / reconnects
                    print(f"[{label}] Client stream ended: {e}")
                finally:
                    try: conn.close()
                    except: pass
                    print(f"[{label}] Unity disconnected.")
    except Exception as e:
        print(f"[{label}] Error: {e}")

def ensure_ready(precision: bool = True):
    """
    IMPORTANT:
    - We DO NOT ClearError/EnableRobot/SpeedFactor per command anymore.
    - Arming happens ONCE on VR connect in command_server().
    """
    global robot_armed
    if not robot_armed:
        print("[SAFETY] Motion ignored: robot DISARMED (no active VR connection).")
        return False
    return True


def ensure_gripper_ready():
    if not gripper_connected:
        print("[SAFETY] Gripper ignored: gripper NOT connected (RS485).")
        return False
    return True


# --- Robot actions ---
# NOTE: Robot action functions (do_home, do_drop, do_nudge_xy, etc.) are now
# imported from actions.py module along with their SystemHandles dependency.


# --- 4. HIGHWAY 2: COMMAND HUB (Logic Bridge) ---
def command_server():
    global STATE, robot_armed, gripper_connected

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', UNITY_PORT_COMMANDS))
    server.listen(1)

    print(f"[CONTROL] Hub ready on {UNITY_PORT_COMMANDS}")
    print("[CONTROL] Commands: HOME | FIX | NUDGE dx dy | DROP | CANCEL | GRIP_TOGGLE | GRIP_OPEN | GRIP_CLOSE | (COMMIT->DROP)")
    print("[CONTROL] NOTE: Robot arms once per VR connection (no per-command arming).")
    print("[CONTROL] NOTE: Gripper is RS485 Modbus (no DH UI Init toggle required).")

    while True:
        conn, addr = server.accept()
        print(f"[CONTROL] VR Connected: {addr}")

        # Reset state machine for new session
        STATE = State.IDLE

        # Initialize fresh stacking session
        current_pick_index = 0
        current_stack_level = 0
        # Dev 8 autonomous stacking loop controls
        stacking_enabled = True
        target_stack_count = min(7, len(cfg.PICK_SEQUENCE))
        controller_busy = False

        # Arm once for this VR session
        try:
            actions.arm_robot_once(handles)
            robot_armed = True
        except Exception as e:
            robot_armed = False
            print(f"[CONTROL] Robot arm FAILED: {e}")

        # Connect gripper once for this VR session
        try:
            actions.connect_gripper_once(handles)
            gripper_connected = True
        except Exception as e:
            gripper_connected = False
            print(f"[CONTROL] Gripper connect FAILED: {e}")

        # Initialize stack session (home + open gripper)
        try:
            actions.initialize_stack_session(handles)
        except Exception as e:
            print(f"[CONTROL] Session init FAILED: {e}")

        conn.settimeout(None)
        buf = ""

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                buf += data.decode('utf-8', errors='ignore')

                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    cmd_str = line.strip()
                    if not cmd_str:
                        continue

                    if cmd_str == "COMMIT":
                        cmd_str = "DROP"

                    print(f"\n[CONTROL] Received: {cmd_str}   (STATE={STATE.name})  (ARMED={robot_armed})  (GRIPPER={gripper_connected})")

                    # SAFE_RESET short-circuit (before parsing)
                    if cmd_str == "SAFE_RESET":
                        try:
                            print("[CONTROL] SAFE_RESET requested.")
                            actions.do_home(handles)
                            actions.do_grip_open(handles)
                            STATE = State.IDLE
                            current_pick_index = 0
                            current_stack_level = 0
                        except Exception as e:
                            print(f"[CONTROL] SAFE_RESET failed: {e}")
                        continue

                    # Parse command into event using state machine
                    try:
                        event, payload = parse_event(cmd_str)
                    except ValueError as e:
                        print(f"[CMD] Reject: {e}")
                        continue

                    # Gate transition using state machine
                    result = step(STATE, event)
                    if not result.allowed:
                        print(f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason={result.reason}")
                        continue

                    # Apply state transition
                    prev = STATE
                    STATE = result.next_state
                    if STATE != prev:
                        print(f"[SM] {prev.name} -> {STATE.name} on {event.name}")

                    # Safety gates (keep command thread alive; just refuse execution)
                    if event in {Event.HOME, Event.FIX, Event.NUDGE_XY, Event.NUDGE_YAW, Event.DROP, Event.START_STACK}:
                        if not robot_armed:
                            print("[GATE] Robot not armed. Ignoring motion command.")
                            continue

                    if event in {Event.GRIP_OPEN, Event.GRIP_CLOSE, Event.GRIP_TOGGLE}:
                        if not gripper_connected:
                            print("[GATE] Gripper not connected. Ignoring gripper command.")
                            continue

                    # Execute side-effects based on event
                    if event == Event.HOME:
                        if controller_busy:
                            print("[GATE] Controller busy.")
                            continue
                        controller_busy = True
                        try:
                            actions.do_home(handles)
                        finally:
                            controller_busy = False

                    elif event == Event.START_STACK:
                        if controller_busy:
                            print("[GATE] Controller busy.")
                            continue
                        controller_busy = True
                        try:
                            # Bounds checking
                            if current_pick_index >= len(cfg.PICK_SEQUENCE):
                                print("[STACK] No more blocks in PICK_SEQUENCE. Ignoring START.")
                                continue

                            if current_stack_level >= 7:
                                print("[STACK] Tower full. Ignoring START.")
                                continue

                            # Execute pick sequence
                            side, level = cfg.PICK_SEQUENCE[current_pick_index]
                            actions.execute_pick_sequence(handles, side, level)

                            # Emit internal progression event
                            result2 = step(STATE, Event.PICK_COMPLETE)
                            if result2.allowed:
                                STATE = result2.next_state
                                print(f"[SM] -> {STATE.name} (PICK_COMPLETE)")

                                # Immediately move to tower hover
                                if STATE == State.MOVING_TO_TOWER_HOVER:
                                    actions.move_to_tower_hover(handles, current_stack_level)

                                    result3 = step(STATE, Event.AT_TOWER_HOVER)
                                    if result3.allowed:
                                        STATE = result3.next_state
                                        print(f"[SM] -> {STATE.name} (AT_TOWER_HOVER)")
                        finally:
                            controller_busy = False

                    elif event == Event.FIX:
                        # no motion yet; just entering NUDGE
                        pass

                    elif event == Event.NUDGE_XY:
                        if controller_busy:
                            print("[GATE] Controller busy.")
                            continue
                        controller_busy = True
                        try:
                            actions.do_nudge_xy(handles, payload["dx"], payload["dy"])
                        finally:
                            controller_busy = False

                    elif event == Event.NUDGE_YAW:
                        if controller_busy:
                            print("[GATE] Controller busy.")
                            continue
                        controller_busy = True
                        try:
                            actions.do_nudge_yaw(handles, payload["dtheta"])
                        finally:
                            controller_busy = False

                    elif event == Event.DROP:
                        if controller_busy:
                            print("[GATE] Controller busy.")
                            continue
                        controller_busy = True
                        try:
                            # Attempt placement with error handling
                            try:
                                actions.complete_place_sequence(handles, current_stack_level)
                            except Exception as e:
                                print(f"[STACK] Place failed: {e}")
                                # Emit fault event
                                fault_result = step(STATE, Event.FAULT)
                                if fault_result.allowed:
                                    STATE = fault_result.next_state
                                    print(f"[SM] -> {STATE.name} (FAULT)")
                                continue

                            # Update stack counters only on success
                            current_stack_level += 1
                            current_pick_index += 1

                            # Emit internal progression event
                            result4 = step(STATE, Event.PLACE_COMPLETE)
                            if result4.allowed:
                                STATE = result4.next_state
                                print(f"[SM] -> {STATE.name} (PLACE_COMPLETE)")

                                # Auto-continue stacking if enabled and targets remain
                                if stacking_enabled and current_stack_level < target_stack_count and current_pick_index < len(cfg.PICK_SEQUENCE):
                                    print("[STACK] Auto-continue to next block.")
                                    auto_result = step(STATE, Event.START_STACK)
                                    if auto_result.allowed:
                                        STATE = auto_result.next_state
                                        side, level = cfg.PICK_SEQUENCE[current_pick_index]
                                        actions.execute_pick_sequence(handles, side, level)
                                        result2 = step(STATE, Event.PICK_COMPLETE)
                                        if result2.allowed:
                                            STATE = result2.next_state
                                            if STATE == State.MOVING_TO_TOWER_HOVER:
                                                actions.move_to_tower_hover(handles, current_stack_level)
                                                result3 = step(STATE, Event.AT_TOWER_HOVER)
                                                if result3.allowed:
                                                    STATE = result3.next_state
                                else:
                                    print("[STACK] Target reached or no more blocks.")
                        finally:
                            controller_busy = False

                    elif event == Event.CANCEL:
                        # returns to WAITING_FOR_DECISION by state machine; no motion needed
                        pass

                    elif event == Event.GRIP_OPEN:
                        actions.do_grip_open(handles)

                    elif event == Event.GRIP_CLOSE:
                        actions.do_grip_close(handles)

                    elif event == Event.GRIP_TOGGLE:
                        actions.do_grip_toggle(handles)

        except Exception as e:
            print(f"[CONTROL] Connection error: {e}")
        finally:
            try:
                conn.close()
            except:
                pass

            # NOTE: handles.gripper.close() not called here because close() is used for
            # gripper actuation (closing the grip) and would cause unintended physical motion.
            # Gripper disconnection is handled by gripper_connected flag reset and eventual
            # reconnection on next VR session.

            robot_armed = False
            STATE = State.IDLE
            print("[CONTROL] VR disconnected. Robot disarmed. STATE reset to IDLE.")

            # Keep the gripper connection status conservative:
            # If Unity reconnects, we'll re-attempt connect.
            gripper_connected = False


# --- 5. START SYSTEM ---
threading.Thread(target=command_server, daemon=True).start()
threading.Thread(target=camera_server, args=(MXID_INSPECTOR, UNITY_PORT_INSPECTOR, "INSPECTOR"), daemon=True).start()
time.sleep(10)
threading.Thread(target=camera_server, args=(MXID_MANAGER, UNITY_PORT_MANAGER, "SITE_MANAGER"), daemon=True).start()

print("TASK CONTROLLER ACTIVE. Press Ctrl+C to stop.")
while True:
    time.sleep(1)
