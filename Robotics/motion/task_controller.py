import time, socket, struct, os, threading, json, sys
import warnings
import depthai as dai
from datetime import timedelta
import hashlib
from uuid import uuid4
from state_machine import State, Event, step, parse_event

import actions
from actions import SystemHandles
import drift_engine
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE  # NEW: RS485 Modbus gripper driver
import robot_config as cfg
from logger import info, warn

warnings.filterwarnings("ignore", category=DeprecationWarning)

info("CONTROL", f"USING ACTIONS FROM: {actions.__file__}")


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
vr_connected = False

# --- Graceful shutdown ---
STOP_EVENT = threading.Event()
_SOCKETS_LOCK = threading.Lock()
_SERVER_SOCKETS: set[socket.socket] = set()
_CLIENT_SOCKETS: set[socket.socket] = set()
_WORKER_THREADS: list[threading.Thread] = []

# --- Session-level participant + telemetry state ---
participant_name = None
session_id = None
tower_attempt_start_ts = None
block_attempt_start_ts = None
holding_block = False
current_session_token = uuid4()
proposed_place_pose = None
proposed_place_stack_level = None


def _track_server_socket(sock: socket.socket) -> None:
    with _SOCKETS_LOCK:
        _SERVER_SOCKETS.add(sock)


def _untrack_server_socket(sock: socket.socket) -> None:
    with _SOCKETS_LOCK:
        _SERVER_SOCKETS.discard(sock)


def _track_client_socket(sock: socket.socket) -> None:
    with _SOCKETS_LOCK:
        _CLIENT_SOCKETS.add(sock)


def _untrack_client_socket(sock: socket.socket) -> None:
    with _SOCKETS_LOCK:
        _CLIENT_SOCKETS.discard(sock)


def _close_socket_quietly(sock: socket.socket) -> None:
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    try:
        sock.close()
    except Exception:
        pass


def _close_all_tracked_sockets() -> list[Exception]:
    errors: list[Exception] = []
    with _SOCKETS_LOCK:
        client_sockets = list(_CLIENT_SOCKETS)
        server_sockets = list(_SERVER_SOCKETS)

    for sock in client_sockets:
        try:
            _close_socket_quietly(sock)
        except Exception as e:
            errors.append(e)
        finally:
            _untrack_client_socket(sock)

    for sock in server_sockets:
        try:
            _close_socket_quietly(sock)
        except Exception as e:
            errors.append(e)
        finally:
            _untrack_server_socket(sock)

    return errors


def _join_worker_threads(timeout_s: float = 1.0) -> list[str]:
    alive: list[str] = []
    for thread in _WORKER_THREADS:
        if not thread.daemon:
            thread.join(timeout=timeout_s)
            if thread.is_alive():
                alive.append(thread.name)
    return alive


def log_event(event: str, **fields) -> None:
    """Append one JSONL event record to cfg.LOG_DIR."""
    try:
        log_dir = getattr(cfg, "LOG_DIR", "logs")
        os.makedirs(log_dir, exist_ok=True)

        sid = session_id or "no_session"
        log_path = os.path.join(log_dir, f"session_{sid}.jsonl")

        record = {
            "timestamp": time.time(),
            "event": event,
            "session_id": session_id,
            "participant_name": participant_name,
            "state": STATE.name if hasattr(STATE, "name") else str(STATE),
            "current_stack_level": globals().get("current_stack_level", None),
            "current_pick_index": globals().get("current_pick_index", None),
        }
        if hasattr(cfg, "DRIFT_SCALE"):
            record["drift_scale"] = cfg.DRIFT_SCALE
        if fields:
            record.update(fields)

        with open(log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
    except Exception as e:
        warn("CONTROL", f"[LOG] Failed to write event {event}: {e}")


# --- SIGNIFIER: prove which file is running ---
def _startup_banner():
    try:
        path = os.path.abspath(__file__)
        with open(path, "rb") as f:
            h = hashlib.sha1(f.read()).hexdigest()[:10]
        info("CONTROL", "=" * 72)
        info("CONTROL", "SIDE QUEST TASK CONTROLLER — ARM-ON-CONNECT BUILD (NO PER-COMMAND ARM)")
        info("CONTROL", f"RUNNING FILE: {path}")
        info("CONTROL", f"FILE SHA1 (first10): {h}")
        info("CONTROL", "=" * 72)
    except Exception as e:
        warn("CONTROL", f"Could not print startup banner: {e}")

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
    server = None
    try:
        with dai.Device(pipeline, dai.DeviceInfo(mxid)) as device:
            if getattr(cfg, "RUN_MODE", "DEBUG") == "COMP":
                try:
                    device.setLogLevel(dai.LogLevel.CRITICAL)
                except Exception:
                    pass
            info("CAM", f"[{label}] Camera Connected.")
            q = device.getOutputQueue("out", maxSize=4, blocking=False)

            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', port))
            server.listen(1)
            server.settimeout(1.0)
            _track_server_socket(server)
            info("CAM", f"[{label}] Streaming on port {port}")

            while not STOP_EVENT.is_set():
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if STOP_EVENT.is_set():
                        break
                    raise
                _track_client_socket(conn)
                conn.settimeout(1.0)
                warn("CAM", f"[{label}] Unity connected from {addr}")
                try:
                    while not STOP_EVENT.is_set():
                        group = q.get()
                        dL = group["left"].getData().tobytes()
                        dR = group["right"].getData().tobytes()
                        conn.sendall(b'L' + struct.pack('>I', len(dL)) + dL)
                        conn.sendall(b'R' + struct.pack('>I', len(dR)) + dR)
                except socket.timeout:
                    continue
                except Exception as e:
                    # Typical when Unity stops play mode / reconnects
                    if not STOP_EVENT.is_set():
                        if getattr(cfg, "RUN_MODE", "DEBUG") == "DEBUG":
                            warn("CAM", f"[{label}] Client stream ended: {e}")
                finally:
                    _close_socket_quietly(conn)
                    _untrack_client_socket(conn)
                    if not STOP_EVENT.is_set():
                        if getattr(cfg, "RUN_MODE", "DEBUG") == "DEBUG":
                            warn("CAM", f"[{label}] Unity disconnected.")
    except Exception as e:
        if not STOP_EVENT.is_set():
            warn("CAM", f"[{label}] Error: {e}")
    finally:
        if server is not None:
            _close_socket_quietly(server)
            _untrack_server_socket(server)

def ensure_ready(precision: bool = True):
    """
    IMPORTANT:
    - We DO NOT ClearError/EnableRobot/SpeedFactor per command anymore.
    - Arming happens ONCE on VR connect in command_server().
    """
    global robot_armed
    if not robot_armed:
        warn("CONTROL", "[SAFETY] Motion ignored: robot DISARMED (no active VR connection).")
        return False
    return True


def ensure_gripper_ready():
    if not gripper_connected:
        warn("CONTROL", "[SAFETY] Gripper ignored: gripper NOT connected (RS485).")
        return False
    return True


def _module_for_source(source: str) -> str:
    return "ADMIN" if source == "ADMIN" else "CONTROL"


def _handle_post_tower_hover(module: str, my_token) -> bool:
    """Handle post-hover fault check and AT_TOWER_HOVER transition in one place."""
    global STATE, block_attempt_start_ts, proposed_place_pose, proposed_place_stack_level

    m = handles.robot.robot_mode()
    if m in (9, 11):
        if my_token != current_session_token:
            return False
        warn(module, f"[FAULT] RobotMode={m} -> entering FAULT")
        fault_result = step(STATE, Event.FAULT)
        if fault_result.allowed:
            STATE = fault_result.next_state
            info(module, f"[SM] -> {STATE.name} (FAULT)")
        return False

    if my_token != current_session_token:
        return False
    result = step(STATE, Event.AT_TOWER_HOVER)
    if result.allowed:
        STATE = result.next_state
        info(module, f"[SM] -> {STATE.name} (AT_TOWER_HOVER)")
        if STATE == State.WAITING_FOR_DECISION:
            if current_stack_level is not None and current_stack_level >= 0:
                base_place = cfg.tower_place_pose(current_stack_level)
                proposed_place_pose = drift_engine.inject_drift(base_place, current_stack_level)
                proposed_place_stack_level = current_stack_level
            else:
                proposed_place_pose = None
                proposed_place_stack_level = None
            if getattr(cfg, "RUN_MODE", "DEBUG") == "DEBUG":
                info("CONTROL", f"READY: waiting for DROP/FIX (stack_level={current_stack_level})")
            block_attempt_start_ts = time.time()

    return True


# --- Robot actions ---
# NOTE: Robot action functions (do_home, do_drop, do_nudge_xy, etc.) are now
# imported from actions.py module along with their SystemHandles dependency.


# --- 3.5. COMMAND HANDLER ---
def handle_command(cmd_str: str, source: str) -> None:
    """
    Process a single incoming command string.
    Uses globals: STATE, robot_armed, gripper_connected, controller_busy,
                  current_pick_index, current_stack_level, stacking_enabled, target_stack_count
    
    source: Log prefix, e.g., "CONTROL", "ADMIN"
    """
    global STATE, robot_armed, gripper_connected, controller_busy
    global current_pick_index, current_stack_level, stacking_enabled, target_stack_count
    global participant_name, session_id, tower_attempt_start_ts, block_attempt_start_ts, holding_block, current_session_token
    global proposed_place_pose, proposed_place_stack_level

    # Normalize command
    if cmd_str == "COMMIT":
        cmd_str = "DROP"

    module = _module_for_source(source)
    if getattr(cfg, "RUN_MODE", "DEBUG") == "DEBUG":
        info(module, f"[{source}] Received: {cmd_str}   (STATE={STATE.name})  (ARMED={robot_armed})  (GRIPPER={gripper_connected})")

    # Raw session commands (before parse_event)
    upper_cmd = cmd_str.upper()
    if upper_cmd == "NAME" or upper_cmd.startswith("NAME "):
        # Accept: NAME <free text>
        name_value = cmd_str[4:].strip()
        if not name_value:
            warn(module, "[GATE] NAME command requires free text (e.g., NAME Alice).")
            return
        participant_name = name_value
        current_stack_level = 0
        current_pick_index = 0
        session_id = f"{int(time.time())}-{uuid4().hex[:8]}"
        tower_attempt_start_ts = None
        block_attempt_start_ts = None
        holding_block = False
        controller_busy = False
        current_session_token = uuid4()
        proposed_place_pose = None
        proposed_place_stack_level = None
        log_event("EVENT_NAME_SET", participant=participant_name, source=source)
        info("CONTROL", f"Participant set: {participant_name}. Ready to START.")
        return

    if upper_cmd == "TUMBLE":
        current_session_token = uuid4()
        controller_busy = False
        # Run summary before any state/session reset
        blocks_placed = current_stack_level
        now_mono = time.monotonic()
        run_time_s = None
        if tower_attempt_start_ts is not None:
            run_time_s = now_mono - tower_attempt_start_ts

        name_for_summary = participant_name if participant_name else "Unknown participant"
        if run_time_s is not None:
            info(module, f"{name_for_summary} successfully placed {blocks_placed} blocks in {run_time_s:.1f} seconds")
        else:
            info(module, f"{name_for_summary} successfully placed {blocks_placed} blocks (time unavailable)")

        log_event(
            "EVENT_RUN_SUMMARY",
            participant=participant_name,
            blocks_placed=blocks_placed,
            run_time_s=run_time_s,
            source=source,
        )

        log_event("EVENT_TUMBLE", source=source, holding_block=holding_block)
        try:
            actions.safe_reset_after_tumble(handles, holding_block)
        except Exception as e:
            warn(module, f"[{source}] TUMBLE reset failed: {e}")
            log_event("EVENT_TUMBLE_RESET_ERROR", source=source, error=str(e))

        STATE = State.IDLE
        current_pick_index = 0
        current_stack_level = 0
        holding_block = False
        controller_busy = False
        participant_name = None
        tower_attempt_start_ts = None
        block_attempt_start_ts = None
        proposed_place_pose = None
        proposed_place_stack_level = None
        if getattr(cfg, "RUN_MODE", "DEBUG") == "COMP" and vr_connected:
            print("Waiting for participant name...")
        return

    # SAFE_RESET short-circuit (before parsing)
    if cmd_str == "SAFE_RESET":
        try:
            info(module, f"[{source}] SAFE_RESET requested.")
            actions.do_home(handles)
            actions.do_grip_open(handles)
            STATE = State.IDLE
            current_pick_index = 0
            current_stack_level = 0
            proposed_place_pose = None
            proposed_place_stack_level = None
        except Exception as e:
            warn(module, f"[{source}] SAFE_RESET failed: {e}")
        return

    # Parse command into event using state machine
    try:
        event, payload = parse_event(cmd_str)
    except ValueError as e:
        warn(module, f"[CMD] Reject: {e}")
        return

    # Session participant gate
    if (participant_name is None or participant_name.strip() == "") and event in {Event.START_STACK, Event.DROP, Event.FIX, Event.NUDGE_XY, Event.NUDGE_YAW}:
        warn(module, f"[GATE] Participant name required. Rejecting: {cmd_str}")
        log_event("EVENT_REJECT_NO_NAME", cmd=cmd_str, source=source)
        return

    # Gate transition using state machine
    result = step(STATE, event)
    if not result.allowed:
        info(module, f"[SM] Blocked: {cmd_str} | state={STATE.name} | reason={result.reason}")
        return

    # Safety gates BEFORE committing state transition
    # AUTO_RECOVER is exempt: it's the recovery path from FAULT and may be 
    # issued when disarmed, since it's responsible for re-enabling the robot.
    if event != Event.AUTO_RECOVER and event in {Event.HOME, Event.FIX, Event.NUDGE_XY, Event.NUDGE_YAW, Event.DROP, Event.START_STACK}:
        if not robot_armed:
            warn(module, "[GATE] Robot not armed. Ignoring motion command.")
            return

    if event in {Event.GRIP_OPEN, Event.GRIP_CLOSE, Event.GRIP_TOGGLE}:
        if not gripper_connected:
            warn(module, "[GATE] Gripper not connected. Ignoring gripper command.")
            return

    # Apply state transition only after passing gates
    # AUTO_RECOVER is special: state is decided by real recovery result, not transition table.
    prev = STATE
    if event != Event.AUTO_RECOVER:
        STATE = result.next_state
        if STATE != prev:
            info(module, f"[SM] {prev.name} -> {STATE.name} on {event.name}")

    # Execute side-effects based on event
    if event == Event.HOME:
        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        controller_busy = True
        try:
            # If recovering from FAULT, call recovery routine first
            if prev == State.FAULT:
                info(module, "[RECOVERY] HOME requested from FAULT. Starting recovery sequence...")
                actions.recover_from_fault(handles)
                mode_after = handles.robot.robot_mode()
                if mode_after not in (9, 11):
                    info(module, f"[RECOVERY] Recovery successful during HOME. Robot mode: {mode_after}")
                    robot_armed = True
                else:
                    warn(module, f"[RECOVERY] Recovery incomplete during HOME. Robot still in fault mode: {mode_after}")
                    robot_armed = False
            else:
                # Normal home
                actions.do_home(handles)
                # Immediate RobotMode check after motion
                m = handles.robot.robot_mode()
                if m in (9, 11):
                    warn(module, f"[FAULT] RobotMode={m} -> entering FAULT")
                    fault_result = step(STATE, Event.FAULT)
                    if fault_result.allowed:
                        STATE = fault_result.next_state
                        info(module, f"[SM] -> {STATE.name} (FAULT)")
                    return
        finally:
            controller_busy = False

    elif event == Event.START_STACK:
        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        controller_busy = True
        try:
            # Bounds checking
            if current_pick_index >= len(cfg.PICK_SEQUENCE):
                warn(module, "[STACK] No more blocks in PICK_SEQUENCE. Ignoring START.")
                return

            if current_stack_level >= 7:
                warn(module, "[STACK] Tower full. Ignoring START.")
                return

            # Run start timing (monotonic) when a new run actually begins
            if current_stack_level == 0:
                current_session_token = uuid4()
                tower_attempt_start_ts = time.monotonic()

            # Execute pick sequence
            my_token = current_session_token
            side, level = cfg.PICK_SEQUENCE[current_pick_index]
            actions.execute_pick_sequence(handles, side, level)
            # Immediate RobotMode check after motion
            m = handles.robot.robot_mode()
            if m in (9, 11):
                warn(module, f"[FAULT] RobotMode={m} -> entering FAULT")
                fault_result = step(STATE, Event.FAULT)
                if fault_result.allowed:
                    STATE = fault_result.next_state
                    info(module, f"[SM] -> {STATE.name} (FAULT)")
                return

            # Emit internal progression event
            if my_token != current_session_token:
                return
            result2 = step(STATE, Event.PICK_COMPLETE)
            if result2.allowed:
                STATE = result2.next_state
                info(module, f"[SM] -> {STATE.name} (PICK_COMPLETE)")

                # Immediately move to tower hover
                if STATE == State.MOVING_TO_TOWER_HOVER:
                    actions.move_to_tower_hover(handles, current_stack_level)
                    if not _handle_post_tower_hover(module, my_token):
                        return
        finally:
            controller_busy = False

    elif event == Event.FIX:
        # no motion yet; just entering NUDGE
        pass

    elif event == Event.NUDGE_XY:
        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        controller_busy = True
        try:
            actions.do_nudge_xy(handles, payload["dx"], payload["dy"])
            if proposed_place_pose is not None and proposed_place_stack_level == current_stack_level:
                proposed_place_pose = (
                    proposed_place_pose[0] + payload["dx"],
                    proposed_place_pose[1] + payload["dy"],
                    proposed_place_pose[2],
                    proposed_place_pose[3],
                    proposed_place_pose[4],
                    proposed_place_pose[5],
                )
            # Immediate RobotMode check after motion
            m = handles.robot.robot_mode()
            if m in (9, 11):
                warn(module, f"[FAULT] RobotMode={m} -> entering FAULT")
                fault_result = step(STATE, Event.FAULT)
                if fault_result.allowed:
                    STATE = fault_result.next_state
                    info(module, f"[SM] -> {STATE.name} (FAULT)")
                return
        finally:
            controller_busy = False

    elif event == Event.NUDGE_YAW:
        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        controller_busy = True
        try:
            actions.do_nudge_yaw(handles, payload["dtheta"])
            if proposed_place_pose is not None and proposed_place_stack_level == current_stack_level:
                proposed_place_pose = (
                    proposed_place_pose[0],
                    proposed_place_pose[1],
                    proposed_place_pose[2],
                    proposed_place_pose[3],
                    proposed_place_pose[4],
                    proposed_place_pose[5] + payload["dtheta"],
                )
            # Immediate RobotMode check after motion
            m = handles.robot.robot_mode()
            if m in (9, 11):
                warn(module, f"[FAULT] RobotMode={m} -> entering FAULT")
                fault_result = step(STATE, Event.FAULT)
                if fault_result.allowed:
                    STATE = fault_result.next_state
                    info(module, f"[SM] -> {STATE.name} (FAULT)")
                return
        finally:
            controller_busy = False

    elif event == Event.DROP:
        decision_time = None
        if block_attempt_start_ts is not None:
            decision_time = time.time() - block_attempt_start_ts
        log_event("EVENT_DROP_RECEIVED", source=source, decision_time=decision_time)
        block_attempt_start_ts = None

        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        controller_busy = True
        try:
            my_token = current_session_token
            # Attempt placement with error handling
            try:
                if proposed_place_pose is not None and proposed_place_stack_level == current_stack_level:
                    actions.complete_place_sequence(handles, current_stack_level, place_pose=proposed_place_pose)
                else:
                    actions.complete_place_sequence(handles, current_stack_level)
                # Immediate RobotMode check after motion
                m = handles.robot.robot_mode()
                if m in (9, 11):
                    warn(module, f"[FAULT] RobotMode={m} -> entering FAULT")
                    fault_result = step(STATE, Event.FAULT)
                    if fault_result.allowed:
                        STATE = fault_result.next_state
                        info(module, f"[SM] -> {STATE.name} (FAULT)")
                    return
                holding_block = False
                proposed_place_pose = None
                proposed_place_stack_level = None
            except Exception as e:
                warn(module, f"[STACK] Place failed: {e}")
                proposed_place_pose = None
                proposed_place_stack_level = None
                # Emit fault event
                fault_result = step(STATE, Event.FAULT)
                if fault_result.allowed:
                    STATE = fault_result.next_state
                    info(module, f"[SM] -> {STATE.name} (FAULT)")
                return

            # Update stack counters only on success
            current_stack_level += 1
            current_pick_index += 1

            # Emit internal progression event
            if my_token != current_session_token:
                return
            result4 = step(STATE, Event.PLACE_COMPLETE)
            if result4.allowed:
                STATE = result4.next_state
                info(module, f"[SM] -> {STATE.name} (PLACE_COMPLETE)")

                # Auto-continue stacking if enabled and targets remain
                if stacking_enabled and current_stack_level < target_stack_count and current_pick_index < len(cfg.PICK_SEQUENCE):
                    info(module, "[STACK] Auto-continue to next block.")
                    if my_token != current_session_token:
                        return
                    auto_result = step(STATE, Event.START_STACK)
                    if auto_result.allowed:
                        STATE = auto_result.next_state
                        my_token = current_session_token
                        side, level = cfg.PICK_SEQUENCE[current_pick_index]
                        actions.execute_pick_sequence(handles, side, level)
                        # Immediate RobotMode check after motion (auto-continue)
                        m = handles.robot.robot_mode()
                        if m in (9, 11):
                            warn(module, f"[FAULT] RobotMode={m} -> entering FAULT")
                            fault_result = step(STATE, Event.FAULT)
                            if fault_result.allowed:
                                STATE = fault_result.next_state
                                info(module, f"[SM] -> {STATE.name} (FAULT)")
                            return
                        if my_token != current_session_token:
                            return
                        result2 = step(STATE, Event.PICK_COMPLETE)
                        if result2.allowed:
                            STATE = result2.next_state
                            if STATE == State.MOVING_TO_TOWER_HOVER:
                                actions.move_to_tower_hover(handles, current_stack_level)
                                if not _handle_post_tower_hover(module, my_token):
                                    return
                else:
                    info(module, "[STACK] Target reached or no more blocks.")
        finally:
            controller_busy = False

    elif event == Event.CANCEL:
        # returns to WAITING_FOR_DECISION by state machine; no motion needed
        pass

    elif event == Event.AUTO_RECOVER:
        # Recovery from FAULT state (or no-op from other states)
        # State is determined by actual recovery success, not transition table next_state.
        if controller_busy:
            warn(module, "[GATE] Controller busy.")
            return
        controller_busy = True
        try:
            info(module, "[RECOVERY] Auto-recover requested. Starting recovery sequence...")
            ok = actions.recover_from_fault(handles)
            if ok:
                info(module, "[RECOVERY] Recovery reported OK. Resetting state to IDLE.")
                STATE = State.IDLE
                current_pick_index = 0
                current_stack_level = 0
                proposed_place_pose = None
                proposed_place_stack_level = None
                robot_armed = True
            else:
                warn(module, "[RECOVERY] Recovery reported FAILURE. Setting state to FAULT.")
                STATE = State.FAULT
                robot_armed = False
        except Exception as e:
            warn(module, f"[RECOVERY] Unexpected error during recovery: {e}")
            STATE = State.FAULT
            robot_armed = False
        finally:
            controller_busy = False

    elif event == Event.GRIP_OPEN:
        actions.do_grip_open(handles)
        holding_block = False

    elif event == Event.GRIP_CLOSE:
        actions.do_grip_close(handles)
        holding_block = True

    elif event == Event.GRIP_TOGGLE:
        actions.do_grip_toggle(handles)


# --- 4. HIGHWAY 2: COMMAND HUB (Logic Bridge) ---
def command_server():
    global STATE, robot_armed, gripper_connected, controller_busy, vr_connected
    global proposed_place_pose, proposed_place_stack_level
    global current_pick_index, current_stack_level, stacking_enabled, target_stack_count

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', UNITY_PORT_COMMANDS))
    server.listen(1)
    server.settimeout(1.0)
    _track_server_socket(server)

    info("CONTROL", f"[CONTROL] Hub ready on {UNITY_PORT_COMMANDS}")
    info("CONTROL", "[CONTROL] Commands: HOME | FIX | NUDGE dx dy | DROP | CANCEL | GRIP_TOGGLE | GRIP_OPEN | GRIP_CLOSE | (COMMIT->DROP)")
    info("CONTROL", "[CONTROL] NOTE: Robot arms once per VR connection (no per-command arming).")
    info("CONTROL", "[CONTROL] NOTE: Gripper is RS485 Modbus (no DH UI Init toggle required).")

    try:
        while not STOP_EVENT.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                if STOP_EVENT.is_set():
                    break
                raise
            _track_client_socket(conn)
            vr_connected = True
            info("CONTROL", f"[CONTROL] VR Connected: {addr}")

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
                if getattr(cfg, "RUN_MODE", "DEBUG") == "COMP":
                    print("Waiting for participant name...")
            except Exception as e:
                robot_armed = False
                warn("CONTROL", f"[CONTROL] Robot arm FAILED: {e}")

            # Connect gripper once for this VR session
            try:
                actions.connect_gripper_once(handles)
                gripper_connected = True
            except Exception as e:
                gripper_connected = False
                warn("CONTROL", f"[CONTROL] Gripper connect FAILED: {e}")

            # Initialize stack session (home + open gripper)
            try:
                actions.initialize_stack_session(handles)
            except Exception as e:
                warn("CONTROL", f"[CONTROL] Session init FAILED: {e}")

            conn.settimeout(1.0)
            buf = ""

            try:
                while not STOP_EVENT.is_set():
                    try:
                        data = conn.recv(1024)
                    except socket.timeout:
                        continue
                    if not data:
                        break

                    buf += data.decode('utf-8', errors='ignore')

                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        cmd_str = line.strip()
                        if not cmd_str:
                            continue

                        # Dispatch to command handler
                        handle_command(cmd_str, "CONTROL")

            except Exception as e:
                if not STOP_EVENT.is_set():
                    warn("CONTROL", f"[CONTROL] Connection error: {e}")
            finally:
                _close_socket_quietly(conn)
                _untrack_client_socket(conn)

            # NOTE: handles.gripper.close() not called here because close() is used for
            # gripper actuation (closing the grip) and would cause unintended physical motion.
            # Gripper disconnection is handled by gripper_connected flag reset and eventual
            # reconnection on next VR session.

                robot_armed = False
                STATE = State.IDLE
                if getattr(cfg, "RUN_MODE", "DEBUG") == "DEBUG":
                    warn("CONTROL", "[CONTROL] VR disconnected. Robot disarmed. STATE reset to IDLE.")

                # Keep the gripper connection status conservative:
                # If Unity reconnects, we'll re-attempt connect.
                gripper_connected = False
                vr_connected = False
                proposed_place_pose = None
                proposed_place_stack_level = None
    finally:
        _close_socket_quietly(server)
        _untrack_server_socket(server)


# --- 4.5. ADMIN SERVER ---
def admin_server():
    """
    Administrative TCP server (localhost only, port 8089).
    Accepts commands and processes them via handle_command with source="ADMIN".
    Does NOT reset session variables or arm/disarm on connect.
    Shares state with command_server (STATE, robot_armed, etc.).
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 8089))
    server.listen(1)
    server.settimeout(1.0)
    _track_server_socket(server)

    info("ADMIN", "[ADMIN] Hub ready on 8089")

    try:
        while not STOP_EVENT.is_set():
            try:
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if STOP_EVENT.is_set():
                        break
                    raise
                _track_client_socket(conn)
                info("ADMIN", f"[ADMIN] Client connected: {addr}")
                
                conn.settimeout(1.0)
                buf = ""

                try:
                    while not STOP_EVENT.is_set():
                        try:
                            data = conn.recv(1024)
                        except socket.timeout:
                            continue
                        if not data:
                            break

                        buf += data.decode('utf-8', errors='ignore')

                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            cmd_str = line.strip()
                            if not cmd_str:
                                continue

                            # Dispatch to command handler with ADMIN source
                            handle_command(cmd_str, "ADMIN")

                except Exception as e:
                    if not STOP_EVENT.is_set():
                        warn("ADMIN", f"[ADMIN] Client error: {e}")
                finally:
                    _close_socket_quietly(conn)
                    _untrack_client_socket(conn)
                    if not STOP_EVENT.is_set():
                        info("ADMIN", f"[ADMIN] Client disconnected: {addr}")

            except Exception as e:
                if not STOP_EVENT.is_set():
                    warn("ADMIN", f"[ADMIN] Server error: {e}")
    finally:
        _close_socket_quietly(server)
        _untrack_server_socket(server)


def facilitator_hotkey_loop():
    """
    Facilitator keyboard hotkeys from stdin.
    Runs in a daemon thread and dispatches commands via handle_command(..., "FACILITATOR").
    """
    if getattr(cfg, "RUN_MODE", "DEBUG") == "DEBUG":
        info("CONTROL", "[FACILITATOR] Console ready: t|tumble, r|recover, n <name>|name <name>, h|help")
    while not STOP_EVENT.is_set():
        try:
            if getattr(cfg, "RUN_MODE", "DEBUG") == "DEBUG":
                raw = input("FACILITATOR> ")
            else:
                raw = input()
            line = raw.strip()
            if not line:
                continue

            lowered = line.lower()
            if lowered in {"h", "help"}:
                info("CONTROL", "[FACILITATOR] Commands:")
                info("CONTROL", "  t | tumble           -> TUMBLE")
                info("CONTROL", "  r | recover          -> AUTO_RECOVER")
                info("CONTROL", "  n <name>             -> NAME <name>")
                info("CONTROL", "  name <name>          -> NAME <name>")
                info("CONTROL", "  h | help             -> this help")
                continue

            if lowered in {"t", "tumble"}:
                cmd = "TUMBLE"
            elif lowered in {"r", "recover"}:
                cmd = "AUTO_RECOVER"
            elif lowered.startswith("n ") or lowered.startswith("name "):
                name = line.split(" ", 1)[1].strip()
                if not name:
                    warn("CONTROL", "[FACILITATOR] Usage: n <name>  (or: name <name>)")
                    continue
                cmd = f"NAME {name}"
            else:
                warn("CONTROL", f"[FACILITATOR] Unknown command: {line} (type 'h' or 'help')")
                continue

            try:
                handle_command(cmd, "FACILITATOR")
            except Exception as e:
                warn("CONTROL", f"[FACILITATOR] Command error: {e}")
        except EOFError:
            time.sleep(0.2)
        except Exception as e:
            warn("CONTROL", f"[FACILITATOR] Input loop error: {e}")


def _start_worker_thread(name: str, target, args=(), daemon: bool = False) -> None:
    thread = threading.Thread(target=target, args=args, name=name, daemon=daemon)
    _WORKER_THREADS.append(thread)
    thread.start()


# --- 5. START SYSTEM ---
_start_worker_thread("command-server", command_server)
_start_worker_thread("admin-server", admin_server)
_start_worker_thread("camera-inspector", camera_server, args=(MXID_INSPECTOR, UNITY_PORT_INSPECTOR, "INSPECTOR"))
time.sleep(10)
_start_worker_thread("camera-site-manager", camera_server, args=(MXID_MANAGER, UNITY_PORT_MANAGER, "SITE_MANAGER"))
if getattr(cfg, "RUN_MODE", "DEBUG") == "COMP":
    print("Start Unity and press Play...")
_start_worker_thread("facilitator-hotkey", facilitator_hotkey_loop, daemon=True)

info("CONTROL", "TASK CONTROLLER ACTIVE. Press Ctrl+C to stop.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    info("CONTROL", "[MAIN] Ctrl+C received. Initiating graceful shutdown...")
    STOP_EVENT.set()
    cleanup_errors = _close_all_tracked_sockets()
    alive_threads = _join_worker_threads(timeout_s=1.0)

    if alive_threads:
        if getattr(cfg, "RUN_MODE", "DEBUG") == "DEBUG":
            warn("CONTROL", f"[MAIN] Threads still alive after join timeout: {alive_threads}")
        cleanup_errors.append(RuntimeError("Worker threads still alive after shutdown timeout"))

    if cleanup_errors:
        if getattr(cfg, "RUN_MODE", "DEBUG") == "DEBUG":
            warn("CONTROL", f"[MAIN] Shutdown encountered {len(cleanup_errors)} issue(s). Exiting with code 1.")
        sys.exit(1)

    info("CONTROL", "[MAIN] Shutdown complete.")
    if getattr(cfg, "RUN_MODE", "DEBUG") == "COMP":
        os._exit(0)
    sys.exit(0)
