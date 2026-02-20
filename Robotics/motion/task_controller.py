import time, socket, struct, os, threading
import depthai as dai
from datetime import timedelta
import hashlib

from dobot_driver import DobotDriver
import robot_config as cfg

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

# --- Robot driver instance ---
robot = DobotDriver()

# --- Simple control mode/state (Week 2 skeleton) ---
MODE = "IDLE"   # IDLE | HOVER_WAIT | NUDGE

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


# --- Robot lifecycle helpers ---
def arm_robot_once():
    """Arm the robot ONCE per VR connection (latency fix)."""
    global robot_armed
    try:
        robot.clear_and_enable(speed_percent=cfg.SPEED_PRECISION)
        robot_armed = True
        print("[ARM] Robot armed on VR connect.")
    except Exception as e:
        robot_armed = False
        print(f"[ARM] FAILED to arm robot on VR connect: {e}")


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


# --- Robot actions ---
def do_home():
    global MODE
    MODE = "IDLE"
    if not ensure_ready(precision=True):
        return
    resp = robot.go_home(speed_percent=cfg.SPEED_PRECISION)
    print(f"[HOME] {resp}")


def do_drop():
    global MODE
    MODE = "IDLE"
    if not ensure_ready(precision=True):
        return
    resp = robot.relmovl_user(0, 0, -20, 0, 0, 0)
    print(f"[DROP] {resp}")


def do_nudge(dx: float, dy: float):
    if not ensure_ready(precision=True):
        return
    resp = robot.relmovl_user(dx, dy, 0, 0, 0, 0)
    print(f"[NUDGE] dx={dx} dy={dy} -> {resp}")


def do_grip_toggle():
    global LAST_GRIP_TS
    if not ensure_ready(precision=True):
        return

    now = time.time()
    if (now - LAST_GRIP_TS) < GRIP_DEBOUNCE_S:
        print(f"[GRIP] Ignored (debounce {now - LAST_GRIP_TS:.3f}s)")
        return

    LAST_GRIP_TS = now

    try:
        robot.grip_toggle()
        print("[GRIP] TOGGLE")
    except Exception as e:
        print(f"[GRIP] FAILED: {e}")


def do_grip_open():
    """
    Temporary: your DobotDriver implements grip_open() as a toggle pulse (no DI feedback).
    This exists so Unity can have separate buttons if you want, but behaviour = toggle.
    """
    if not ensure_ready(precision=True):
        return
    try:
        robot.grip_open()
        print("[GRIP] OPEN (currently toggle-backed)")
    except Exception as e:
        print(f"[GRIP] OPEN FAILED: {e}")


def do_grip_close():
    """
    Temporary: your DobotDriver implements grip_close() as a toggle pulse (no DI feedback).
    Behaviour = toggle until we wire DI feedback.
    """
    if not ensure_ready(precision=True):
        return
    try:
        robot.grip_close()
        print("[GRIP] CLOSE (currently toggle-backed)")
    except Exception as e:
        print(f"[GRIP] CLOSE FAILED: {e}")


# --- 4. HIGHWAY 2: COMMAND HUB (Logic Bridge) ---
def command_server():
    global MODE, robot_armed

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', UNITY_PORT_COMMANDS))
    server.listen(1)

    print(f"[CONTROL] Hub ready on {UNITY_PORT_COMMANDS}")
    print("[CONTROL] Commands: HOME | FIX | NUDGE dx dy | DROP | CANCEL | GRIP_TOGGLE | GRIP_OPEN | GRIP_CLOSE | (COMMIT->DROP)")
    print("[CONTROL] NOTE: Robot arms once per VR connection (no per-command arming).")
    print("[CONTROL] NOTE: Gripper requires DH UI Init + IO Mode ON before it will respond.")

    while True:
        conn, addr = server.accept()
        print(f"[CONTROL] VR Connected: {addr}")

        # Arm once for this VR session
        arm_robot_once()

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
                    msg = line.strip()
                    if not msg:
                        continue

                    if msg == "COMMIT":
                        msg = "DROP"

                    print(f"\n[CONTROL] Received: {msg}   (MODE={MODE})  (ARMED={robot_armed})")

                    if msg == "HOME":
                        do_home()
                        continue

                    if msg == "FIX":
                        MODE = "NUDGE"
                        print("[MODE] Entered NUDGE mode (Z locked).")
                        continue

                    if msg == "CANCEL":
                        if MODE != "NUDGE":
                            print("[SAFETY] CANCEL ignored (not in NUDGE).")
                            continue
                        MODE = "HOVER_WAIT"
                        print("[MODE] Exited NUDGE -> HOVER_WAIT.")
                        continue

                    if msg.startswith("NUDGE"):
                        if MODE != "NUDGE":
                            print("[SAFETY] NUDGE ignored (not in NUDGE mode).")
                            continue

                        parts = msg.split()
                        if len(parts) != 3:
                            print("[SAFETY] NUDGE format must be: NUDGE dx dy (e.g., 'NUDGE 3 0')")
                            continue

                        try:
                            dx = float(parts[1])
                            dy = float(parts[2])
                        except ValueError:
                            print("[SAFETY] NUDGE dx/dy must be numbers.")
                            continue

                        do_nudge(dx, dy)
                        continue

                    if msg == "DROP":
                        do_drop()
                        continue

                    # --- Gripper commands ---
                    if msg == "GRIP_TOGGLE":
                        do_grip_toggle()
                        continue

                    if msg == "GRIP_OPEN":
                        do_grip_open()
                        continue

                    if msg == "GRIP_CLOSE":
                        do_grip_close()
                        continue

                    print(f"[CONTROL] Unknown command: {msg}")

        except Exception as e:
            print(f"[CONTROL] Connection error: {e}")
        finally:
            try:
                conn.close()
            except:
                pass

            robot_armed = False
            print("[CONTROL] VR disconnected. Robot disarmed.")


# --- 5. START SYSTEM ---
threading.Thread(target=command_server, daemon=True).start()
threading.Thread(target=camera_server, args=(MXID_INSPECTOR, UNITY_PORT_INSPECTOR, "INSPECTOR"), daemon=True).start()
time.sleep(10)
threading.Thread(target=camera_server, args=(MXID_MANAGER, UNITY_PORT_MANAGER, "SITE_MANAGER"), daemon=True).start()

print("TASK CONTROLLER ACTIVE. Press Ctrl+C to stop.")
while True:
    time.sleep(1)
