import time, socket, struct, os, threading
import depthai as dai
from datetime import timedelta

# NEW: use your clean driver + config
from dobot_driver import DobotDriver
import robot_config as cfg

# --- 0. STABILITY & PORT CONFIG ---
os.environ["DEPTHAI_WATCHDOG_TIMEOUT"] = "5000"

UNITY_PORT_INSPECTOR = 8085
UNITY_PORT_MANAGER   = 8086
UNITY_PORT_COMMANDS  = 8088

MXID_INSPECTOR = "19443010B14C872F00"
MXID_MANAGER   = "194430108183F12E00"

# --- Robot driver instance ---
robot = DobotDriver()

# --- Simple control mode/state (Week 2 skeleton) ---
MODE = "IDLE"   # IDLE | HOVER_WAIT | NUDGE


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


# --- Robot action helpers ---
def ensure_ready(precision: bool = True):
    """Clear+enable and set speed profile."""
    spd = cfg.SPEED_PRECISION if precision else cfg.SPEED_TRAVEL
    robot.clear_and_enable(speed_percent=spd)

def do_home():
    global MODE
    MODE = "IDLE"
    ensure_ready(precision=True)
    resp = robot.go_home(speed_percent=cfg.SPEED_PRECISION)
    print(f"[HOME] {resp}")

def do_drop():
    global MODE
    # For now: a simple 20mm descent. Later: open gripper + retract + loop.
    MODE = "IDLE"
    ensure_ready(precision=True)
    resp = robot.relmovl_user(0, 0, -20, 0, 0, 0)
    print(f"[DROP] {resp}")

def do_nudge(dx: float, dy: float):
    # Z locked by design; rotations locked by design (Week 2)
    ensure_ready(precision=True)
    resp = robot.relmovl_user(dx, dy, 0, 0, 0, 0)
    print(f"[NUDGE] dx={dx} dy={dy} -> {resp}")


# --- 4. HIGHWAY 2: COMMAND HUB (Logic Bridge) ---
def command_server():
    global MODE

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', UNITY_PORT_COMMANDS))
    server.listen(1)

    print(f"[CONTROL] Hub ready on {UNITY_PORT_COMMANDS}")
    print("[CONTROL] Commands: HOME | FIX | NUDGE dx dy | DROP | CANCEL | (COMMIT->DROP)")

    while True:
        conn, addr = server.accept()
        print(f"[CONTROL] VR Connected: {addr}")

        # Don't auto-timeout; Unity can sit idle.
        conn.settimeout(None)

        buf = ""
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                buf += data.decode('utf-8', errors='ignore')

                # Parse newline-delimited commands (recommended)
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    msg = line.strip()
                    if not msg:
                        continue

                    # Backward compatibility
                    if msg == "COMMIT":
                        msg = "DROP"

                    print(f"\n[CONTROL] Received: {msg}   (MODE={MODE})")

                    # --- HOME works anytime ---
                    if msg == "HOME":
                        do_home()
                        continue

                    # --- FIX enters NUDGE mode (for now allow anytime; later gate to HOVER_WAIT) ---
                    if msg == "FIX":
                        # If you want strict gating later:
                        # if MODE != "HOVER_WAIT": ignore
                        MODE = "NUDGE"
                        print("[MODE] Entered NUDGE mode (Z locked).")
                        continue

                    # --- CANCEL exits NUDGE back to hover wait (or idle for now) ---
                    if msg == "CANCEL":
                        if MODE != "NUDGE":
                            print("[SAFETY] CANCEL ignored (not in NUDGE).")
                            continue
                        MODE = "HOVER_WAIT"
                        print("[MODE] Exited NUDGE -> HOVER_WAIT.")
                        continue

                    # --- NUDGE dx dy (base XY) ---
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

                    # --- DROP allowed from hover or nudge (for now allow anytime) ---
                    if msg == "DROP":
                        do_drop()
                        continue

                    print(f"[CONTROL] Unknown command: {msg}")

        except Exception as e:
            print(f"[CONTROL] Connection error: {e}")
        finally:
            try: conn.close()
            except: pass
            print("[CONTROL] VR disconnected.")


# --- 5. START SYSTEM ---
threading.Thread(target=command_server, daemon=True).start()
threading.Thread(target=camera_server, args=(MXID_INSPECTOR, UNITY_PORT_INSPECTOR, "INSPECTOR"), daemon=True).start()
time.sleep(10)
threading.Thread(target=camera_server, args=(MXID_MANAGER, UNITY_PORT_MANAGER, "SITE_MANAGER"), daemon=True).start()

print("TASK CONTROLLER ACTIVE. Press Ctrl+C to stop.")
while True:
    time.sleep(1)

