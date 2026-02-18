import time, socket, struct, os, threading
import depthai as dai
from datetime import timedelta

# --- 0. STABILITY & IP CONFIG ---
os.environ["DEPTHAI_WATCHDOG_TIMEOUT"] = "5000"
ROBOT_IP = "192.168.5.1"
ROBOT_DASHBOARD_PORT = 29999
UNITY_PORT_INSPECTOR = 8085
UNITY_PORT_MANAGER = 8086
UNITY_PORT_COMMANDS = 8088

MXID_INSPECTOR = "19443010B14C872F00" 
MXID_MANAGER = "194430108183F12E00"

# --- 1. ROBOT UTILITY ---
def send_to_dobot(msg):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0); s.connect((ROBOT_IP, ROBOT_DASHBOARD_PORT))
        s.send(msg.encode('utf-8'))
        resp = s.recv(1024).decode('utf-8')
        s.close()
        return resp
    except Exception as e: return f"Error: {e}"

# --- 2. CAMERA PIPELINE ---
def create_pipeline():
    pipeline = dai.Pipeline()
    monoL = pipeline.create(dai.node.MonoCamera)
    monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoR = pipeline.create(dai.node.MonoCamera)
    monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    
    encL = pipeline.create(dai.node.VideoEncoder)
    encL.setDefaultProfilePreset(20, dai.VideoEncoderProperties.Profile.MJPEG)
    encL.setQuality(40)
    encR = pipeline.create(dai.node.VideoEncoder)
    encR.setDefaultProfilePreset(20, dai.VideoEncoderProperties.Profile.MJPEG)
    encR.setQuality(40)
    
    monoL.out.link(encL.input); monoR.out.link(encR.input)
    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(timedelta(milliseconds=50))
    encL.bitstream.link(sync.inputs["left"]); encR.bitstream.link(sync.inputs["right"])
    xout = pipeline.create(dai.node.XLinkOut); xout.setStreamName("out"); sync.out.link(xout.input)
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
            server.bind(('0.0.0.0', port)); server.listen(1)
            while True:
                conn, addr = server.accept()
                try:
                    while True:
                        group = q.get()
                        dL = group["left"].getData().tobytes(); dR = group["right"].getData().tobytes()
                        conn.sendall(b'L' + struct.pack('>I', len(dL)) + dL)
                        conn.sendall(b'R' + struct.pack('>I', len(dR)) + dR)
                except: conn.close()
    except Exception as e: print(f"[{label}] Error: {e}")

# --- 4. HIGHWAY 2: COMMAND HUB (The Logic Bridge) ---
def command_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', UNITY_PORT_COMMANDS)); server.listen(1)
    print(f"[CONTROL] Hub ready on {UNITY_PORT_COMMANDS}")
    
    while True:
        conn, addr = server.accept()
        print(f"[CONTROL] VR Connected.")
        try:
            while True:
                data = conn.recv(1024)
                if not data: break
                msg = data.decode('utf-8').strip()
                
                if msg == "COMMIT":
                    print(">>> VR SIGNAL: COMMIT. Executing Drop...")
                    # 1. Enable just in case
                    send_to_dobot("EnableRobot()")
                    # 2. Linear relative move: Z = -20mm
                    # RelMovL(x,y,z,rx,ry,rz)
                    resp = send_to_dobot("RelMovLUser(0,0,-20,0,0,0)")
                    print(f"Robot Result: {resp}")
        except: conn.close()

# --- 5. START SYSTEM ---
threading.Thread(target=command_server, daemon=True).start()
threading.Thread(target=camera_server, args=(MXID_INSPECTOR, UNITY_PORT_INSPECTOR, "INSPECTOR"), daemon=True).start()
time.sleep(10)
threading.Thread(target=camera_server, args=(MXID_MANAGER, UNITY_PORT_MANAGER, "SITE_MANAGER"), daemon=True).start()

print("V5 HYBRID BRIDGE ACTIVE. Press Ctrl+C to stop.")
while True: time.sleep(1)
