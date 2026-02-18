import socket
import time

# --- CONFIG ---
ROBOT_IP = "192.168.5.1"
DASHBOARD_PORT = 29999

def send(cmd):
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5.0); client.connect((ROBOT_IP, DASHBOARD_PORT))
        # Commands to the E6 MUST end with a newline or semicolon depending on firmware
        # The V4 controller usually accepts them as-is.
        full_cmd = f"{cmd}"
        print(f"Sending: {full_cmd}")
        client.send(full_cmd.encode('utf-8'))
        resp = client.recv(1024).decode('utf-8')
        client.close()
        return resp
    except Exception as e: return f"Error: {e}"

print("=== DOBOT E6: MOVING TO WORK POSITION ===")

# 1. Clear any errors & Enable
send("ClearError()")
send("EnableRobot()")
time.sleep(1)

# 2. Set Speed (Important for safety!)
# Sets the global speed to 20% so it doesn't jump too fast
send("SpeedFactor(20)")
time.sleep(0.5)

# 3. Move to Work Pose using MovJ (Joint Motion)
# This avoids the "Straight-Up Singularity" crash.
# Target: X=350, Y=0, Z=200, Rx=180 (Pointing Down), Ry=0, Rz=0
target_pose = "pose={350, 0, 300, 180, 0, 0}"
move_cmd = f"MovJ({target_pose})"

print(f"Moving to: {target_pose}...")
response = send(move_cmd)
print(f"Robot Response: {response}")

print("\nDONE. The robot should now be in a 'V' shape ready for work.")
