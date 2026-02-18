"""
robot_config.py
Central configuration for Side Quest robot control.

Keep ALL "magic numbers" here:
- IPs/ports
- poses
- speed profiles
- nudge step sizes
"""

# ----------------------------
# Network / Dobot Controller
# ----------------------------
ROBOT_IP = "192.168.5.1"
DASHBOARD_PORT = 29999

# ----------------------------
# Speed Profiles (SpeedFactor %)
# ----------------------------
SPEED_TRAVEL = 30        # used for larger moves (home <-> pick <-> hover)
SPEED_PRECISION = 15     # used for nudges + placing (fine motion)

# Combo mode (future): add +10–20% on travel only
COMBO_SPEED_BONUS = 10   # example: travel becomes 40% when combo active

# ----------------------------
# Poses (Cartesian pose={x,y,z,rx,ry,rz})
# Units: mm and degrees (Dobot convention)
# ----------------------------
# Safe Home / Idle pose (you already tested raising Z by +100mm)
SAFE_HOME_POSE = (350, 0, 300, 180, 0, 0)

# Placeholders for Week 2 (fill once you measure them)
PICK_POSE = (0, 0, 0, 0, 0, 0)     # TODO: replace
HOVER_POSE = (0, 0, 0, 0, 0, 0)    # TODO: replace

# ----------------------------
# Nudge Settings (Week 2)
# ----------------------------
# Discrete nudge step sizes
NUDGE_STEP_MM = 3         # XY step per button press
NUDGE_YAW_DEG = 2         # yaw step per button press (optional)

# Z is locked during nudge by design
NUDGE_DZ_MM = 0

# ----------------------------
# Gripper I/O (NOT IMPLEMENTED YET)
# ----------------------------
# You controlled it via Controller DO/DO panel in DobotStudio.
# We still need to confirm which DO channel actually actuates the gripper.
GRIPPER_DO_CHANNEL = 1    # TODO: confirm which DO_x triggers gripper action

# If your gripper needs "pulse" control rather than steady ON/OFF, configure here:
GRIPPER_PULSE_S = 0.2     # TODO: adjust if needed (0.0 = no pulse)
