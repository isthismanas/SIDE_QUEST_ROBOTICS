"""
robot_config.py
Central configuration for Side Quest robot + gripper control.

Keep ALL magic numbers here:
- network/IP/ports
- speed profiles
- canonical poses
- nudge step sizes
- gripper RS485 parameters (Dev 7+)

Units:
- Poses: mm and degrees (Dobot dashboard convention)
"""

# ----------------------------
# Network / Dobot Dashboard
# ----------------------------
ROBOT_IP = "192.168.5.1"
DASHBOARD_PORT = 29999
SOCKET_TIMEOUT_S = 5.0

# ----------------------------
# Speed Profiles (SpeedFactor %)
# ----------------------------
SPEED_TRAVEL = 30        # larger moves (home <-> pick <-> hover)
SPEED_PRECISION = 15     # nudges + placing (fine motion)

# Combo mode (future): +10–20% travel only
COMBO_SPEED_BONUS = 10

# ----------------------------
# Poses (Cartesian pose={x,y,z,rx,ry,rz})
# ----------------------------
# Safe Home / Idle pose (tested; adjust as needed)
SAFE_HOME_POSE = (273.2320, -23.7896, 378.5702, -180.0, 0.0, -124.0)
NEUTRAL_2 = (273.2320, -23.7896, 294.4369, -180.0, 0.0, -124.0)

# Placeholders — replace with measured/validated poses
PICK_POSE = (0, 0, 0, 0, 0, 0)     # TODO: fill
HOVER_POSE = (0, 0, 0, 0, 0, 0)    # TODO: fill

# Optional: a safe "above-place" pose if you want one later
# PLACE_APPROACH_POSE = (...)

# ----------------------------
# Nudge Settings
# ----------------------------
NUDGE_STEP_MM = 3         # XY step per button press
NUDGE_YAW_DEG = 2         # yaw step per button press (optional)
NUDGE_DZ_MM = 0           # Z locked by design

# ----------------------------
# Stacking (Deterministic Pick & Place)
# ----------------------------
PICK_CLEARANCE_MM = 40.0      # height above block before/after pick
PLACE_CLEARANCE_MM = 40.0     # height above tower during approach
BLOCK_HEIGHT_MM = 37.0        # physical block height

# Stacking sequence: list of (side, level) tuples defining pick order
# side: "L" (left) or "R" (right) source stack
# level: 1-indexed level on source stack
PICK_SEQUENCE = [
    ("L", 4),
    ("R", 3),
    ("L", 3),
    ("R", 2),
    ("L", 2),
    ("R", 1),
    ("L", 1),
]

# Tower capacity
TOWER_LEVELS = 7

# Pick positions (left & right source stacks) - measured/calibrated
LEFT_PICK_BASE = (273.2320, 49.1437, 155.6196, -180.0, 0.0, -124.0)
RIGHT_PICK_BASE = (362.9314, 49.1437, 155.8725, -180.0, 0.0, -124.0)
LEFT_PICK_STEP_MM = 37.0                   # height step per level (left stack)
RIGHT_PICK_STEP_MM = 36.6                  # height step per level (right stack)

# Tower position (destination) - measured/calibrated
TOWER_BASE_POSE = (193.4597, -91.4552, 157.8142, 180.0, 0.0, -124.0)

# ----------------------------
# Gripper (Dev 7+) — RS485 Modbus RTU
# ----------------------------
# NOTE: The gripper is no longer controlled via Dobot DO pulses.
# It is controlled directly by the Raspberry Pi over RS485 (FT232).

GRIPPER_PORT = "/dev/ttyUSB0"
GRIPPER_BAUDRATE = 115200
GRIPPER_SLAVE_ID = 1

# Deterministic positions (0..1000 scale per DH PGE docs)
GRIPPER_OPEN_POS = 900
GRIPPER_CLOSE_POS = 50

# Driver behaviour
GRIPPER_COMMAND_TIMEOUT_S = 3.0    # max time to wait for move completion
GRIPPER_POLL_PERIOD_S = 0.05       # status poll interval during move


# ----------------------------
# Helper Functions: Stacking Poses
# ----------------------------
def left_pick_pose(level: int):
    """
    Compute pose to pick a block from the left stack at a given level.
    level: 1-indexed (1 = bottom block on left stack)
    """
    x, y, z0, rx, ry, rz = LEFT_PICK_BASE
    return (x, y, z0 + (level - 1) * LEFT_PICK_STEP_MM, rx, ry, rz)


def right_pick_pose(level: int):
    """
    Compute pose to pick a block from the right stack at a given level.
    level: 1-indexed (1 = bottom block on right stack)
    """
    x, y, z0, rx, ry, rz = RIGHT_PICK_BASE
    return (x, y, z0 + (level - 1) * RIGHT_PICK_STEP_MM, rx, ry, rz)


def tower_place_pose(level: int):
    """
    Compute pose to place a block on the tower at a given level (final position).
    level: 0-indexed (0 = bottom of tower, 1 = on top of first block, etc.)
    """
    x, y, z0, rx, ry, rz = TOWER_BASE_POSE
    return (x, y, z0 + level * BLOCK_HEIGHT_MM, rx, ry, rz)


def tower_hover_pose(level: int):
    """
    Compute pose to hover above the tower at a given level (safe approach height).
    level: 0-indexed (0 = above base, 1 = above first block, etc.)
    """
    x, y, z0, rx, ry, rz = TOWER_BASE_POSE
    return (x, y, z0 + level * BLOCK_HEIGHT_MM + PLACE_CLEARANCE_MM, rx, ry, rz)