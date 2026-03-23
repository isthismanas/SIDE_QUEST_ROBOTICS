"""
robot_config.py
Central configuration for Side Quest robot + gripper control.

Keep ALL magic numbers here:
- network/IP/ports
- speed profiles
- Poses: mm and degrees (Dobot dashboard convention)
"""

import os

# ----------------------------
# Logging Configuration
# ----------------------------
# Runtime debug verbosity switch (terminal diagnostics only)
DEBUG_ENABLED = False

# Run mode
# Allowed: "COMP", "DEBUG"
RUN_MODE = "DEBUG"

# LOG_LEVEL sets the default logging level for all modules.
# LOG_MODULES defines per-module overrides; module-level values supersede LOG_LEVEL.
# Allowed levels: "DEBUG", "INFO", "WARN", "ERROR", "QUIET"
LOG_LEVEL = "INFO"
LOG_MODULES = {
    "DOBOT": "WARN",
    "CAM": "WARN",
    "CONTROL": "INFO",
    "ADMIN": "INFO",
    "STACK": "WARN",
    "DRIFT": "WARN"
}

if RUN_MODE == "COMP":
    LOG_LEVEL = "WARN"
    LOG_MODULES = {
        "DOBOT": "WARN",
        "STACK": "WARN",
        "DRIFT": "WARN",
        "CAM": "QUIET",
        "CONTROL": "INFO",
        "ADMIN": "INFO"
    }
elif RUN_MODE == "DEBUG":
    LOG_LEVEL = "DEBUG"

# ----------------------------
# Network / Dobot Dashboard
# ----------------------------
ROBOT_IP = "192.168.5.1"
DASHBOARD_PORT = 29999
SOCKET_TIMEOUT_S = 5.0

# Camera identifiers. Preserve legacy MXIDs but allow current PoE device ids.
INSPECTOR_CAMERA_IDS = (
    "19443010B14C872F00",
    "169.254.1.223",
)
MANAGER_CAMERA_IDS = (
    "194430108183F12E00",
    "169.254.1.222",
)

# ----------------------------
# Speed Profiles
# ----------------------------
SPEED_TRAVEL = 30        # larger moves (home <-> pick <-> hover)
SPEED_PRECISION = 15     # nudges + placing (fine motion)

# Combo mode
COMBO_SPEED_BONUS = 10 #may be deprecated in favor of manual speed values provided
COMBO_ENABLED = True
COMBO_GREEN_PLACEMENTS_TARGET = 3
MOVEJ_SPEED_NORMAL = 20
MOVEJ_SPEED_COMBO = 90

# ----------------------------
# Poses (Cartesian pose={x,y,z,rx,ry,rz})
# ----------------------------
# Safe Home / Idle pose (tested; adjust as needed)
SAFE_HOME_POSE = (273.2320, -23.7896, 378.5702, -180.0, 0.0, -124.0)
NEUTRAL_1 = (273.2320, -23.7896, 212.9035, -180.0, 0.0, -124.0)
NEUTRAL_2 = (273.2320, -23.7896, 294.4369, -180.0, 0.0, -124.0)
NEUTRAL_3 = (273.2320, -23.7896, 378.5702, -180.0, 0.0, -124.0)

# ----------------------------
# Nudge Settings
# ----------------------------
NUDGE_STEP_MM = 1         # XY step per button press
NUDGE_YAW_DEG = 2         # yaw step per button press (optional)
NUDGE_DZ_MM = 0           # Z locked by design
NUDGE_COOLDOWN_S = 0.20   # minimum time between NUDGE commands
NUDGE_MAX_OFFSET_MM = 10  # max allowed XY offset from nominal placement center

# ----------------------------
# Stacking (Deterministic Pick & Place)
# ----------------------------
PICK_POSE_MODE = "deterministic"  # "deterministic" | "vision"
CAMERA_STREAM_ENABLED = True       # enables OAK camera streaming independent of pick mode
VISION_ASSIST_ENABLED = True       # calibrated perception in shadow mode; does not control pick motion
VISION_REFERENCE_MARKER_ID = 0     # marker used for camera-to-robot reference tracking
VISION_SHADOW_LOGS_ENABLED = True  # logs vision-derived comparisons without changing authority
VISION_TRACK_FRESHNESS_S = 1.5     # observational tracking accepts markers seen within this recent window
VISION_MARKER_CHANGE_DELTA_MM = 8.0  # only emit marker-position updates when motion exceeds this XY threshold
VISION_MARKER_CHANGE_ROT_DELTA_RAD = 0.15  # suppress jittery relogs when orientation noise is small
VISION_MARKER_CHANGE_LOG_ROTATION = False  # rotation flips alone do not count as meaningful block movement
VISION_PICK_OBJECT_PERMANENCE_ENABLED = True  # keep last-seen pickup marker positions until that block is picked
VISION_PICK_OBJECT_PERMANENCE_MAX_AGE_S = 2.0  # only trust remembered pickup markers for a very short time after last sighting
VISION_DYNAMIC_PICK_SELECTION_ENABLED = True  # when vision pick mode is authoritative, choose the next source block from live marker observations
VISION_PICK_TEMPLATE_TARGET = "P3"  # provides fixed Z/orientation for single-shot perception picks
VISION_PICK_WORKSPACE_X_MM = (210.0, 430.0)  # bounded XY workspace for arbitrary-on-plane perception picks
VISION_PICK_WORKSPACE_Y_MM = (-80.0, 60.0)
# Optional residual-learning layer for pickup-plane XY correction.
# Safety boundary:
# - disabled by default
# - affects pickup estimation only
# - does not change deterministic controller motion
# - falls back to the base calibrated affine map when unavailable
VISION_PICK_ML_ENABLED = False
VISION_PICK_ML_MODEL_JSON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "perception",
    "calibration_data",
    "pick_plane_marker13_20260318_ml_residual.json",
)
VISION_PICK_X_OFFSET_MM = 0.0
VISION_PICK_Y_OFFSET_MM = 0.0
VISION_CALIBRATION_JSON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "perception",
    "calibration_data",
    "pick_plane_marker13_20260321_retry_phase3_solution.json",
)
VISION_PICK_MARKER_MAP = {
    "P1": 11,
    "P2": 12,
    "P3": 13,
    "P4": 14,
    "P5": 15,
    "P6": 16,
    "P7": 17,
}
VISION_DROP_MARKER_MAP = {
    # Example: "T1": 21,
}
PICK_CLEARANCE_MM = 40.0      # height above block before/after pick
PLACE_CLEARANCE_MM = 40.0     # height above tower during approach
BLOCK_HEIGHT_MM = 37.0        # physical block height

# Softening lift applied to explicit build-point final placement Z.
# Positive values reduce downward compression on contact.
PLACE_Z_SOFTEN_MM = 3

# Layout switches (legacy fallback preserved)
# PICK_LAYOUT_MODE: "legacy_towers" | "pickup_plate"
# BUILD_LAYOUT_MODE: "tower_stack" | "explicit_points"
PICK_LAYOUT_MODE = "pickup_plate"
BUILD_LAYOUT_MODE = "explicit_points"

# Tower capacity
TOWER_LEVELS = 7

# Pick positions (left & right source stacks) - measured/calibrated
LEFT_PICK_BASE = (273.2320, 49.1437, 155.6196, -180.0, 0.0, -124.0)
RIGHT_PICK_BASE = (362.9314, 49.1437, 155.8725, -180.0, 0.0, -124.0)
LEFT_PICK_STEP_MM = 37.0                   # height step per level (left stack)
RIGHT_PICK_STEP_MM = 36.6                  # height step per level (right stack)

# Tower position (destination) - measured/calibrated
TOWER_BASE_POSE = (193.4597, -91.4552, 157.8142, 180.0, 0.0, -124.0)

# Safe dump pose (used when tower tumbles)
SAFE_DUMP_POSE = (197.4741, 49.1437, 156.6749, 180.0, 0.0, -124.0)

# Explicit pickup-plate points
PICKUP_POINTS = {
    "P1": (404.2777, -67.4403, 157.2881, -180.0, 0.0, -124.0),
    "P2": (322.1818, -67.4403, 157.2881, -180.0, 0.0, -124.0),
    "P3": (362.3367, -13.4532, 157.2881, -180.0, 0.0, -124.0),
    "P4": (278.4949, -13.4532, 157.2881, -180.0, 0.0, -124.0),
    "P5": (403.8273, 40.8819, 157.2881, -180.0, 0.0, -124.0),
    "P6": (323.1831, 38.1755, 157.2881, -180.0, 0.0, -124.0),
    "P7": (238.8195, 36.9264, 157.2881, -180.0, 0.0, -124.0),
}

# Explicit build points
BUILD_POINTS = {
    "T1": (197.8699, -93.1587, 160.3303, 179.9999, -0.0001, -124.0),
    "T2": (197.8699, -93.1587, 197.4850, 179.9999, -0.0001, -124.0),
    "T3": (197.8699, -93.1587, 234.4143, 179.9999, -0.0001, -124.0),
    "T4": (197.8699, -93.1587, 271.4543, 179.9999, -0.0001, -124.0),
    "T5": (197.8699, -93.1587, 308.7479, 179.9999, -0.0001, -124.0),
    "T6": (197.8699, -93.1587, 345.7816, 179.9999, -0.0001, -124.0),
    "T7": (197.8699, -93.1587, 383.2371, 179.9999, -0.0001, -124.0),
}

# Explicit tumble dump point
EXPLICIT_TUMBLE_DUMP_POSE = (195.5782, -15.3110, 157.2881, 179.9999, -0.0001, -124.0)

# Explicit neutral corridor points
EXPLICIT_NEUTRALS = {
    1: (289.5256, -18.6253, 226.9103, 179.9999, -0.0001, -124.0),
    2: (289.5256, -18.6253, 310.4436, 179.9999, -0.0001, -124.0),
    3: (289.5256, -18.6253, 373.5770, 179.9999, -0.0001, -124.0),
}

LEGACY_PICK_SEQUENCE = ["L4", "R3", "L3", "R2", "L2", "R1", "L1"]
PLATE_PICK_SEQUENCE = ["P1", "P2", "P3", "P4", "P5", "P6", "P7"]
BUILD_SEQUENCE = ["T1", "T2", "T3", "T4", "T5", "T6", "T7"]
PICK_SEQUENCE = PLATE_PICK_SEQUENCE if str(PICK_LAYOUT_MODE).strip().lower() == "pickup_plate" else LEGACY_PICK_SEQUENCE

# Explicit mode top-stack escape shuffle after retract hover (T6/T7)
EXPLICIT_POST_PLACE_Y_SHUFFLE_MM = 60.0

# Session logging
LOG_DIR = "logs"

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


def _is_explicit_pickup_layout() -> bool:
    return str(PICK_LAYOUT_MODE).strip().lower() == "pickup_plate"


def _is_explicit_build_layout() -> bool:
    return str(BUILD_LAYOUT_MODE).strip().lower() == "explicit_points"


def stack_target_count() -> int:
    return min(int(TOWER_LEVELS), len(PICK_SEQUENCE), len(BUILD_SEQUENCE))


def neutral_pose_for_slot(slot: int):
    if _is_explicit_pickup_layout() or _is_explicit_build_layout():
        return EXPLICIT_NEUTRALS[int(slot)]
    if int(slot) <= 1:
        return NEUTRAL_1
    if int(slot) == 2:
        return NEUTRAL_2
    return NEUTRAL_3


def pick_gateway_neutrals(stack_level: int) -> tuple[int, int]:
    lvl = int(stack_level)
    if _is_explicit_pickup_layout():
        pre_slots = [3, 1, 1, 2, 2, 2, 3]
        post_slots = [1, 1, 2, 2, 2, 3, 3]
        idx = max(0, min(lvl, len(pre_slots) - 1))
        return pre_slots[idx], post_slots[idx]
    return 2, 2


def place_exit_neutral_slot(stack_level: int) -> int:
    lvl = int(stack_level)
    if _is_explicit_build_layout():
        slots = [1, 1, 2, 2, 2, 3, 3]
        idx = max(0, min(lvl, len(slots) - 1))
        return slots[idx]
    return 3 if lvl >= 3 else 2


def requires_post_place_y_shuffle(stack_level: int) -> bool:
    return _is_explicit_build_layout() and int(stack_level) in (4, 5, 6)


def post_place_y_shuffle_mm(stack_level: int) -> float:
    return EXPLICIT_POST_PLACE_Y_SHUFFLE_MM if requires_post_place_y_shuffle(stack_level) else 0.0


def tumble_dump_pose():
    if _is_explicit_build_layout():
        return EXPLICIT_TUMBLE_DUMP_POSE
    return SAFE_DUMP_POSE


def pick_target_pose(target_id: str):
    tid = str(target_id).strip().upper()
    if tid.startswith("P"):
        if tid not in PICKUP_POINTS:
            raise ValueError(f"Unknown pickup target id: {target_id}")
        return PICKUP_POINTS[tid]
    if len(tid) >= 2 and tid[0] in {"L", "R"} and tid[1:].isdigit():
        level = int(tid[1:])
        return left_pick_pose(level) if tid[0] == "L" else right_pick_pose(level)
    raise ValueError(f"Unsupported pickup target id: {target_id}")


def pick_target_hover_pose(target_id: str):
    pose = pick_target_pose(target_id)
    return (pose[0], pose[1], pose[2] + PICK_CLEARANCE_MM, pose[3], pose[4], pose[5])


def build_target_id_for_level(level: int) -> str:
    idx = max(0, min(int(level), len(BUILD_SEQUENCE) - 1))
    return BUILD_SEQUENCE[idx]


def build_target_pose(target_id: str):
    tid = str(target_id).strip().upper()
    if _is_explicit_build_layout():
        if tid not in BUILD_POINTS:
            raise ValueError(f"Unknown build target id: {target_id}")
        x, y, z, rx, ry, rz = BUILD_POINTS[tid]
        return (x, y, z + PLACE_Z_SOFTEN_MM, rx, ry, rz)
    if tid.startswith("T") and tid[1:].isdigit():
        level = int(tid[1:]) - 1
        x, y, z0, rx, ry, rz = TOWER_BASE_POSE
        return (x, y, z0 + level * BLOCK_HEIGHT_MM, rx, ry, rz)
    raise ValueError(f"Unsupported build target id: {target_id}")


def tower_place_pose(level: int):
    """
    Compute pose to place a block on the tower at a given level (final position).
    level: 0-indexed (0 = bottom of tower, 1 = on top of first block, etc.)
    """
    target_id = build_target_id_for_level(level)
    return build_target_pose(target_id)


def tower_hover_pose(level: int):
    """
    Compute pose to hover above the tower at a given level (safe approach height).
    level: 0-indexed (0 = above base, 1 = above first block, etc.)
    """
    place_pose = tower_place_pose(level)
    return (
        place_pose[0],
        place_pose[1],
        place_pose[2] + PLACE_CLEARANCE_MM,
        place_pose[3],
        place_pose[4],
        place_pose[5],
    )


# ----------------------------
# Tolerance Engine (Robot-only, Dev 12)
# ----------------------------
TOL_GREEN_MM = 0.5
TOL_YELLOW_MM = 2.5
TOLERANCE_SCALE = 1.0

# Risk escalation: if placement zone is RED, increase drift for next block
DRIFT_RISK_INCREMENT = 0.15


# ----------------------------
# Drift Engine (Dev 10)
# ----------------------------

# Master toggle
DRIFT_ENABLED = True

# Deterministic run seed (change only when you want a new reproducible pattern)
DRIFT_RUN_SEED = 12345

# Runtime run seed controls
# True: include per-run runtime seed in drift randomness
# False: ignore runtime seed and keep deterministic baseline by level
DRIFT_USE_RUNTIME_SEED = True

# Optional fixed runtime seed for reproducible debugging (None = random each run)
DRIFT_FORCE_RUN_SEED = None

# Runtime-populated by controller at run start
DRIFT_RUNTIME_RUN_SEED = None
DRIFT_RUNTIME_PARTICIPANT = ""

# Baseline max XY drift in mm at 1x scale
DRIFT_MAX_XY_MM = 5.0

# Experimental scale multiplier (only knob you change during experiments)
# 0.0 = no drift
# 1.0 = baseline
# 2.0 = double drift
DRIFT_SCALE = 0
DRIFT_SCALE_DEFAULT = DRIFT_SCALE

# Distribution mode
# "uniform" = random within square bounds
# "grid"    = deterministic grid offsets
# "fixed"   = always same offset direction
DRIFT_MODE = "uniform"

# Axis mask mode
# "X"  -> drift only along X
# "Y"  -> drift only along Y
# "XY" -> drift along both axes (original behavior)
DRIFT_AXIS_MODE = "XY"
