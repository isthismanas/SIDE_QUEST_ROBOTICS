import time
from typing import Tuple, Dict, List

import robot_config as cfg
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE

Pose = Tuple[float, float, float, float, float, float]


# ----------------------------
# POSES FROM DOBOT STUDIO (your screenshots)
# ----------------------------
POSES: Dict[str, Pose] = {
    # --- Tower (build site) ---
    "T1": (193.4597, -91.4552, 157.8142,  180.0, 0.0, -124.0),
    "T2": (193.4597, -91.4552, 195.2472,  180.0, 0.0, -124.0),
    "T3": (193.4597, -91.4552, 232.0248,  180.0, 0.0, -124.0),
    "T4": (193.4597, -91.4552, 269.0821,  180.0, 0.0, -124.0),
    "T5": (193.4597, -91.4552, 304.8309,  180.0, 0.0, -124.0),
    "T6": (193.4597, -91.4552, 341.6388,  180.0, 0.0, -124.0),
    "T7": (193.4597, -91.4552, 379.1380,  180.0, 0.0, -124.0),
    "T7_Hover": (193.4597, -91.4552, 429.6714, 180.0, 0.0, -124.0),

    # --- Right pickup stack ---
    "R1": (362.9314, 49.1437, 155.8725, -180.0, 0.0, -124.0),
    "R2": (362.9314, 49.1437, 192.4539, -180.0, 0.0, -124.0),
    "R3": (362.9314, 49.1437, 230.2771, -180.0, 0.0, -124.0),
    "R4": (362.9314, 49.1437, 267.6572, -180.0, 0.0, -124.0),
    "R4_Hover": (362.9314, 49.1437, 330.3239, -180.0, 0.0, -124.0),

    # --- Left pickup stack ---
    "L1": (273.2320, 49.1437, 155.6196, -180.0, 0.0, -124.0),
    "L2": (273.2320, 49.1437, 192.9385, -180.0, 0.0, -124.0),
    "L3": (273.2320, 49.1437, 230.8392, -180.0, 0.0, -124.0),
    "L4": (273.2320, 49.1437, 269.3900, -180.0, 0.0, -124.0),
    "L4_Hover": (273.2320, 49.1437, 340.1021, -180.0, 0.0, -124.0),

    # --- Neutral corridor (collision avoidance) ---
    "Neutral_1": (273.2320, -23.7896, 212.9035, -180.0, 0.0, -124.0),
    "Neutral_2": (273.2320, -23.7896, 294.4369, -180.0, 0.0, -124.0),
    "Neutral_3": (273.2320, -23.7896, 378.5702, -180.0, 0.0, -124.0),
}

# Choose which neutral point to always pass through:
NEUTRAL = "Neutral_3"

# Hover points for approach/retract at pickup:
PICK_HOVER_FOR = {
    "L": "L4_Hover",
    "R": "R4_Hover",
}

# Hover point for approach/retract at tower:
TOWER_HOVER = "T7_Hover"

# Pick order you requested:
PICK_ORDER: List[str] = ["L3", "L2", "L1", "R4", "R3", "R2", "R1"]

# Place order (bottom to top):
PLACE_ORDER: List[str] = ["T1", "T2", "T3", "T4", "T5", "T6", "T7"]


def pause(msg: str, seconds: float = 0.2) -> None:
    print(msg)
    time.sleep(seconds)


def movj(robot: DobotDriver, name: str) -> None:
    pose = POSES[name]
    resp = robot.movj_pose(pose)
    print(f"[ARM] MovJ -> {name} -> {resp}")


def pick(robot: DobotDriver, grip: DHGripperPGE, pick_name: str) -> None:
    side = pick_name[0]  # 'L' or 'R'
    hover = PICK_HOVER_FOR[side]

    pause(f"\n=== PICK {pick_name} ===", 0.1)

    # Always travel via neutral corridor
    movj(robot, NEUTRAL)
    movj(robot, hover)

    # Ensure open before going down
    pause("[GRIPPER] OPEN", 0.05)
    grip.open(timeout_s=5)

    # Go down to pick
    movj(robot, pick_name)

    # Close to grab
    pause("[GRIPPER] CLOSE", 0.05)
    grip.close(timeout_s=5)

    # Retract back up
    movj(robot, hover)

    # Back through neutral
    movj(robot, NEUTRAL)


def place(robot: DobotDriver, grip: DHGripperPGE, place_name: str) -> None:
    pause(f"\n=== PLACE {place_name} ===", 0.1)

    # Always travel via neutral corridor, then tower hover
    movj(robot, NEUTRAL)
    movj(robot, TOWER_HOVER)

    # Approach place
    movj(robot, place_name)

    # Release
    pause("[GRIPPER] OPEN", 0.05)
    grip.open(timeout_s=5)

    # Retract
    movj(robot, TOWER_HOVER)

    # Back through neutral
    movj(robot, NEUTRAL)


def main():
    robot = DobotDriver()
    gripper = DHGripperPGE(open_pos=900, close_pos=50)

    print("[SETUP] Arming robot (ClearError/Enable/SpeedFactor)...")
    robot.clear_and_enable(speed_percent=cfg.SPEED_PRECISION)

    print("[SETUP] Connecting gripper (RS485 Modbus)...")
    if not gripper.connect():
        raise RuntimeError("Gripper connect failed")
    print("[SETUP] Gripper status:", gripper.status())

    # Start safe: go to home and open gripper
    pause("\n[START] Going HOME", 0.1)
    robot.go_home(speed_percent=cfg.SPEED_PRECISION)
    pause("[START] Gripper OPEN", 0.05)
    gripper.open(timeout_s=5)

    # Quick warning about Rx flip
    print("\n[NOTE] Your Tower poses use Rx=+180 while L/R use Rx=-180.")
    print("       They are equivalent angles, but the robot may choose different joint solutions.")
    print("       If you see weird wrist flips, we will normalize Rx to -180 everywhere.\n")

    # Stack 7 blocks
    for i, pick_name in enumerate(PICK_ORDER):
        place_name = PLACE_ORDER[i]
        print(f"\n########## BLOCK {i+1}/7 : {pick_name} -> {place_name} ##########")
        pick(robot, gripper, pick_name)
        place(robot, gripper, place_name)

    pause("\n[END] Returning HOME", 0.1)
    robot.go_home(speed_percent=cfg.SPEED_PRECISION)

    # Close Modbus connection
    try:
        gripper.close()  # closes connection
    except Exception:
        pass

    print("\nDONE.")


if __name__ == "__main__":
    main()
