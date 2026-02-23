import time
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE

POSE_NEUTRAL_3 = (273.2320, -23.7896, 378.5702, -180.0, 0.0, -124.0)
POSE_R4_HOVER  = (362.9314,  49.1437, 330.3239, -180.0, 0.0, -124.0)
POSE_R3_PICK   = (362.9314,  49.1437, 230.2771, -180.0, 0.0, -124.0)

def show(grip, label):
    try:
        print(f"[GRIPPER] {label}: {grip.status()}")
    except Exception as e:
        print(f"[GRIPPER] {label}: status read failed: {e}")

def main():
    robot = DobotDriver()
    grip = DHGripperPGE(open_pos=900, close_pos=50)

    print("[SETUP] Arm enable")
    robot.clear_and_enable(speed_percent=10)

    print("[SETUP] Gripper connect")
    if not grip.connect():
        raise RuntimeError("Gripper connect failed")

    show(grip, "after connect")

    print("\n[STEP 1] OPEN gripper (force)")
    grip.open(timeout_s=5)
    time.sleep(0.5)
    show(grip, "after open")

    # Travel move (joint) to safe corridor
    print("\n[STEP 2] MovJ -> Neutral_3")
    robot.movj_pose(POSE_NEUTRAL_3)
    time.sleep(0.5)
    show(grip, "after neutral")

    # Approach moves (linear)
    print("\n[STEP 3] MovL -> R4_Hover")
    robot.movl_pose(POSE_R4_HOVER)
    time.sleep(0.5)
    show(grip, "after R4 hover")

    print("\n[STEP 4] MovL -> R3 Pick")
    robot.movl_pose(POSE_R3_PICK)
    time.sleep(0.5)
    show(grip, "after R3 pick (before close)")

    print("\n[STEP 5] CLOSE gripper")
    grip.close(timeout_s=5)
    time.sleep(0.5)
    show(grip, "after close")

    print("\n[STEP 6] MovL -> R4_Hover")
    robot.movl_pose(POSE_R4_HOVER)
    time.sleep(0.5)
    show(grip, "after return hover")

    print("\n[DONE]")

if __name__ == "__main__":
    main()
