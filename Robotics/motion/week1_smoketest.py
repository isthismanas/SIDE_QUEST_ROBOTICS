"""
week1_smoketest.py

Purpose:
Validate that:
- TCP/IP communication works
- Safe Home works
- SpeedFactor works
- Relative motion works
- Driver abstraction works

Run:
    python3 week1_smoketest.py
"""

import time
from dobot_driver import DobotDriver
import robot_config as cfg


def main():
    robot = DobotDriver()

    print("=== WEEK 1 SMOKE TEST START ===")

    # 1. Clear + Enable at precision speed
    robot.clear_and_enable(speed_percent=cfg.SPEED_PRECISION)

    # 2. Move to Safe Home
    print("\nMoving to SAFE_HOME...")
    resp = robot.go_home(speed_percent=cfg.SPEED_PRECISION)
    print(f"Response: {resp}")
    time.sleep(1)

    # 3. Small relative move (sanity check)
    print("\nPerforming small relative move (+20mm Z)...")
    resp = robot.relmovl_user(0, 0, 20, 0, 0, 0)
    print(f"Response: {resp}")
    time.sleep(1)

    # 4. Return to Safe Home
    print("\nReturning to SAFE_HOME...")
    resp = robot.go_home(speed_percent=cfg.SPEED_PRECISION)
    print(f"Response: {resp}")
    time.sleep(1)

    print("\n=== WEEK 1 SMOKE TEST COMPLETE ===")


if __name__ == "__main__":
    main()
