from dobot_driver import DobotDriver
import time

CANDIDATES = [
    # Execution model probes
    "Continue()",

    # Digital input probes (we're testing what exists)
    "DI(1)",
    "DI(2)",
    "GetDI(1)",
    "GetDI(2)",
    "GetInput(1)",
    "GetInput(2)",
    "GetIO()",
    "GetDO(1)",
    "GetDO(2)",
]

def main():
    d = DobotDriver()
    print(d.send("ClearError()"))
    print(d.send("EnableRobot()"))
    print(d.send("SpeedFactor(10)"))

    # Test whether DI_1 / DI_2 change when we toggle the gripper
    for i in range(6):
        print(f"\n--- Cycle {i+1} ---")
        print("DI(1) before ->", d.send("DI(1)"))
        print("DI(2) before ->", d.send("DI(2)"))

        print("DO(1,1) ->", d.send("DO(1,1)"))
        print("Continue() ->", d.send("Continue()"))
        time.sleep(0.2)
        print("DO(1,0) ->", d.send("DO(1,0)"))
        print("Continue() ->", d.send("Continue()"))

        time.sleep(0.5)  # give gripper time to move
        print("DI(1) after  ->", d.send("DI(1)"))
        print("DI(2) after  ->", d.send("DI(2)"))

    # Cleanly close the persistent socket
    d.close()
    print("\n[OK] Closed dashboard connection.")

if __name__ == "__main__":
    main()
