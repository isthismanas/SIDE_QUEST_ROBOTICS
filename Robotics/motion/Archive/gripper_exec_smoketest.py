import time
from dobot_driver import DobotDriver

def main():
    d = DobotDriver()

    print(d.send("ClearError()"))
    print(d.send("EnableRobot()"))
    print(d.send("SpeedFactor(10)"))

    print("\n--- DO 1 ON ---")
    print(d.send("DO(1,1)"))
    print(d.send("Continue()"))
    time.sleep(1)

    print("--- DO 1 OFF ---")
    print(d.send("DO(1,0)"))
    print(d.send("Continue()"))
    time.sleep(1)

    print("Done.")

if __name__ == "__main__":
    main()
