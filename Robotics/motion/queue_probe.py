from dobot_driver import DobotDriver
import time

CANDIDATES = [
    "Play()",
    "Start()",
    "Run()",
    "Continue()",
    "Resume()",
    "StartScript()",
    "RunScript()",
]

def main():
    d = DobotDriver()
    print(d.send("ClearError()"))
    print(d.send("EnableRobot()"))
    print(d.send("SpeedFactor(10)"))

    # Queue a tool IO toggle
    print("ToolDO(1,1) ->", d.send("ToolDO(1,1)"))
    time.sleep(0.2)
    print("ToolDO(1,0) ->", d.send("ToolDO(1,0)"))

    print("\n--- Trying execution commands ---")
    for cmd in CANDIDATES:
        resp = d.send(cmd)
        print(f"{cmd:12s} -> {resp}")

if __name__ == "__main__":
    main()
