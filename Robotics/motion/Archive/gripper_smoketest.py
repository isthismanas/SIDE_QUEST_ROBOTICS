from dobot_driver import DobotDriver

CANDIDATES = [
    "ToolDOExecute(1,1)",
    "ToolDOExecute(1,0)",
    "ToolDO(1,1)",
    "ToolDO(1,0)",
    "SetToolDO(1,1)",
    "SetToolDO(1,0)",
    "DO(1,1)",
    "DO(1,0)",
    "SetDO(1,1)",
    "SetDO(1,0)",
]

def main():
    d = DobotDriver()
    print(d.send("ClearError()"))
    print(d.send("EnableRobot()"))
    print(d.send("SpeedFactor(10)"))

    for cmd in CANDIDATES:
        resp = d.send(cmd)
        print(f"{cmd:18s} -> {resp}")

if __name__ == "__main__":
    main()
