import time
from dh_gripper import DHGripperPGE

def main():
    g = DHGripperPGE(open_pos=900, close_pos=50)
    if not g.connect():
        raise RuntimeError("connect failed")

    print("Status after connect:", g.status())

    print("\nFORCE OPEN now.")
    g.open(timeout_s=5)
    time.sleep(0.2)
    print("Status after open:", g.status())

    print("\nWatching for 15 seconds. If it moves, something else is controlling it or it’s not holding.")
    for i in range(30):
        time.sleep(0.5)
        print(f"t={0.5*(i+1):4.1f}s  status={g.status()}")

if __name__ == "__main__":
    main()
