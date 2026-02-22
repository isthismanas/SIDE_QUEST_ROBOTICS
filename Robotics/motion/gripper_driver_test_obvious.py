from dh_gripper import DHGripperPGE
import time

g = DHGripperPGE(open_pos=900, close_pos=50)

print("Connecting...")
if not g.connect():
    print("Failed to connect")
    raise SystemExit(1)

print("\nInitial status:", g.status())
time.sleep(2)

print("\nCOMMAND: OPEN (900). Watch it move now.")
st = g.open(timeout_s=5)
print("After OPEN:", st)
time.sleep(3)

print("\nCOMMAND: CLOSE (50). Watch it move now.")
st = g.close(timeout_s=5)
print("After CLOSE:", st)
time.sleep(2)

g.close()
print("\nDone. Connection closed.")
