from dh_gripper import DHGripperPGE
import time

g = DHGripperPGE(open_pos=900, close_pos=50)

print("Connecting...")
if not g.connect():
    print("Failed to connect")
    raise SystemExit(1)

print("Status:", g.status())

print("Closing...")
print(g.close(timeout_s=5))
time.sleep(0.5)

print("Opening...")
print(g.open(timeout_s=5))

g.close()
print("Done.")
