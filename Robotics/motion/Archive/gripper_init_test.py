from dh_gripper import DHGripperPGE

g = DHGripperPGE()

g.connect()
print(g.ensure_initialized())
g.disconnect()