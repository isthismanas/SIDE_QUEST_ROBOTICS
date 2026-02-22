from pymodbus.client import ModbusSerialClient
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
SLAVE_ID = 1

REG_INIT_STATE = 0x0200   # 0=not initialized, 1=initialized
REG_GRIP_STATE = 0x0201   # 0=in motion, 1=reached, 2=caught, 3=dropped
REG_CUR_POS    = 0x0202   # actual position
REG_REF_POS    = 0x0103   # target/reference position (0..1000)


def read_reg(client, addr):
    result = client.read_holding_registers(
        address=addr,
        count=1,
        device_id=SLAVE_ID,
    )
    if result.isError():
        print(f"Read error @0x{addr:04X} -> {result}")
        return None
    return result.registers[0]


def write_reg(client, addr, value):
    result = client.write_register(
        address=addr,
        value=value,
        device_id=SLAVE_ID,
    )
    if result.isError():
        print(f"Write error @0x{addr:04X} -> {result}")
        return False
    return True


def main():
    print("Connecting to gripper...")

    client = ModbusSerialClient(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=1,
    )

    if not client.connect():
        print("Failed to connect.")
        return

    print("Connected.")

    init_state = read_reg(client, REG_INIT_STATE)
    grip_state = read_reg(client, REG_GRIP_STATE)
    cur_pos = read_reg(client, REG_CUR_POS)

    print(f"Init state (0x0200): {init_state}")
    print(f"Grip state (0x0201): {grip_state}")
    print(f"Current pos (0x0202): {cur_pos}")

    targets = [900, 600, 400, 250, 150, 100, 50]

    for target in targets:
        print(f"\nCommanding target position {target}...")
        ok = write_reg(client, REG_REF_POS, target)
        if not ok:
            client.close()
            return

        reached = False

        for i in range(25):  # up to 5 seconds
            time.sleep(0.2)
            grip_state = read_reg(client, REG_GRIP_STATE)
            cur_pos = read_reg(client, REG_CUR_POS)
            print(f"  t={0.2*(i+1):.1f}s  state={grip_state}  pos={cur_pos}")

            if grip_state == 1:  # reached
                reached = True
                break

        if not reached:
            print("  WARNING: did not reach target within timeout.")

    client.close()
    print("Done.")


if __name__ == "__main__":
    main()
