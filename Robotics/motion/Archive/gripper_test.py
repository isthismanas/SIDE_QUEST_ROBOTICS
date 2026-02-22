from pymodbus.client import ModbusSerialClient
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
SLAVE_ID = 1


def main():
    print("Opening Modbus connection...")

    client = ModbusSerialClient(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=1,
    )

    if not client.connect():
        print("Failed to connect to gripper.")
        return

    print("Connected.")

    try:
        # Set target position to 200
        print("Writing position 200 to register 0x0103...")
        result = client.write_register(
            address=0x0103,
            value=200,
            device_id=SLAVE_ID,
        )

        if result.isError():
            print("Write error:", result)
        else:
            print("Position command sent.")

    except Exception as e:
        print("Exception during write:", e)

    time.sleep(1)

    client.close()
    print("Connection closed.")


if __name__ == "__main__":
    main()
