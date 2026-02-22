from pymodbus.client import ModbusSerialClient
import time


class DHGripperPGE:
    """
    DH Robotics PGE gripper over RS485 Modbus RTU.

    Registers (from DH PGE external controller manual):
      0x0200 init state: 0=not initialized, 1=initialized
      0x0201 gripper state: 0=moving, 1=reached, 2=caught, 3=dropped
      0x0202 current position
      0x0103 target/reference position (0..1000)
    """

    REG_INIT_STATE = 0x0200
    REG_GRIP_STATE = 0x0201
    REG_CUR_POS = 0x0202
    REG_REF_POS = 0x0103

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200,
        device_id=1,
        timeout=1,
        open_pos=900,
        close_pos=50,
    ):
        self.port = port
        self.baudrate = baudrate
        self.device_id = device_id
        self.timeout = timeout
        self.open_pos = open_pos
        self.close_pos = close_pos

        self.client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout,
        )

    def connect(self) -> bool:
        return self.client.connect()

    def close(self) -> None:
        self.client.close()

    def _read(self, addr: int):
        r = self.client.read_holding_registers(address=addr, count=1, device_id=self.device_id)
        if r.isError():
            raise RuntimeError(f"Read error @0x{addr:04X}: {r}")
        return r.registers[0]

    def _write(self, addr: int, value: int):
        r = self.client.write_register(address=addr, value=int(value), device_id=self.device_id)
        if r.isError():
            raise RuntimeError(f"Write error @0x{addr:04X}: {r}")
        return True

    def status(self) -> dict:
        return {
            "init_state": self._read(self.REG_INIT_STATE),
            "grip_state": self._read(self.REG_GRIP_STATE),
            "pos": self._read(self.REG_CUR_POS),
        }

    def goto(self, target: int, timeout_s: float = 5.0, poll_s: float = 0.2) -> dict:
        """
        Command a target position and wait until it stops moving.
        Returns latest status dict.
        """
        self._write(self.REG_REF_POS, target)

        t0 = time.time()
        while True:
            st = self.status()
            # grip_state: 0=moving, 1=reached, 2=caught, 3=dropped
            if st["grip_state"] != 0:
                return st
            if (time.time() - t0) > timeout_s:
                raise TimeoutError(f"Timeout waiting for target {target}. Last status: {st}")
            time.sleep(poll_s)

    def open(self, **kwargs) -> dict:
        return self.goto(self.open_pos, **kwargs)

    def close(self, **kwargs) -> dict:
        return self.goto(self.close_pos, **kwargs)
