from pymodbus.client import ModbusSerialClient
import time


class DHGripperPGE:
    """
    DH Robotics PGE gripper over RS485 Modbus RTU.

    Registers (official DH PGE external-controller manual):
      0x0100 init command:
          0x0001 = initialize in configured direction
          0x00A5 = full initialization
      0x0103 target/reference position (0..1000)

      0x0200 init state:
          0 = not initialized
          1 = initialized

      0x0201 gripper state:
          0 = moving
          1 = reached
          2 = caught
          3 = dropped

      0x0202 current position
    """

    REG_INIT_CMD = 0x0100
    REG_REF_POS = 0x0103

    REG_INIT_STATE = 0x0200
    REG_GRIP_STATE = 0x0201
    REG_CUR_POS = 0x0202

    CMD_INIT = 0x0001
    CMD_FULL_INIT = 0x00A5

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200,
        device_id=1,
        timeout=1.0,
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

    def disconnect(self) -> None:
        self.client.close()

    def _read(self, addr: int) -> int:
        r = self.client.read_holding_registers(
            address=addr,
            count=1,
            slave=self.device_id,
        )
        if r.isError():
            raise RuntimeError(f"Read error @0x{addr:04X}: {r}")
        if not hasattr(r, "registers") or len(r.registers) != 1:
            raise RuntimeError(f"Unexpected read response @0x{addr:04X}: {r}")
        return int(r.registers[0])

    def _write(self, addr: int, value: int) -> bool:
        r = self.client.write_register(
            address=addr,
            value=int(value),
            slave=self.device_id,
        )
        if r.isError():
            raise RuntimeError(f"Write error @0x{addr:04X}: {r}")
        return True

    def get_init_state(self) -> int:
        return self._read(self.REG_INIT_STATE)

    def get_grip_state(self) -> int:
        return self._read(self.REG_GRIP_STATE)

    def get_position(self) -> int:
        return self._read(self.REG_CUR_POS)

    def status(self) -> dict:
        return {
            "init_state": self.get_init_state(),
            "grip_state": self.get_grip_state(),
            "pos": self.get_position(),
        }

    def initialize(
        self,
        full: bool = True,
        timeout_s: float = 15.0,
        poll_s: float = 0.1,
    ) -> dict:
        """
        Initialize the gripper and wait until init_state == 1.

        full=True  -> write 0xA5 to 0x0100 (full initialization)
        full=False -> write 0x01 to 0x0100 (normal initialization)
        """
        cmd = self.CMD_FULL_INIT if full else self.CMD_INIT
        self._write(self.REG_INIT_CMD, cmd)

        t0 = time.time()
        last_state = None

        while True:
            last_state = self.get_init_state()
            if last_state == 1:
                return self.status()

            if (time.time() - t0) > timeout_s:
                raise TimeoutError(
                    f"Timeout waiting for initialization. Last init_state={last_state}"
                )

            time.sleep(poll_s)

    def ensure_initialized(
        self,
        full: bool = True,
        timeout_s: float = 15.0,
        poll_s: float = 0.1,
    ) -> dict:
        """
        Only initialize if needed.
        """
        if self.get_init_state() == 1:
            return self.status()
        return self.initialize(full=full, timeout_s=timeout_s, poll_s=poll_s)

    def goto(self, target: int, timeout_s: float = 5.0, poll_s: float = 0.2) -> dict:
        """
        Command a target position and wait until motion completes.

        grip_state:
          0 = moving
          1 = reached
          2 = caught
          3 = dropped
        """
        target = int(target)
        if not 0 <= target <= 1000:
            raise ValueError(f"Target must be between 0 and 1000, got {target}")

        self._write(self.REG_REF_POS, target)

        t0 = time.time()
        while True:
            st = self.status()

            if st["grip_state"] != 0:
                return st

            if (time.time() - t0) > timeout_s:
                raise TimeoutError(
                    f"Timeout waiting for target {target}. Last status: {st}"
                )

            time.sleep(poll_s)

    def open(self, **kwargs) -> dict:
        return self.goto(self.open_pos, **kwargs)

    def close(self, **kwargs) -> dict:
        return self.goto(self.close_pos, **kwargs)