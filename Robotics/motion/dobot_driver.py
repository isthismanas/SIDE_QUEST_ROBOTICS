"""
dobot_driver.py
Dobot Magician E6 Dashboard TCP driver (port 29999) + gripper primitives.

Verified on your controller:
- ToolDOExecute(...) is NOT supported (returns -10000)
- ToolDO(...) IS supported (queued) and returns a queue id
- DO(...) IS supported (queued) and returns a queue id
- Continue() IS supported and is required to execute queued commands

Verified on your hardware wiring + behaviour:
- DH Robotics PGE5 gripper is connected to the BASE I/O (not wrist/tool flange)
- Gripper responds to BASE DO_1 after DH UI Init (I/O Mode ON)
- Behaviour suggests edge-trigger / pulse style control (twitch then returns if we treat DO as a level)

Therefore, gripper control is implemented as:
    PULSE on DO(GRIP_DO_INDEX) using DO(...) -> Continue()

NOTE:
- This driver does NOT (yet) automate DH "Init" sequence. You must Init in DH-Robotics UI first.
- For deterministic open/close (instead of toggle), we will later read DI feedback and pulse-until-state.
"""

import socket
import time
from typing import Optional, Tuple


Pose = Tuple[float, float, float, float, float, float]


def _try_import_robot_config():
    """
    Optional config hook: if you already have robot_config.py, we use it.
    Otherwise fall back to safe defaults.
    """
    try:
        import robot_config as cfg  # type: ignore
        return cfg
    except Exception:
        return None


_cfg = _try_import_robot_config()


class DobotDriver:
    # ----------------------------
    # Defaults (override via robot_config.py if present)
    # ----------------------------
    ROBOT_IP = getattr(_cfg, "ROBOT_IP", "192.168.5.1")
    DASHBOARD_PORT = getattr(_cfg, "DASHBOARD_PORT", 29999)
    SOCKET_TIMEOUT_S = getattr(_cfg, "SOCKET_TIMEOUT_S", 5.0)

    SPEED_PRECISION = getattr(_cfg, "SPEED_PRECISION", 10)

    SAFE_HOME_POSE: Pose = getattr(
        _cfg,
        "SAFE_HOME_POSE",
        (300.0, 0.0, 300.0, 180.0, 0.0, 0.0),
    )

    def __init__(
        self,
        robot_ip: Optional[str] = None,
        dashboard_port: Optional[int] = None,
        timeout_s: Optional[float] = None,
    ):
        self.robot_ip = robot_ip or self.ROBOT_IP
        self.port = dashboard_port or self.DASHBOARD_PORT
        self.timeout_s = timeout_s or self.SOCKET_TIMEOUT_S

        # ----------------------------
        # Gripper config (BASE DO) — YOUR real wiring
        # ----------------------------
        # You verified it responds to DO_1 in DobotStudio after DH UI Init.
        self.GRIP_DO_INDEX = getattr(_cfg, "GRIP_DO_INDEX", 1)

        # Pulse timing (tweakable)
        self.GRIP_PULSE_WIDTH_S = getattr(_cfg, "GRIP_PULSE_WIDTH_S", 0.25)
        self.GRIP_SETTLE_S = getattr(_cfg, "GRIP_SETTLE_S", 0.35)

        # Pulse shape:
        # pulse_high=True means: ensure low -> high -> low
        # If your installation expects inverse edges, set to False in robot_config.py
        self.GRIP_PULSE_HIGH = getattr(_cfg, "GRIP_PULSE_HIGH", True)

        # Persistent dashboard socket (prevents connect/disconnect spam)
        self._sock: Optional[socket.socket] = None

    # ----------------------------
    # Connection management (persistent socket)
    # ----------------------------
    def connect(self) -> None:
        """
        Open a persistent connection to the dashboard server.
        This avoids rapid connect/disconnect causing 'port occupied' errors.
        """
        if self._sock is not None:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout_s)
        s.connect((self.robot_ip, self.port))
        self._sock = s

    def close(self) -> None:
        """
        Close the persistent dashboard connection.
        """
        if self._sock is None:
            return
        try:
            self._sock.close()
        finally:
            self._sock = None

    def __del__(self):
        # Best-effort cleanup if user forgets to close()
        try:
            self.close()
        except Exception:
            pass

    # ----------------------------
    # Core TCP/IP send (persistent)
    # ----------------------------
    def send(self, cmd: str) -> str:
        """
        Send a single dashboard command and return the raw response.
        Reuses a persistent socket connection.
        """
        cmd = cmd.strip()
        payload = (cmd + "\n").encode("utf-8")

        if self._sock is None:
            self.connect()

        try:
            assert self._sock is not None
            self._sock.sendall(payload)
            resp = self._sock.recv(4096).decode("utf-8", errors="ignore").strip()
            return resp
        except (socket.timeout, ConnectionError, OSError):
            # If the connection died or got weird, reset once and retry.
            self.close()
            self.connect()
            assert self._sock is not None
            self._sock.sendall(payload)
            resp = self._sock.recv(4096).decode("utf-8", errors="ignore").strip()
            return resp

    # ----------------------------
    # Dashboard response checks
    # ----------------------------
    @staticmethod
    def _assert_ok(resp: str, cmd: str) -> None:
        """
        Success responses typically start with '0,'.
        Example: '0,{},EnableRobot();' or '0,{126},DO(1,1);'
        """
        if resp is None or resp == "":
            raise RuntimeError(f"No response for: {cmd}")
        if not resp.strip().startswith("0,"):
            raise RuntimeError(f"Dashboard error for {cmd} -> {resp}")

    # ----------------------------
    # Robot setup helpers
    # ----------------------------
    def clear_error(self) -> str:
        cmd = "ClearError()"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        return resp

    def enable(self) -> str:
        cmd = "EnableRobot()"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        return resp

    def speed_factor(self, speed_percent: int) -> str:
        speed_percent = int(max(1, min(100, speed_percent)))
        cmd = f"SpeedFactor({speed_percent})"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        return resp

    def clear_and_enable(self, speed_percent: Optional[int] = None) -> None:
        """
        Convenience: clears error, enables, and sets speed.
        """
        print(f"[DOBOT] ClearError -> {self.clear_error()}")
        print(f"[DOBOT] EnableRobot -> {self.enable()}")

        if speed_percent is not None:
            print(f"[DOBOT] SpeedFactor({speed_percent}) -> {self.speed_factor(speed_percent)}")

        time.sleep(0.2)

    # ----------------------------
    # Queue execution control (YOUR firmware supports Continue())
    # ----------------------------
    def continue_queue(self) -> str:
        cmd = "Continue()"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        return resp

    # ----------------------------
    # Digital outputs (queued)
    # ----------------------------
    def tool_do(self, index: int, status: int, execute: bool = True) -> str:
        """
        Tool digital output (queued): ToolDO(index,status)
        Not used for your current gripper wiring, but kept for future tooling.
        """
        status = 1 if status else 0
        cmd = f"ToolDO({index},{status})"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        if execute:
            self.continue_queue()
        return resp

    def do(self, index: int, status: int, execute: bool = True) -> str:
        """
        Base/controller digital output (queued): DO(index,status)
        Your gripper is on BASE DO_1.
        """
        status = 1 if status else 0
        cmd = f"DO({index},{status})"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        if execute:
            self.continue_queue()
        return resp

    # ----------------------------
    # Gripper primitives (BASE DO pulse + Continue)
    # IMPORTANT:
    # - DH UI Init must already have been performed (IO Mode ON).
    # - Without DI feedback, open/close are treated as TOGGLE pulses.
    # ----------------------------
    def pulse_do(self, index: int, width_s: Optional[float] = None, pulse_high: Optional[bool] = None) -> None:
        """
        Generate a clean pulse on DO(index).

        pulse_high=True : ensure low -> high -> low
        pulse_high=False: ensure high -> low -> high
        """
        if width_s is None:
            width_s = self.GRIP_PULSE_WIDTH_S
        if pulse_high is None:
            pulse_high = self.GRIP_PULSE_HIGH

        on = 1 if pulse_high else 0
        off = 0 if pulse_high else 1

        # Ensure known starting level (best-effort)
        r = self.do(index, off, execute=False)
        print(f"[GRIPPER] DO({index},{off}) -> {r}")
        print(f"[GRIPPER] Continue -> {self.continue_queue()}")
        time.sleep(0.05)

        # Pulse ON
        r = self.do(index, on, execute=False)
        print(f"[GRIPPER] DO({index},{on}) -> {r}")
        print(f"[GRIPPER] Continue -> {self.continue_queue()}")
        time.sleep(width_s)

        # Return OFF
        r = self.do(index, off, execute=False)
        print(f"[GRIPPER] DO({index},{off}) -> {r}")
        print(f"[GRIPPER] Continue -> {self.continue_queue()}")
        time.sleep(0.05)

        # Let mechanics settle
        time.sleep(self.GRIP_SETTLE_S)

    def grip_toggle(self) -> None:
        print(f"[GRIPPER] PULSE TOGGLE on DO({self.GRIP_DO_INDEX})")
        self.pulse_do(self.GRIP_DO_INDEX)

    def grip_open(self) -> None:
        """
        TEMPORARY: Without DI feedback we cannot guarantee state.
        Treat open/close as toggle pulses.
        """
        self.grip_toggle()

    def grip_close(self) -> None:
        """
        TEMPORARY: Without DI feedback we cannot guarantee state.
        Treat open/close as toggle pulses.
        """
        self.grip_toggle()

    # ----------------------------
    # Motion primitives (optional / minimal)
    # ----------------------------
    @staticmethod
    def _pose_to_cmd(pose: Pose) -> str:
        x, y, z, rx, ry, rz = pose
        return f"pose={{{x},{y},{z},{rx},{ry},{rz}}}"

    def movj_pose(self, pose: Pose) -> str:
        cmd = f"MovJ({self._pose_to_cmd(pose)})"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        self.continue_queue()
        return resp

    def relmovl_user(self, dx: float, dy: float, dz: float, drx: float, dry: float, drz: float) -> str:
        cmd = f"RelMovLUser({dx},{dy},{dz},{drx},{dry},{drz})"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        self.continue_queue()
        return resp

    def go_home(self, speed_percent: Optional[int] = None) -> str:
        if speed_percent is None:
            speed_percent = self.SPEED_PRECISION
        self.speed_factor(speed_percent)
        return self.movj_pose(self.SAFE_HOME_POSE)
