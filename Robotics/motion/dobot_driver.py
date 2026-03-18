"""
dobot_driver.py

Dobot Magician E6 Dashboard TCP driver (port 29999).

Dev 7+ note:
- Gripper control has moved to RS485 Modbus (see dh_gripper.py).
- This driver is now arm/motion-focused.
- DO/ToolDO helpers remain for general I/O, but are NOT "the gripper".

This driver uses a persistent TCP socket to avoid rapid connect/disconnect issues.
"""

import socket
import time
import re
from typing import Optional, Tuple
from logger import debug, info, warn


Pose = Tuple[float, float, float, float, float, float]


def _try_import_robot_config():
    """
    Optional config hook:
    If robot_config.py exists, use it.
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

        # Persistent dashboard socket
        self._sock: Optional[socket.socket] = None

    # ----------------------------
    # Connection management (persistent socket)
    # ----------------------------
    def connect(self) -> None:
        """
        Open a persistent connection to the dashboard server.
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
        try:
            self.close()
        except Exception:
            pass

    # ----------------------------
    # Core TCP send (persistent)
    # ----------------------------
    def send(self, cmd: str) -> str:
        """
        Send a single dashboard command and return the raw response.
        Retries once on connection failure.
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
            # Reset and retry once
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
        Example: '0,{},EnableRobot();'
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
        Convenience: ClearError + EnableRobot + optional SpeedFactor.
        Intended to be called once per VR session, not per command.
        """
        info("DOBOT", f"ClearError -> {self.clear_error()}")
        info("DOBOT", f"EnableRobot -> {self.enable()}")

        if speed_percent is not None:
            info("DOBOT", f"SpeedFactor({speed_percent}) -> {self.speed_factor(speed_percent)}")

        time.sleep(0.2)

    # ----------------------------
    # Queue execution control
    # ----------------------------
    def continue_queue(self) -> str:
        """
        Send Continue() to the robot.
        Never raises RuntimeError. Prints warnings for empty or non-0 responses.
        Returns resp for higher layer handling.
        """
        cmd = "Continue()"
        resp = self.send(cmd)
        if resp is None or resp == "":
            warn("DOBOT", f"Warning: No response for {cmd}")
            return resp
        if not resp.strip().startswith("0,"):
            warn("DOBOT", f"Warning: Continue() returned non-0 response: {resp}")
            return resp
        return resp

    def robot_mode(self) -> int:
        """
        Query the robot mode via Dashboard `RobotMode()` command.

        Returns the integer mode on success, or -1 on parse failure.
        Expected response format: ErrorID,{value},RobotMode();
        """
        cmd = "RobotMode()"
        try:
            resp = self.send(cmd)
        except Exception as e:
            warn("DOBOT", f"RobotMode() failed: {e}")
            return -1

        # Try to parse second CSV field as integer
        try:
            parts = resp.split(',')
            if len(parts) >= 2:
                val = parts[1].strip()
                # Remove any non-digit characters (defensive)
                # but allow negative sign if present
                filtered = ''.join(ch for ch in val if (ch.isdigit() or ch == '-'))
                if filtered == '' or filtered == '-' or filtered is None:
                    raise ValueError(f"No numeric value in RobotMode response: {resp}")
                return int(filtered)
        except Exception as e:
            if getattr(_cfg, "RUN_MODE", "DEBUG") == "DEBUG":
                warn("DOBOT", f"Failed to parse RobotMode() response '{resp}': {e}")
            return -1

    def get_tcp_pose(self) -> Optional[Pose]:
        """
        Query current TCP Cartesian pose from dashboard and return
        (x, y, z, rx, ry, rz) as floats.

        Returns None on communication/parse failure.
        """
        cmd = "GetPose()"
        try:
            resp = self.send(cmd)
        except Exception as e:
            warn("DOBOT", f"GetPose() failed: {e}")
            return None

        try:
            # Extract all numeric tokens, including scientific notation.
            nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", resp)]
            if len(nums) < 6:
                raise ValueError(f"expected >=6 numeric values, got {len(nums)}")

            # Typical responses include leading error code 0; drop it when present.
            if resp.strip().startswith("0,") and len(nums) >= 7:
                nums = nums[1:]

            x, y, z, rx, ry, rz = nums[:6]
            return (x, y, z, rx, ry, rz)
        except Exception as e:
            warn("DOBOT", f"Failed to parse GetPose() response '{resp}': {e}")
            return None

    def wait_until_idle(self, timeout_s: float = 30.0, poll_s: float = 0.1) -> None:
        """
        Poll `RobotMode()` until the robot reports an idle-like mode.
        Treat mode==5 as idle. Logs observed modes. If parsing fails,
        falls back to a conservative short sleep and returns.
        If mode indicates ERROR (9) or COLLISION (11), warn and return early.
        """
        start = time.time()
        last_mode = None
        while True:
            mode = self.robot_mode()
            if mode >= 0:
                if mode != last_mode:
                    debug("DOBOT", f"RobotMode -> {mode}")
                    last_mode = mode
                # Treat 5 as idle (ROBOT_MODE_ENABLE). Do NOT treat 0 as idle.
                if mode == 5:
                    return
                # Handle error/collision modes conservatively
                if mode in (9, 11):
                    warn("DOBOT", f"RobotMode indicates fault/collision: {mode}")
                    return
            else:
                # Parsing failed; fallback
                warn("DOBOT", "RobotMode parsing failed; falling back to short sleep")
                time.sleep(0.5)
                return

            if (time.time() - start) > timeout_s:
                warn("DOBOT", f"wait_until_idle timeout after {timeout_s}s (last mode={mode})")
                return

            time.sleep(poll_s)

    # ----------------------------
    # Digital outputs (queued)
    # ----------------------------
    def tool_do(self, index: int, status: int, execute: bool = True) -> str:
        """
        Tool digital output (queued): ToolDO(index,status)
        Not used for the current gripper wiring (Dev 7+ uses RS485).
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
        Kept for generic I/O use.
        """
        status = 1 if status else 0
        cmd = f"DO({index},{status})"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        if execute:
            self.continue_queue()
        return resp

    # ----------------------------
    # Motion primitives
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
        # Wait until motion completes (replace Sync())
        try:
            self.wait_until_idle()
        except Exception as e:
            warn("DOBOT", f"wait_until_idle failed after MovJ: {e}")
        return resp

    def movl_pose(self, pose: Pose) -> str:
        """
        Linear move in Cartesian space (straight-line path).
        """
        cmd = f"MovL({self._pose_to_cmd(pose)})"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        self.continue_queue()
        try:
            self.wait_until_idle()
        except Exception as e:
            warn("DOBOT", f"wait_until_idle failed after MovL: {e}")
        return resp

    def relmovl_user(self, dx: float, dy: float, dz: float, drx: float, dry: float, drz: float) -> str:
        """
        Relative linear move in user/base frame (used for NUDGE and small controlled descents).
        """
        cmd = f"RelMovLUser({dx},{dy},{dz},{drx},{dry},{drz})"
        resp = self.send(cmd)
        self._assert_ok(resp, cmd)
        self.continue_queue()
        try:
            self.wait_until_idle()
        except Exception as e:
            warn("DOBOT", f"wait_until_idle failed after RelMovLUser: {e}")
        return resp

    def go_home(self, speed_percent: Optional[int] = None) -> str:
        """
        Move to SAFE_HOME_POSE at precision speed by default.
        """
        if speed_percent is None:
            speed_percent = self.SPEED_PRECISION
        self.speed_factor(speed_percent)
        return self.movj_pose(self.SAFE_HOME_POSE)
