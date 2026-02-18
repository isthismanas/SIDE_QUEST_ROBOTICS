"""
dobot_driver.py
Low-level Dobot Magician E6 TCP/IP driver (Dashboard Port 29999).

- Owns socket comms + command formatting
- Provides reusable primitives: enable, speed, movj, relmovl_user, go_home
- Gripper functions are stubbed for now (we'll implement once DO channel is confirmed)
"""

import socket
import time
from typing import Tuple, Optional

import robot_config as cfg


Pose = Tuple[float, float, float, float, float, float]


class DobotDriver:
    def __init__(self,
                 robot_ip: str = cfg.ROBOT_IP,
                 port: int = cfg.DASHBOARD_PORT,
                 timeout_s: float = 5.0):
        self.robot_ip = robot_ip
        self.port = port
        self.timeout_s = timeout_s

    # ----------------------------
    # Core TCP/IP send
    # ----------------------------
    def send(self, cmd: str) -> str:
        """
        Send one command to Dobot dashboard port and return response.
        Dobot expects newline-terminated commands (safe to always send '\n').
        """
        cmd = cmd.strip()
        if not cmd.endswith(")"):
            # not strictly required, but helps catch accidental garbage
            pass

        payload = cmd + "\n"

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(self.timeout_s)

        try:
            client.connect((self.robot_ip, self.port))
            client.sendall(payload.encode("utf-8"))
            resp = client.recv(4096).decode("utf-8", errors="ignore").strip()
            return resp
        finally:
            try:
                client.close()
            except Exception:
                pass

    # ----------------------------
    # Common robot setup
    # ----------------------------
    def clear_error(self) -> str:
        return self.send("ClearError()")

    def enable(self) -> str:
        return self.send("EnableRobot()")

    def disable(self) -> str:
        # Some firmware supports DisableRobot(); if yours doesn't, we'll remove later.
        return self.send("DisableRobot()")

    def set_speed(self, speed_percent: int) -> str:
        """
        SpeedFactor: 1–100 (percent). Keep it conservative.
        """
        speed_percent = int(max(1, min(100, speed_percent)))
        return self.send(f"SpeedFactor({speed_percent})")

    def clear_and_enable(self, speed_percent: Optional[int] = None, settle_s: float = 0.5) -> None:
        """
        Convenience: clear error + enable + optional speed set.
        """
        r1 = self.clear_error()
        print(f"[DOBOT] ClearError -> {r1}")

        r2 = self.enable()
        print(f"[DOBOT] EnableRobot -> {r2}")

        time.sleep(settle_s)

        if speed_percent is not None:
            r3 = self.set_speed(speed_percent)
            print(f"[DOBOT] SpeedFactor({speed_percent}) -> {r3}")
            time.sleep(0.2)

    # ----------------------------
    # Motion primitives
    # ----------------------------
    @staticmethod
    def _pose_to_str(pose: Pose) -> str:
        x, y, z, rx, ry, rz = pose
        # Keep formatting clean for Dobot parser
        return f"pose={{{{ {x}, {y}, {z}, {rx}, {ry}, {rz} }}}}".replace("{{", "{").replace("}}", "}")

    def movj_pose(self, pose: Pose) -> str:
        """
        Joint-interpolated move to a Cartesian pose target.
        """
        pose_str = self._pose_to_str(pose)
        return self.send(f"MovJ({pose_str})")

    def relmovl_user(self,
                    dx: float, dy: float, dz: float,
                    drx: float, dry: float, drz: float) -> str:
        """
        Linear relative move in USER frame.
        """
        return self.send(f"RelMovLUser({dx},{dy},{dz},{drx},{dry},{drz})")

    def go_home(self, speed_percent: int = cfg.SPEED_PRECISION) -> str:
        """
        Safe home pose.
        """
        self.set_speed(speed_percent)
        return self.movj_pose(cfg.SAFE_HOME_POSE)

    # ----------------------------
    # Gripper (stub)
    # ----------------------------
    def grip_open(self) -> None:
        """
        Not implemented yet.
        You currently toggle Controller DO in DobotStudio.
        Once we confirm which DO channel and logic, we'll implement SetDO/DOExecute here.
        """
        print("[GRIPPER] grip_open() NOT IMPLEMENTED (needs DO channel/command confirmation).")

    def grip_close(self) -> None:
        print("[GRIPPER] grip_close() NOT IMPLEMENTED (needs DO channel/command confirmation).")
