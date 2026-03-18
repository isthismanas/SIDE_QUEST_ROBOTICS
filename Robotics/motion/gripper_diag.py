#!/usr/bin/env python3
"""
Minimal RS485 gripper diagnostic for the DH PGE gripper.

This does not move the robot. By default it only:
- prints configured serial parameters
- checks whether the serial device exists
- connects to the Modbus client
- attempts to read the standard status registers

Optional flags can request initialization or an open command when you are ready.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import robot_config as cfg
from dh_gripper import DHGripperPGE


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Diagnose DH gripper RS485 communication.")
    parser.add_argument(
        "--init",
        action="store_true",
        help="Attempt gripper ensure_initialized() after status reads.",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Attempt gripper open() after status reads/init.",
    )
    return parser.parse_args()


def _print_result(label: str, fn) -> bool:
    try:
        value = fn()
        print(f"[GRIPPER_DIAG] {label}: {value}")
        return True
    except Exception as exc:
        print(f"[GRIPPER_DIAG] {label}_error: {exc}")
        return False


def main() -> int:
    args = parse_args()

    port = str(cfg.GRIPPER_PORT)
    print(f"[GRIPPER_DIAG] port={port}")
    print(f"[GRIPPER_DIAG] baudrate={cfg.GRIPPER_BAUDRATE}")
    print(f"[GRIPPER_DIAG] slave_id={cfg.GRIPPER_SLAVE_ID}")
    print(f"[GRIPPER_DIAG] port_exists={Path(port).exists()}")
    if Path(port).exists():
        try:
            st = os.stat(port)
            print(f"[GRIPPER_DIAG] device_mode={oct(st.st_mode)}")
        except Exception as exc:
            print(f"[GRIPPER_DIAG] stat_error: {exc}")

    gripper = DHGripperPGE(
        port=cfg.GRIPPER_PORT,
        baudrate=cfg.GRIPPER_BAUDRATE,
        device_id=cfg.GRIPPER_SLAVE_ID,
        timeout=1.0,
        open_pos=cfg.GRIPPER_OPEN_POS,
        close_pos=cfg.GRIPPER_CLOSE_POS,
    )

    try:
        connected = gripper.connect()
        print(f"[GRIPPER_DIAG] connect={connected}")
        if not connected:
            return 1

        _print_result("init_state", gripper.get_init_state)
        _print_result("grip_state", gripper.get_grip_state)
        _print_result("position", gripper.get_position)

        if args.init:
            _print_result("ensure_initialized", gripper.ensure_initialized)

        if args.open:
            _print_result("open", gripper.open)

        print("[GRIPPER_DIAG] done")
        return 0
    finally:
        try:
            gripper.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
