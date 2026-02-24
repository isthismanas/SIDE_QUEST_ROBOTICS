"""
actions.py

Side-effect actions for Side Quest Dev 7+.

This module contains *only* the operations that actually move the robot or actuate the gripper.
No TCP. No video. No socket parsing. No state transitions.

task_controller.py should:
- parse incoming command -> event
- state_machine decides if allowed + next state
- if allowed, call these action functions

Design:
- keep everything parameterized by robot_config.py
- keep Dobot primitives in dobot_driver.py
- keep Gripper primitives in dh_gripper.py
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import robot_config as cfg
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE


@dataclass
class SystemHandles:
    """Holds live interfaces to hardware."""
    robot: DobotDriver
    gripper: DHGripperPGE


# ----------------------------
# Session-level setup
# ----------------------------

def arm_robot_once(handles: SystemHandles) -> None:
    """
    Clears errors + enables robot + sets a safe speed.
    Intended to be called when Unity connects (once per session).
    """
    handles.robot.clear_and_enable(speed_percent=cfg.SPEED_PRECISION)


def connect_gripper_once(handles: SystemHandles) -> None:
    """
    Opens RS485 connection to the gripper.
    Intended to be called when Unity connects (once per session).
    """
    handles.gripper.connect()


def initialize_stack_session(handles: SystemHandles) -> None:
    """
    Initialize a fresh stacking session.
    - Move robot to safe home pose (joint motion)
    - Open gripper to ready state
    
    Called after arming + gripper connect on each VR session start.
    """
    # Move to safe home using joint motion
    handles.robot.movj_pose(cfg.SAFE_HOME_POSE)
    
    # Open gripper
    handles.gripper.open()


# ----------------------------
# Fault recovery
# ----------------------------

def recover_from_fault(handles: SystemHandles) -> bool:
    """
    Best-effort recovery from a faulted state.
    
    Steps:
    1. Query initial robot mode
    2. If already in fault mode (9, 11), skip motion and return
    3. Clear any error state
    4. Re-enable the robot
    5. Set safe speed factor
    6. Move to safe home pose (joint motion)
    7. Open gripper (gracefully skip if not connected)
    8. Query final robot mode
    
    This is designed to be called after a FAULT event to attempt safe recovery.
    Exceptions are caught and logged; fails gracefully rather than re-raising.
    """
    # Local helper to query robot mode safely
    def _mode():
        try:
            m = handles.robot.robot_mode()
            return None if m == -1 else m
        except Exception:
            return None

    print("[RECOVERY] recover_from_fault: start")

    # Query initial mode (for logging)
    mode_before = _mode()
    print(f"[RECOVERY] mode_before={mode_before}")

    # If robot is not in fault, run lightweight re-center and return OK
    if mode_before is not None and mode_before not in (9, 11):
        try:
            handles.robot.speed_factor(cfg.SPEED_PRECISION)
            print(f"[RECOVERY] speed_factor={cfg.SPEED_PRECISION}")
        except Exception as e:
            print(f"[RECOVERY] speed_factor failed: {e}")
        try:
            handles.robot.movj_pose(cfg.SAFE_HOME_POSE)
            handles.robot.wait_until_idle()
            print("[RECOVERY] moved to SAFE_HOME_POSE")
        except Exception as e:
            print(f"[RECOVERY] movj failed: {e}")
            return False
        try:
            handles.gripper.open()
            print("[RECOVERY] gripper opened")
        except Exception as e:
            print(f"[RECOVERY] gripper open failed: {e}")
        print("[RECOVERY] recover_from_fault: complete (idempotent)")
        return True

    # Otherwise, proceed with full recovery
    try:
        resp = handles.robot.clear_error()
        print(f"[RECOVERY] clear_error -> {resp}")
    except Exception as e:
        print(f"[RECOVERY] clear_error failed: {e}")

    try:
        resp = handles.robot.enable()
        print(f"[RECOVERY] enable -> {resp}")
    except Exception as e:
        print(f"[RECOVERY] enable failed: {e}")

    # Poll RobotMode up to 3s after ClearError + EnableRobot.
    # Fail only on hard fault (9/11). Any other valid mode (>=0) is recoverable.
    start_t = time.time()
    timeout = 3.0
    mode_after = None
    while time.time() - start_t < timeout:
        m = _mode()
        if m is None:
            time.sleep(0.25)
            continue
        if m in (9, 11):
            print(f"[RECOVERY] mode_before={mode_before}")
            print(f"[RECOVERY] mode_after={m}")
            print("[RECOVERY] HARD FAULT detected during poll. Aborting.")
            return False
        if m >= 0:
            mode_after = m
            break
        time.sleep(0.25)

    if mode_after is None:
        print(f"[RECOVERY] mode_before={mode_before}")
        print(f"[RECOVERY] mode_after={mode_after}")
        print("[RECOVERY] No valid RobotMode within timeout. Aborting.")
        return False

    print(f"[RECOVERY] mode_before={mode_before}")
    print(f"[RECOVERY] mode_after={mode_after}")

    # Proceed with recovery motions
    try:
        handles.robot.speed_factor(cfg.SPEED_PRECISION)
        print(f"[RECOVERY] speed_factor={cfg.SPEED_PRECISION}")
    except Exception as e:
        print(f"[RECOVERY] speed_factor failed: {e}")

    try:
        handles.robot.movj_pose(cfg.SAFE_HOME_POSE)
        handles.robot.wait_until_idle()
        print("[RECOVERY] moved to SAFE_HOME_POSE")
    except Exception as e:
        print(f"[RECOVERY] movj failed: {e}")
        return False

    try:
        handles.gripper.open()
        print("[RECOVERY] gripper opened")
    except Exception as e:
        print(f"[RECOVERY] gripper open failed: {e}")

    print("[RECOVERY] recover_from_fault: complete")
    return True



def do_home(handles: SystemHandles) -> None:
    """Go to safe home pose."""
    handles.robot.speed_factor(cfg.SPEED_PRECISION)
    handles.robot.go_home(speed_percent=cfg.SPEED_PRECISION)


def do_drop(handles: SystemHandles, dz_mm: float = -20.0) -> None:
    """
    Minimal 'drop' primitive used currently.
    This is NOT the final placing routine; just a controlled linear descent.
    """
    handles.robot.speed_factor(cfg.SPEED_PRECISION)
    handles.robot.relmovl_user(0, 0, dz_mm, 0, 0, 0)


def do_nudge_xy(handles: SystemHandles, dx_mm: float, dy_mm: float) -> None:
    """XY nudge in base/user frame."""
    handles.robot.speed_factor(cfg.SPEED_PRECISION)
    handles.robot.relmovl_user(dx_mm, dy_mm, 0, 0, 0, 0)


def do_nudge_yaw(handles: SystemHandles, dtheta_deg: float) -> None:
    """
    Optional yaw nudge.
    Only call this if your Dobot supports RelMovLUser with rotation in your setup.
    """
    handles.robot.speed_factor(cfg.SPEED_PRECISION)
    handles.robot.relmovl_user(0, 0, 0, 0, 0, dtheta_deg)


# ----------------------------
# Stacking: Pick & Place (Hybrid MovJ/MovL)
# ----------------------------

def execute_pick_sequence(handles: SystemHandles, side: str, level: int) -> None:
    """
    Deterministic left/right pick sequence with hybrid strategy.
    - MovJ to hover zone (joint motion, faster)
    - MovL vertical descent to block (linear, controlled)
    - Close gripper
    - MovL vertical retract
    - MovJ to neutral exit pose

    Args:
        side: "L" for left stack, "R" for right stack
        level: 1-indexed block level on source stack
    """
    robot = handles.robot
    gripper = handles.gripper

    if side == "L":
        pick_pose = cfg.left_pick_pose(level)
    else:
        pick_pose = cfg.right_pick_pose(level)

    hover_pose = (
        pick_pose[0],
        pick_pose[1],
        pick_pose[2] + cfg.PICK_CLEARANCE_MM,
        pick_pose[3],
        pick_pose[4],
        pick_pose[5],
    )

    print(f"[STACK] pick target side={side} level={level} pick_pose={pick_pose}")
    print(f"[STACK] pick hover_pose={hover_pose}")

    # Joint transition into region
    robot.movj_pose(hover_pose)
    # Wait for joint motion to complete before linear descent
    robot.wait_until_idle()

    # Linear vertical descent
    robot.movl_pose(pick_pose)
    # Ensure linear descent completed before actuating gripper
    robot.wait_until_idle()

    gripper.close()
    time.sleep(0.5)

    # Linear vertical retract
    robot.movl_pose(hover_pose)
    # Wait for retract to finish before joint exit
    robot.wait_until_idle()

    # Joint exit to neutral
    robot.movj_pose(cfg.NEUTRAL_2)


def move_to_tower_hover(handles: SystemHandles, stack_level: int) -> None:
    """
    Move to safe hover position above tower using joint motion (MovJ).
    
    Args:
        stack_level: 0-indexed (0 = above base, 1 = above first block, etc.)
    """
    robot = handles.robot
    hover_pose = cfg.tower_hover_pose(stack_level)
    print(f"[STACK] tower hover level={stack_level} hover_pose={hover_pose}")
    robot.movj_pose(hover_pose)


def complete_place_sequence(handles: SystemHandles, stack_level: int) -> None:
    """
    Deterministic placement completion with hybrid strategy.
    - MovL vertical descent to place position
    - Open gripper
    - MovL vertical retract to hover
    - MovJ to neutral exit pose

    Args:
        stack_level: 0-indexed tower level (0 = base, 1 = on first block, etc.)
    """
    robot = handles.robot
    gripper = handles.gripper

    place_pose = cfg.tower_place_pose(stack_level)
    hover_pose = cfg.tower_hover_pose(stack_level)

    # Linear vertical descent
    robot.movl_pose(place_pose)

    gripper.open()
    time.sleep(0.3)

    # Linear vertical retract
    robot.movl_pose(hover_pose)
    # Ensure retract finished before joint exit
    robot.wait_until_idle()

    # Optional sidestep for very high stacks to reduce joint travel over tower
    if stack_level >= 5:
        sidestep_pose = (
            hover_pose[0],
            -10.0,
            hover_pose[2],
            hover_pose[3],
            hover_pose[4],
            hover_pose[5],
        )
        robot.movl_pose(sidestep_pose)
        robot.wait_until_idle()

    # Select neutral exit based on stack height
    if stack_level >= 3:
        robot.movj_pose(cfg.NEUTRAL_3)
    else:
        robot.movj_pose(cfg.NEUTRAL_2)


# ----------------------------
# Gripper actions (RS485)
# ----------------------------

def do_grip_open(handles: SystemHandles) -> None:
    handles.gripper.goto(cfg.GRIPPER_OPEN_POS)


def do_grip_close(handles: SystemHandles) -> None:
    handles.gripper.goto(cfg.GRIPPER_CLOSE_POS)


_last_toggle_t = 0.0

def do_grip_toggle(handles: SystemHandles, debounce_s: float = 0.40) -> None:
    """
    Toggle based on current position relative to midpoint.
    Includes a debounce to avoid repeated rapid toggles on button bounce.
    """
    global _last_toggle_t
    t = time.time()
    if (t - _last_toggle_t) < debounce_s:
        return
    _last_toggle_t = t

    st = handles.gripper.status()
    pos = st.get("pos", None)
    if pos is None:
        # if no position read, default to close
        do_grip_close(handles)
        return

    midpoint = (cfg.GRIPPER_OPEN_POS + cfg.GRIPPER_CLOSE_POS) / 2.0
    if pos > midpoint:
        do_grip_close(handles)
    else:
        do_grip_open(handles)