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
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import robot_config as cfg
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE
import drift_engine
from logger import info, debug, warn


cfg_pose_type = tuple[float, float, float, float, float, float]


class PickPoseProvider(ABC):
    @abstractmethod
    def get_pick_pose(self, side: str, level: int) -> tuple[Optional[cfg_pose_type], str]:
        """Return (pose, reason). pose=None means unavailable."""


class DeterministicPickPoseProvider(PickPoseProvider):
    def get_pick_pose(self, side: str, level: int) -> tuple[Optional[cfg_pose_type], str]:
        if side == "L":
            return cfg.left_pick_pose(level), "ok"
        if side == "R":
            return cfg.right_pick_pose(level), "ok"
        return None, f"Unknown pick side '{side}'"


class VisionPickPoseProvider(PickPoseProvider):
    def get_pick_pose(self, side: str, level: int) -> tuple[Optional[cfg_pose_type], str]:
        return None, "Vision pick pose provider not implemented"


class PickPoseUnavailableError(RuntimeError):
    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


def _active_pick_pose_provider() -> PickPoseProvider:
    mode = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).lower()
    if mode in {"vision", "perception"}:
        return VisionPickPoseProvider()
    return DeterministicPickPoseProvider()


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

def _is_gripper_holding_block(handles: SystemHandles, fallback_holding: Optional[bool]) -> bool:
    """
    Conservative holding detector for tumble routing.
    Priority:
    1) grip_state==2 (caught) -> holding
    2) current position near closed side -> holding
    3) fallback_holding when status unavailable
    4) conservative default True
    """
    try:
        st = handles.gripper.status()
        grip_state = st.get("grip_state")
        pos = st.get("pos")
        if grip_state == 2:
            return True
        if isinstance(pos, (int, float)):
            midpoint = (cfg.GRIPPER_OPEN_POS + cfg.GRIPPER_CLOSE_POS) / 2.0
            return pos <= midpoint
    except Exception as e:
        warn("STACK", f"[TUMBLE] Gripper status read failed, using fallback: {e}")

    if fallback_holding is not None:
        return bool(fallback_holding)
    return True


def execute_tumble_sequence(handles: SystemHandles, fallback_holding: Optional[bool] = None) -> bool:
    """
    Deterministic tumble flow.
    Returns detected holding state before executing motions.

    A) Holding -> SAFE_DUMP_POSE -> open -> NEUTRAL_3
    B) Not holding -> NEUTRAL_3
    """
    holding = _is_gripper_holding_block(handles, fallback_holding)
    info("STACK", f"[TUMBLE] holding_detected={holding} fallback_holding={fallback_holding}")

    if holding:
        dump_pose = cfg.SAFE_DUMP_POSE
        dump_hover_pose = (
            dump_pose[0],
            dump_pose[1],
            dump_pose[2] + cfg.PLACE_CLEARANCE_MM,
            dump_pose[3],
            dump_pose[4],
            dump_pose[5],
        )

        info("STACK", f"[TUMBLE] branch=A step=move_dump_hover pose={dump_hover_pose}")
        handles.robot.movj_pose(dump_hover_pose)
        handles.robot.wait_until_idle()

        info("STACK", f"[TUMBLE] branch=A step=movl_down_dump pose={dump_pose}")
        handles.robot.speed_factor(cfg.SPEED_PRECISION)
        handles.robot.movl_pose(dump_pose)
        handles.robot.wait_until_idle()

        info("STACK", "[TUMBLE] branch=A step=open_gripper")
        handles.gripper.open()
        time.sleep(0.2)

        info("STACK", f"[TUMBLE] branch=A step=movl_up_hover pose={dump_hover_pose}")
        handles.robot.speed_factor(cfg.SPEED_PRECISION)
        handles.robot.movl_pose(dump_hover_pose)
        handles.robot.wait_until_idle()

        info("STACK", f"[TUMBLE] branch=A step=move_neutral3 pose={cfg.NEUTRAL_3}")
        handles.robot.movj_pose(cfg.NEUTRAL_3)
    else:
        info("STACK", f"[TUMBLE] branch=B step=move_neutral3 pose={cfg.NEUTRAL_3}")
        handles.robot.movj_pose(cfg.NEUTRAL_3)

    return holding



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


def _movj_speed_percent(handles: SystemHandles) -> int:
    if getattr(cfg, "COMBO_ENABLED", True) and getattr(handles, "combo_active", False):
        return int(max(1, min(100, getattr(cfg, "MOVEJ_SPEED_COMBO", 70))))
    return int(max(1, min(100, getattr(cfg, "MOVEJ_SPEED_NORMAL", 35))))


def movj_pose_combo(handles: SystemHandles, pose: cfg_pose_type) -> str:
    desired_speed = _movj_speed_percent(handles)

    if not hasattr(handles, "_last_movj_speed"):
        handles._last_movj_speed = None

    if desired_speed != handles._last_movj_speed:
        warn("COMBO", f"MoveJ speed set to {desired_speed}% (combo_active={getattr(handles, 'combo_active', False)})")
        handles._last_movj_speed = desired_speed

    handles.robot.speed_factor(desired_speed)
    return handles.robot.movj_pose(pose)


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

    provider = _active_pick_pose_provider()
    pick_pose, reason = provider.get_pick_pose(side, level)
    if pick_pose is None:
        raise PickPoseUnavailableError(reason)

    hover_pose = (
        pick_pose[0],
        pick_pose[1],
        pick_pose[2] + cfg.PICK_CLEARANCE_MM,
        pick_pose[3],
        pick_pose[4],
        pick_pose[5],
    )

    info("STACK", f"pick target side={side} level={level} pick_pose={pick_pose}")
    debug("STACK", f"pick hover_pose={hover_pose}")

    # Joint transition into region
    movj_pose_combo(handles, hover_pose)
    # Wait for joint motion to complete before linear descent
    robot.wait_until_idle()

    # Linear vertical descent
    robot.speed_factor(cfg.SPEED_PRECISION)
    robot.movl_pose(pick_pose)
    # Ensure linear descent completed before actuating gripper
    robot.wait_until_idle()

    gripper.close()
    time.sleep(0.5)

    # Linear vertical retract
    robot.speed_factor(cfg.SPEED_PRECISION)
    robot.movl_pose(hover_pose)
    # Wait for retract to finish before joint exit
    robot.wait_until_idle()

    # Joint exit to neutral
    movj_pose_combo(handles, cfg.NEUTRAL_2)


def move_to_tower_hover(handles: SystemHandles, stack_level: int) -> None:
    """
    Move to safe hover position above tower using joint motion (MovJ).
    
    Args:
        stack_level: 0-indexed (0 = above base, 1 = above first block, etc.)
    """
    robot = handles.robot
    hover_pose = cfg.tower_hover_pose(stack_level)
    info("STACK", f"tower hover level={stack_level} hover_pose={hover_pose}")
    movj_pose_combo(handles, hover_pose)


def complete_place_neutral_exit(handles: SystemHandles, stack_level: int) -> None:
    """Final neutral MoveJ after placement, combo-aware."""
    if stack_level >= 3:
        movj_pose_combo(handles, cfg.NEUTRAL_3)
    else:
        movj_pose_combo(handles, cfg.NEUTRAL_2)


def complete_place_sequence(
    handles: SystemHandles,
    stack_level: int,
    place_pose: Optional[cfg_pose_type] = None,
    perform_neutral_exit: bool = True,
) -> None:
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

    if place_pose is None:
        base_place_pose = cfg.tower_place_pose(stack_level)
        place_pose = drift_engine.inject_drift(base_place_pose, stack_level)
        info("DRIFT", f"stack_level={stack_level} base={base_place_pose} drifted={place_pose}")
    else:
        debug("DRIFT", f"using proposed pose stack_level={stack_level} pose={place_pose}")
    retract_hover_pose = (
        place_pose[0],
        place_pose[1],
        place_pose[2] + cfg.PLACE_CLEARANCE_MM,
        place_pose[3],
        place_pose[4],
        place_pose[5],
    )

    # Linear vertical descent
    robot.speed_factor(cfg.SPEED_PRECISION)
    robot.movl_pose(place_pose)

    gripper.open()
    time.sleep(0.3)

    # Linear vertical retract
    robot.speed_factor(cfg.SPEED_PRECISION)
    robot.movl_pose(retract_hover_pose)
    # Ensure retract finished before joint exit
    robot.wait_until_idle()

    # Optional sidestep for very high stacks to reduce joint travel over tower
    if stack_level >= 5:
        sidestep_pose = (
            retract_hover_pose[0],
            -10.0,
            retract_hover_pose[2],
            retract_hover_pose[3],
            retract_hover_pose[4],
            retract_hover_pose[5],
        )
        robot.speed_factor(cfg.SPEED_PRECISION)
        robot.movl_pose(sidestep_pose)
        robot.wait_until_idle()

    if perform_neutral_exit:
        complete_place_neutral_exit(handles, stack_level)


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