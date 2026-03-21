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

import re
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import robot_config as cfg
import block_tracker
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE
import drift_engine
import vision_bridge
from logger import info, debug, warn, error, write_jsonl_event


cfg_pose_type = tuple[float, float, float, float, float, float]


class PickPoseProvider(ABC):
    @abstractmethod
    def get_pick_pose(self, target_id: str) -> tuple[Optional[cfg_pose_type], str]:
        """Return (pose, reason). pose=None means unavailable."""


class DeterministicPickPoseProvider(PickPoseProvider):
    def get_pick_pose(self, target_id: str) -> tuple[Optional[cfg_pose_type], str]:
        try:
            return cfg.pick_target_pose(target_id), "ok"
        except Exception as e:
            return None, str(e)


class VisionPickPoseProvider(PickPoseProvider):
    def get_pick_pose(self, target_id: str) -> tuple[Optional[cfg_pose_type], str]:
        tracking = block_tracker.track_pick_target(target_id)
        if not tracking.get("configured", False):
            return None, str(tracking.get("reason", "marker_unmapped"))
        if not tracking.get("available", False):
            return None, str(tracking.get("reason", "marker_unavailable"))
        observation_source = str(tracking.get("observation_source", "")).strip().lower()
        if observation_source != "live":
            return None, f"pick_observation_not_live:{observation_source or 'unknown'}"

        try:
            template_pose = cfg.pick_target_pose(target_id)
        except Exception as e:
            return None, str(e)

        robot_x, robot_y = tracking["robot_xy"]
        return (
            (
                float(robot_x),
                float(robot_y),
                float(template_pose[2]),
                float(template_pose[3]),
                float(template_pose[4]),
                float(template_pose[5]),
            ),
            "ok",
        )


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


def _emit_motion_event(event_name: str, **fields) -> None:
    payload = {
        "event": "motion_trigger",
        "module": "CONTROL",
        "motion": event_name,
    }
    payload.update(fields)
    write_jsonl_event("motion_trace", payload)


def _pose_dict(pose: Optional[cfg_pose_type]) -> Optional[dict[str, float]]:
    if pose is None:
        return None
    return {
        "x": round(float(pose[0]), 3),
        "y": round(float(pose[1]), 3),
        "z": round(float(pose[2]), 3),
        "rx": round(float(pose[3]), 3),
        "ry": round(float(pose[4]), 3),
        "rz": round(float(pose[5]), 3),
    }


def _log_motion_step(
    handles: SystemHandles,
    motion_name: str,
    step_name: str,
    phase: str,
    commanded_pose: Optional[cfg_pose_type] = None,
    commanded_delta_mm_deg: Optional[tuple[float, float, float, float, float, float]] = None,
    **fields,
) -> None:
    pose = handles.robot.get_tcp_pose()
    payload = {
        "event": "motion_step",
        "module": "CONTROL",
        "motion": motion_name,
        "step": step_name,
        "phase": phase,
    }
    payload.update(fields)
    if commanded_pose is not None:
        payload["commanded_pose_mm_deg"] = _pose_dict(commanded_pose)
    if commanded_delta_mm_deg is not None:
        payload["commanded_delta_mm_deg"] = _pose_dict(commanded_delta_mm_deg)
    if pose is None:
        payload["tcp_pose_available"] = False
    else:
        payload["tcp_pose_available"] = True
        payload["tcp_pose_mm_deg"] = _pose_dict(pose)
    write_jsonl_event("motion_trace", payload)


def _log_motion_trigger(handles: SystemHandles, event_name: str, **fields) -> None:
    pose = handles.robot.get_tcp_pose()
    payload = dict(fields)
    if pose is None:
        payload["tcp_pose_available"] = False
    else:
        payload["tcp_pose_available"] = True
        payload["tcp_pose_mm_deg"] = _pose_dict(pose)
    _emit_motion_event(event_name, **payload)


def _resolve_known_pick_target_id(target_label: str) -> Optional[str]:
    text = str(target_label).strip().upper()
    match = re.search(r"(P[1-7])", text)
    if not match:
        return None
    target_id = str(match.group(1))
    if target_id not in getattr(cfg, "PICKUP_POINTS", {}):
        return None
    return target_id


def _emit_pick_runtime_residual_event(
    handles: SystemHandles,
    *,
    motion_name: str,
    target_label: str,
    stack_level: int,
    commanded_pick_pose: cfg_pose_type,
) -> None:
    tcp_pose = handles.robot.get_tcp_pose()
    if tcp_pose is None:
        return

    payload: dict[str, object] = {
        "event": "pickup_runtime_residual",
        "module": "CONTROL",
        "motion": motion_name,
        "target_label": str(target_label),
        "stack_level": int(stack_level),
        "commanded_pick_pose_mm_deg": _pose_dict(commanded_pick_pose),
        "tcp_pick_pose_mm_deg": _pose_dict(tcp_pose),
        "residual_to_commanded_mm": {
            "x": round(float(tcp_pose[0]) - float(commanded_pick_pose[0]), 3),
            "y": round(float(tcp_pose[1]) - float(commanded_pick_pose[1]), 3),
            "z": round(float(tcp_pose[2]) - float(commanded_pick_pose[2]), 3),
        },
    }
    payload.update(vision_bridge.current_pick_runtime_metadata())

    known_target_id = _resolve_known_pick_target_id(target_label)
    if known_target_id is not None:
        expected_pose = cfg.pick_target_pose(known_target_id)
        payload["expected_pick_target_id"] = known_target_id
        payload["expected_pick_pose_mm_deg"] = _pose_dict(expected_pose)
        payload["residual_to_expected_mm"] = {
            "x": round(float(tcp_pose[0]) - float(expected_pose[0]), 3),
            "y": round(float(tcp_pose[1]) - float(expected_pose[1]), 3),
            "z": round(float(tcp_pose[2]) - float(expected_pose[2]), 3),
        }

    write_jsonl_event("pickup_runtime_residual", payload)


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
    _log_motion_step(
        handles,
        "initialize_stack_session",
        "safe_home",
        "start",
        commanded_pose=cfg.SAFE_HOME_POSE,
    )
    handles.robot.movj_pose(cfg.SAFE_HOME_POSE)
    _log_motion_step(
        handles,
        "initialize_stack_session",
        "safe_home",
        "complete",
        commanded_pose=cfg.SAFE_HOME_POSE,
    )
    
    # Open gripper
    _log_motion_step(handles, "initialize_stack_session", "gripper_open", "start")
    handles.gripper.open()
    _log_motion_step(handles, "initialize_stack_session", "gripper_open", "complete")


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

    if bool(getattr(cfg, "DEBUG_ENABLED", False)):
        info("STACK", "[RECOVERY] recover_from_fault: start")

    # Query initial mode (for logging)
    mode_before = _mode()
    if bool(getattr(cfg, "DEBUG_ENABLED", False)):
        info("STACK", f"[RECOVERY] mode_before={mode_before}")

    # If robot is not in fault, run lightweight re-center and return OK
    if mode_before is not None and mode_before not in (9, 11):
        try:
            handles.robot.speed_factor(cfg.SPEED_PRECISION)
            if bool(getattr(cfg, "DEBUG_ENABLED", False)):
                info("STACK", f"[RECOVERY] speed_factor={cfg.SPEED_PRECISION}")
        except Exception as e:
            error("STACK", f"[RECOVERY] speed_factor failed: {e}")
        try:
            handles.robot.movj_pose(cfg.SAFE_HOME_POSE)
            handles.robot.wait_until_idle()
            if bool(getattr(cfg, "DEBUG_ENABLED", False)):
                info("STACK", "[RECOVERY] moved to SAFE_HOME_POSE")
        except Exception as e:
            error("STACK", f"[RECOVERY] movj failed: {e}")
            return False
        try:
            handles.gripper.open()
            if bool(getattr(cfg, "DEBUG_ENABLED", False)):
                info("STACK", "[RECOVERY] gripper opened")
        except Exception as e:
            error("STACK", f"[RECOVERY] gripper open failed: {e}")
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("STACK", "[RECOVERY] recover_from_fault: complete (idempotent)")
        return True

    # Otherwise, proceed with full recovery
    try:
        resp = handles.robot.clear_error()
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("STACK", f"[RECOVERY] clear_error -> {resp}")
    except Exception as e:
        error("STACK", f"[RECOVERY] clear_error failed: {e}")

    try:
        resp = handles.robot.enable()
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("STACK", f"[RECOVERY] enable -> {resp}")
    except Exception as e:
        error("STACK", f"[RECOVERY] enable failed: {e}")

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
            error("STACK", f"[RECOVERY] mode_before={mode_before}")
            error("STACK", f"[RECOVERY] mode_after={m}")
            error("STACK", "[RECOVERY] HARD FAULT detected during poll. Aborting.")
            return False
        if m >= 0:
            mode_after = m
            break
        time.sleep(0.25)

    if mode_after is None:
        error("STACK", f"[RECOVERY] mode_before={mode_before}")
        error("STACK", f"[RECOVERY] mode_after={mode_after}")
        error("STACK", "[RECOVERY] No valid RobotMode within timeout. Aborting.")
        return False

    if bool(getattr(cfg, "DEBUG_ENABLED", False)):
        info("STACK", f"[RECOVERY] mode_before={mode_before}")
        info("STACK", f"[RECOVERY] mode_after={mode_after}")

    # Proceed with recovery motions
    try:
        handles.robot.speed_factor(cfg.SPEED_PRECISION)
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("STACK", f"[RECOVERY] speed_factor={cfg.SPEED_PRECISION}")
    except Exception as e:
        error("STACK", f"[RECOVERY] speed_factor failed: {e}")

    try:
        handles.robot.movj_pose(cfg.SAFE_HOME_POSE)
        handles.robot.wait_until_idle()
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("STACK", "[RECOVERY] moved to SAFE_HOME_POSE")
    except Exception as e:
        error("STACK", f"[RECOVERY] movj failed: {e}")
        return False

    try:
        handles.gripper.open()
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("STACK", "[RECOVERY] gripper opened")
    except Exception as e:
        error("STACK", f"[RECOVERY] gripper open failed: {e}")

    if bool(getattr(cfg, "DEBUG_ENABLED", False)):
        info("STACK", "[RECOVERY] recover_from_fault: complete")
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
def execute_tumble_sequence(handles: SystemHandles, fallback_holding: Optional[bool] = None, run_terminating: bool = False) -> bool:
    """
    Deterministic tumble flow.
    Returns detected holding state before executing motions.

    A) Holding -> SAFE_DUMP_POSE -> open -> NEUTRAL_3
    B) Not holding -> NEUTRAL_3
    """
    info("STACK", f"[TUMBLE] >>> ENTER fallback_holding={fallback_holding} run_terminating={run_terminating}")
    holding = _is_gripper_holding_block(handles, fallback_holding)
    info("STACK", f"[TUMBLE] holding_detected={holding} fallback_holding={fallback_holding} run_terminating={run_terminating}")

    if holding:
        dump_pose = cfg.tumble_dump_pose()
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
        info("STACK", "[TUMBLE] branch=A wait_until_idle=done after move_dump_hover")

        info("STACK", f"[TUMBLE] branch=A step=movl_down_dump pose={dump_pose}")
        handles.robot.speed_factor(cfg.SPEED_PRECISION)
        handles.robot.movl_pose(dump_pose)
        handles.robot.wait_until_idle()
        info("STACK", "[TUMBLE] branch=A wait_until_idle=done after movl_down_dump")

        info("STACK", "[TUMBLE] branch=A step=open_gripper (calling)")
        try:
            handles.gripper.open()
            info("STACK", "[TUMBLE] branch=A step=open_gripper success")
        except Exception as _e:
            warn("STACK", f"[TUMBLE] branch=A step=open_gripper FAILED: {_e}")
        time.sleep(0.2)

        info("STACK", f"[TUMBLE] branch=A step=movl_up_hover pose={dump_hover_pose}")
        handles.robot.speed_factor(cfg.SPEED_PRECISION)
        handles.robot.movl_pose(dump_hover_pose)
        handles.robot.wait_until_idle()
        info("STACK", "[TUMBLE] branch=A wait_until_idle=done after movl_up_hover")

        neutral3 = cfg.neutral_pose_for_slot(3)
        info("STACK", f"[TUMBLE] branch=A step=move_neutral3 pose={neutral3}")
        handles.robot.movj_pose(neutral3)
        info("STACK", "[TUMBLE] branch=A movj for neutral3 issued (no wait)")
    else:
        neutral3 = cfg.neutral_pose_for_slot(3)
        try:
            handles.gripper.open()
            info("STACK", "[TUMBLE] branch=B step=open_gripper done")
        except Exception as e:
            warn("STACK", f"[TUMBLE] branch=B gripper open failed: {e}")
        info("STACK", f"[TUMBLE] branch=B step=move_neutral3 pose={neutral3}")
        handles.robot.movj_pose(neutral3)
        info("STACK", "[TUMBLE] branch=B movj for neutral3 issued (no wait)")

    info("STACK", f"[TUMBLE] <<< EXIT returning holding={holding} run_terminating={run_terminating}")
    return holding


def ensure_gripper_open_at_run_start(handles: SystemHandles) -> None:
    """
    Lightweight run-start safeguard: open gripper if detected as closed.
    Called once per new participant run. Does nothing if already open or status unreadable.
    """
    try:
        st = handles.gripper.status()
        grip_state = st.get("grip_state")
        pos = st.get("pos")
        midpoint = (cfg.GRIPPER_OPEN_POS + cfg.GRIPPER_CLOSE_POS) / 2.0
        is_closed = (grip_state == 2) or (isinstance(pos, (int, float)) and pos <= midpoint)
        if is_closed:
            info("STACK", "[RUN_START] Gripper detected closed at run start — opening")
            handles.gripper.open()
        elif bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("STACK", "[RUN_START] Gripper already open at run start")
    except Exception as e:
        warn("STACK", f"[RUN_START] Gripper run-start check failed (ignored): {e}")


def do_home(handles: SystemHandles) -> None:
    """Go to safe home pose."""
    _log_motion_trigger(handles, "do_home", target_pose=cfg.SAFE_HOME_POSE)
    _log_motion_step(handles, "do_home", "safe_home", "start", commanded_pose=cfg.SAFE_HOME_POSE)
    handles.robot.speed_factor(cfg.SPEED_PRECISION)
    handles.robot.go_home(speed_percent=cfg.SPEED_PRECISION)
    _log_motion_step(handles, "do_home", "safe_home", "complete", commanded_pose=cfg.SAFE_HOME_POSE)


def do_drop(handles: SystemHandles, dz_mm: float = -20.0) -> None:
    """
    Minimal 'drop' primitive used currently.
    This is NOT the final placing routine; just a controlled linear descent.
    """
    _log_motion_trigger(handles, "do_drop", dz_mm=float(dz_mm))
    handles.robot.speed_factor(cfg.SPEED_PRECISION)
    handles.robot.relmovl_user(0, 0, dz_mm, 0, 0, 0)


def do_nudge_xy(handles: SystemHandles, dx_mm: float, dy_mm: float) -> None:
    """XY nudge in base/user frame."""
    _log_motion_trigger(handles, "do_nudge_xy", dx_mm=float(dx_mm), dy_mm=float(dy_mm))
    handles.robot.speed_factor(cfg.SPEED_PRECISION)
    handles.robot.relmovl_user(dx_mm, dy_mm, 0, 0, 0, 0)


def do_nudge_yaw(handles: SystemHandles, dtheta_deg: float) -> None:
    """
    Optional yaw nudge.
    Only call this if your Dobot supports RelMovLUser with rotation in your setup.
    """
    _log_motion_trigger(handles, "do_nudge_yaw", dtheta_deg=float(dtheta_deg))
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
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("COMBO", f"MoveJ speed set to {desired_speed}% (combo_active={getattr(handles, 'combo_active', False)})")
        handles._last_movj_speed = desired_speed

    handles.robot.speed_factor(desired_speed)
    return handles.robot.movj_pose(pose)


# ----------------------------
# Stacking: Pick & Place (Hybrid MovJ/MovL)
# ----------------------------

def _execute_pick_with_pose(
    handles: SystemHandles,
    pick_pose: cfg_pose_type,
    stack_level: int,
    target_label: str,
    motion_name: str,
) -> None:
    robot = handles.robot
    gripper = handles.gripper

    hover_pose = (
        pick_pose[0],
        pick_pose[1],
        pick_pose[2] + cfg.PICK_CLEARANCE_MM,
        pick_pose[3],
        pick_pose[4],
        pick_pose[5],
    )

    pre_slot, post_slot = cfg.pick_gateway_neutrals(stack_level)
    pre_neutral = cfg.neutral_pose_for_slot(pre_slot)
    post_neutral = cfg.neutral_pose_for_slot(post_slot)

    _log_motion_trigger(
        handles,
        motion_name,
        target_id=target_label,
        stack_level=int(stack_level),
        pick_pose=pick_pose,
        hover_pose=hover_pose,
        pre_neutral_slot=int(pre_slot),
        post_neutral_slot=int(post_slot),
    )
    info("STACK", f"pick target={target_label} lvl={stack_level} pre_neutral={pre_slot} post_neutral={post_slot} pick_pose={pick_pose}")
    debug("STACK", f"pick hover_pose={hover_pose}")

    # Route through neutral gateway before entering pickup zone
    _log_motion_step(
        handles,
        motion_name,
        "pre_neutral",
        "start",
        commanded_pose=pre_neutral,
        target_id=target_label,
        stack_level=int(stack_level),
    )
    movj_pose_combo(handles, pre_neutral)
    robot.wait_until_idle()
    _log_motion_step(
        handles,
        motion_name,
        "pre_neutral",
        "complete",
        commanded_pose=pre_neutral,
        target_id=target_label,
        stack_level=int(stack_level),
    )

    # Joint transition into region
    _log_motion_step(
        handles,
        motion_name,
        "pick_hover_entry",
        "start",
        commanded_pose=hover_pose,
        target_id=target_label,
        stack_level=int(stack_level),
    )
    movj_pose_combo(handles, hover_pose)
    # Wait for joint motion to complete before linear descent
    robot.wait_until_idle()
    _log_motion_step(
        handles,
        motion_name,
        "pick_hover_entry",
        "complete",
        commanded_pose=hover_pose,
        target_id=target_label,
        stack_level=int(stack_level),
    )

    # Linear vertical descent
    _log_motion_step(
        handles,
        motion_name,
        "pick_descent",
        "start",
        commanded_pose=pick_pose,
        target_id=target_label,
        stack_level=int(stack_level),
    )
    robot.speed_factor(cfg.SPEED_PRECISION)
    robot.movl_pose(pick_pose)
    # Ensure linear descent completed before actuating gripper
    robot.wait_until_idle()
    _log_motion_step(
        handles,
        motion_name,
        "pick_descent",
        "complete",
        commanded_pose=pick_pose,
        target_id=target_label,
        stack_level=int(stack_level),
    )
    _emit_pick_runtime_residual_event(
        handles,
        motion_name=motion_name,
        target_label=target_label,
        stack_level=int(stack_level),
        commanded_pick_pose=pick_pose,
    )

    _log_motion_step(handles, motion_name, "gripper_close", "start", target_id=target_label, stack_level=int(stack_level))
    gripper.close()
    time.sleep(0.5)
    _log_motion_step(handles, motion_name, "gripper_close", "complete", target_id=target_label, stack_level=int(stack_level))

    # Linear vertical retract
    _log_motion_step(
        handles,
        motion_name,
        "pick_retract",
        "start",
        commanded_pose=hover_pose,
        target_id=target_label,
        stack_level=int(stack_level),
    )
    robot.speed_factor(cfg.SPEED_PRECISION)
    robot.movl_pose(hover_pose)
    # Wait for retract to finish before joint exit
    robot.wait_until_idle()
    _log_motion_step(
        handles,
        motion_name,
        "pick_retract",
        "complete",
        commanded_pose=hover_pose,
        target_id=target_label,
        stack_level=int(stack_level),
    )

    # Joint exit to neutral gateway for this block
    _log_motion_step(
        handles,
        motion_name,
        "post_neutral",
        "start",
        commanded_pose=post_neutral,
        target_id=target_label,
        stack_level=int(stack_level),
    )
    movj_pose_combo(handles, post_neutral)
    _log_motion_step(
        handles,
        motion_name,
        "post_neutral",
        "complete",
        commanded_pose=post_neutral,
        target_id=target_label,
        stack_level=int(stack_level),
    )


def execute_pick_pose(handles: SystemHandles, pick_pose: cfg_pose_type, stack_level: int, target_label: str = "explicit_pick") -> None:
    """
    Execute the standard safe pick path using an explicit Cartesian pick pose.

    This is intended for guarded lab tooling where perception computes the XY
    but we still want to reuse the same neutral/hover/descent/retract motion
    path as the deterministic runtime.
    """
    _execute_pick_with_pose(
        handles=handles,
        pick_pose=pick_pose,
        stack_level=stack_level,
        target_label=target_label,
        motion_name="execute_pick_pose",
    )


def execute_pick_sequence(handles: SystemHandles, pick_target_id: str, stack_level: int) -> None:
    """
    Deterministic pick sequence with hybrid strategy.
    - MovJ to hover zone (joint motion, faster)
    - MovL vertical descent to block (linear, controlled)
    - Close gripper
    - MovL vertical retract
    - MovJ to neutral exit pose

    Args:
        pick_target_id: pickup target id (legacy: L4/R3..., plate: P1..P7)
        stack_level: 0-indexed build level used for neutral gateway selection
    """
    provider = _active_pick_pose_provider()
    pick_pose, reason = provider.get_pick_pose(pick_target_id)
    if pick_pose is None:
        raise PickPoseUnavailableError(reason)

    _execute_pick_with_pose(
        handles=handles,
        pick_pose=pick_pose,
        stack_level=stack_level,
        target_label=pick_target_id,
        motion_name="execute_pick_sequence",
    )


def move_to_tower_hover(
    handles: SystemHandles,
    stack_level: int,
    target_xy: Optional[tuple[float, float]] = None,
    target_id: Optional[str] = None,
) -> None:
    """
    Move to safe hover position above tower using joint motion (MovJ).
    
    Args:
        stack_level: 0-indexed (0 = above base, 1 = above first block, etc.)
    """
    robot = handles.robot
    hover_pose = cfg.tower_hover_pose(stack_level)
    if target_xy is not None:
        hover_pose = (
            target_xy[0],
            target_xy[1],
            hover_pose[2],
            hover_pose[3],
            hover_pose[4],
            hover_pose[5],
        )
    _log_motion_trigger(
        handles,
        "move_to_tower_hover",
        stack_level=int(stack_level),
        target_id=target_id,
        target_xy=target_xy,
        hover_pose=hover_pose,
    )
    info("STACK", f"tower hover level={stack_level} hover_pose={hover_pose}")
    _log_motion_step(
        handles,
        "move_to_tower_hover",
        "tower_hover_entry",
        "start",
        commanded_pose=hover_pose,
        stack_level=int(stack_level),
        target_id=target_id,
    )
    movj_pose_combo(handles, hover_pose)
    _log_motion_step(
        handles,
        "move_to_tower_hover",
        "tower_hover_entry",
        "complete",
        commanded_pose=hover_pose,
        stack_level=int(stack_level),
        target_id=target_id,
    )


def move_to_hover_xy(handles: SystemHandles, target_x: float, target_y: float, stack_level: int) -> None:
    """
    Move to the requested XY while holding tower hover Z/orientation for this stack level.
    """
    robot = handles.robot
    nominal_hover = cfg.tower_hover_pose(stack_level)
    hover_xy_pose = (
        target_x,
        target_y,
        nominal_hover[2],
        nominal_hover[3],
        nominal_hover[4],
        nominal_hover[5],
    )
    _log_motion_trigger(
        handles,
        "move_to_hover_xy",
        stack_level=int(stack_level),
        target_x=float(target_x),
        target_y=float(target_y),
        hover_xy_pose=hover_xy_pose,
    )
    info("STACK", f"tower hover xy align level={stack_level} hover_xy_pose={hover_xy_pose}")
    robot.speed_factor(cfg.SPEED_PRECISION)
    robot.movl_pose(hover_xy_pose)


def complete_place_neutral_exit(handles: SystemHandles, stack_level: int, target_id: Optional[str] = None) -> None:
    """Final neutral MoveJ after placement, combo-aware."""
    slot = cfg.place_exit_neutral_slot(stack_level)
    target_neutral = cfg.neutral_pose_for_slot(slot)
    _log_motion_trigger(
        handles,
        "complete_place_neutral_exit",
        stack_level=int(stack_level),
        target_id=target_id,
        neutral_slot=int(slot),
        target_neutral=target_neutral,
    )
    _log_motion_step(
        handles,
        "complete_place_neutral_exit",
        "neutral_exit",
        "start",
        commanded_pose=target_neutral,
        stack_level=int(stack_level),
        target_id=target_id,
        neutral_slot=int(slot),
    )
    movj_pose_combo(handles, target_neutral)
    _log_motion_step(
        handles,
        "complete_place_neutral_exit",
        "neutral_exit",
        "complete",
        commanded_pose=target_neutral,
        stack_level=int(stack_level),
        target_id=target_id,
        neutral_slot=int(slot),
    )


def complete_place_sequence(
    handles: SystemHandles,
    stack_level: int,
    place_pose: Optional[cfg_pose_type] = None,
    perform_neutral_exit: bool = True,
    target_id: Optional[str] = None,
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
    _log_motion_trigger(
        handles,
        "complete_place_sequence",
        stack_level=int(stack_level),
        target_id=target_id,
        place_pose=place_pose,
        retract_hover_pose=retract_hover_pose,
        perform_neutral_exit=bool(perform_neutral_exit),
    )

    # Pure vertical-only drop using relative Z from configured hover/place levels
    hover_z = cfg.tower_hover_pose(stack_level)[2]
    place_z = cfg.tower_place_pose(stack_level)[2]
    drop_mm = max(0.0, hover_z - place_z)
    _log_motion_step(
        handles,
        "complete_place_sequence",
        "place_drop",
        "start",
        commanded_pose=place_pose,
        commanded_delta_mm_deg=(0.0, 0.0, -drop_mm, 0.0, 0.0, 0.0),
        stack_level=int(stack_level),
        target_id=target_id,
    )
    robot.speed_factor(cfg.SPEED_PRECISION)
    info("CONTROL", f"[DROP] lvl={stack_level} drop_mm={drop_mm:.2f} using=relmovl_user dz=-{drop_mm:.2f}")
    robot.relmovl_user(0, 0, -drop_mm, 0, 0, 0)

    _log_motion_step(handles, "complete_place_sequence", "gripper_open", "start", stack_level=int(stack_level), target_id=target_id)
    gripper.open()
    time.sleep(0.3)
    _log_motion_step(handles, "complete_place_sequence", "gripper_open", "complete", stack_level=int(stack_level), target_id=target_id)

    # Linear vertical retract
    _log_motion_step(
        handles,
        "complete_place_sequence",
        "place_retract",
        "start",
        commanded_pose=retract_hover_pose,
        stack_level=int(stack_level),
        target_id=target_id,
    )
    robot.speed_factor(cfg.SPEED_PRECISION)
    info("CONTROL", f"[RETRACT] lvl={stack_level} retract_pose=({retract_hover_pose[0]:.2f},{retract_hover_pose[1]:.2f},{retract_hover_pose[2]:.2f})")
    robot.movl_pose(retract_hover_pose)
    # Ensure retract finished before joint exit
    robot.wait_until_idle()
    _log_motion_step(
        handles,
        "complete_place_sequence",
        "place_retract",
        "complete",
        commanded_pose=retract_hover_pose,
        stack_level=int(stack_level),
        target_id=target_id,
    )

    # Explicit mode: after T6/T7 retract hover, perform Y+ shuffle before neutral exit.
    # Legacy mode keeps existing high-stack sidestep behavior.
    if cfg.requires_post_place_y_shuffle(stack_level):
        shuffle_pose = (
            retract_hover_pose[0],
            retract_hover_pose[1] + cfg.post_place_y_shuffle_mm(stack_level),
            retract_hover_pose[2],
            retract_hover_pose[3],
            retract_hover_pose[4],
            retract_hover_pose[5],
        )
        _log_motion_step(
            handles,
            "complete_place_sequence",
            "post_place_y_shuffle",
            "start",
            commanded_pose=shuffle_pose,
            stack_level=int(stack_level),
            target_id=target_id,
            shuffle_mm=float(cfg.post_place_y_shuffle_mm(stack_level)),
        )
        robot.speed_factor(cfg.SPEED_PRECISION)
        info("CONTROL", f"[ESCAPE] lvl={stack_level} explicit_y_shuffle_mm={cfg.post_place_y_shuffle_mm(stack_level):.2f} pose=({shuffle_pose[0]:.2f},{shuffle_pose[1]:.2f},{shuffle_pose[2]:.2f})")
        robot.movl_pose(shuffle_pose)
        robot.wait_until_idle()
        _log_motion_step(
            handles,
            "complete_place_sequence",
            "post_place_y_shuffle",
            "complete",
            commanded_pose=shuffle_pose,
            stack_level=int(stack_level),
            target_id=target_id,
            shuffle_mm=float(cfg.post_place_y_shuffle_mm(stack_level)),
        )
    elif stack_level >= 5:
        sidestep_pose = (
            retract_hover_pose[0],
            -10.0,
            retract_hover_pose[2],
            retract_hover_pose[3],
            retract_hover_pose[4],
            retract_hover_pose[5],
        )
        _log_motion_step(
            handles,
            "complete_place_sequence",
            "legacy_sidestep",
            "start",
            commanded_pose=sidestep_pose,
            stack_level=int(stack_level),
            target_id=target_id,
        )
        robot.speed_factor(cfg.SPEED_PRECISION)
        robot.movl_pose(sidestep_pose)
        robot.wait_until_idle()
        _log_motion_step(
            handles,
            "complete_place_sequence",
            "legacy_sidestep",
            "complete",
            commanded_pose=sidestep_pose,
            stack_level=int(stack_level),
            target_id=target_id,
        )

    if perform_neutral_exit:
        complete_place_neutral_exit(handles, stack_level, target_id=target_id)


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
