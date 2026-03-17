from __future__ import annotations

import os
import sys
from typing import Any

import block_tracker
import robot_config as cfg
import tolerance_engine
from logger import info, warn, write_jsonl_event


PICK_POSE_MODE = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
VISION_MODE_ENABLED = PICK_POSE_MODE in {"vision", "perception"}
VISION_ASSIST_ENABLED = bool(getattr(cfg, "VISION_ASSIST_ENABLED", False))
PERCEPTION_ASSIST_ENABLED = VISION_MODE_ENABLED or VISION_ASSIST_ENABLED

PERC_AVAILABLE = False
_PERC_ENGINE = None

if not PERCEPTION_ASSIST_ENABLED:
    if bool(getattr(cfg, "DEBUG_ENABLED", False)):
        info("PERC", "PERC bypassed (deterministic mode)")
else:
    try:
        perc_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "perception")
        if perc_path not in sys.path:
            sys.path.append(perc_path)
        from perception_engine import engine as _PERC_ENGINE  # type: ignore[reportMissingImports]

        PERC_AVAILABLE = True
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            mode_label = "vision authority" if VISION_MODE_ENABLED else "shadow assist"
            info("PERC", f"Perception module enabled ({mode_label})")
    except Exception as exc:
        warn("PERC", f"Perception module disabled: {exc}")
        _PERC_ENGINE = None


def get_perception_engine():
    return _PERC_ENGINE


def camera_stream_perception_binding(label: str) -> tuple[bool, Any]:
    enable_raw = (label == "INSPECTOR" and PERCEPTION_ASSIST_ENABLED and PERC_AVAILABLE and _PERC_ENGINE is not None)
    return enable_raw, _PERC_ENGINE if enable_raw else None


def _shadow_logs_enabled() -> bool:
    return bool(getattr(cfg, "VISION_SHADOW_LOGS_ENABLED", False))


def log_shadow_pose(context: str, pose, authoritative_zone: str, debug_enabled: bool = False) -> None:
    if not _shadow_logs_enabled():
        return

    try:
        assessment = tolerance_engine.vision_shadow_assessment(pose)
    except Exception as exc:
        if debug_enabled:
            warn("CONTROL", f"[VISION_SHADOW] {context} failed: {exc}")
        return

    if not assessment.get("available", False):
        if debug_enabled:
            info("CONTROL", f"[VISION_SHADOW] {context} unavailable reason={assessment.get('reason', 'unknown')}")
        return

    center_x, center_y = assessment["center_xy"]
    err_x, err_y = assessment["axis_error_mm"]
    shadow_zone = str(assessment["zone"])
    if debug_enabled or shadow_zone != authoritative_zone:
        info(
            "CONTROL",
            f"[VISION_SHADOW] {context} center=({center_x:.2f},{center_y:.2f}) "
            f"err=({err_x:.2f},{err_y:.2f}) shadow_zone={shadow_zone} authoritative_zone={authoritative_zone}",
        )


def log_pick_tracking(target_id: str, stack_level: int, debug_enabled: bool = False) -> None:
    _log_block_tracking(
        context=f"PICK target={target_id} lvl={stack_level}",
        tracking=block_tracker.track_pick_target(target_id),
        debug_enabled=debug_enabled,
    )


def log_drop_tracking(stack_level: int, debug_enabled: bool = False) -> None:
    _log_block_tracking(
        context=f"DROP target={cfg.build_target_id_for_level(stack_level)} lvl={stack_level}",
        tracking=block_tracker.track_drop_level(stack_level),
        debug_enabled=debug_enabled,
    )


def _log_block_tracking(context: str, tracking: dict[str, Any], debug_enabled: bool = False) -> None:
    if not _shadow_logs_enabled():
        return

    event_payload: dict[str, Any] = {
        "event": "block_track",
        "module": "CONTROL",
        "context": context,
        "configured": bool(tracking.get("configured", False)),
        "available": bool(tracking.get("available", False)),
        "reason": str(tracking.get("reason", "unknown")),
        "role": tracking.get("role"),
        "target_id": tracking.get("target_id"),
        "marker_id": tracking.get("marker_id"),
    }
    if tracking.get("last_seen_age_s") is not None:
        event_payload["last_seen_age_s"] = round(float(tracking["last_seen_age_s"]), 3)
    if tracking.get("last_seen_utc") is not None:
        event_payload["last_seen_utc"] = str(tracking["last_seen_utc"])
    if tracking.get("freshness_window_s") is not None:
        event_payload["freshness_window_s"] = round(float(tracking["freshness_window_s"]), 3)

    if not tracking.get("configured", False):
        write_jsonl_event("block_track", event_payload)
        return

    if not tracking.get("available", False):
        write_jsonl_event("block_track", event_payload)
        return

    robot_x, robot_y = tracking["robot_xy"]
    expected_x, expected_y = tracking["expected_xy"]
    err_x, err_y = tracking["axis_error_mm"]
    radial_err = float(tracking["radial_error_mm"])
    event_payload.update(
        {
            "camera_pose": {
                "x": round(float(tracking["camera_pose"][0]), 6),
                "y": round(float(tracking["camera_pose"][1]), 6),
                "z": round(float(tracking["camera_pose"][2]), 6),
                "roll": round(float(tracking["camera_pose"][3]), 6),
                "pitch": round(float(tracking["camera_pose"][4]), 6),
                "yaw": round(float(tracking["camera_pose"][5]), 6),
            },
            "robot_xy_mm": {
                "x": round(float(robot_x), 3),
                "y": round(float(robot_y), 3),
            },
            "expected_xy_mm": {
                "x": round(float(expected_x), 3),
                "y": round(float(expected_y), 3),
            },
            "axis_error_mm": {
                "x": round(float(err_x), 3),
                "y": round(float(err_y), 3),
            },
            "radial_error_mm": round(radial_err, 3),
        }
    )
    write_jsonl_event("block_track", event_payload)
