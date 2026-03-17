from __future__ import annotations

from math import sqrt
from typing import Any, Optional

import robot_config as cfg
import tolerance_engine
import vision_bridge

TRACK_FRESHNESS_S = float(getattr(cfg, "VISION_TRACK_FRESHNESS_S", 1.5))


def _tracking_state() -> dict[int, tuple[float, float, float, float, float, float]]:
    engine = getattr(tolerance_engine, "perc_engine", None)
    if engine is None:
        return {}
    try:
        return engine.get_latest_state()
    except Exception:
        return {}


def _last_seen_marker(marker_id: int) -> Optional[dict[str, Any]]:
    engine = getattr(tolerance_engine, "perc_engine", None)
    if engine is None:
        return None
    try:
        getter = getattr(engine, "get_last_seen_marker", None)
        if getter is None:
            return None
        return getter(int(marker_id))
    except Exception:
        return None


def _track_target(target_id: str, marker_map: dict[str, int], expected_pose: tuple[float, float, float, float, float, float], role: str) -> dict[str, Any]:
    marker_id = marker_map.get(target_id)
    if marker_id is None:
        return {
            "configured": False,
            "available": False,
            "reason": "marker_unmapped",
            "role": role,
            "target_id": target_id,
        }

    tracking_state = _tracking_state()
    marker_pose = tracking_state.get(int(marker_id))
    age_s = 0.0
    last_seen_utc = None
    if marker_pose is None:
        last_seen = _last_seen_marker(int(marker_id))
        if last_seen is None:
            return {
                "configured": True,
                "available": False,
                "reason": "marker_never_seen",
                "role": role,
                "target_id": target_id,
                "marker_id": int(marker_id),
            }

        age_s = float(last_seen["age_s"])
        if age_s > TRACK_FRESHNESS_S:
            return {
                "configured": True,
                "available": False,
                "reason": "marker_stale",
                "role": role,
                "target_id": target_id,
                "marker_id": int(marker_id),
                "last_seen_age_s": age_s,
                "last_seen_utc": last_seen["last_seen_utc"],
                "freshness_window_s": TRACK_FRESHNESS_S,
            }

        marker_pose = tuple(last_seen["pose"])
        age_s = float(last_seen["age_s"])
        last_seen_utc = str(last_seen["last_seen_utc"])
    else:
        last_seen = _last_seen_marker(int(marker_id))
        age_s = float(last_seen["age_s"]) if last_seen is not None else 0.0
        last_seen_utc = str(last_seen["last_seen_utc"]) if last_seen is not None else None

    if marker_pose is None:
        return {
            "configured": True,
            "available": False,
            "reason": "marker_not_visible",
            "role": role,
            "target_id": target_id,
            "marker_id": int(marker_id),
        }

    robot_xy, reason = vision_bridge.camera_xy_to_robot_xy_mm(marker_pose[0], marker_pose[1])
    if robot_xy is None:
        return {
            "configured": True,
            "available": False,
            "reason": reason,
            "role": role,
            "target_id": target_id,
            "marker_id": int(marker_id),
        }

    err_x = robot_xy[0] - expected_pose[0]
    err_y = robot_xy[1] - expected_pose[1]
    return {
        "configured": True,
        "available": True,
        "reason": "ok",
        "role": role,
        "target_id": target_id,
        "marker_id": int(marker_id),
        "camera_pose": marker_pose,
        "last_seen_age_s": age_s,
        "last_seen_utc": last_seen_utc,
        "freshness_window_s": TRACK_FRESHNESS_S,
        "robot_xy": robot_xy,
        "expected_xy": (expected_pose[0], expected_pose[1]),
        "axis_error_mm": (err_x, err_y),
        "radial_error_mm": sqrt((err_x ** 2) + (err_y ** 2)),
    }


def track_pick_target(target_id: str) -> dict[str, Any]:
    marker_map = dict(getattr(cfg, "VISION_PICK_MARKER_MAP", {}))
    expected_pose = cfg.pick_target_pose(target_id)
    return _track_target(target_id=target_id, marker_map=marker_map, expected_pose=expected_pose, role="pick")


def track_drop_target(target_id: str) -> dict[str, Any]:
    marker_map = dict(getattr(cfg, "VISION_DROP_MARKER_MAP", {}))
    expected_pose = cfg.build_target_pose(target_id)
    return _track_target(target_id=target_id, marker_map=marker_map, expected_pose=expected_pose, role="drop")


def track_drop_level(level: int) -> dict[str, Any]:
    target_id = cfg.build_target_id_for_level(level)
    return track_drop_target(target_id)
