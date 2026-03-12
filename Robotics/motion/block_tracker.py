from __future__ import annotations

from math import sqrt
from typing import Any, Optional

import robot_config as cfg
import tolerance_engine
import vision_bridge


def _tracking_state() -> dict[int, tuple[float, float, float, float, float, float]]:
    engine = getattr(tolerance_engine, "perc_engine", None)
    if engine is None:
        return {}
    try:
        return engine.get_latest_state()
    except Exception:
        return {}


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
