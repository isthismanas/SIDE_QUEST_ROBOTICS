from __future__ import annotations

from math import sqrt
from typing import Any, Optional

import robot_config as cfg
import tolerance_engine
import vision_bridge

TRACK_FRESHNESS_S = float(getattr(cfg, "VISION_TRACK_FRESHNESS_S", 1.5))
PICK_OBJECT_PERMANENCE_ENABLED = bool(getattr(cfg, "VISION_PICK_OBJECT_PERMANENCE_ENABLED", True))
PICK_OBJECT_PERMANENCE_MAX_AGE_S = float(getattr(cfg, "VISION_PICK_OBJECT_PERMANENCE_MAX_AGE_S", 300.0))


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


def _invalidate_marker(marker_id: int) -> None:
    engine = getattr(tolerance_engine, "perc_engine", None)
    if engine is None:
        return
    try:
        invalidator = getattr(engine, "invalidate_marker", None)
        if invalidator is not None:
            invalidator(int(marker_id))
    except Exception:
        return


def _reset_marker_memory(marker_ids: list[int] | None = None) -> None:
    engine = getattr(tolerance_engine, "perc_engine", None)
    if engine is None:
        return
    try:
        resetter = getattr(engine, "reset_marker_memory", None)
        if resetter is not None:
            resetter(marker_ids)
    except Exception:
        return


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
    observation_source = "live"
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
        invalidated = bool(last_seen.get("invalidated", False))
        if invalidated:
            return {
                "configured": True,
                "available": False,
                "reason": "marker_invalidated_after_pick",
                "role": role,
                "target_id": target_id,
                "marker_id": int(marker_id),
                "last_seen_age_s": age_s,
                "last_seen_utc": last_seen["last_seen_utc"],
            }

        permanence_allowed = (
            role == "pick"
            and PICK_OBJECT_PERMANENCE_ENABLED
            and age_s <= PICK_OBJECT_PERMANENCE_MAX_AGE_S
        )
        if not permanence_allowed and age_s > TRACK_FRESHNESS_S:
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
        observation_source = "cached_memory" if permanence_allowed else "recent_last_seen"
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

    if role == "pick":
        robot_xy, reason = vision_bridge.camera_pose_to_pick_robot_xy_mm(
            marker_pose[0],
            marker_pose[1],
            camera_pose=marker_pose,
        )
    else:
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
        "observation_source": observation_source,
        "robot_xy": robot_xy,
        "expected_xy": (expected_pose[0], expected_pose[1]),
        "axis_error_mm": (err_x, err_y),
        "radial_error_mm": sqrt((err_x ** 2) + (err_y ** 2)),
    }


def track_pick_target(target_id: str) -> dict[str, Any]:
    marker_map = dict(getattr(cfg, "VISION_PICK_MARKER_MAP", {}))
    expected_pose = cfg.pick_target_pose(target_id)
    return _track_target(target_id=target_id, marker_map=marker_map, expected_pose=expected_pose, role="pick")


def select_pick_target(target_ids: list[str], allow_cached: bool = False) -> Optional[dict[str, Any]]:
    source_priority = {
        "live": 0,
        "recent_last_seen": 1,
        "cached_memory": 2,
    }

    candidates: list[dict[str, Any]] = []
    for target_id in target_ids:
        tracking = track_pick_target(str(target_id))
        if not tracking.get("configured", False):
            continue
        if not tracking.get("available", False):
            continue

        observation_source = str(tracking.get("observation_source", "")).strip().lower()
        if not allow_cached and observation_source != "live":
            continue
        if allow_cached and observation_source not in source_priority:
            continue
        candidates.append(tracking)

    if not candidates:
        return None

    candidates.sort(
        key=lambda item: (
            int(source_priority.get(str(item.get("observation_source", "")).strip().lower(), 10**6)),
            float(item.get("last_seen_age_s", 10**6)),
            int(item.get("marker_id", 10**9)),
            str(item.get("target_id", "")),
        )
    )
    return candidates[0]


def select_live_pick_target(target_ids: list[str]) -> Optional[dict[str, Any]]:
    return select_pick_target(target_ids, allow_cached=False)


def track_drop_target(target_id: str) -> dict[str, Any]:
    marker_map = dict(getattr(cfg, "VISION_DROP_MARKER_MAP", {}))
    expected_pose = cfg.build_target_pose(target_id)
    return _track_target(target_id=target_id, marker_map=marker_map, expected_pose=expected_pose, role="drop")


def track_drop_level(level: int) -> dict[str, Any]:
    target_id = cfg.build_target_id_for_level(level)
    return track_drop_target(target_id)


def forget_pick_target(target_id: str) -> None:
    marker_map = dict(getattr(cfg, "VISION_PICK_MARKER_MAP", {}))
    marker_id = marker_map.get(str(target_id))
    if marker_id is not None:
        _invalidate_marker(int(marker_id))


def reset_pick_tracking_memory() -> None:
    marker_map = dict(getattr(cfg, "VISION_PICK_MARKER_MAP", {}))
    marker_ids = [int(marker_id) for marker_id in marker_map.values()]
    _reset_marker_memory(marker_ids)
