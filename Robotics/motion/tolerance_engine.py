from __future__ import annotations

import os, sys
from math import sqrt
from typing import Any, Optional, Tuple

import robot_config as cfg
import vision_bridge

PICK_POSE_MODE = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
VISION_MODE_ENABLED = PICK_POSE_MODE == "experimental"
VISION_ASSIST_ENABLED = bool(getattr(cfg, "VISION_ASSIST_ENABLED", False))
PERCEPTION_ASSIST_ENABLED = VISION_MODE_ENABLED or VISION_ASSIST_ENABLED

perc_engine = None
if PERCEPTION_ASSIST_ENABLED:
	try:
		perc_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "perception")
		if perc_path not in sys.path:
			sys.path.append(perc_path)
		from perception_engine import engine as perc_engine
	except Exception:
		perc_engine = None

Pose = Tuple[float, float, float, float, float, float]


def convert_camera_to_robot(cam_x_m: float, cam_y_m: float) -> Tuple[float, float]:
	"""
	Translate camera-frame XY into robot-frame XY using the current Phase 3 planar calibration.
	"""
	robot_xy, reason = vision_bridge.camera_xy_to_robot_xy_mm(cam_x_m, cam_y_m)
	if robot_xy is None:
		raise ValueError(f"camera_to_robot_unavailable:{reason}")
	return robot_xy


def reference_marker_center_xy(marker_id: Optional[int] = None) -> Tuple[Optional[Tuple[float, float]], str]:
	if (not PERCEPTION_ASSIST_ENABLED) or (perc_engine is None):
		return None, "perception_disabled"

	tracking_state = perc_engine.get_latest_state()
	reference_marker_id = int(getattr(cfg, "VISION_REFERENCE_MARKER_ID", 0) if marker_id is None else marker_id)
	marker_pose = tracking_state.get(reference_marker_id)
	if marker_pose is None:
		return None, "reference_marker_missing"

	try:
		return convert_camera_to_robot(marker_pose[0], marker_pose[1]), "ok"
	except Exception as exc:
		return None, str(exc)


def vision_shadow_assessment(pose: Pose, marker_id: Optional[int] = None) -> dict[str, Any]:
	center_xy, reason = reference_marker_center_xy(marker_id=marker_id)
	if center_xy is None:
		return {
			"available": False,
			"reason": reason,
		}

	ax_mm, ay_mm = axis_error_mm(pose, center_xy=center_xy)
	return {
		"available": True,
		"reason": "ok",
		"center_xy": center_xy,
		"axis_error_mm": (ax_mm, ay_mm),
		"zone": classify_pose(pose, center_xy=center_xy),
	}


def radial_error_mm(pose: Pose) -> float:
	center_xy, _ = reference_marker_center_xy()
	if center_xy is None:
		x0, y0 = cfg.TOWER_BASE_POSE[0], cfg.TOWER_BASE_POSE[1]
	else:
		x0, y0 = center_xy

	target_x, target_y = pose[0], pose[1]
	return sqrt((target_x - x0) ** 2 + (target_y - y0) ** 2)


def axis_error_mm(pose: Pose, center_xy: Optional[Tuple[float, float]] = None) -> Tuple[float, float]:
	target_x, target_y = pose[0], pose[1]
	if center_xy is None:
		x0, y0 = cfg.TOWER_BASE_POSE[0], cfg.TOWER_BASE_POSE[1]
	else:
		x0, y0 = center_xy
	return abs(target_x - x0), abs(target_y - y0)


def classify_pose(pose: Pose, center_xy: Optional[Tuple[float, float]] = None) -> str:
	green_mm = getattr(cfg, "TOL_GREEN_MM", 3.0)
	yellow_mm = getattr(cfg, "TOL_YELLOW_MM", 6.0)
	green_thr = green_mm * cfg.TOLERANCE_SCALE
	yellow_thr = yellow_mm * cfg.TOLERANCE_SCALE
	ax_mm, ay_mm = axis_error_mm(pose, center_xy=center_xy)

	if ax_mm <= green_thr and ay_mm <= green_thr:
		return "GREEN"
	if ax_mm <= yellow_thr and ay_mm <= yellow_thr:
		return "YELLOW"
	return "RED"
