from __future__ import annotations

import os, sys
from math import sqrt
from typing import Optional, Tuple

import robot_config as cfg

PICK_POSE_MODE = str(getattr(cfg, "PICK_POSE_MODE", "deterministic")).strip().lower()
VISION_MODE_ENABLED = PICK_POSE_MODE in {"vision", "perception"}

perc_engine = None
if VISION_MODE_ENABLED:
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
	Translates Optical Camera Space into Dobot Coordinate Space.
	(Note: This assumes the camera is looking straight down).
	"""
	cam_x_mm = cam_x_m * 1000.0
	cam_y_mm = cam_y_m * 1000.0

	# 2. Enter the offsets you physically measured in Step 3.1
	OFFSET_X = 150.0 
	OFFSET_Y = 50.0  

	return (cam_x_mm + OFFSET_X, cam_y_mm + OFFSET_Y)


def radial_error_mm(pose: Pose) -> float:
	if (not VISION_MODE_ENABLED) or (perc_engine is None):
		x0, y0 = cfg.TOWER_BASE_POSE[0], cfg.TOWER_BASE_POSE[1]
	else:
		tracking_state = perc_engine.get_latest_state()
		base_marker_data = tracking_state.get(0)
		if base_marker_data is None:
			x0, y0 = cfg.TOWER_BASE_POSE[0], cfg.TOWER_BASE_POSE[1]
		else:
			cam_x_m, cam_y_m = base_marker_data[0], base_marker_data[1]
			x0, y0 = convert_camera_to_robot(cam_x_m, cam_y_m)

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

