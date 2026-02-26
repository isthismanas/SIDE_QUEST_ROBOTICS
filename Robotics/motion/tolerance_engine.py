from __future__ import annotations

from math import sqrt
from typing import Tuple

import robot_config as cfg


Pose = Tuple[float, float, float, float, float, float]


def radial_error_mm(pose: Pose) -> float:
	x0, y0 = cfg.TOWER_BASE_POSE[0], cfg.TOWER_BASE_POSE[1]
	x, y = pose[0], pose[1]
	return sqrt((x - x0) ** 2 + (y - y0) ** 2)


def classify_pose(pose: Pose) -> str:
	r_mm = radial_error_mm(pose)
	green_mm = getattr(cfg, "TOL_GREEN_MM", 3.0)
	yellow_mm = getattr(cfg, "TOL_YELLOW_MM", 6.0)
	green_thr = green_mm * cfg.TOLERANCE_SCALE
	yellow_thr = yellow_mm * cfg.TOLERANCE_SCALE

	if r_mm <= green_thr:
		return "GREEN"
	if r_mm <= yellow_thr:
		return "YELLOW"
	return "RED"

