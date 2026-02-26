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
	r = radial_error_mm(pose)
	g = cfg.TOLERANCE_GREEN_MM * cfg.TOLERANCE_SCALE
	y = cfg.TOLERANCE_YELLOW_MM * cfg.TOLERANCE_SCALE

	if r <= g:
		return "GREEN"
	if r <= y:
		return "YELLOW"
	return "RED"

