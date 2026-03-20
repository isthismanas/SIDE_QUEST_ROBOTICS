from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Optional

import robot_config as cfg
import vision_pick_ml


@dataclass(frozen=True)
class PlanarAffineCalibration:
    source_path: str
    coeffs: tuple[tuple[float, float], tuple[float, float], tuple[float, float]]

    def transform_xy_mm(self, camera_x_mm: float, camera_y_mm: float) -> tuple[float, float]:
        # The Phase 3 solver stores a 3x2 matrix:
        # [camera_x, camera_y, bias] -> [robot_x, robot_y]
        return (
            (camera_x_mm * self.coeffs[0][0]) + (camera_y_mm * self.coeffs[1][0]) + self.coeffs[2][0],
            (camera_x_mm * self.coeffs[0][1]) + (camera_y_mm * self.coeffs[1][1]) + self.coeffs[2][1],
        )


_CACHED_CALIBRATION: Optional[PlanarAffineCalibration] = None
_CACHED_KEY: Optional[tuple[str, float, int]] = None


def _calibration_path() -> str:
    configured = str(getattr(cfg, "VISION_CALIBRATION_JSON", "")).strip()
    if not configured:
        return ""
    if os.path.isabs(configured):
        return configured
    return os.path.abspath(os.path.join(os.path.dirname(__file__), configured))


def _cache_key(path: str) -> tuple[str, float, int]:
    stat = os.stat(path)
    return (path, stat.st_mtime, stat.st_size)


def _load_planar_affine(path: str) -> PlanarAffineCalibration:
    with open(path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)

    planar = payload.get("planar_xy_affine", {})
    coeffs_raw = planar.get("coefficients")
    if not isinstance(coeffs_raw, list) or len(coeffs_raw) != 3:
        raise ValueError("Invalid planar_xy_affine coefficients in calibration JSON")

    coeffs: list[tuple[float, float]] = []
    for row in coeffs_raw:
        if not isinstance(row, list) or len(row) != 2:
            raise ValueError("Expected a 3x2 planar_xy_affine coefficient matrix")
        coeffs.append((float(row[0]), float(row[1])))

    return PlanarAffineCalibration(
        source_path=path,
        coeffs=(coeffs[0], coeffs[1], coeffs[2]),
    )


def get_planar_affine() -> tuple[Optional[PlanarAffineCalibration], str]:
    global _CACHED_CALIBRATION, _CACHED_KEY

    path = _calibration_path()
    if not path:
        return None, "calibration_path_unset"
    if not os.path.exists(path):
        return None, "calibration_file_missing"

    try:
        key = _cache_key(path)
    except OSError:
        return None, "calibration_file_unreadable"

    if _CACHED_CALIBRATION is not None and _CACHED_KEY == key:
        return _CACHED_CALIBRATION, "ok"

    try:
        calibration = _load_planar_affine(path)
    except Exception as exc:
        return None, f"calibration_load_error:{exc}"

    _CACHED_CALIBRATION = calibration
    _CACHED_KEY = key
    return calibration, "ok"


def camera_xy_to_robot_xy_mm(cam_x_m: float, cam_y_m: float) -> tuple[Optional[tuple[float, float]], str]:
    calibration, reason = get_planar_affine()
    if calibration is None:
        return None, reason
    robot_xy = calibration.transform_xy_mm(cam_x_m * 1000.0, cam_y_m * 1000.0)
    return robot_xy, "ok"


def camera_xy_to_pick_robot_xy_mm_base(cam_x_m: float, cam_y_m: float) -> tuple[Optional[tuple[float, float]], str]:
    robot_xy, reason = camera_xy_to_robot_xy_mm(cam_x_m, cam_y_m)
    if robot_xy is None:
        return None, reason
    return robot_xy, "ok"


def apply_pick_ml_residual_mm(
    robot_x_mm: float,
    robot_y_mm: float,
    *,
    camera_pose=None,
) -> tuple[tuple[float, float], str]:
    residual_xy_mm, reason = vision_pick_ml.predict_pick_residual_mm(camera_pose)
    if residual_xy_mm is None:
        return (float(robot_x_mm), float(robot_y_mm)), reason
    return (
        float(robot_x_mm) + float(residual_xy_mm[0]),
        float(robot_y_mm) + float(residual_xy_mm[1]),
    ), f"ml_applied:{reason}"


def apply_pick_xy_offsets_mm(robot_x_mm: float, robot_y_mm: float) -> tuple[float, float]:
    return (
        float(robot_x_mm) + float(getattr(cfg, "VISION_PICK_X_OFFSET_MM", 0.0)),
        float(robot_y_mm) + float(getattr(cfg, "VISION_PICK_Y_OFFSET_MM", 0.0)),
    )


def camera_pose_to_pick_robot_xy_mm(
    cam_x_m: float,
    cam_y_m: float,
    *,
    camera_pose=None,
) -> tuple[Optional[tuple[float, float]], str]:
    robot_xy, reason = camera_xy_to_pick_robot_xy_mm_base(cam_x_m, cam_y_m)
    if robot_xy is None:
        return None, reason
    corrected_xy, ml_reason = apply_pick_ml_residual_mm(
        robot_xy[0],
        robot_xy[1],
        camera_pose=camera_pose,
    )
    return apply_pick_xy_offsets_mm(corrected_xy[0], corrected_xy[1]), f"ok:{ml_reason}"


def camera_xy_to_pick_robot_xy_mm(cam_x_m: float, cam_y_m: float) -> tuple[Optional[tuple[float, float]], str]:
    return camera_pose_to_pick_robot_xy_mm(
        cam_x_m,
        cam_y_m,
        camera_pose={
            "x_m": float(cam_x_m),
            "y_m": float(cam_y_m),
        },
    )
