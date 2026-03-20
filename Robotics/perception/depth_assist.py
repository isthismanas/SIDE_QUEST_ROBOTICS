from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class DepthAssistConfig:
    inner_polygon_scale: float = 0.65
    min_valid_depth_mm: float = 50.0
    max_valid_depth_mm: float = 2500.0
    center_window_radius_px: int = 4


class DepthAssistModule:
    """
    Optional perception-side depth summarizer.

    This module only extracts local depth-map statistics around detected markers.
    It does not alter deterministic controller motion or state-machine authority.
    """

    def __init__(self, config: DepthAssistConfig | None = None) -> None:
        self.config = config or DepthAssistConfig()

    def extract_marker_depth(self, corners, ids, depth_frame_mm: np.ndarray | None) -> dict[int, dict[str, float]]:
        if depth_frame_mm is None or ids is None or len(ids) == 0:
            return {}

        observations: dict[int, dict[str, float]] = {}
        for marker_id, corner in zip(ids.flatten(), corners):
            stats = self._depth_stats_for_marker(depth_frame_mm, corner[0].astype(np.float32))
            if stats is not None:
                observations[int(marker_id)] = stats
        return observations

    def _depth_stats_for_marker(self, depth_frame_mm: np.ndarray, marker_corners_xy: np.ndarray) -> dict[str, float] | None:
        if depth_frame_mm.ndim != 2 or marker_corners_xy.shape != (4, 2):
            return None

        polygon = self._inner_polygon(marker_corners_xy)
        min_x = max(0, int(np.floor(np.min(polygon[:, 0]))))
        max_x = min(depth_frame_mm.shape[1] - 1, int(np.ceil(np.max(polygon[:, 0]))))
        min_y = max(0, int(np.floor(np.min(polygon[:, 1]))))
        max_y = min(depth_frame_mm.shape[0] - 1, int(np.ceil(np.max(polygon[:, 1]))))
        if max_x <= min_x or max_y <= min_y:
            return None

        roi = depth_frame_mm[min_y : max_y + 1, min_x : max_x + 1]
        if roi.size == 0:
            return None

        mask = np.zeros((roi.shape[0], roi.shape[1]), dtype=np.uint8)
        shifted_polygon = polygon.copy()
        shifted_polygon[:, 0] -= float(min_x)
        shifted_polygon[:, 1] -= float(min_y)
        cv2.fillConvexPoly(mask, np.round(shifted_polygon).astype(np.int32), 255)

        valid_depth = roi[(mask > 0) & self._valid_depth_mask(roi)]
        roi_area_px = int(np.count_nonzero(mask))
        if roi_area_px <= 0 or valid_depth.size == 0:
            return None

        center_xy = np.mean(marker_corners_xy, axis=0)
        center_depth_mm = self._center_depth_mm(depth_frame_mm, center_xy)
        if center_depth_mm is None:
            center_depth_mm = float(np.median(valid_depth))

        return {
            "center_depth_mm": float(center_depth_mm),
            "median_depth_mm": float(np.median(valid_depth)),
            "mean_depth_mm": float(np.mean(valid_depth)),
            "std_depth_mm": float(np.std(valid_depth)),
            "min_depth_mm": float(np.min(valid_depth)),
            "max_depth_mm": float(np.max(valid_depth)),
            "valid_fraction": float(valid_depth.size / float(roi_area_px)),
            "valid_pixel_count": float(valid_depth.size),
            "roi_area_px": float(roi_area_px),
        }

    def _inner_polygon(self, marker_corners_xy: np.ndarray) -> np.ndarray:
        center = np.mean(marker_corners_xy, axis=0, keepdims=True)
        return center + ((marker_corners_xy - center) * float(self.config.inner_polygon_scale))

    def _valid_depth_mask(self, depth_values: np.ndarray) -> np.ndarray:
        return (
            np.isfinite(depth_values)
            & (depth_values >= float(self.config.min_valid_depth_mm))
            & (depth_values <= float(self.config.max_valid_depth_mm))
        )

    def _center_depth_mm(self, depth_frame_mm: np.ndarray, center_xy: np.ndarray) -> float | None:
        x = int(round(float(center_xy[0])))
        y = int(round(float(center_xy[1])))
        radius = max(1, int(self.config.center_window_radius_px))
        min_x = max(0, x - radius)
        max_x = min(depth_frame_mm.shape[1] - 1, x + radius)
        min_y = max(0, y - radius)
        max_y = min(depth_frame_mm.shape[0] - 1, y + radius)
        window = depth_frame_mm[min_y : max_y + 1, min_x : max_x + 1]
        valid = window[self._valid_depth_mask(window)]
        if valid.size == 0:
            return None
        return float(np.median(valid))
