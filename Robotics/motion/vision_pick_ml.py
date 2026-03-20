from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Any, Optional

import robot_config as cfg


DEFAULT_FEATURE_NAMES = (
    "camera_x_m",
    "camera_y_m",
    "camera_z_m",
    "camera_pitch_rad",
    "camera_yaw_rad",
)


@dataclass(frozen=True)
class ResidualSample:
    label: str
    features: tuple[float, ...]
    residual_xy_mm: tuple[float, float]


@dataclass(frozen=True)
class PickResidualModel:
    source_path: str
    feature_names: tuple[str, ...]
    feature_mean: tuple[float, ...]
    feature_scale: tuple[float, ...]
    knn_k: int
    max_neighbor_distance: float
    max_correction_norm_mm: float
    weight_power: float
    samples: tuple[ResidualSample, ...]

    def predict_residual_mm(self, feature_map: dict[str, float]) -> tuple[Optional[tuple[float, float]], str]:
        feature_vector: list[float] = []
        for name in self.feature_names:
            if name not in feature_map:
                return None, f"missing_feature:{name}"
            feature_vector.append(float(feature_map[name]))

        if not self.samples:
            return None, "model_empty"

        distances: list[tuple[float, ResidualSample]] = []
        for sample in self.samples:
            dist_sq = 0.0
            for index, value in enumerate(feature_vector):
                scale = float(self.feature_scale[index])
                denom = scale if abs(scale) > 1e-9 else 1.0
                delta = (float(value) - float(sample.features[index])) / denom
                dist_sq += delta * delta
            distances.append((math.sqrt(dist_sq), sample))

        distances.sort(key=lambda item: item[0])
        neighbors = distances[: max(1, int(self.knn_k))]
        if not neighbors:
            return None, "no_neighbors"

        nearest_distance = float(neighbors[0][0])
        if nearest_distance > float(self.max_neighbor_distance):
            return None, f"neighbor_too_far:{nearest_distance:.4f}"

        weighted_dx = 0.0
        weighted_dy = 0.0
        weight_total = 0.0
        for distance, sample in neighbors:
            weight = 1.0 / ((max(float(distance), 1e-6)) ** float(self.weight_power))
            weighted_dx += weight * float(sample.residual_xy_mm[0])
            weighted_dy += weight * float(sample.residual_xy_mm[1])
            weight_total += weight

        if weight_total <= 0.0:
            return None, "zero_weight"

        dx_mm = weighted_dx / weight_total
        dy_mm = weighted_dy / weight_total
        norm_mm = math.sqrt((dx_mm * dx_mm) + (dy_mm * dy_mm))
        max_norm_mm = float(self.max_correction_norm_mm)
        if max_norm_mm > 0.0 and norm_mm > max_norm_mm:
            scale = max_norm_mm / norm_mm
            dx_mm *= scale
            dy_mm *= scale
            return (dx_mm, dy_mm), "ok_clamped"

        return (dx_mm, dy_mm), "ok"


_CACHED_MODEL: Optional[PickResidualModel] = None
_CACHED_KEY: Optional[tuple[str, float, int]] = None


def _model_path() -> str:
    configured = str(getattr(cfg, "VISION_PICK_ML_MODEL_JSON", "")).strip()
    if not configured:
        return ""
    if os.path.isabs(configured):
        return configured
    return os.path.abspath(os.path.join(os.path.dirname(__file__), configured))


def _cache_key(path: str) -> tuple[str, float, int]:
    stat = os.stat(path)
    return (path, stat.st_mtime, stat.st_size)


def _float_tuple(values: list[Any], *, expected_len: int, label: str) -> tuple[float, ...]:
    if len(values) != expected_len:
        raise ValueError(f"{label} length mismatch: expected {expected_len}, got {len(values)}")
    return tuple(float(value) for value in values)


def _load_model(path: str) -> PickResidualModel:
    with open(path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)

    model_type = str(payload.get("model_type", "")).strip().lower()
    if model_type != "pick_residual_knn_v1":
        raise ValueError(f"Unsupported model_type: {payload.get('model_type')}")

    feature_names_raw = payload.get("feature_names")
    if not isinstance(feature_names_raw, list) or not feature_names_raw:
        raise ValueError("Missing feature_names")
    feature_names = tuple(str(name) for name in feature_names_raw)

    feature_mean = _float_tuple(list(payload.get("feature_mean", [])), expected_len=len(feature_names), label="feature_mean")
    feature_scale = _float_tuple(list(payload.get("feature_scale", [])), expected_len=len(feature_names), label="feature_scale")

    samples_raw = payload.get("samples")
    if not isinstance(samples_raw, list) or not samples_raw:
        raise ValueError("Missing samples")

    samples: list[ResidualSample] = []
    for index, sample in enumerate(samples_raw):
        if not isinstance(sample, dict):
            raise ValueError(f"Sample {index} must be an object")
        features = _float_tuple(list(sample.get("features", [])), expected_len=len(feature_names), label=f"samples[{index}].features")
        residual_xy_mm = _float_tuple(
            [
                sample.get("residual_xy_mm", {}).get("x"),
                sample.get("residual_xy_mm", {}).get("y"),
            ],
            expected_len=2,
            label=f"samples[{index}].residual_xy_mm",
        )
        samples.append(
            ResidualSample(
                label=str(sample.get("sample_label", f"sample_{index + 1}")),
                features=features,
                residual_xy_mm=(float(residual_xy_mm[0]), float(residual_xy_mm[1])),
            )
        )

    return PickResidualModel(
        source_path=path,
        feature_names=feature_names,
        feature_mean=feature_mean,
        feature_scale=feature_scale,
        knn_k=max(1, int(payload.get("knn_k", 3))),
        max_neighbor_distance=float(payload.get("max_neighbor_distance", 3.0)),
        max_correction_norm_mm=float(payload.get("max_correction_norm_mm", 8.0)),
        weight_power=float(payload.get("weight_power", 2.0)),
        samples=tuple(samples),
    )


def get_pick_residual_model() -> tuple[Optional[PickResidualModel], str]:
    global _CACHED_MODEL, _CACHED_KEY

    if not bool(getattr(cfg, "VISION_PICK_ML_ENABLED", False)):
        return None, "ml_disabled"

    path = _model_path()
    if not path:
        return None, "model_path_unset"
    if not os.path.exists(path):
        return None, "model_file_missing"

    try:
        key = _cache_key(path)
    except OSError:
        return None, "model_file_unreadable"

    if _CACHED_MODEL is not None and _CACHED_KEY == key:
        return _CACHED_MODEL, "ok"

    try:
        model = _load_model(path)
    except Exception as exc:
        return None, f"model_load_error:{exc}"

    _CACHED_MODEL = model
    _CACHED_KEY = key
    return model, "ok"


def feature_map_from_camera_pose(camera_pose: Any) -> tuple[Optional[dict[str, float]], str]:
    if camera_pose is None:
        return None, "camera_pose_missing"

    if isinstance(camera_pose, dict):
        raw = dict(camera_pose)
        aliases = {
            "camera_x_m": ("camera_x_m", "x_m", "x"),
            "camera_y_m": ("camera_y_m", "y_m", "y"),
            "camera_z_m": ("camera_z_m", "z_m", "z"),
            "camera_roll_rad": ("camera_roll_rad", "roll_rad", "roll"),
            "camera_pitch_rad": ("camera_pitch_rad", "pitch_rad", "pitch"),
            "camera_yaw_rad": ("camera_yaw_rad", "yaw_rad", "yaw"),
        }
        feature_map: dict[str, float] = {}
        for canonical_name, keys in aliases.items():
            value = None
            for key in keys:
                if key in raw:
                    value = raw[key]
                    break
            if value is not None:
                feature_map[canonical_name] = float(value)
        if not feature_map:
            return None, "camera_pose_dict_empty"
        return feature_map, "ok"

    if isinstance(camera_pose, (tuple, list)) and len(camera_pose) >= 6:
        return (
            {
                "camera_x_m": float(camera_pose[0]),
                "camera_y_m": float(camera_pose[1]),
                "camera_z_m": float(camera_pose[2]),
                "camera_roll_rad": float(camera_pose[3]),
                "camera_pitch_rad": float(camera_pose[4]),
                "camera_yaw_rad": float(camera_pose[5]),
            },
            "ok",
        )

    return None, "camera_pose_unsupported"


def predict_pick_residual_mm(camera_pose: Any) -> tuple[Optional[tuple[float, float]], str]:
    model, reason = get_pick_residual_model()
    if model is None:
        return None, reason

    feature_map, pose_reason = feature_map_from_camera_pose(camera_pose)
    if feature_map is None:
        return None, pose_reason

    return model.predict_residual_mm(feature_map)
