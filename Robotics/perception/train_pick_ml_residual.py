#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from datetime import datetime, timezone
from typing import Any

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MOTION_DIR = os.path.normpath(os.path.join(THIS_DIR, "..", "motion"))

if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)
if MOTION_DIR not in sys.path:
    sys.path.append(MOTION_DIR)

import robot_config as cfg
import vision_bridge
import vision_pick_ml


def _load_jsonl(path: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                rows.append(json.loads(stripped))
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON in {path} line {line_number}: {exc}") from exc
    return rows


def _default_input_jsonl() -> str:
    solution_path = str(getattr(cfg, "VISION_CALIBRATION_JSON", "")).strip()
    if solution_path and os.path.exists(solution_path):
        with open(solution_path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        input_jsonl = str(payload.get("input_jsonl", "")).strip()
        if input_jsonl and os.path.exists(input_jsonl):
            return input_jsonl
    return os.path.join(THIS_DIR, "calibration_data", "pick_plane_marker13_20260318.jsonl")


def _default_output_json() -> str:
    return os.path.join(THIS_DIR, "calibration_data", "pick_plane_marker13_20260318_ml_residual.json")


def _default_grab_pick_log_jsonl() -> str:
    return os.path.join(THIS_DIR, "logs", "grab_pick.jsonl")


def _default_pickup_runtime_residual_jsonl() -> str:
    return os.path.join(THIS_DIR, "logs", "pickup_runtime_residual.jsonl")


def _canonical_lineage_path(path: str | None) -> str:
    text = str(path or "").strip()
    if not text:
        return ""
    normalized = text.replace("\\", "/")
    marker = "/Robotics/perception/calibration_data/"
    index = normalized.rfind(marker)
    if index >= 0:
        return normalized[index + 1 :]
    return os.path.basename(normalized)


def _parse_utc_timestamp(value: str | None) -> datetime | None:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        return datetime.fromisoformat(text.replace("Z", "+00:00")).astimezone(timezone.utc)
    except ValueError:
        return None


def _active_training_lineage(input_jsonl: str) -> dict[str, object] | None:
    calibration_path = str(getattr(cfg, "VISION_CALIBRATION_JSON", "")).strip()
    if not calibration_path:
        return None
    if not os.path.isabs(calibration_path):
        calibration_path = os.path.abspath(os.path.join(MOTION_DIR, calibration_path))
    if not os.path.exists(calibration_path):
        return None

    with open(calibration_path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)

    resolved_input_jsonl = str(payload.get("input_jsonl", "")).strip() or os.path.abspath(input_jsonl)
    return {
        "calibration_json": calibration_path,
        "calibration_json_label": _canonical_lineage_path(calibration_path),
        "calibration_input_jsonl": resolved_input_jsonl,
        "calibration_input_label": _canonical_lineage_path(resolved_input_jsonl),
        "calibration_generated_at_utc": str(payload.get("generated_at_utc", "")).strip(),
        "pick_offset_x_mm": float(getattr(cfg, "VISION_PICK_X_OFFSET_MM", 0.0)),
        "pick_offset_y_mm": float(getattr(cfg, "VISION_PICK_Y_OFFSET_MM", 0.0)),
    }


def _row_pick_offset_xy_mm(row: dict[str, Any]) -> tuple[float, float] | None:
    direct = row.get("vision_pick_offset_xy_mm")
    if isinstance(direct, dict):
        x = direct.get("x")
        y = direct.get("y")
        if x is not None and y is not None:
            return float(x), float(y)

    projection = row.get("pick_projection")
    if isinstance(projection, dict):
        nested = projection.get("offset_xy_mm")
        if isinstance(nested, dict):
            x = nested.get("x")
            y = nested.get("y")
            if x is not None and y is not None:
                return float(x), float(y)
    return None


def _row_matches_training_lineage(row: dict[str, Any], lineage: dict[str, object]) -> bool:
    if bool(row.get("pick_ml_enabled", False)):
        return False

    row_offsets = _row_pick_offset_xy_mm(row)
    if row_offsets is None:
        return False
    if not (
        math.isclose(row_offsets[0], float(lineage["pick_offset_x_mm"]), abs_tol=1e-6)
        and math.isclose(row_offsets[1], float(lineage["pick_offset_y_mm"]), abs_tol=1e-6)
    ):
        return False

    row_input_label = _canonical_lineage_path(row.get("vision_calibration_input_jsonl"))
    row_calibration_label = _canonical_lineage_path(row.get("vision_calibration_json"))
    expected_input_label = str(lineage["calibration_input_label"])
    expected_calibration_label = str(lineage["calibration_json_label"])
    if row_input_label or row_calibration_label:
        if row_input_label and row_input_label != expected_input_label:
            return False
        if row_calibration_label and row_calibration_label != expected_calibration_label:
            return False
        return True

    calibration_generated_at = _parse_utc_timestamp(str(lineage["calibration_generated_at_utc"]))
    row_ts = _parse_utc_timestamp(row.get("ts_utc"))
    if calibration_generated_at is None or row_ts is None:
        return False
    return row_ts >= calibration_generated_at


def _feature_vector(record: dict[str, Any], feature_names: tuple[str, ...]) -> tuple[float, ...]:
    median_pose = dict(record["camera_window"]["median_pose"])
    feature_map, reason = vision_pick_ml.feature_map_from_camera_pose(median_pose)
    if feature_map is None:
        raise ValueError(f"Unable to extract features: {reason}")
    return tuple(float(feature_map[name]) for name in feature_names)


def _base_pick_xy(record: dict[str, Any]) -> tuple[float, float]:
    median_pose = dict(record["camera_window"]["median_pose"])
    robot_xy, reason = vision_bridge.camera_xy_to_pick_robot_xy_mm_base(
        float(median_pose["x_m"]),
        float(median_pose["y_m"]),
    )
    if robot_xy is None:
        raise ValueError(f"Base pickup projection unavailable: {reason}")
    return float(robot_xy[0]), float(robot_xy[1])


def _actual_pick_xy(record: dict[str, Any]) -> tuple[float, float]:
    robot_pose = dict(record["robot_pose"])
    return float(robot_pose["x_mm"]), float(robot_pose["y_mm"])


def _pick_target_xy_mm(target_id: str) -> tuple[float, float]:
    pose = cfg.pick_target_pose(str(target_id).strip().upper())
    return float(pose[0]), float(pose[1])


def _feature_vector_from_camera_summary(summary: dict[str, Any], feature_names: tuple[str, ...]) -> tuple[float, ...]:
    median_pose = dict(summary["median_pose"])
    feature_map, reason = vision_pick_ml.feature_map_from_camera_pose(median_pose)
    if feature_map is None:
        raise ValueError(f"Unable to extract features from log camera summary: {reason}")
    return tuple(float(feature_map[name]) for name in feature_names)


def _base_pick_xy_from_camera_summary(summary: dict[str, Any]) -> tuple[float, float]:
    median_pose = dict(summary["median_pose"])
    robot_xy, reason = vision_bridge.camera_xy_to_pick_robot_xy_mm_base(
        float(median_pose["x_m"]),
        float(median_pose["y_m"]),
    )
    if robot_xy is None:
        raise ValueError(f"Base pickup projection unavailable from log camera summary: {reason}")
    return float(robot_xy[0]), float(robot_xy[1])


def _infer_actual_target_from_plan(plan_event: dict[str, Any], max_target_distance_mm: float) -> tuple[str | None, str]:
    participant_name = str(plan_event.get("participant_name", "")).strip().upper()
    match = re.search(r"(^|[^A-Z0-9])(P[1-7])([^A-Z0-9]|$)", participant_name)
    if match:
        return str(match.group(2)), "participant_name"

    diagnostics = plan_event.get("pickup_grid_diagnostics")
    if not isinstance(diagnostics, dict):
        return None, "pickup_grid_missing"

    source_target_id = diagnostics.get("source_target_id")
    source_delta = diagnostics.get("source_delta_mm")
    if isinstance(source_target_id, str) and isinstance(source_delta, dict):
        source_norm = source_delta.get("norm")
        if source_norm is not None and float(source_norm) <= float(max_target_distance_mm):
            return str(source_target_id).strip().upper(), "source_delta"

    nearest_target_id = diagnostics.get("nearest_pick_target_id")
    nearest_delta = diagnostics.get("nearest_pick_delta_mm")
    if isinstance(nearest_target_id, str) and isinstance(nearest_delta, dict):
        nearest_norm = nearest_delta.get("norm")
        if nearest_norm is not None and float(nearest_norm) <= float(max_target_distance_mm):
            return str(nearest_target_id).strip().upper(), "nearest_delta"

    return None, "target_unresolved"


def _load_log_confirmation_samples(
    path: str,
    feature_names: tuple[str, ...],
    *,
    max_target_distance_mm: float,
    lineage: dict[str, object] | None,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    rows = _load_jsonl(path)
    completed_keys: set[tuple[str, int | None, str]] = set()
    for row in rows:
        event_name = str(row.get("event", "")).strip()
        if event_name not in {"vision_pick_place_execute_complete", "grab_pick_execute_complete"}:
            continue
        completed_keys.add(
            (
                str(row.get("run_id", "")),
                None if row.get("cycle_index") is None else int(row.get("cycle_index")),
                event_name,
            )
        )

    samples: list[dict[str, Any]] = []
    considered_plan_rows = 0
    skipped_lineage_rows = 0
    for row in rows:
        event_name = str(row.get("event", "")).strip()
        if event_name not in {"vision_pick_place_plan", "grab_pick_plan"}:
            continue
        considered_plan_rows += 1
        if lineage is not None and not _row_matches_training_lineage(row, lineage):
            skipped_lineage_rows += 1
            continue

        completion_event = "vision_pick_place_execute_complete" if event_name == "vision_pick_place_plan" else "grab_pick_execute_complete"
        run_key = (
            str(row.get("run_id", "")),
            None if row.get("cycle_index") is None else int(row.get("cycle_index")),
            completion_event,
        )
        if run_key not in completed_keys:
            continue

        camera_summary = row.get("camera_pose_summary")
        if not isinstance(camera_summary, dict):
            continue

        actual_target_id, target_reason = _infer_actual_target_from_plan(row, max_target_distance_mm)
        if actual_target_id is None:
            continue

        feature_vector = _feature_vector_from_camera_summary(camera_summary, feature_names)
        base_x_mm, base_y_mm = _base_pick_xy_from_camera_summary(camera_summary)
        actual_x_mm, actual_y_mm = _pick_target_xy_mm(actual_target_id)
        residual_x_mm = actual_x_mm - base_x_mm
        residual_y_mm = actual_y_mm - base_y_mm
        cycle_index = row.get("cycle_index")
        label_suffix = f"cycle{int(cycle_index)}" if cycle_index is not None else "single"
        samples.append(
            {
                "sample_label": f"log_{str(row.get('run_id', 'unknown'))}_{label_suffix}",
                "marker_id": int(row.get("marker_id", -1)),
                "features": tuple(float(value) for value in feature_vector),
                "base_xy_mm": (base_x_mm, base_y_mm),
                "actual_xy_mm": (actual_x_mm, actual_y_mm),
                "residual_xy_mm": (residual_x_mm, residual_y_mm),
                "sample_source": "log_confirmation",
                "sample_source_path": os.path.abspath(path),
                "actual_target_id": actual_target_id,
                "actual_target_reason": target_reason,
                "source_event": event_name,
                "participant_name": str(row.get("participant_name", "")),
            }
        )
    return samples, {
        "considered_plan_rows": considered_plan_rows,
        "skipped_lineage_rows": skipped_lineage_rows,
    }


def _runtime_target_id(row: dict[str, Any]) -> str | None:
    expected_target_id = row.get("expected_pick_target_id")
    if isinstance(expected_target_id, str) and expected_target_id.strip():
        return str(expected_target_id).strip().upper()
    target_label = str(row.get("target_label", "")).strip().upper()
    match = re.search(r"(P[1-7])", target_label)
    if match:
        return str(match.group(1))
    return None


def _build_plan_index(
    rows: list[dict[str, Any]],
    *,
    lineage: dict[str, object] | None,
) -> dict[tuple[str, str], dict[str, Any]]:
    indexed: dict[tuple[str, str], dict[str, Any]] = {}
    ambiguous: set[tuple[str, str]] = set()
    for row in rows:
        event_name = str(row.get("event", "")).strip()
        if event_name not in {"vision_pick_place_plan", "grab_pick_plan"}:
            continue
        if lineage is not None and not _row_matches_training_lineage(row, lineage):
            continue
        run_id = str(row.get("run_id", "")).strip()
        if not run_id:
            continue
        target_id = str(row.get("target_id", "")).strip().upper()
        if not target_id:
            inferred_target_id, _ = _infer_actual_target_from_plan(row, max_target_distance_mm=1e9)
            if inferred_target_id is None:
                continue
            target_id = inferred_target_id
        key = (run_id, target_id)
        if key in indexed:
            ambiguous.add(key)
        else:
            indexed[key] = row
    for key in ambiguous:
        indexed.pop(key, None)
    return indexed


def _load_runtime_residual_samples(
    plan_log_path: str,
    runtime_log_path: str,
    feature_names: tuple[str, ...],
    *,
    max_runtime_expected_residual_mm: float,
    lineage: dict[str, object] | None,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    plan_rows = _load_jsonl(plan_log_path)
    runtime_rows = _load_jsonl(runtime_log_path)
    plan_index = _build_plan_index(plan_rows, lineage=lineage)

    samples: list[dict[str, Any]] = []
    considered_runtime_rows = 0
    joined_runtime_rows = 0
    for row in runtime_rows:
        event_name = str(row.get("event", "")).strip()
        if event_name != "pickup_runtime_residual":
            continue
        considered_runtime_rows += 1

        run_id = str(row.get("run_id", "")).strip()
        if not run_id:
            continue
        target_id = _runtime_target_id(row)
        if target_id is None:
            continue
        residual_to_expected = row.get("residual_to_expected_mm")
        if not isinstance(residual_to_expected, dict):
            continue
        residual_norm = math.sqrt(
            float(residual_to_expected.get("x", 0.0)) ** 2
            + float(residual_to_expected.get("y", 0.0)) ** 2
        )
        if residual_norm > float(max_runtime_expected_residual_mm):
            continue

        plan_row = plan_index.get((run_id, target_id))
        if plan_row is None:
            continue
        joined_runtime_rows += 1
        camera_summary = plan_row.get("camera_pose_summary")
        if not isinstance(camera_summary, dict):
            continue

        feature_vector = _feature_vector_from_camera_summary(camera_summary, feature_names)
        base_x_mm, base_y_mm = _base_pick_xy_from_camera_summary(camera_summary)
        actual_x_mm, actual_y_mm = _pick_target_xy_mm(target_id)
        residual_x_mm = actual_x_mm - base_x_mm
        residual_y_mm = actual_y_mm - base_y_mm
        tcp_pose = row.get("tcp_pick_pose_mm_deg") if isinstance(row.get("tcp_pick_pose_mm_deg"), dict) else {}
        samples.append(
            {
                "sample_label": f"runtime_{run_id}_{target_id}",
                "marker_id": int(plan_row.get("marker_id", -1)),
                "features": tuple(float(value) for value in feature_vector),
                "base_xy_mm": (base_x_mm, base_y_mm),
                "actual_xy_mm": (actual_x_mm, actual_y_mm),
                "residual_xy_mm": (residual_x_mm, residual_y_mm),
                "sample_source": "runtime_pick_residual",
                "sample_source_path": os.path.abspath(runtime_log_path),
                "actual_target_id": target_id,
                "actual_target_reason": "runtime_expected_target",
                "participant_name": str(plan_row.get("participant_name", "")),
                "source_event": event_name,
                "runtime_tcp_xy_mm": {
                    "x": float(tcp_pose.get("x", 0.0)),
                    "y": float(tcp_pose.get("y", 0.0)),
                },
                "runtime_residual_to_expected_mm": {
                    "x": float(residual_to_expected.get("x", 0.0)),
                    "y": float(residual_to_expected.get("y", 0.0)),
                    "norm": float(residual_norm),
                },
            }
        )
    return samples, {
        "considered_runtime_rows": considered_runtime_rows,
        "joined_runtime_rows": joined_runtime_rows,
    }


def _as_mm_norm(dx_mm: float, dy_mm: float) -> float:
    return math.sqrt((dx_mm * dx_mm) + (dy_mm * dy_mm))


def _predict_from_arrays(
    feature_matrix: np.ndarray,
    residual_matrix: np.ndarray,
    feature_mean: np.ndarray,
    feature_scale: np.ndarray,
    feature_vector: np.ndarray,
    *,
    knn_k: int,
    max_neighbor_distance: float,
    max_correction_norm_mm: float,
    weight_power: float,
    exclude_index: int | None = None,
) -> tuple[np.ndarray | None, str]:
    if feature_matrix.shape[0] == 0:
        return None, "no_samples"

    distances: list[tuple[float, int]] = []
    for index in range(feature_matrix.shape[0]):
        if exclude_index is not None and index == exclude_index:
            continue
        normalized = (feature_vector - feature_matrix[index]) / feature_scale
        distances.append((float(np.linalg.norm(normalized)), index))

    if not distances:
        return None, "no_neighbors"

    distances.sort(key=lambda item: item[0])
    neighbors = distances[: max(1, int(knn_k))]
    if float(neighbors[0][0]) > float(max_neighbor_distance):
        return None, f"neighbor_too_far:{neighbors[0][0]:.4f}"

    weighted = np.zeros((2,), dtype=np.float64)
    weight_total = 0.0
    for distance, index in neighbors:
        weight = 1.0 / ((max(float(distance), 1e-6)) ** float(weight_power))
        weighted += weight * residual_matrix[index]
        weight_total += weight

    if weight_total <= 0.0:
        return None, "zero_weight"

    predicted = weighted / weight_total
    norm_mm = float(np.linalg.norm(predicted))
    if max_correction_norm_mm > 0.0 and norm_mm > float(max_correction_norm_mm):
        predicted = predicted * (float(max_correction_norm_mm) / norm_mm)
        return predicted, "ok_clamped"
    return predicted, "ok"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Train a bounded kNN residual model for pickup-plane XY correction. "
            "This model sits between the affine calibration and the manual pickup offsets."
        )
    )
    parser.add_argument("--input-jsonl", default=_default_input_jsonl(), help="Calibration capture JSONL with paired camera and robot poses.")
    parser.add_argument("--output-json", default=_default_output_json(), help="Output JSON path for the trained residual model.")
    parser.add_argument(
        "--use-log-confirmation-samples",
        action="store_true",
        help=(
            "Supplement the calibration dataset with successful guarded-run plans from grab_pick.jsonl "
            "when the actual pickup target can be resolved defensibly."
        ),
    )
    parser.add_argument(
        "--grab-pick-log-jsonl",
        default=_default_grab_pick_log_jsonl(),
        help="Guarded-run JSONL log path used with --use-log-confirmation-samples.",
    )
    parser.add_argument(
        "--max-log-target-distance-mm",
        type=float,
        default=12.0,
        help="Only accept a log-derived target label when its resolved target is within this distance threshold.",
    )
    parser.add_argument(
        "--use-runtime-residual-samples",
        action="store_true",
        help=(
            "Supplement the dataset with runtime TCP residual samples by joining "
            "pickup_runtime_residual.jsonl back to matching guarded plan logs."
        ),
    )
    parser.add_argument(
        "--pickup-runtime-residual-jsonl",
        default=_default_pickup_runtime_residual_jsonl(),
        help="Runtime pickup residual JSONL emitted during guarded execute picks.",
    )
    parser.add_argument(
        "--max-runtime-expected-residual-mm",
        type=float,
        default=8.0,
        help="Only trust runtime residual samples when the live TCP XY stayed within this distance of the expected P target.",
    )
    parser.add_argument("--knn-k", type=int, default=2, help="Number of neighbors to use for residual prediction.")
    parser.add_argument(
        "--max-neighbor-distance",
        type=float,
        default=1.5,
        help="Maximum standardized neighbor distance allowed at inference time.",
    )
    parser.add_argument(
        "--max-correction-mm",
        type=float,
        default=8.0,
        help="Maximum residual correction norm applied by the model.",
    )
    parser.add_argument("--weight-power", type=float, default=2.0, help="Inverse-distance weighting power.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    feature_names = tuple(vision_pick_ml.DEFAULT_FEATURE_NAMES)
    records = _load_jsonl(args.input_jsonl)
    if len(records) < 3:
        raise ValueError(f"Need at least 3 calibration records, found {len(records)}.")
    lineage = _active_training_lineage(args.input_jsonl)

    samples: list[dict[str, Any]] = []
    for record in records:
        feature_vector = _feature_vector(record, feature_names)
        base_x_mm, base_y_mm = _base_pick_xy(record)
        actual_x_mm, actual_y_mm = _actual_pick_xy(record)
        residual_x_mm = actual_x_mm - base_x_mm
        residual_y_mm = actual_y_mm - base_y_mm
        samples.append(
            {
                "sample_label": str(record.get("sample_label", f"sample_{len(samples) + 1}")),
                "marker_id": int(record.get("marker_id", -1)),
                "features": tuple(float(value) for value in feature_vector),
                "base_xy_mm": (base_x_mm, base_y_mm),
                "actual_xy_mm": (actual_x_mm, actual_y_mm),
                "residual_xy_mm": (residual_x_mm, residual_y_mm),
                "sample_source": "calibration_capture",
                "sample_source_path": os.path.abspath(args.input_jsonl),
            }
        )

    log_sample_count = 0
    log_lineage_stats = {"considered_plan_rows": 0, "skipped_lineage_rows": 0}
    if bool(args.use_log_confirmation_samples):
        if not os.path.exists(args.grab_pick_log_jsonl):
            raise ValueError(f"Log confirmation dataset not found: {args.grab_pick_log_jsonl}")
        log_samples, log_lineage_stats = _load_log_confirmation_samples(
            args.grab_pick_log_jsonl,
            feature_names,
            max_target_distance_mm=float(args.max_log_target_distance_mm),
            lineage=lineage,
        )
        samples.extend(log_samples)
        log_sample_count = len(log_samples)

    runtime_sample_count = 0
    runtime_lineage_stats = {"considered_runtime_rows": 0, "joined_runtime_rows": 0}
    if bool(args.use_runtime_residual_samples):
        if not os.path.exists(args.grab_pick_log_jsonl):
            raise ValueError(f"Plan log dataset not found for runtime joins: {args.grab_pick_log_jsonl}")
        if not os.path.exists(args.pickup_runtime_residual_jsonl):
            raise ValueError(f"Runtime residual dataset not found: {args.pickup_runtime_residual_jsonl}")
        runtime_samples, runtime_lineage_stats = _load_runtime_residual_samples(
            args.grab_pick_log_jsonl,
            args.pickup_runtime_residual_jsonl,
            feature_names,
            max_runtime_expected_residual_mm=float(args.max_runtime_expected_residual_mm),
            lineage=lineage,
        )
        samples.extend(runtime_samples)
        runtime_sample_count = len(runtime_samples)

    feature_matrix = np.array([sample["features"] for sample in samples], dtype=np.float64)
    residual_matrix = np.array([sample["residual_xy_mm"] for sample in samples], dtype=np.float64)
    base_matrix = np.array([sample["base_xy_mm"] for sample in samples], dtype=np.float64)
    actual_matrix = np.array([sample["actual_xy_mm"] for sample in samples], dtype=np.float64)

    feature_mean = feature_matrix.mean(axis=0)
    feature_scale = feature_matrix.std(axis=0)
    feature_scale = np.where(feature_scale < 1e-9, 1.0, feature_scale)

    train_predictions: list[np.ndarray] = []
    for index in range(feature_matrix.shape[0]):
        predicted_residual, _ = _predict_from_arrays(
            feature_matrix=feature_matrix,
            residual_matrix=residual_matrix,
            feature_mean=feature_mean,
            feature_scale=feature_scale,
            feature_vector=feature_matrix[index],
            knn_k=int(args.knn_k),
            max_neighbor_distance=float(args.max_neighbor_distance),
            max_correction_norm_mm=float(args.max_correction_mm),
            weight_power=float(args.weight_power),
            exclude_index=None,
        )
        if predicted_residual is None:
            predicted_residual = np.zeros((2,), dtype=np.float64)
        train_predictions.append(base_matrix[index] + predicted_residual)

    loo_predictions: list[np.ndarray] = []
    loo_available = 0
    for index in range(feature_matrix.shape[0]):
        predicted_residual, _ = _predict_from_arrays(
            feature_matrix=feature_matrix,
            residual_matrix=residual_matrix,
            feature_mean=feature_mean,
            feature_scale=feature_scale,
            feature_vector=feature_matrix[index],
            knn_k=int(args.knn_k),
            max_neighbor_distance=float(args.max_neighbor_distance),
            max_correction_norm_mm=float(args.max_correction_mm),
            weight_power=float(args.weight_power),
            exclude_index=index,
        )
        if predicted_residual is None:
            loo_predictions.append(base_matrix[index])
            continue
        loo_predictions.append(base_matrix[index] + predicted_residual)
        loo_available += 1

    train_predictions_arr = np.array(train_predictions, dtype=np.float64)
    loo_predictions_arr = np.array(loo_predictions, dtype=np.float64)

    base_error = actual_matrix - base_matrix
    train_error = actual_matrix - train_predictions_arr
    loo_error = actual_matrix - loo_predictions_arr

    payload = {
        "model_type": "pick_residual_knn_v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_calibration_jsonl": os.path.abspath(args.input_jsonl),
        "source_log_confirmation_jsonl": os.path.abspath(args.grab_pick_log_jsonl) if bool(args.use_log_confirmation_samples) else None,
        "source_runtime_residual_jsonl": os.path.abspath(args.pickup_runtime_residual_jsonl) if bool(args.use_runtime_residual_samples) else None,
        "training_lineage": lineage,
        "description": (
            "Bounded pickup-plane residual model. Runtime order is: affine calibration -> "
            "optional ML residual correction -> manual pickup XY offsets."
        ),
        "feature_names": list(feature_names),
        "feature_mean": [float(value) for value in feature_mean.tolist()],
        "feature_scale": [float(value) for value in feature_scale.tolist()],
        "knn_k": int(args.knn_k),
        "max_neighbor_distance": float(args.max_neighbor_distance),
        "max_correction_norm_mm": float(args.max_correction_mm),
        "weight_power": float(args.weight_power),
        "metrics": {
            "sample_count": int(len(samples)),
            "calibration_sample_count": int(len(records)),
            "log_confirmation_sample_count": int(log_sample_count),
            "runtime_residual_sample_count": int(runtime_sample_count),
            "base_rmse_total_mm": float(np.sqrt(np.mean(np.sum(np.square(base_error), axis=1)))),
            "train_rmse_total_mm": float(np.sqrt(np.mean(np.sum(np.square(train_error), axis=1)))),
            "loo_rmse_total_mm": float(np.sqrt(np.mean(np.sum(np.square(loo_error), axis=1)))),
            "loo_available_count": int(loo_available),
        },
        "samples": [],
    }

    for index, sample in enumerate(samples):
        payload["samples"].append(
            {
                "sample_label": sample["sample_label"],
                "marker_id": int(sample["marker_id"]),
                "features": [float(value) for value in sample["features"]],
                "base_xy_mm": {
                    "x": float(sample["base_xy_mm"][0]),
                    "y": float(sample["base_xy_mm"][1]),
                },
                "actual_xy_mm": {
                    "x": float(sample["actual_xy_mm"][0]),
                    "y": float(sample["actual_xy_mm"][1]),
                },
                "residual_xy_mm": {
                    "x": float(sample["residual_xy_mm"][0]),
                    "y": float(sample["residual_xy_mm"][1]),
                    "norm": float(_as_mm_norm(sample["residual_xy_mm"][0], sample["residual_xy_mm"][1])),
                },
                "sample_source": str(sample.get("sample_source", "unknown")),
                "sample_source_path": str(sample.get("sample_source_path", "")),
                "actual_target_id": sample.get("actual_target_id"),
                "actual_target_reason": sample.get("actual_target_reason"),
                "participant_name": sample.get("participant_name"),
                "source_event": sample.get("source_event"),
                "runtime_tcp_xy_mm": sample.get("runtime_tcp_xy_mm"),
                "runtime_residual_to_expected_mm": sample.get("runtime_residual_to_expected_mm"),
                "train_predicted_xy_mm": {
                    "x": float(train_predictions_arr[index, 0]),
                    "y": float(train_predictions_arr[index, 1]),
                },
                "loo_predicted_xy_mm": {
                    "x": float(loo_predictions_arr[index, 0]),
                    "y": float(loo_predictions_arr[index, 1]),
                },
            }
        )

    os.makedirs(os.path.dirname(args.output_json), exist_ok=True)
    with open(args.output_json, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")

    print(f"[PICK_ML] input_jsonl={os.path.abspath(args.input_jsonl)}")
    print(f"[PICK_ML] output_json={os.path.abspath(args.output_json)}")
    print(f"[PICK_ML] sample_count={payload['metrics']['sample_count']}")
    print(f"[PICK_ML] calibration_sample_count={payload['metrics']['calibration_sample_count']}")
    print(f"[PICK_ML] log_confirmation_sample_count={payload['metrics']['log_confirmation_sample_count']}")
    print(f"[PICK_ML] runtime_residual_sample_count={payload['metrics']['runtime_residual_sample_count']}")
    if bool(args.use_log_confirmation_samples):
        print(
            "[PICK_ML] "
            f"log_plan_rows_considered={log_lineage_stats['considered_plan_rows']} "
            f"log_plan_rows_skipped_lineage={log_lineage_stats['skipped_lineage_rows']}"
        )
    if bool(args.use_runtime_residual_samples):
        print(
            "[PICK_ML] "
            f"runtime_rows_considered={runtime_lineage_stats['considered_runtime_rows']} "
            f"runtime_rows_joined={runtime_lineage_stats['joined_runtime_rows']}"
        )
    print(f"[PICK_ML] base_rmse_total_mm={payload['metrics']['base_rmse_total_mm']:.3f}")
    print(f"[PICK_ML] train_rmse_total_mm={payload['metrics']['train_rmse_total_mm']:.3f}")
    print(f"[PICK_ML] loo_rmse_total_mm={payload['metrics']['loo_rmse_total_mm']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
