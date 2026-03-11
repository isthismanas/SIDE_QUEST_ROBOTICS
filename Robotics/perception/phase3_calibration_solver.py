#!/usr/bin/env python3
"""
Offline Phase 3 calibration solver.

Consumes Phase 2 JSONL capture records and fits camera-to-robot calibration
models without touching the live robot or cameras.

Outputs:
- planar XY affine fit: camera (x,y) mm -> robot (x,y) mm
- exploratory XYZ affine fit: camera (x,y,z) mm -> robot (x,y,z) mm
- residual summaries and per-sample errors
"""

from __future__ import annotations

import argparse
import json
import os
from datetime import datetime, timezone
from typing import Any

import numpy as np


def _load_records(path: str) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                payload = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON on line {line_number}: {exc}") from exc
            records.append(payload)
    return records


def _extract_arrays(records: list[dict[str, Any]]) -> dict[str, Any]:
    labels: list[str] = []
    camera_xyz_mm: list[list[float]] = []
    robot_xyz_mm: list[list[float]] = []

    for index, record in enumerate(records):
        camera_window = record.get("camera_window", {})
        median_pose = camera_window.get("median_pose", {})
        robot_pose = record.get("robot_pose", {})
        label = str(record.get("sample_label") or f"sample_{index + 1}")

        required_camera = ("x_m", "y_m", "z_m")
        required_robot = ("x_mm", "y_mm", "z_mm")

        missing_camera = [key for key in required_camera if key not in median_pose]
        missing_robot = [key for key in required_robot if key not in robot_pose]
        if missing_camera or missing_robot:
            raise ValueError(
                f"Record '{label}' is missing fields. "
                f"camera_missing={missing_camera} robot_missing={missing_robot}"
            )

        labels.append(label)
        camera_xyz_mm.append(
            [
                float(median_pose["x_m"]) * 1000.0,
                float(median_pose["y_m"]) * 1000.0,
                float(median_pose["z_m"]) * 1000.0,
            ]
        )
        robot_xyz_mm.append(
            [
                float(robot_pose["x_mm"]),
                float(robot_pose["y_mm"]),
                float(robot_pose["z_mm"]),
            ]
        )

    return {
        "labels": labels,
        "camera_xyz_mm": np.array(camera_xyz_mm, dtype=np.float64),
        "robot_xyz_mm": np.array(robot_xyz_mm, dtype=np.float64),
    }


def _fit_affine(inputs: np.ndarray, outputs: np.ndarray) -> dict[str, Any]:
    design = np.concatenate(
        [inputs, np.ones((inputs.shape[0], 1), dtype=np.float64)],
        axis=1,
    )
    coeffs, _, _, _ = np.linalg.lstsq(design, outputs, rcond=None)
    predicted = design @ coeffs
    residual = outputs - predicted
    row_error = np.linalg.norm(residual, axis=1)
    rmse_by_axis = np.sqrt(np.mean(np.square(residual), axis=0))
    rmse_total = float(np.sqrt(np.mean(np.square(row_error))))

    return {
        "coefficients": coeffs,
        "predicted": predicted,
        "residual": residual,
        "row_error": row_error,
        "rmse_by_axis": rmse_by_axis,
        "rmse_total": rmse_total,
    }


def _serialise_matrix(matrix: np.ndarray) -> list[list[float]]:
    return [[float(value) for value in row] for row in matrix.tolist()]


def _per_sample_rows(
    labels: list[str],
    actual: np.ndarray,
    predicted: np.ndarray,
    residual: np.ndarray,
    row_error: np.ndarray,
    axis_labels: tuple[str, ...],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for index, label in enumerate(labels):
        row = {
            "sample_label": label,
            "actual": {
                axis: float(actual[index, axis_index])
                for axis_index, axis in enumerate(axis_labels)
            },
            "predicted": {
                axis: float(predicted[index, axis_index])
                for axis_index, axis in enumerate(axis_labels)
            },
            "residual": {
                axis: float(residual[index, axis_index])
                for axis_index, axis in enumerate(axis_labels)
            },
            "residual_norm_mm": float(row_error[index]),
        }
        rows.append(row)
    return rows


def _print_fit_summary(
    title: str,
    fit: dict[str, Any],
    axis_labels: tuple[str, ...],
) -> None:
    print(f"[PHASE3] {title}")
    for axis_index, axis in enumerate(axis_labels):
        print(
            f"[PHASE3]   rmse_{axis}_mm={fit['rmse_by_axis'][axis_index]:.3f}"
        )
    print(f"[PHASE3]   rmse_total_mm={fit['rmse_total']:.3f}")


def _default_output_path(input_path: str) -> str:
    directory = os.path.dirname(input_path) or "."
    stem = os.path.splitext(os.path.basename(input_path))[0]
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return os.path.join(directory, f"{stem}_phase3_solution_{timestamp}.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline Phase 3 solver for camera-to-robot calibration."
    )
    parser.add_argument(
        "input_jsonl",
        help="Phase 2 calibration capture JSONL file.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional output JSON path for the solved calibration.",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=4,
        help="Minimum number of records required to solve calibration.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    records = _load_records(args.input_jsonl)
    if len(records) < args.min_samples:
        raise ValueError(
            f"Need at least {args.min_samples} samples, found {len(records)}."
        )

    data = _extract_arrays(records)
    labels: list[str] = data["labels"]
    camera_xyz_mm: np.ndarray = data["camera_xyz_mm"]
    robot_xyz_mm: np.ndarray = data["robot_xyz_mm"]

    xy_fit = _fit_affine(
        inputs=camera_xyz_mm[:, :2],
        outputs=robot_xyz_mm[:, :2],
    )
    xyz_fit = _fit_affine(
        inputs=camera_xyz_mm,
        outputs=robot_xyz_mm,
    )

    _print_fit_summary("Planar XY affine fit", xy_fit, ("x", "y"))
    _print_fit_summary("Exploratory XYZ affine fit", xyz_fit, ("x", "y", "z"))

    output_path = args.output or _default_output_path(args.input_jsonl)
    payload = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "input_jsonl": os.path.abspath(args.input_jsonl),
        "sample_count": len(records),
        "sample_labels": labels,
        "planar_xy_affine": {
            "description": "Maps camera-frame (x_mm, y_mm, 1) to robot-frame (x_mm, y_mm).",
            "input_axes_mm": ["camera_x_mm", "camera_y_mm", "bias"],
            "output_axes_mm": ["robot_x_mm", "robot_y_mm"],
            "coefficients": _serialise_matrix(xy_fit["coefficients"]),
            "rmse_by_axis_mm": {
                "x": float(xy_fit["rmse_by_axis"][0]),
                "y": float(xy_fit["rmse_by_axis"][1]),
            },
            "rmse_total_mm": float(xy_fit["rmse_total"]),
            "per_sample": _per_sample_rows(
                labels=labels,
                actual=robot_xyz_mm[:, :2],
                predicted=xy_fit["predicted"],
                residual=xy_fit["residual"],
                row_error=xy_fit["row_error"],
                axis_labels=("x", "y"),
            ),
        },
        "affine_xyz_exploratory": {
            "description": (
                "Maps camera-frame (x_mm, y_mm, z_mm, 1) to robot-frame "
                "(x_mm, y_mm, z_mm). Use for analysis first, not direct robot motion."
            ),
            "input_axes_mm": ["camera_x_mm", "camera_y_mm", "camera_z_mm", "bias"],
            "output_axes_mm": ["robot_x_mm", "robot_y_mm", "robot_z_mm"],
            "coefficients": _serialise_matrix(xyz_fit["coefficients"]),
            "rmse_by_axis_mm": {
                "x": float(xyz_fit["rmse_by_axis"][0]),
                "y": float(xyz_fit["rmse_by_axis"][1]),
                "z": float(xyz_fit["rmse_by_axis"][2]),
            },
            "rmse_total_mm": float(xyz_fit["rmse_total"]),
            "per_sample": _per_sample_rows(
                labels=labels,
                actual=robot_xyz_mm,
                predicted=xyz_fit["predicted"],
                residual=xyz_fit["residual"],
                row_error=xyz_fit["row_error"],
                axis_labels=("x", "y", "z"),
            ),
        },
    }

    with open(output_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")

    print(f"[PHASE3] Wrote calibration solution to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
