#!/usr/bin/env python3
"""
Build and optionally solve a pickup-plane calibration dataset from runtime logs.

This tool pairs:
- marker_positions.jsonl observations from the top-down camera
- block_state.jsonl deterministic pick ground truth

For each deterministic pick, it finds the most recent observed marker pose for the
same target within a short age window and writes a Phase-3-compatible JSONL file.
If enough pairs exist, it also solves a planar affine fit immediately.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from typing import Any

MOTION_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "motion"))
if MOTION_DIR not in sys.path:
    sys.path.append(MOTION_DIR)

from data_lineage import current_data_lineage_tag, tagged_log_stream_path, tagged_path


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


def _solver_api():
    from phase3_calibration_solver import (
        _extract_arrays,
        _fit_affine,
        _load_records,
        _per_sample_rows,
        _serialise_matrix,
    )

    return {
        "extract_arrays": _extract_arrays,
        "fit_affine": _fit_affine,
        "load_records": _load_records,
        "per_sample_rows": _per_sample_rows,
        "serialise_matrix": _serialise_matrix,
    }


def _parse_ts(ts_utc: str) -> datetime:
    if ts_utc.endswith("Z"):
        ts_utc = ts_utc[:-1] + "+00:00"
    return datetime.fromisoformat(ts_utc)


def _runtime_scope(block_rows: list[dict[str, Any]], participant_name: str | None, run_id: str | None, session_id: str | None) -> tuple[str | None, str | None, str | None]:
    filtered = [row for row in block_rows if row.get("event") == "block_inferred_picked"]
    if participant_name:
        filtered = [row for row in filtered if row.get("participant_name") == participant_name]
    if run_id:
        filtered = [row for row in filtered if row.get("run_id") == run_id]
    if session_id:
        filtered = [row for row in filtered if row.get("session_id") == session_id]
    if not filtered:
        return participant_name, run_id, session_id
    filtered.sort(key=lambda row: row.get("ts_utc", ""))
    latest = filtered[-1]
    return (
        str(latest.get("participant_name")) if latest.get("participant_name") is not None else participant_name,
        str(latest.get("run_id")) if latest.get("run_id") is not None else run_id,
        str(latest.get("session_id")) if latest.get("session_id") is not None else session_id,
    )


def _filter_rows(rows: list[dict[str, Any]], participant_name: str | None, run_id: str | None, session_id: str | None) -> list[dict[str, Any]]:
    filtered = rows
    if participant_name is not None:
        filtered = [row for row in filtered if row.get("participant_name") == participant_name]
    if run_id is not None:
        filtered = [row for row in filtered if row.get("run_id") == run_id]
    if session_id is not None:
        filtered = [row for row in filtered if row.get("session_id") == session_id]
    return filtered


def _sample_label(participant_name: str | None, target_id: str, pick_ts_utc: str) -> str:
    prefix = participant_name or "runtime"
    safe_ts = pick_ts_utc.replace(":", "").replace("-", "").replace(".", "")
    return f"{prefix}_{target_id}_{safe_ts}"


def _build_capture_rows(
    marker_rows: list[dict[str, Any]],
    block_rows: list[dict[str, Any]],
    max_age_s: float,
) -> tuple[list[dict[str, Any]], list[str]]:
    observation_by_target: dict[str, list[dict[str, Any]]] = {}
    for row in marker_rows:
        if row.get("event") != "marker_pose_changed":
            continue
        target_id = row.get("target_id")
        camera_pose = row.get("camera_pose")
        if not isinstance(target_id, str) or not isinstance(camera_pose, dict):
            continue
        observation_by_target.setdefault(target_id, []).append(row)

    for entries in observation_by_target.values():
        entries.sort(key=lambda row: row.get("ts_utc", ""))

    capture_rows: list[dict[str, Any]] = []
    misses: list[str] = []
    for block_row in block_rows:
        if block_row.get("event") != "block_inferred_picked":
            continue
        target_id = block_row.get("target_id")
        pick_pose = block_row.get("pick_pose")
        pick_ts_utc = block_row.get("ts_utc")
        if not isinstance(target_id, str) or not isinstance(pick_pose, list) or not isinstance(pick_ts_utc, str):
            continue

        pick_ts = _parse_ts(pick_ts_utc)
        candidates = observation_by_target.get(target_id, [])
        match: dict[str, Any] | None = None
        for candidate in reversed(candidates):
            obs_ts_utc = candidate.get("ts_utc")
            if not isinstance(obs_ts_utc, str):
                continue
            obs_ts = _parse_ts(obs_ts_utc)
            age_s = (pick_ts - obs_ts).total_seconds()
            if age_s < 0:
                continue
            if age_s <= max_age_s:
                match = candidate
                break

        if match is None:
            misses.append(target_id)
            continue

        camera_pose = dict(match["camera_pose"])
        capture_rows.append(
            {
                "sample_label": _sample_label(block_row.get("participant_name"), target_id, pick_ts_utc),
                "sample_type": "runtime_pick_pair",
                "participant_name": block_row.get("participant_name"),
                "run_id": block_row.get("run_id"),
                "session_id": block_row.get("session_id"),
                "target_id": target_id,
                "marker_id": match.get("marker_id"),
                "pick_ts_utc": pick_ts_utc,
                "marker_ts_utc": match.get("ts_utc"),
                "observation_age_s": round((pick_ts - _parse_ts(str(match["ts_utc"]))).total_seconds(), 6),
                "camera_window": {
                    "median_pose": {
                        "x_m": float(camera_pose["x"]),
                        "y_m": float(camera_pose["y"]),
                        "z_m": float(camera_pose["z"]),
                        "roll_rad": float(camera_pose["roll"]),
                        "pitch_rad": float(camera_pose["pitch"]),
                        "yaw_rad": float(camera_pose["yaw"]),
                    },
                },
                "robot_pose": {
                    "x_mm": float(pick_pose[0]),
                    "y_mm": float(pick_pose[1]),
                    "z_mm": float(pick_pose[2]),
                    "rx_deg": float(pick_pose[3]),
                    "ry_deg": float(pick_pose[4]),
                    "rz_deg": float(pick_pose[5]),
                },
            }
        )

    return capture_rows, misses


def _write_jsonl(path: str, rows: list[dict[str, Any]]) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True))
            handle.write("\n")


def _solve_capture(input_jsonl: str, output_json: str) -> dict[str, Any]:
    solver_api = _solver_api()
    records = solver_api["load_records"](input_jsonl)
    data = solver_api["extract_arrays"](records)
    labels: list[str] = data["labels"]
    camera_xyz_mm = data["camera_xyz_mm"]
    robot_xyz_mm = data["robot_xyz_mm"]

    xy_fit = solver_api["fit_affine"](inputs=camera_xyz_mm[:, :2], outputs=robot_xyz_mm[:, :2])
    payload = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "data_lineage_tag": current_data_lineage_tag() or None,
        "input_jsonl": os.path.abspath(input_jsonl),
        "sample_count": len(records),
        "sample_labels": labels,
        "planar_xy_affine": {
            "description": "Runtime-derived planar fit from marker_positions + block_state logs.",
            "input_axes_mm": ["camera_x_mm", "camera_y_mm", "bias"],
            "output_axes_mm": ["robot_x_mm", "robot_y_mm"],
            "coefficients": solver_api["serialise_matrix"](xy_fit["coefficients"]),
            "rmse_by_axis_mm": {
                "x": float(xy_fit["rmse_by_axis"][0]),
                "y": float(xy_fit["rmse_by_axis"][1]),
            },
            "rmse_total_mm": float(xy_fit["rmse_total"]),
            "per_sample": solver_api["per_sample_rows"](
                labels=labels,
                actual=robot_xyz_mm[:, :2],
                predicted=xy_fit["predicted"],
                residual=xy_fit["residual"],
                row_error=xy_fit["row_error"],
                axis_labels=("x", "y"),
            ),
        },
    }
    with open(output_json, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
    return payload


def _default_capture_path(participant_name: str | None) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    label = participant_name or "runtime"
    return tagged_path(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "calibration_data",
        f"{label}_runtime_pick_pairs_{stamp}.jsonl",
    ))


def _default_solution_path(input_jsonl: str) -> str:
    directory = os.path.dirname(input_jsonl) or "."
    stem = os.path.splitext(os.path.basename(input_jsonl))[0]
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return tagged_path(os.path.join(directory, f"{stem}_phase3_solution_{timestamp}.json"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fit pickup-plane calibration from runtime logs.")
    parser.add_argument("--marker-log", default=tagged_log_stream_path(os.path.join("Robotics", "perception", "logs"), "marker_positions"))
    parser.add_argument("--block-log", default=tagged_log_stream_path(os.path.join("Robotics", "perception", "logs"), "block_state"))
    parser.add_argument("--participant-name", default=None, help="Optional participant label. Defaults to latest available run.")
    parser.add_argument("--run-id", default=None, help="Optional run id filter.")
    parser.add_argument("--session-id", default=None, help="Optional session id filter.")
    parser.add_argument("--max-age-s", type=float, default=2.0, help="Maximum allowed marker-to-pick age for pairing.")
    parser.add_argument("--min-samples", type=int, default=4, help="Minimum matched pairs required to solve.")
    parser.add_argument("--output-jsonl", default=None, help="Optional output JSONL path for matched runtime pairs.")
    parser.add_argument("--output-solution", default=None, help="Optional solved calibration JSON path.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    marker_rows = _load_jsonl(args.marker_log)
    block_rows = _load_jsonl(args.block_log)

    participant_name, run_id, session_id = _runtime_scope(
        block_rows=block_rows,
        participant_name=args.participant_name,
        run_id=args.run_id,
        session_id=args.session_id,
    )
    marker_rows = _filter_rows(marker_rows, participant_name=participant_name, run_id=run_id, session_id=session_id)
    block_rows = _filter_rows(block_rows, participant_name=participant_name, run_id=run_id, session_id=session_id)

    capture_rows, misses = _build_capture_rows(marker_rows=marker_rows, block_rows=block_rows, max_age_s=float(args.max_age_s))
    output_jsonl = args.output_jsonl or _default_capture_path(participant_name)
    os.makedirs(os.path.dirname(output_jsonl), exist_ok=True)
    _write_jsonl(output_jsonl, capture_rows)

    print("[RUNTIME_FIT] scope")
    print(f"[RUNTIME_FIT]   participant_name={participant_name}")
    print(f"[RUNTIME_FIT]   session_id={session_id}")
    print(f"[RUNTIME_FIT]   run_id={run_id}")
    print(f"[RUNTIME_FIT]   max_age_s={float(args.max_age_s):.3f}")
    print(f"[RUNTIME_FIT] matched_pairs={len(capture_rows)}")
    print(f"[RUNTIME_FIT] unmatched_targets={','.join(misses) if misses else 'none'}")
    print(f"[RUNTIME_FIT] output_jsonl={output_jsonl}")

    if len(capture_rows) < int(args.min_samples):
        print(
            f"[RUNTIME_FIT] insufficient pairs to solve "
            f"(need {int(args.min_samples)}, have {len(capture_rows)})."
        )
        return 0

    output_solution = args.output_solution or _default_solution_path(output_jsonl)
    solution = _solve_capture(input_jsonl=output_jsonl, output_json=output_solution)
    rmse_total_mm = float(solution["planar_xy_affine"]["rmse_total_mm"])
    print(f"[RUNTIME_FIT] solved_planar_rmse_total_mm={rmse_total_mm:.3f}")
    print(f"[RUNTIME_FIT] output_solution={output_solution}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
