#!/usr/bin/env python3
"""
Capture camera/robot XY pairs for arbitrary pickup-plane recalibration.

Workflow:
- place a tagged block anywhere on the pickup plane
- manually jog the robot TCP over the intended grasp point for that block
- capture a stable marker window and the live robot TCP pose together

The output JSONL intentionally matches the Phase 2 schema so the existing
phase3_calibration_solver.py can solve a new affine fit directly.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Optional

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MOTION_DIR = os.path.normpath(os.path.join(THIS_DIR, "..", "motion"))

if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)
if MOTION_DIR not in sys.path:
    sys.path.append(MOTION_DIR)

from dobot_driver import DobotDriver
import robot_config as cfg
import vision_bridge
from phase2_calibration_capture import (
    DEFAULT_TOPDOWN_DEVICE_ID,
    DEFAULT_TOPDOWN_LABEL,
    CaptureSession,
    _format_pose_block,
    _resolve_device,
    _start_capture_thread,
)


def _timestamp_utc() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _default_output_path(explicit_path: Optional[str]) -> str:
    if explicit_path:
        output_path = explicit_path
    else:
        out_dir = os.path.join(THIS_DIR, "calibration_data")
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output_path = os.path.join(out_dir, f"pick_plane_capture_{timestamp}.jsonl")
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    return output_path


def _robot_pose_payload(tcp_pose: tuple[float, float, float, float, float, float]) -> dict[str, float]:
    return {
        "x_mm": float(tcp_pose[0]),
        "y_mm": float(tcp_pose[1]),
        "z_mm": float(tcp_pose[2]),
        "rx_deg": float(tcp_pose[3]),
        "ry_deg": float(tcp_pose[4]),
        "rz_deg": float(tcp_pose[5]),
    }


def _projection_summary(summary: dict[str, object], robot_pose: dict[str, float]) -> dict[str, object]:
    median_pose = summary["median_pose"]
    robot_xy, reason = vision_bridge.camera_xy_to_robot_xy_mm(
        float(median_pose["x_m"]),
        float(median_pose["y_m"]),
    )
    if robot_xy is None:
        return {"available": False, "reason": reason}

    pred_x = float(robot_xy[0])
    pred_y = float(robot_xy[1])
    err_x = pred_x - float(robot_pose["x_mm"])
    err_y = pred_y - float(robot_pose["y_mm"])
    return {
        "available": True,
        "reason": "ok",
        "predicted_robot_xy_mm": {
            "x": round(pred_x, 3),
            "y": round(pred_y, 3),
        },
        "residual_to_live_tcp_mm": {
            "x": round(err_x, 3),
            "y": round(err_y, 3),
            "norm": round((err_x ** 2 + err_y ** 2) ** 0.5, 3),
        },
    }


def _load_existing_records(output_path: str) -> list[dict[str, object]]:
    if not os.path.exists(output_path):
        return []

    records: list[dict[str, object]] = []
    with open(output_path, "r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                payload = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"Invalid JSON in existing capture file {output_path} line {line_number}: {exc}"
                ) from exc
            if isinstance(payload, dict):
                records.append(payload)
    return records


def _next_sample_index(records: list[dict[str, object]]) -> int:
    next_index = 0
    for record in records:
        raw_index = record.get("sample_index")
        if raw_index is None:
            continue
        try:
            next_index = max(next_index, int(raw_index) + 1)
        except (TypeError, ValueError):
            continue
    return next_index


def _dataset_summary(records: list[dict[str, object]]) -> Optional[str]:
    robot_x: list[float] = []
    robot_y: list[float] = []
    camera_x: list[float] = []
    camera_y: list[float] = []

    for record in records:
        robot_pose = record.get("robot_pose")
        if isinstance(robot_pose, dict):
            x_mm = robot_pose.get("x_mm")
            y_mm = robot_pose.get("y_mm")
            if x_mm is not None and y_mm is not None:
                robot_x.append(float(x_mm))
                robot_y.append(float(y_mm))

        camera_window = record.get("camera_window")
        if isinstance(camera_window, dict):
            median_pose = camera_window.get("median_pose")
            if isinstance(median_pose, dict):
                x_m = median_pose.get("x_m")
                y_m = median_pose.get("y_m")
                if x_m is not None and y_m is not None:
                    camera_x.append(float(x_m) * 1000.0)
                    camera_y.append(float(y_m) * 1000.0)

    if not robot_x:
        return None

    parts = [
        f"samples={len(records)}",
        f"robot_xy_span_mm=x[{min(robot_x):.1f},{max(robot_x):.1f}]",
        f"y[{min(robot_y):.1f},{max(robot_y):.1f}]",
    ]
    if camera_x and camera_y:
        parts.extend(
            [
                f"camera_xy_span_mm=x[{min(camera_x):.1f},{max(camera_x):.1f}]",
                f"y[{min(camera_y):.1f},{max(camera_y):.1f}]",
            ]
        )
    return " ".join(parts)


def _wait_for_stable_summary(
    session: CaptureSession,
    marker_id: int,
    min_samples: int,
    max_xy_std_mm: float,
    timeout_s: float,
) -> dict[str, object]:
    deadline = time.time() + float(timeout_s)
    last_status_print = 0.0

    while time.time() < deadline:
        summary = session.capture_summary(marker_id=marker_id, min_samples=min_samples)
        std_x_mm = None
        std_y_mm = None
        if summary is not None:
            std_pose = summary["std_pose"]
            std_x_mm = float(std_pose["x_m"]) * 1000.0
            std_y_mm = float(std_pose["y_m"]) * 1000.0
            if std_x_mm <= max_xy_std_mm and std_y_mm <= max_xy_std_mm:
                return summary

        now = time.time()
        if (now - last_status_print) >= 1.0:
            status = session.status()
            marker_status = status["markers"].get(marker_id, {})
            print(
                "[PICK_CAL] "
                f"waiting marker={marker_id} frames={status['frames']} "
                f"buffer={marker_status.get('buffer_count')} "
                f"ratio={float(marker_status.get('detection_ratio', 0.0)):.3f} "
                f"last_detection_age_s={marker_status.get('last_detection_age_s')} "
                f"std_xy_mm=({std_x_mm if std_x_mm is not None else 'n/a'}, "
                f"{std_y_mm if std_y_mm is not None else 'n/a'})"
            )
            last_status_print = now

        time.sleep(0.1)

    raise RuntimeError(
        f"Timed out waiting for stable marker {marker_id} detection window "
        f"(min_samples={min_samples}, max_xy_std_mm={max_xy_std_mm})."
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture stable marker observations paired with live Dobot TCP poses "
            "for arbitrary pickup-plane recalibration."
        )
    )
    parser.add_argument("--marker-id", type=int, required=True, help="Marker id to calibrate from.")
    parser.add_argument(
        "--device-id",
        default=DEFAULT_TOPDOWN_DEVICE_ID,
        help="DepthAI device id for the top-down inspector camera.",
    )
    parser.add_argument(
        "--device-label",
        default=DEFAULT_TOPDOWN_LABEL,
        help="Human-readable camera label stored in captures.",
    )
    parser.add_argument("--fps", type=float, default=20.0, help="Requested camera FPS.")
    parser.add_argument("--buffer-size", type=int, default=60, help="Rolling detection buffer size.")
    parser.add_argument(
        "--min-samples",
        type=int,
        default=20,
        help="Minimum buffered detections required before a capture is allowed.",
    )
    parser.add_argument(
        "--max-xy-std-mm",
        type=float,
        default=6.5,
        help="Maximum allowed camera-frame X/Y std in mm for a stable capture.",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=30.0,
        help="Maximum wait time for a stable marker window during capture.",
    )
    parser.add_argument(
        "--participant-name",
        default="pick_plane_calibration",
        help="Run label stored in the capture records.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional output JSONL path. Defaults under Robotics/perception/calibration_data/.",
    )
    return parser.parse_args()


def _print_robot_pose(robot: DobotDriver) -> None:
    tcp_pose = robot.get_tcp_pose()
    if tcp_pose is None:
        print("[PICK_CAL] Failed to read live TCP pose.")
        return
    print(
        "[PICK_CAL] tcp_pose_mm_deg="
        f"({tcp_pose[0]:.3f}, {tcp_pose[1]:.3f}, {tcp_pose[2]:.3f}, "
        f"{tcp_pose[3]:.3f}, {tcp_pose[4]:.3f}, {tcp_pose[5]:.3f})"
    )


def _interactive_loop(
    session: CaptureSession,
    robot: DobotDriver,
    output_path: str,
    device_id: str,
    device_label: str,
    marker_id: int,
    participant_name: str,
    min_samples: int,
    max_xy_std_mm: float,
    timeout_s: float,
) -> int:
    records = _load_existing_records(output_path)
    sample_index = _next_sample_index(records)
    print("[PICK_CAL] Commands: status | pose | summary | capture | quit")
    print("[PICK_CAL] Align the TCP over the intended grasp point before capture.")
    if records:
        summary = _dataset_summary(records)
        print(
            f"[PICK_CAL] Resuming existing capture file with {len(records)} samples. "
            f"next_sample_index={sample_index}"
        )
        if summary is not None:
            print(f"[PICK_CAL] dataset {summary}")

    with open(output_path, "a", encoding="utf-8") as handle:
        while True:
            try:
                command = input("pickcal> ").strip().lower()
            except EOFError:
                print()
                break

            if command in {"quit", "q", "exit"}:
                break

            if command in {"status", "s"}:
                status = session.status()
                marker_status = status["markers"].get(marker_id, {})
                print(f"[PICK_CAL] status frames={status['frames']} marker={marker_id}")
                print(
                    "[PICK_CAL] "
                    f"detection_frames={marker_status.get('detection_frames')} "
                    f"ratio={float(marker_status.get('detection_ratio', 0.0)):.3f} "
                    f"buffer={marker_status.get('buffer_count')} "
                    f"last_detection_age_s={marker_status.get('last_detection_age_s')}"
                )
                if "window_summary" in marker_status:
                    print("[PICK_CAL] " + _format_pose_block(marker_status["window_summary"]))
                continue

            if command in {"pose", "p"}:
                _print_robot_pose(robot)
                continue

            if command in {"summary", "sum"}:
                summary = _dataset_summary(records)
                if summary is None:
                    print("[PICK_CAL] No saved samples yet.")
                else:
                    print(f"[PICK_CAL] dataset {summary}")
                continue

            if command in {"capture", "c", ""}:
                summary = _wait_for_stable_summary(
                    session=session,
                    marker_id=marker_id,
                    min_samples=min_samples,
                    max_xy_std_mm=max_xy_std_mm,
                    timeout_s=timeout_s,
                )
                print("[PICK_CAL] stable summary " + _format_pose_block(summary))

                tcp_pose = robot.get_tcp_pose()
                if tcp_pose is None:
                    print("[PICK_CAL] Failed to read live TCP pose. Capture aborted.")
                    continue

                robot_pose = _robot_pose_payload(tcp_pose)
                print(
                    "[PICK_CAL] live tcp_pose_mm_deg="
                    f"({robot_pose['x_mm']:.3f}, {robot_pose['y_mm']:.3f}, {robot_pose['z_mm']:.3f}, "
                    f"{robot_pose['rx_deg']:.3f}, {robot_pose['ry_deg']:.3f}, {robot_pose['rz_deg']:.3f})"
                )

                sample_label = input("sample label: ").strip() or f"pick_plane_{sample_index + 1:02d}"
                notes = input("notes (optional): ").strip()

                projection = _projection_summary(summary, robot_pose)
                if projection.get("available"):
                    residual = projection["residual_to_live_tcp_mm"]
                    predicted = projection["predicted_robot_xy_mm"]
                    print(
                        "[PICK_CAL] current_projection "
                        f"pred=({predicted['x']:.3f}, {predicted['y']:.3f}) "
                        f"residual=({residual['x']:.3f}, {residual['y']:.3f}) "
                        f"norm={residual['norm']:.3f}mm"
                    )
                else:
                    print(f"[PICK_CAL] current_projection unavailable reason={projection.get('reason')}")

                record = {
                    "sample_index": sample_index,
                    "sample_label": sample_label,
                    "timestamp_utc": _timestamp_utc(),
                    "device_id": device_id,
                    "device_label": device_label,
                    "marker_id": int(marker_id),
                    "participant_name": participant_name,
                    "camera_window": summary,
                    "robot_pose": robot_pose,
                    "projection_check": projection,
                    "notes": notes,
                }
                handle.write(json.dumps(record) + "\n")
                handle.flush()
                records.append(record)

                sample_index += 1
                print(f"[PICK_CAL] Saved sample {sample_index} to {output_path}")
                dataset_summary = _dataset_summary(records)
                if dataset_summary is not None:
                    print(f"[PICK_CAL] dataset {dataset_summary}")
                continue

            print("[PICK_CAL] Unknown command. Use: status | pose | summary | capture | quit")

    return sample_index


def main() -> int:
    args = parse_args()

    output_path = _default_output_path(args.output)
    device_info, resolved_device_id = _resolve_device(args.device_id)

    print(
        f"[PICK_CAL] Using device {resolved_device_id} label={args.device_label} "
        f"marker_id={int(args.marker_id)}"
    )
    print(f"[PICK_CAL] Writing captures to {output_path}")

    session = CaptureSession(buffer_size=int(args.buffer_size), marker_ids=[int(args.marker_id)])
    stop_event = threading.Event()
    capture_thread = _start_capture_thread(
        device_info=device_info,
        fps=float(args.fps),
        session=session,
        stop_event=stop_event,
    )

    robot = DobotDriver(
        robot_ip=cfg.ROBOT_IP,
        dashboard_port=cfg.DASHBOARD_PORT,
        timeout_s=cfg.SOCKET_TIMEOUT_S,
    )
    robot.connect()

    def _request_stop(_signum=None, _frame=None) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    time.sleep(1.0)
    if not capture_thread.is_alive():
        stop_event.set()
        capture_thread.join(timeout=1.0)
        robot.close()
        raise RuntimeError("Capture thread failed to start. Check the selected camera/device.")

    sample_count = 0
    try:
        sample_count = _interactive_loop(
            session=session,
            robot=robot,
            output_path=output_path,
            device_id=resolved_device_id,
            device_label=args.device_label,
            marker_id=int(args.marker_id),
            participant_name=str(args.participant_name),
            min_samples=int(args.min_samples),
            max_xy_std_mm=float(args.max_xy_std_mm),
            timeout_s=float(args.timeout_s),
        )
    finally:
        stop_event.set()
        capture_thread.join(timeout=2.0)
        robot.close()

    print(f"[PICK_CAL] Finished. Saved {sample_count} samples to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
