#!/usr/bin/env python3
"""
Standalone Phase 2 calibration capture tool.

Purpose:
- attach to the confirmed perception camera only
- track one ArUco marker continuously
- maintain a rolling buffer of recent camera-frame poses
- let the operator capture stable samples and pair them with manually-entered
  robot-frame coordinates

This file is intentionally isolated from the live task controller and motion
runtime. It is a perception-side lab utility for collecting calibration data.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
import warnings
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

import cv2
import numpy as np

try:
    import depthai as dai
except Exception as exc:  # pragma: no cover - depends on lab environment
    raise SystemExit(
        "depthai import failed. Activate the robotics environment before "
        f"running this tool: {exc}"
    )

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)

from aruco_tracker import ArucoTracker

warnings.filterwarnings("ignore", category=DeprecationWarning)

DEFAULT_TOPDOWN_DEVICE_ID = "169.254.1.223"
DEFAULT_TOPDOWN_LABEL = "OAK_2"


@dataclass
class LivePose:
    timestamp_s: float
    pose: tuple[float, float, float, float, float, float]


class CaptureSession:
    def __init__(self, buffer_size: int) -> None:
        self.buffer = deque(maxlen=buffer_size)
        self.lock = threading.Lock()
        self.frames = 0
        self.detection_frames = 0
        self.last_pose: Optional[LivePose] = None
        self.last_detection_ts: Optional[float] = None

    def record_frame(
        self,
        pose: Optional[tuple[float, float, float, float, float, float]],
        timestamp_s: float,
    ) -> None:
        with self.lock:
            self.frames += 1
            if pose is None:
                return
            self.detection_frames += 1
            live_pose = LivePose(timestamp_s=timestamp_s, pose=pose)
            self.last_pose = live_pose
            self.last_detection_ts = timestamp_s
            self.buffer.append(live_pose)

    def status(self) -> dict[str, object]:
        with self.lock:
            pose_count = len(self.buffer)
            detection_ratio = (
                0.0 if self.frames == 0 else self.detection_frames / float(self.frames)
            )
            payload: dict[str, object] = {
                "frames": self.frames,
                "detection_frames": self.detection_frames,
                "detection_ratio": detection_ratio,
                "buffer_count": pose_count,
                "last_detection_age_s": (
                    None
                    if self.last_detection_ts is None
                    else max(0.0, time.time() - self.last_detection_ts)
                ),
            }

            if self.last_pose is not None:
                payload["last_pose"] = self.last_pose.pose

            if pose_count > 0:
                payload["window_summary"] = _summarize_pose_window(
                    [sample.pose for sample in self.buffer]
                )

            return payload

    def capture_summary(self, min_samples: int) -> Optional[dict[str, object]]:
        with self.lock:
            if len(self.buffer) < min_samples:
                return None
            poses = [sample.pose for sample in self.buffer]
            time_span_s = self.buffer[-1].timestamp_s - self.buffer[0].timestamp_s
            summary = _summarize_pose_window(poses)
            summary["sample_count"] = len(poses)
            summary["time_span_s"] = max(0.0, time_span_s)
            summary["first_timestamp_s"] = self.buffer[0].timestamp_s
            summary["last_timestamp_s"] = self.buffer[-1].timestamp_s
            return summary


def _create_node(pipeline: dai.Pipeline, node_name: str):
    node_namespace = getattr(dai, "node", None)
    node_cls = getattr(node_namespace, node_name, None) if node_namespace is not None else None
    if node_cls is not None:
        return pipeline.create(node_cls)

    factory_name = f"create{node_name}"
    factory = getattr(pipeline, factory_name, None)
    if callable(factory):
        return factory()

    legacy_cls = getattr(dai, node_name, None)
    if legacy_cls is not None:
        return pipeline.create(legacy_cls)

    raise RuntimeError(f"DepthAI node '{node_name}' is unavailable in this environment.")


def _supports_v2_xlink() -> bool:
    node_namespace = getattr(dai, "node", None)
    if node_namespace is not None and getattr(node_namespace, "XLinkOut", None) is not None:
        return True
    return callable(getattr(dai.Pipeline, "createXLinkOut", None))


def _supports_v3_camera_api() -> bool:
    node_namespace = getattr(dai, "node", None)
    camera_cls = getattr(node_namespace, "Camera", None) if node_namespace is not None else None
    return camera_cls is not None and not _supports_v2_xlink()


def _device_mxid(device_info: object, index: int) -> str:
    getter = getattr(device_info, "getMxId", None)
    if callable(getter):
        mxid = getter()
        if mxid:
            return str(mxid)

    for attr_name in ("mxid", "mxId", "name"):
        value = getattr(device_info, attr_name, None)
        if value:
            return str(value)

    return f"UNKNOWN_{index}"


def _resolve_device(device_id: Optional[str]) -> tuple[dai.DeviceInfo, str]:
    available_devices = list(dai.Device.getAllAvailableDevices())
    if not available_devices:
        raise RuntimeError("No OAK-D devices found.")

    indexed = [
        (device, _device_mxid(device, index))
        for index, device in enumerate(available_devices, start=1)
    ]

    if device_id:
        for device, mxid in indexed:
            if mxid == device_id:
                return device, mxid
        available_text = ", ".join(mxid for _, mxid in indexed)
        raise RuntimeError(
            f"Requested device '{device_id}' was not found. Available devices: {available_text}"
        )

    if len(indexed) == 1:
        return indexed[0]

    for device, mxid in indexed:
        if mxid == DEFAULT_TOPDOWN_DEVICE_ID:
            return device, mxid

    available_text = ", ".join(mxid for _, mxid in indexed)
    raise RuntimeError(
        "Multiple OAK-D devices found. Pass --device-id explicitly. "
        f"Available devices: {available_text}"
    )


def _build_v2_pipeline(fps: float) -> dai.Pipeline:
    pipeline = dai.Pipeline()

    mono_left = _create_node(pipeline, "MonoCamera")
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_left.setFps(fps)

    xout_raw = _create_node(pipeline, "XLinkOut")
    xout_raw.setStreamName("rawL")
    mono_left.out.link(xout_raw.input)

    return pipeline


def _v3_pipeline_context(device_info: dai.DeviceInfo):
    device = dai.Device(device_info)
    return dai.Pipeline(device)


def _summarize_pose_window(
    poses: list[tuple[float, float, float, float, float, float]]
) -> dict[str, object]:
    array = np.array(poses, dtype=np.float64)
    median = np.median(array, axis=0)
    mean = np.mean(array, axis=0)
    std = np.std(array, axis=0)

    return {
        "median_pose": {
            "x_m": float(median[0]),
            "y_m": float(median[1]),
            "z_m": float(median[2]),
            "roll_rad": float(median[3]),
            "pitch_rad": float(median[4]),
            "yaw_rad": float(median[5]),
        },
        "mean_pose": {
            "x_m": float(mean[0]),
            "y_m": float(mean[1]),
            "z_m": float(mean[2]),
            "roll_rad": float(mean[3]),
            "pitch_rad": float(mean[4]),
            "yaw_rad": float(mean[5]),
        },
        "std_pose": {
            "x_m": float(std[0]),
            "y_m": float(std[1]),
            "z_m": float(std[2]),
            "roll_rad": float(std[3]),
            "pitch_rad": float(std[4]),
            "yaw_rad": float(std[5]),
        },
    }


def _format_pose_block(summary: dict[str, object]) -> str:
    median_pose = summary["median_pose"]
    std_pose = summary["std_pose"]
    return (
        "median="
        f"({median_pose['x_m']:.4f}, {median_pose['y_m']:.4f}, {median_pose['z_m']:.4f}, "
        f"{median_pose['roll_rad']:.4f}, {median_pose['pitch_rad']:.4f}, {median_pose['yaw_rad']:.4f}) "
        "std="
        f"({std_pose['x_m']:.4f}, {std_pose['y_m']:.4f}, {std_pose['z_m']:.4f}, "
        f"{std_pose['roll_rad']:.4f}, {std_pose['pitch_rad']:.4f}, {std_pose['yaw_rad']:.4f})"
    )


def _output_path(explicit_path: Optional[str]) -> str:
    if explicit_path:
        output_path = explicit_path
    else:
        out_dir = os.path.join(THIS_DIR, "calibration_data")
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output_path = os.path.join(out_dir, f"phase2_capture_{timestamp}.jsonl")

    output_dir = os.path.dirname(output_path) or "."
    os.makedirs(output_dir, exist_ok=True)
    return output_path


def _parse_robot_pose(raw: str) -> dict[str, Optional[float]]:
    parts = raw.strip().split()
    if len(parts) not in {3, 6}:
        raise ValueError("Enter either 3 values (x y z) or 6 values (x y z rx ry rz).")

    values = [float(part) for part in parts]
    payload = {
        "x_mm": values[0],
        "y_mm": values[1],
        "z_mm": values[2],
        "rx_deg": None,
        "ry_deg": None,
        "rz_deg": None,
    }
    if len(values) == 6:
        payload["rx_deg"] = values[3]
        payload["ry_deg"] = values[4]
        payload["rz_deg"] = values[5]
    return payload


def _capture_worker_v2(
    device_info: dai.DeviceInfo,
    fps: float,
    tracker: ArucoTracker,
    marker_id: int,
    session: CaptureSession,
    stop_event: threading.Event,
) -> None:
    pipeline = _build_v2_pipeline(fps=fps)
    with dai.Device(pipeline, device_info) as device:
        queue = device.getOutputQueue("rawL", maxSize=4, blocking=False)
        calib = device.readCalibration()
        intrinsics = np.array(
            calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 1280, 720),
            dtype=np.float64,
        )
        dist_coeffs = np.array(
            calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B),
            dtype=np.float64,
        )

        while not stop_event.is_set():
            frame_packet = queue.get()
            _process_packet(
                frame_packet=frame_packet,
                tracker=tracker,
                intrinsics=intrinsics,
                dist_coeffs=dist_coeffs,
                marker_id=marker_id,
                session=session,
            )


def _capture_worker_v3(
    device_info: dai.DeviceInfo,
    fps: float,
    tracker: ArucoTracker,
    marker_id: int,
    session: CaptureSession,
    stop_event: threading.Event,
) -> None:
    with _v3_pipeline_context(device_info) as pipeline:
        camera = _create_node(pipeline, "Camera").build(
            dai.CameraBoardSocket.CAM_B,
            sensorFps=fps,
        )
        output = camera.requestOutput((1280, 720), type=dai.ImgFrame.Type.GRAY8)
        queue = output.createOutputQueue()

        device = pipeline.getDefaultDevice()
        calib = device.readCalibration()
        intrinsics = np.array(
            calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 1280, 720),
            dtype=np.float64,
        )
        dist_coeffs = np.array(
            calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B),
            dtype=np.float64,
        )

        pipeline.start()
        try:
            while pipeline.isRunning() and not stop_event.is_set():
                frame_packet = queue.get()
                _process_packet(
                    frame_packet=frame_packet,
                    tracker=tracker,
                    intrinsics=intrinsics,
                    dist_coeffs=dist_coeffs,
                    marker_id=marker_id,
                    session=session,
                )
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass


def _process_packet(
    frame_packet,
    tracker: ArucoTracker,
    intrinsics: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_id: int,
    session: CaptureSession,
) -> None:
    frame = frame_packet.getCvFrame()
    if len(frame.shape) == 2:
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    else:
        frame_bgr = frame

    poses = tracker.compute_poses(frame_bgr, intrinsics, dist_coeffs)
    pose = poses.get(marker_id)
    session.record_frame(pose=pose, timestamp_s=time.time())


def _start_capture_thread(
    device_info: dai.DeviceInfo,
    fps: float,
    marker_id: int,
    session: CaptureSession,
    stop_event: threading.Event,
) -> threading.Thread:
    tracker = ArucoTracker()

    if _supports_v2_xlink():
        target = _capture_worker_v2
    elif _supports_v3_camera_api():
        target = _capture_worker_v3
    else:
        raise RuntimeError(
            "Unsupported DepthAI runtime: neither v2 XLink nor v3 Camera API is available."
        )

    thread = threading.Thread(
        target=target,
        kwargs={
            "device_info": device_info,
            "fps": fps,
            "tracker": tracker,
            "marker_id": marker_id,
            "session": session,
            "stop_event": stop_event,
        },
        name="phase2-capture",
        daemon=True,
    )
    thread.start()
    return thread


def _interactive_loop(
    session: CaptureSession,
    output_path: str,
    device_id: str,
    device_label: str,
    marker_id: int,
    min_samples: int,
) -> int:
    sample_index = 0
    print("[PHASE2] Commands: status | capture | quit")

    with open(output_path, "a", encoding="utf-8") as handle:
        while True:
            try:
                command = input("phase2> ").strip().lower()
            except EOFError:
                print()
                break

            if command in {"quit", "q", "exit"}:
                break

            if command in {"status", "s"}:
                status = session.status()
                print(
                    "[PHASE2] status "
                    f"frames={status['frames']} "
                    f"detection_frames={status['detection_frames']} "
                    f"ratio={status['detection_ratio']:.3f} "
                    f"buffer={status['buffer_count']} "
                    f"last_detection_age_s={status['last_detection_age_s']}"
                )
                if "window_summary" in status:
                    print(
                        "[PHASE2] "
                        + _format_pose_block(status["window_summary"])
                    )
                continue

            if command in {"capture", "c", ""}:
                summary = session.capture_summary(min_samples=min_samples)
                if summary is None:
                    status = session.status()
                    print(
                        "[PHASE2] Not enough buffered detections to capture yet. "
                        f"buffer={status['buffer_count']} min_required={min_samples}"
                    )
                    continue

                print("[PHASE2] Camera window ready.")
                print("[PHASE2] " + _format_pose_block(summary))

                label = input("sample label: ").strip() or f"sample_{sample_index + 1:02d}"

                while True:
                    robot_raw = input(
                        "robot pose mm/deg (x y z [rx ry rz]): "
                    ).strip()
                    try:
                        robot_pose = _parse_robot_pose(robot_raw)
                        break
                    except ValueError as exc:
                        print(f"[PHASE2] {exc}")

                notes = input("notes (optional): ").strip()

                record = {
                    "sample_index": sample_index,
                    "sample_label": label,
                    "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                    "device_id": device_id,
                    "device_label": device_label,
                    "marker_id": marker_id,
                    "camera_window": summary,
                    "robot_pose": robot_pose,
                    "notes": notes,
                }
                handle.write(json.dumps(record) + "\n")
                handle.flush()

                sample_index += 1
                print(
                    f"[PHASE2] Saved sample {sample_index} to {output_path}"
                )
                continue

            print("[PHASE2] Unknown command. Use: status | capture | quit")

    return sample_index


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Phase 2 calibration capture tool for camera-to-robot pairing."
    )
    parser.add_argument(
        "--device-id",
        default=DEFAULT_TOPDOWN_DEVICE_ID,
        help="Device identifier from Phase 1. Defaults to the confirmed top-down camera.",
    )
    parser.add_argument(
        "--device-label",
        default=DEFAULT_TOPDOWN_LABEL,
        help="Human-readable label stored in captured records.",
    )
    parser.add_argument(
        "--marker-id",
        type=int,
        default=0,
        help="Marker id to capture. Defaults to the confirmed calibration marker 0.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=20.0,
        help="Requested camera FPS.",
    )
    parser.add_argument(
        "--buffer-size",
        type=int,
        default=60,
        help="Rolling detection buffer size used for median/std summaries.",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=25,
        help="Minimum buffered detections required before a capture is allowed.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional output JSONL path. Defaults under Robotics/perception/calibration_data/.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    output_path = _output_path(args.output)
    device_info, resolved_device_id = _resolve_device(args.device_id)

    print(
        f"[PHASE2] Using device {resolved_device_id} "
        f"label={args.device_label} marker_id={args.marker_id}"
    )
    print(f"[PHASE2] Writing captures to {output_path}")

    session = CaptureSession(buffer_size=args.buffer_size)
    stop_event = threading.Event()
    thread = _start_capture_thread(
        device_info=device_info,
        fps=args.fps,
        marker_id=args.marker_id,
        session=session,
        stop_event=stop_event,
    )

    def _request_stop(_signum=None, _frame=None) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    time.sleep(1.0)
    if not thread.is_alive():
        stop_event.set()
        thread.join(timeout=1.0)
        raise RuntimeError("Capture thread failed to start. Check the selected device and DepthAI runtime.")

    sample_count = 0
    try:
        sample_count = _interactive_loop(
            session=session,
            output_path=output_path,
            device_id=resolved_device_id,
            device_label=args.device_label,
            marker_id=args.marker_id,
            min_samples=args.min_samples,
        )
    finally:
        stop_event.set()
        thread.join(timeout=2.0)

    print(f"[PHASE2] Finished. Saved {sample_count} samples to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
