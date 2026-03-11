#!/usr/bin/env python3
"""
Standalone Phase 1 diagnostic for OAK-D stream identification.

This tool is intentionally isolated from the live task controller. It connects
to available OAK-D devices, reads the left mono feed used by the current
perception path, and reports ArUco detections per device so the top-down camera
can be identified before calibration work begins.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

try:
    import depthai as dai
except Exception as exc:  # pragma: no cover - depends on lab environment
    raise SystemExit(
        "depthai import failed. Activate the robotics environment before "
        f"running this probe: {exc}"
    )

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)

from aruco_tracker import ArucoTracker


DEFAULT_LABELS_BY_MXID = {
    "19443010B14C872F00": "INSPECTOR",
    "194430108183F12E00": "SITE_MANAGER",
}


@dataclass
class ProbeStats:
    label: str
    mxid: str
    frames: int = 0
    detection_frames: int = 0
    last_seen_ts: Optional[float] = None
    last_print_ts: float = 0.0
    markers_seen: set[int] = field(default_factory=set)
    last_pose_by_marker: dict[int, tuple[float, float, float, float, float, float]] = field(
        default_factory=dict
    )
    error: Optional[str] = None

    def detection_ratio(self) -> float:
        if self.frames == 0:
            return 0.0
        return self.detection_frames / float(self.frames)


def _build_pipeline(fps: float) -> dai.Pipeline:
    pipeline = dai.Pipeline()

    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_left.setFps(fps)

    xout_raw = pipeline.create(dai.node.XLinkOut)
    xout_raw.setStreamName("rawL")
    mono_left.out.link(xout_raw.input)

    return pipeline


def _resolve_label(mxid: str, index: int) -> str:
    return DEFAULT_LABELS_BY_MXID.get(mxid, f"OAK_{index}")


def _device_mxid(device_info: object, index: int) -> str:
    """
    DepthAI has exposed device identifiers differently across releases.
    Resolve the most specific available MXID without assuming one API shape.
    """
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


def _summarize_pose(pose: tuple[float, float, float, float, float, float]) -> str:
    x, y, z, roll, pitch, yaw = pose
    return (
        f"x={x:.3f}m y={y:.3f}m z={z:.3f}m "
        f"roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f}"
    )


def _print_summary(stats_list: list[ProbeStats]) -> None:
    print("\n[SUMMARY] Phase 1 probe results")
    for stats in stats_list:
        markers = sorted(stats.markers_seen)
        marker_text = ",".join(str(marker) for marker in markers) if markers else "none"
        print(
            f"[SUMMARY] {stats.label} mxid={stats.mxid} frames={stats.frames} "
            f"detection_frames={stats.detection_frames} "
            f"detection_ratio={stats.detection_ratio():.3f} markers={marker_text}"
        )
        if stats.error:
            print(f"[SUMMARY] {stats.label} error={stats.error}")

    ranked = sorted(stats_list, key=lambda item: item.detection_ratio(), reverse=True)
    if ranked and ranked[0].detection_frames > 0:
        winner = ranked[0]
        print(
            f"[SUMMARY] Most likely active marker-view camera: "
            f"{winner.label} ({winner.mxid})"
        )
    else:
        print("[SUMMARY] No markers were detected on any probed device.")


def _filter_poses(
    poses: dict[int, tuple[float, float, float, float, float, float]],
    marker_id: Optional[int],
) -> dict[int, tuple[float, float, float, float, float, float]]:
    if marker_id is None:
        return poses
    pose = poses.get(marker_id)
    return {} if pose is None else {marker_id: pose}


def _probe_device(
    device_info: dai.DeviceInfo,
    label: str,
    fps: float,
    marker_size_m: float,
    marker_id: Optional[int],
    detection_print_interval_s: float,
    heartbeat_interval_s: float,
    stop_event: threading.Event,
    stats: ProbeStats,
) -> None:
    tracker = ArucoTracker(marker_size=marker_size_m)
    pipeline = _build_pipeline(fps=fps)

    try:
        with dai.Device(pipeline, device_info) as device:
            try:
                device.setLogLevel(dai.LogLevel.CRITICAL)
            except Exception:
                pass

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

            print(
                f"[PHASE1] {label} mxid={stats.mxid} connected. "
                "Watching CAM_B mono feed for ArUco markers."
            )

            last_heartbeat_ts = time.time()
            while not stop_event.is_set():
                frame_packet = queue.get()
                frame = frame_packet.getCvFrame()
                if len(frame.shape) == 2:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                else:
                    frame_bgr = frame

                stats.frames += 1
                poses = tracker.compute_poses(frame_bgr, intrinsics, dist_coeffs)
                filtered = _filter_poses(poses, marker_id)

                now = time.time()
                if filtered:
                    stats.detection_frames += 1
                    stats.last_seen_ts = now
                    stats.markers_seen.update(filtered.keys())
                    stats.last_pose_by_marker.update(filtered)

                    if now - stats.last_print_ts >= detection_print_interval_s:
                        for detected_marker, pose in sorted(filtered.items()):
                            print(
                                f"[PHASE1] {label} marker={detected_marker} "
                                f"{_summarize_pose(pose)}"
                            )
                        stats.last_print_ts = now

                if now - last_heartbeat_ts >= heartbeat_interval_s:
                    print(
                        f"[PHASE1] {label} heartbeat frames={stats.frames} "
                        f"detection_frames={stats.detection_frames} "
                        f"ratio={stats.detection_ratio():.3f}"
                    )
                    last_heartbeat_ts = now
    except Exception as exc:
        stats.error = str(exc)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Phase 1 OAK-D camera probe for identifying the top-down ArUco stream."
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=20.0,
        help="How long to run the probe. Use 0 for infinite until Ctrl-C.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=20.0,
        help="Mono camera FPS for the probe pipeline.",
    )
    parser.add_argument(
        "--marker-size-m",
        type=float,
        default=0.0375,
        help="ArUco marker size in meters.",
    )
    parser.add_argument(
        "--marker-id",
        type=int,
        default=None,
        help="Optional specific marker id to track.",
    )
    parser.add_argument(
        "--detection-print-interval-s",
        type=float,
        default=1.0,
        help="Minimum time between detection printouts per camera.",
    )
    parser.add_argument(
        "--heartbeat-interval-s",
        type=float,
        default=5.0,
        help="How often to print a no-marker heartbeat per camera.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    available_devices = list(dai.Device.getAllAvailableDevices())
    if not available_devices:
        print("[PHASE1] No OAK-D devices found.")
        return 1

    print("[PHASE1] Available devices:")
    stats_list: list[ProbeStats] = []
    for index, device in enumerate(available_devices, start=1):
        mxid = _device_mxid(device, index)
        label = _resolve_label(mxid, index)
        print(f"[PHASE1]   {label} mxid={mxid}")
        stats_list.append(ProbeStats(label=label, mxid=mxid))

    stop_event = threading.Event()

    def _request_stop(_signum=None, _frame=None) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    threads: list[threading.Thread] = []
    for device, stats in zip(available_devices, stats_list):
        thread = threading.Thread(
            target=_probe_device,
            kwargs={
                "device_info": device,
                "label": stats.label,
                "fps": args.fps,
                "marker_size_m": args.marker_size_m,
                "marker_id": args.marker_id,
                "detection_print_interval_s": args.detection_print_interval_s,
                "heartbeat_interval_s": args.heartbeat_interval_s,
                "stop_event": stop_event,
                "stats": stats,
            },
            name=f"probe-{stats.label}",
            daemon=True,
        )
        threads.append(thread)
        thread.start()

    if args.duration_s > 0:
        stop_event.wait(timeout=args.duration_s)
        stop_event.set()
    else:
        while not stop_event.is_set():
            time.sleep(0.2)

    for thread in threads:
        thread.join(timeout=2.0)

    _print_summary(stats_list)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
