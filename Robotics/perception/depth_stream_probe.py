#!/usr/bin/env python3
"""
Perception-side diagnostic for current depth-stream availability.

What this checks:
- which OAK devices are currently visible to DepthAI
- which runtime ports/camera ids the controller is configured to use
- whether a local StereoDepth pipeline can actually receive depth frames now

Safety boundary:
- perception only
- no robot motion
- no Unity command traffic
"""

from __future__ import annotations

import argparse
import os
import re
import socket
import sys
import time
import warnings
from pathlib import Path
from typing import Any

import numpy as np

try:
    import depthai as dai
except Exception as exc:  # pragma: no cover - depends on lab environment
    raise SystemExit(
        "depthai import failed. Activate the robotics environment before "
        f"running this probe: {exc}"
    )

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MOTION_DIR = os.path.normpath(os.path.join(THIS_DIR, "..", "motion"))

if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)
if MOTION_DIR not in sys.path:
    sys.path.append(MOTION_DIR)

import robot_config as cfg
from phase2_calibration_capture import (
    _create_node,
    _supports_v2_xlink,
    _supports_v3_camera_api,
)

warnings.filterwarnings("ignore", category=DeprecationWarning)

STREAM_WIDTH = 1280
STREAM_HEIGHT = 720


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Probe current OAK hardware and report whether depth frames are "
            "actually available through the repo's perception-side StereoDepth path."
        )
    )
    parser.add_argument(
        "--device-id",
        default=None,
        help=(
            "Optional explicit OAK device id/MXID to probe. If omitted, the script "
            "enumerates and probes every currently visible device."
        ),
    )
    parser.add_argument("--fps", type=float, default=20.0, help="Probe pipeline FPS.")
    parser.add_argument(
        "--duration-s",
        type=float,
        default=5.0,
        help="How long to sample each device for depth frames.",
    )
    parser.add_argument(
        "--min-valid-depth-mm",
        type=float,
        default=50.0,
        help="Minimum depth counted as valid for statistics.",
    )
    parser.add_argument(
        "--max-valid-depth-mm",
        type=float,
        default=2500.0,
        help="Maximum depth counted as valid for statistics.",
    )
    return parser.parse_args()


def _task_controller_port_map() -> dict[str, int | None]:
    task_controller_path = Path(MOTION_DIR) / "task_controller.py"
    payload = {
        "UNITY_PORT_INSPECTOR": None,
        "UNITY_PORT_MANAGER": None,
        "UNITY_PORT_COMMANDS": None,
    }
    if not task_controller_path.exists():
        return payload

    text = task_controller_path.read_text(encoding="utf-8")
    for key in list(payload.keys()):
        match = re.search(rf"^{key}\s*=\s*(\d+)", text, flags=re.MULTILINE)
        if match:
            payload[key] = int(match.group(1))
    return payload


def _guess_local_ipv4() -> str | None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("192.0.2.1", 80))
        host_ip = sock.getsockname()[0]
        if host_ip and host_ip != "0.0.0.0":
            return str(host_ip)
    except Exception:
        return None
    finally:
        sock.close()
    return None


def _device_name(device_info: object) -> str | None:
    value = getattr(device_info, "name", None)
    if value:
        return str(value)
    return None


def _device_identifiers(device_info: object, index: int) -> tuple[str, ...]:
    identifiers: set[str] = set()

    getter = getattr(device_info, "getMxId", None)
    if callable(getter):
        try:
            value = getter()
            if value:
                identifiers.add(str(value))
        except Exception:
            pass

    for attr_name in ("mxid", "mxId", "name"):
        value = getattr(device_info, attr_name, None)
        if value:
            identifiers.add(str(value))

    if not identifiers:
        identifiers.add(f"UNKNOWN_{index}")

    return tuple(sorted(identifiers))


def _primary_device_id(device_info: object, index: int) -> str:
    getter = getattr(device_info, "getMxId", None)
    if callable(getter):
        try:
            value = getter()
            if value:
                return str(value)
        except Exception:
            pass

    for attr_name in ("mxid", "mxId", "name"):
        value = getattr(device_info, attr_name, None)
        if value:
            return str(value)

    identifiers = _device_identifiers(device_info, index)
    return str(identifiers[0])


def _role_match_details(identifiers: tuple[str, ...]) -> dict[str, tuple[str, ...]]:
    identifier_set = {str(value) for value in identifiers}
    inspector_matches = tuple(
        str(value) for value in getattr(cfg, "INSPECTOR_CAMERA_IDS", ()) if str(value) in identifier_set
    )
    manager_matches = tuple(
        str(value) for value in getattr(cfg, "MANAGER_CAMERA_IDS", ()) if str(value) in identifier_set
    )
    return {
        "INSPECTOR": inspector_matches,
        "SITE_MANAGER": manager_matches,
    }


def _configured_roles(details: dict[str, tuple[str, ...]]) -> list[str]:
    roles: list[str] = []
    for role_name in ("INSPECTOR", "SITE_MANAGER"):
        if details.get(role_name):
            roles.append(role_name)
    return roles


def _role_conflict(details: dict[str, tuple[str, ...]]) -> bool:
    return len(_configured_roles(details)) > 1


def _resolve_target_devices(
    requested_id: str,
    available: list[tuple[dai.DeviceInfo, str, tuple[str, ...]]],
) -> list[tuple[dai.DeviceInfo, str, tuple[str, ...]]]:
    requested = str(requested_id).strip()
    for device_info, primary_id, identifiers in available:
        if requested in identifiers:
            return [(device_info, primary_id, identifiers)]
    known = ", ".join("/".join(identifiers) for _, _, identifiers in available)
    raise RuntimeError(f"Requested device '{requested}' was not found. Available devices: {known}")


def _available_devices() -> list[tuple[dai.DeviceInfo, str, tuple[str, ...]]]:
    return [
        (
            device_info,
            _primary_device_id(device_info, index),
            _device_identifiers(device_info, index),
        )
        for index, device_info in enumerate(dai.Device.getAllAvailableDevices(), start=1)
    ]


def _build_depth_probe_v2_pipeline(fps: float) -> dai.Pipeline:
    pipeline = dai.Pipeline()

    mono_left = _create_node(pipeline, "MonoCamera")
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_left.setFps(fps)

    mono_right = _create_node(pipeline, "MonoCamera")
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_right.setFps(fps)

    stereo = _create_node(pipeline, "StereoDepth")
    try:
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
    except Exception:
        pass
    try:
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    except Exception:
        pass
    try:
        stereo.setLeftRightCheck(True)
    except Exception:
        pass

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    xout_raw = _create_node(pipeline, "XLinkOut")
    xout_raw.setStreamName("rawL")
    mono_left.out.link(xout_raw.input)

    xout_depth = _create_node(pipeline, "XLinkOut")
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    return pipeline


def _probe_device_v2(
    device_info: dai.DeviceInfo,
    fps: float,
    duration_s: float,
    min_valid_depth_mm: float,
    max_valid_depth_mm: float,
) -> dict[str, Any]:
    started_at = time.monotonic()
    raw_frames = 0
    depth_frames = 0
    valid_ratios: list[float] = []
    valid_median_depths_mm: list[float] = []
    raw_shape = None
    depth_shape = None
    depth_dtype = None
    first_raw_ts = None
    last_raw_ts = None
    first_depth_ts = None
    last_depth_ts = None

    pipeline = _build_depth_probe_v2_pipeline(fps=fps)
    with dai.Device(pipeline, device_info) as device:
        try:
            device.setLogLevel(dai.LogLevel.CRITICAL)
        except Exception:
            pass

        raw_queue = device.getOutputQueue("rawL", maxSize=4, blocking=False)
        depth_queue = device.getOutputQueue("depth", maxSize=4, blocking=False)

        while (time.monotonic() - started_at) < duration_s:
            did_work = False

            raw_packet = raw_queue.tryGet()
            while raw_packet is not None:
                raw_frame = raw_packet.getCvFrame()
                raw_frames += 1
                raw_shape = tuple(int(v) for v in raw_frame.shape)
                now = time.monotonic()
                if first_raw_ts is None:
                    first_raw_ts = now
                last_raw_ts = now
                did_work = True
                raw_packet = raw_queue.tryGet()

            depth_packet = depth_queue.tryGet()
            while depth_packet is not None:
                depth_frame = depth_packet.getFrame()
                depth_frames += 1
                depth_shape = tuple(int(v) for v in depth_frame.shape)
                depth_dtype = str(depth_frame.dtype)
                now = time.monotonic()
                if first_depth_ts is None:
                    first_depth_ts = now
                last_depth_ts = now

                valid_mask = (
                    np.isfinite(depth_frame)
                    & (depth_frame >= float(min_valid_depth_mm))
                    & (depth_frame <= float(max_valid_depth_mm))
                )
                valid_count = int(np.count_nonzero(valid_mask))
                total_count = int(depth_frame.size)
                valid_ratio = 0.0 if total_count <= 0 else (valid_count / float(total_count))
                valid_ratios.append(valid_ratio)
                if valid_count > 0:
                    valid_values = depth_frame[valid_mask]
                    valid_median_depths_mm.append(float(np.median(valid_values)))

                did_work = True
                depth_packet = depth_queue.tryGet()

            if not did_work:
                time.sleep(0.01)

    elapsed_s = max(0.0, time.monotonic() - started_at)
    raw_stream_span_s = None if first_raw_ts is None or last_raw_ts is None else max(0.0, last_raw_ts - first_raw_ts)
    depth_stream_span_s = None if first_depth_ts is None or last_depth_ts is None else max(0.0, last_depth_ts - first_depth_ts)

    return {
        "api_path": "v2_xlink",
        "probe_elapsed_s": float(elapsed_s),
        "raw_frames": int(raw_frames),
        "depth_frames": int(depth_frames),
        "raw_observed_fps": (float(raw_frames / elapsed_s) if elapsed_s > 0.0 else 0.0),
        "depth_observed_fps": (float(depth_frames / elapsed_s) if elapsed_s > 0.0 else 0.0),
        "raw_stream_span_s": raw_stream_span_s,
        "depth_stream_span_s": depth_stream_span_s,
        "raw_shape": raw_shape,
        "depth_shape": depth_shape,
        "depth_dtype": depth_dtype,
        "depth_available": bool(depth_frames > 0),
        "valid_depth_observed": bool(any(ratio > 0.0 for ratio in valid_ratios)),
        "mean_valid_ratio": (float(np.mean(valid_ratios)) if valid_ratios else 0.0),
        "max_valid_ratio": (float(np.max(valid_ratios)) if valid_ratios else 0.0),
        "median_valid_depth_mm": (
            float(np.median(valid_median_depths_mm)) if valid_median_depths_mm else None
        ),
        "last_depth_age_s": (
            None if last_depth_ts is None else max(0.0, time.monotonic() - float(last_depth_ts))
        ),
    }


def _probe_device(
    device_info: dai.DeviceInfo,
    fps: float,
    duration_s: float,
    min_valid_depth_mm: float,
    max_valid_depth_mm: float,
) -> dict[str, Any]:
    if _supports_v2_xlink():
        return _probe_device_v2(
            device_info=device_info,
            fps=fps,
            duration_s=duration_s,
            min_valid_depth_mm=min_valid_depth_mm,
            max_valid_depth_mm=max_valid_depth_mm,
        )
    if _supports_v3_camera_api():
        return {
            "api_path": "v3_camera_api",
            "depth_available": False,
            "note": (
                "Current repo depth probing is implemented on the v2/XLink path only. "
                "No v3 depth probe exists yet."
            ),
        }
    return {
        "api_path": "unknown",
        "depth_available": False,
        "note": "Unsupported DepthAI runtime: neither v2 XLink nor v3 Camera API was detected.",
    }


def _print_runtime_map() -> None:
    port_map = _task_controller_port_map()
    host_ip = _guess_local_ipv4()
    host_text = host_ip if host_ip else "unresolved"

    print("[DEPTH_PROBE] Current configured runtime endpoints")
    print(
        "[DEPTH_PROBE] "
        f"inspector_stereo_mjpeg bind=0.0.0.0:{port_map['UNITY_PORT_INSPECTOR']} "
        f"client_target={host_text}:{port_map['UNITY_PORT_INSPECTOR']} "
        f"camera_ids={tuple(getattr(cfg, 'INSPECTOR_CAMERA_IDS', ()))}"
    )
    print(
        "[DEPTH_PROBE] "
        f"site_manager_stereo_mjpeg bind=0.0.0.0:{port_map['UNITY_PORT_MANAGER']} "
        f"client_target={host_text}:{port_map['UNITY_PORT_MANAGER']} "
        f"camera_ids={tuple(getattr(cfg, 'MANAGER_CAMERA_IDS', ()))}"
    )
    print(
        "[DEPTH_PROBE] "
        f"unity_command_channel bind=0.0.0.0:{port_map['UNITY_PORT_COMMANDS']} "
        f"client_target={host_text}:{port_map['UNITY_PORT_COMMANDS']}"
    )
    print(
        "[DEPTH_PROBE] "
        "perception_depth_stream transport=local_xlink stream_name=depth "
        "source=StereoDepth(CAM_B,CAM_C) enabled_by=--use-depth-module"
    )
    print(
        "[DEPTH_PROBE] "
        "perception_raw_left transport=local_xlink stream_name=rawL source=CAM_B mono"
    )
    print(
        "[DEPTH_PROBE] "
        "note=current Unity camera streamer exposes stereo MJPEG only; "
        "it does not publish depth on a TCP port."
    )


def _print_available_devices(devices: list[tuple[dai.DeviceInfo, str, tuple[str, ...]]]) -> None:
    print(f"[DEPTH_PROBE] Visible OAK devices={len(devices)}")
    for device_info, device_id, identifiers in devices:
        name = _device_name(device_info)
        role_details = _role_match_details(identifiers)
        roles = _configured_roles(role_details)
        role_text = ",".join(roles) if roles else "unmapped"
        extra = f" name={name}" if name else ""
        print(
            "[DEPTH_PROBE] "
            f"device_id={device_id} identifiers={identifiers} roles={role_text}{extra}"
        )
        print(
            "[DEPTH_PROBE] "
            f"role_match_details={role_details} role_conflict={_role_conflict(role_details)}"
        )


def _run_probe(args: argparse.Namespace) -> int:
    _print_runtime_map()

    available = _available_devices()
    if not available:
        print("[DEPTH_PROBE] No OAK devices are currently visible to DepthAI.")
        return 1

    _print_available_devices(available)

    if args.device_id:
        targets = _resolve_target_devices(args.device_id, available)
    else:
        targets = available

    print(
        "[DEPTH_PROBE] "
        f"probing api_path={'v2_xlink' if _supports_v2_xlink() else 'v3_camera_api' if _supports_v3_camera_api() else 'unknown'} "
        f"duration_s={float(args.duration_s):.1f} fps={float(args.fps):.1f}"
    )

    failure_count = 0
    for device_info, device_id, identifiers in targets:
        role_details = _role_match_details(identifiers)
        roles = _configured_roles(role_details)
        role_text = ",".join(roles) if roles else "unmapped"
        print(
            f"[DEPTH_PROBE] --- device_id={device_id} identifiers={identifiers} roles={role_text} ---"
        )
        print(
            f"[DEPTH_PROBE] role_match_details={role_details} role_conflict={_role_conflict(role_details)}"
        )
        try:
            summary = _probe_device(
                device_info=device_info,
                fps=float(args.fps),
                duration_s=float(args.duration_s),
                min_valid_depth_mm=float(args.min_valid_depth_mm),
                max_valid_depth_mm=float(args.max_valid_depth_mm),
            )
            for key in (
                "api_path",
                "probe_elapsed_s",
                "raw_frames",
                "depth_frames",
                "raw_observed_fps",
                "depth_observed_fps",
                "raw_stream_span_s",
                "depth_stream_span_s",
                "raw_shape",
                "depth_shape",
                "depth_dtype",
                "depth_available",
                "valid_depth_observed",
                "mean_valid_ratio",
                "max_valid_ratio",
                "median_valid_depth_mm",
                "last_depth_age_s",
                "note",
            ):
                if key in summary:
                    print(f"[DEPTH_PROBE] {key}={summary[key]}")
            if not bool(summary.get("depth_available", False)):
                failure_count += 1
        except Exception as exc:
            failure_count += 1
            print(f"[DEPTH_PROBE] probe_error={exc}")

    return 0 if failure_count < len(targets) else 2


def main() -> int:
    args = parse_args()
    return _run_probe(args)


if __name__ == "__main__":
    raise SystemExit(main())
