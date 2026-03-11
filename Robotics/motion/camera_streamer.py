import socket
import struct
import traceback
from datetime import timedelta
from typing import Iterable

import cv2
import depthai as dai
import numpy as np


JPEG_QUALITY = 40
STREAM_WIDTH = 1280
STREAM_HEIGHT = 720
STREAM_FPS = 20


def _create_node(pipeline, node_name: str):
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


def _device_identifiers(device_info: object, index: int) -> set[str]:
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

    return identifiers


def _candidate_list(target_ref) -> list[str]:
    if isinstance(target_ref, (list, tuple, set)):
        return [str(item) for item in target_ref if str(item).strip()]
    return [str(target_ref)]


def _resolve_device_info(target_ref, label: str):
    candidates = _candidate_list(target_ref)
    available_devices = list(dai.Device.getAllAvailableDevices())
    if not available_devices:
        raise RuntimeError(f"No OAK-D devices found for {label}.")

    available_summary: list[str] = []
    for index, device in enumerate(available_devices, start=1):
        identifiers = _device_identifiers(device, index)
        available_summary.append("/".join(sorted(identifiers)))
        for candidate in candidates:
            if candidate in identifiers:
                return device, candidate

    available_text = ", ".join(available_summary)
    raise RuntimeError(
        f"[{label}] Could not resolve camera id from candidates {candidates}. "
        f"Available devices: {available_text}"
    )


def _build_v2_pipeline(enable_rawL: bool = False) -> dai.Pipeline:
    pipeline = dai.Pipeline()

    monoL = _create_node(pipeline, "MonoCamera")
    monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoL.setFps(STREAM_FPS)

    monoR = _create_node(pipeline, "MonoCamera")
    monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoR.setFps(STREAM_FPS)

    encL = _create_node(pipeline, "VideoEncoder")
    encL.setDefaultProfilePreset(STREAM_FPS, dai.VideoEncoderProperties.Profile.MJPEG)
    encL.setQuality(JPEG_QUALITY)

    encR = _create_node(pipeline, "VideoEncoder")
    encR.setDefaultProfilePreset(STREAM_FPS, dai.VideoEncoderProperties.Profile.MJPEG)
    encR.setQuality(JPEG_QUALITY)

    monoL.out.link(encL.input)
    monoR.out.link(encR.input)

    sync = _create_node(pipeline, "Sync")
    sync.setSyncThreshold(timedelta(milliseconds=50))
    encL.bitstream.link(sync.inputs["left"])
    encR.bitstream.link(sync.inputs["right"])

    xout = _create_node(pipeline, "XLinkOut")
    xout.setStreamName("out")
    sync.out.link(xout.input)

    if enable_rawL:
        xout_rawL = _create_node(pipeline, "XLinkOut")
        xout_rawL.setStreamName("rawL")
        monoL.out.link(xout_rawL.input)

    return pipeline


def _v3_pipeline_context(device_info):
    device = dai.Device(device_info)
    return dai.Pipeline(device)


def _to_bgr(frame):
    if len(frame.shape) == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame


def _to_jpeg_bytes(frame) -> bytes:
    frame_bgr = _to_bgr(frame)
    ok, encoded = cv2.imencode(
        ".jpg",
        frame_bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
    )
    if not ok:
        raise RuntimeError("Failed to JPEG-encode frame.")
    return encoded.tobytes()


def _send_stereo_payload(conn: socket.socket, left_bytes: bytes, right_bytes: bytes) -> None:
    conn.sendall(b"L" + struct.pack(">I", len(left_bytes)) + left_bytes)
    conn.sendall(b"R" + struct.pack(">I", len(right_bytes)) + right_bytes)


def _configure_perception(device, raw_queue, perc_engine, label: str) -> None:
    try:
        calib_data = device.readCalibration()
        camera_matrix = np.array(
            calib_data.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, STREAM_WIDTH, STREAM_HEIGHT)
        )
        dist_coeffs = np.array(
            calib_data.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)
        )
        perc_engine.update_intrinsics(camera_matrix, dist_coeffs)
        perc_engine.start_worker(raw_queue, None)
        print(f"[CAM] [{label}] Perception worker attached to uncompressed feed.")
    except Exception as e:
        print(f"[CAM] [{label}] Perception spawn failed: {e}")


def _serve_socket_loop(server: socket.socket, stop_event, label: str, stream_frames) -> None:
    while not stop_event.is_set():
        try:
            conn, addr = server.accept()
        except socket.timeout:
            continue
        except OSError:
            if stop_event.is_set():
                break
            raise

        conn.settimeout(1.0)
        print(f"[CAM] [{label}] Unity joined stream from {addr}")

        try:
            while not stop_event.is_set():
                left_bytes, right_bytes = stream_frames()
                _send_stereo_payload(conn, left_bytes, right_bytes)
        except socket.timeout:
            continue
        except Exception:
            if not stop_event.is_set():
                pass
        finally:
            conn.close()


def _run_v2_server(device_info, matched_id: str, port: int, label: str, enable_rawL: bool, stop_event, perc_engine=None):
    pipeline = _build_v2_pipeline(enable_rawL=enable_rawL)
    with dai.Device(pipeline, device_info) as device:
        try:
            device.setLogLevel(dai.LogLevel.CRITICAL)
        except Exception:
            pass

        print(f"[CAM] [{label}] Camera Connected using id {matched_id}.")
        q = device.getOutputQueue("out", maxSize=4, blocking=False)

        if enable_rawL and perc_engine is not None:
            q_raw = device.getOutputQueue("rawL", maxSize=4, blocking=False)
            _configure_perception(device, q_raw, perc_engine, label)

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", port))
            server.listen(1)
            server.settimeout(1.0)
            print(f"[CAM] [{label}] Streaming MJPEG to Unity on port {port}")

            def _stream_frames():
                group = q.get()
                return (
                    group["left"].getData().tobytes(),
                    group["right"].getData().tobytes(),
                )

            _serve_socket_loop(server, stop_event, label, _stream_frames)
        finally:
            server.close()


def _run_v3_server(device_info, matched_id: str, port: int, label: str, enable_rawL: bool, stop_event, perc_engine=None):
    with _v3_pipeline_context(device_info) as pipeline:
        cam_left = _create_node(pipeline, "Camera").build(
            dai.CameraBoardSocket.CAM_B,
            sensorFps=STREAM_FPS,
        )
        cam_right = _create_node(pipeline, "Camera").build(
            dai.CameraBoardSocket.CAM_C,
            sensorFps=STREAM_FPS,
        )

        left_stream_output = cam_left.requestOutput((STREAM_WIDTH, STREAM_HEIGHT), type=dai.ImgFrame.Type.GRAY8)
        right_stream_output = cam_right.requestOutput((STREAM_WIDTH, STREAM_HEIGHT), type=dai.ImgFrame.Type.GRAY8)
        left_stream_queue = left_stream_output.createOutputQueue()
        right_stream_queue = right_stream_output.createOutputQueue()

        raw_queue = None
        if enable_rawL:
            left_raw_output = cam_left.requestOutput((STREAM_WIDTH, STREAM_HEIGHT), type=dai.ImgFrame.Type.GRAY8)
            raw_queue = left_raw_output.createOutputQueue()

        device = pipeline.getDefaultDevice()
        print(f"[CAM] [{label}] Camera Connected using id {matched_id}.")

        if enable_rawL and perc_engine is not None and raw_queue is not None:
            _configure_perception(device, raw_queue, perc_engine, label)

        pipeline.start()
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", port))
            server.listen(1)
            server.settimeout(1.0)
            print(f"[CAM] [{label}] Streaming MJPEG to Unity on port {port}")

            def _stream_frames():
                left_frame = left_stream_queue.get().getCvFrame()
                right_frame = right_stream_queue.get().getCvFrame()
                return _to_jpeg_bytes(left_frame), _to_jpeg_bytes(right_frame)

            _serve_socket_loop(server, stop_event, label, _stream_frames)
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass
            server.close()


def start_camera_server(mxid, port: int, label: str, enable_rawL: bool, stop_event, perc_engine=None):
    try:
        device_info, matched_id = _resolve_device_info(mxid, label)
        if _supports_v2_xlink():
            _run_v2_server(
                device_info=device_info,
                matched_id=matched_id,
                port=port,
                label=label,
                enable_rawL=enable_rawL,
                stop_event=stop_event,
                perc_engine=perc_engine,
            )
            return

        if _supports_v3_camera_api():
            _run_v3_server(
                device_info=device_info,
                matched_id=matched_id,
                port=port,
                label=label,
                enable_rawL=enable_rawL,
                stop_event=stop_event,
                perc_engine=perc_engine,
            )
            return

        raise RuntimeError("Unsupported DepthAI runtime: no compatible camera API found.")
    except Exception:
        print(f"[CAM] [{label}] Fatal Error: {traceback.format_exc()}")
