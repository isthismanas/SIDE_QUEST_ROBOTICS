import traceback
import struct
import socket
import logging
import depthai as dai
from datetime import timedelta
from typing import Optional

def create_pipeline(enable_rawL: bool = False) -> dai.Pipeline:
    pipeline = dai.Pipeline()

    monoL = pipeline.create(dai.node.MonoCamera)
    monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoL.setFps(20)

    monoR = pipeline.create(dai.node.MonoCamera)
    monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoR.setFps(20)

    encL = pipeline.create(dai.node.VideoEncoder)
    encL.setDefaultProfilePreset(20, dai.VideoEncoderProperties.Profile.MJPEG)
    encL.setQuality(40)

    encR = pipeline.create(dai.node.VideoEncoder)
    encR.setDefaultProfilePreset(20, dai.VideoEncoderProperties.Profile.MJPEG)
    encR.setQuality(40)

    monoL.out.link(encL.input)
    monoR.out.link(encR.input)

    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(timedelta(milliseconds=50))
    encL.bitstream.link(sync.inputs["left"])
    encR.bitstream.link(sync.inputs["right"])

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("out")
    sync.out.link(xout.input)

    if enable_rawL:
        xout_rawL = pipeline.create(dai.node.XLinkOut)
        xout_rawL.setStreamName("rawL")
        monoL.out.link(xout_rawL.input)

    return pipeline

def start_camera_server(mxid: str, port: int, label: str, enable_rawL: bool, stop_event, perc_engine=None):
    pipeline = create_pipeline(enable_rawL=enable_rawL)
    server = None
    
    try:
        with dai.Device(pipeline, dai.DeviceInfo(mxid)) as device:
            try:
                device.setLogLevel(dai.LogLevel.CRITICAL)
            except Exception:
                pass
                
            print(f"[CAM] [{label}] Camera Connected using MxId {mxid}.")
            q = device.getOutputQueue("out", maxSize=4, blocking=False)
            
            # Perception Integration
            if enable_rawL and perc_engine is not None:
                try:
                    q_raw = device.getOutputQueue("rawL", maxSize=4, blocking=False)
                    import numpy as np
                    calibData = device.readCalibration()
                    camera_matrix = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 1280, 720))
                    dist_coeffs = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B))
                    perc_engine.update_intrinsics(camera_matrix, dist_coeffs)
                    perc_engine.start_worker(q_raw, None)
                    print(f"[CAM] [{label}] Perception worker attached to uncompressed feed.")
                except Exception as e:
                    print(f"[CAM] [{label}] Perception spawn failed: {e}")

            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', port))
            server.listen(1)
            server.settimeout(1.0)
            print(f"[CAM] [{label}] Streaming MJPEG to Unity on port {port}")

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
                        group = q.get()
                        dL = group["left"].getData().tobytes()
                        dR = group["right"].getData().tobytes()
                        conn.sendall(b'L' + struct.pack('>I', len(dL)) + dL)
                        conn.sendall(b'R' + struct.pack('>I', len(dR)) + dR)
                except socket.timeout:
                    continue
                except Exception as e:
                    if not stop_event.is_set():
                        pass 
                finally:
                    conn.close()
    except Exception as e:
        print(f"[CAM] [{label}] Fatal Error: {traceback.format_exc()}")
    finally:
        if server:
            server.close()
