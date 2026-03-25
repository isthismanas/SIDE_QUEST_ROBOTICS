import os
import sys
import threading
import time
from datetime import datetime, timezone

import cv2
from aruco_tracker import ArucoTracker
from logger import info, warn, write_jsonl_event

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MOTION_DIR = os.path.normpath(os.path.join(THIS_DIR, "..", "motion"))
if MOTION_DIR not in sys.path:
    sys.path.append(MOTION_DIR)

import robot_config as cfg
import vision_bridge


def _timestamp_utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class PerceptionEngine:
    def __init__(self, camera_matrix=None, dist_coeffs=None):
        self.tracker = ArucoTracker()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.latest_state = {}
        self.last_seen_by_marker = {}
        self.invalidated_markers = set()
        self.state_lock = threading.Lock()
        self.last_snapshot_log_t = 0.0
        self.last_change_log_by_marker = {}
        
        self.running = False
        self.worker_thread = None

    def _tracked_marker_context(self, marker_id: int):
        reference_marker = int(getattr(cfg, "VISION_REFERENCE_MARKER_ID", 0))
        if marker_id == reference_marker:
            return "reference", "REF"

        for target_id, mapped_marker_id in dict(getattr(cfg, "VISION_PICK_MARKER_MAP", {})).items():
            if int(mapped_marker_id) == int(marker_id):
                return "pickup_block", str(target_id)

        for target_id, mapped_marker_id in dict(getattr(cfg, "VISION_DROP_MARKER_MAP", {})).items():
            if int(mapped_marker_id) == int(marker_id):
                return "tower_block", str(target_id)

        return None, None

    def _should_log_marker_change(self, marker_id: int, pose):
        role, target_id = self._tracked_marker_context(marker_id)
        if role is None:
            return False, None

        last_logged_pose = self.last_change_log_by_marker.get(int(marker_id))
        if last_logged_pose is None:
            return True, {
                "reason": "initial",
                "role": role,
                "target_id": target_id,
            }

        xy_delta_mm = (
            (((float(pose[0]) - float(last_logged_pose[0])) ** 2) + ((float(pose[1]) - float(last_logged_pose[1])) ** 2)) ** 0.5
        ) * 1000.0
        z_delta_mm = abs(float(pose[2]) - float(last_logged_pose[2])) * 1000.0
        rot_delta_rad = max(
            abs(float(pose[3]) - float(last_logged_pose[3])),
            abs(float(pose[4]) - float(last_logged_pose[4])),
            abs(float(pose[5]) - float(last_logged_pose[5])),
        )
        translation_moved = (
            xy_delta_mm >= float(getattr(cfg, "VISION_MARKER_CHANGE_DELTA_MM", 8.0))
            or z_delta_mm >= float(getattr(cfg, "VISION_MARKER_CHANGE_DELTA_MM", 8.0))
        )
        rotation_moved = (
            bool(getattr(cfg, "VISION_MARKER_CHANGE_LOG_ROTATION", False))
            and rot_delta_rad >= float(getattr(cfg, "VISION_MARKER_CHANGE_ROT_DELTA_RAD", 0.15))
        )
        if translation_moved or rotation_moved:
            return True, {
                "reason": "moved",
                "role": role,
                "target_id": target_id,
                "delta_xy_mm": round(float(xy_delta_mm), 3),
                "delta_z_mm": round(float(z_delta_mm), 3),
                "delta_rot_rad": round(float(rot_delta_rad), 4),
            }

        return False, None

    def _log_marker_change(self, marker_id: int, pose) -> None:
        should_log, meta = self._should_log_marker_change(marker_id, pose)
        if not should_log or meta is None:
            return

        robot_xy, robot_xy_reason = vision_bridge.camera_xy_to_robot_xy_mm(float(pose[0]), float(pose[1]))
        payload = {
            "event": "marker_pose_changed",
            "module": "PERCEPTION",
            "marker_id": int(marker_id),
            "role": meta["role"],
            "target_id": meta["target_id"],
            "change_reason": meta["reason"],
            "camera_pose": {
                "x": round(float(pose[0]), 6),
                "y": round(float(pose[1]), 6),
                "z": round(float(pose[2]), 6),
                "roll": round(float(pose[3]), 6),
                "pitch": round(float(pose[4]), 6),
                "yaw": round(float(pose[5]), 6),
            },
        }
        if "delta_xy_mm" in meta:
            payload["delta_xy_mm"] = meta["delta_xy_mm"]
            payload["delta_z_mm"] = meta["delta_z_mm"]
            payload["delta_rot_rad"] = meta["delta_rot_rad"]
        if robot_xy is not None:
            payload["robot_xy_mm"] = {
                "x": round(float(robot_xy[0]), 3),
                "y": round(float(robot_xy[1]), 3),
            }
        else:
            payload["robot_xy_reason"] = robot_xy_reason

        write_jsonl_event("marker_positions", payload)
        self.last_change_log_by_marker[int(marker_id)] = tuple(float(v) for v in pose)

    def update_intrinsics(self, camera_matrix, dist_coeffs):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def process_frame(self, frame_bgr):
        if self.camera_matrix is None or self.dist_coeffs is None:
            # Wait until calibration data is provided
            return
            
        poses = self.tracker.compute_poses(frame_bgr, self.camera_matrix, self.dist_coeffs)
        
        now = time.monotonic()
        snapshot_ts_utc = _timestamp_utc()

        if poses:
            if (now - self.last_snapshot_log_t) >= 1.0:
                marker_snapshot = []
                for m_id in sorted(poses):
                    x, y, z, roll, pitch, yaw = poses[m_id]
                    marker_snapshot.append(
                        {
                            "marker_id": int(m_id),
                            "position_m": {
                                "x": round(float(x), 6),
                                "y": round(float(y), 6),
                                "z": round(float(z), 6),
                            },
                            "rotation_rad": {
                                "roll": round(float(roll), 6),
                                "pitch": round(float(pitch), 6),
                                "yaw": round(float(yaw), 6),
                            },
                        }
                    )

                write_jsonl_event(
                    "aruco_tracker",
                    {
                        "event": "aruco_snapshot",
                        "module": "PERCEPTION",
                        "marker_count": len(marker_snapshot),
                        "markers": marker_snapshot,
                    },
                )
                self.last_snapshot_log_t = now

            for marker_id, pose in poses.items():
                self._log_marker_change(int(marker_id), pose)

        with self.state_lock:
            self.latest_state = poses
            for marker_id, pose in poses.items():
                self.last_seen_by_marker[int(marker_id)] = {
                    "pose": tuple(float(v) for v in pose),
                    "seen_mono": float(now),
                    "seen_utc": snapshot_ts_utc,
                }
            
    def get_latest_state(self):
        with self.state_lock:
            return dict(self.latest_state)

    def get_last_seen_marker(self, marker_id):
        with self.state_lock:
            record = self.last_seen_by_marker.get(int(marker_id))
            if record is None:
                return None

            age_s = max(0.0, time.monotonic() - float(record["seen_mono"]))
            return {
                "marker_id": int(marker_id),
                "pose": tuple(record["pose"]),
                "age_s": age_s,
                "last_seen_utc": str(record["seen_utc"]),
                "invalidated": bool(int(marker_id) in self.invalidated_markers),
            }

    def invalidate_marker(self, marker_id: int) -> None:
        with self.state_lock:
            self.invalidated_markers.add(int(marker_id))

    def reset_marker_memory(self, marker_ids=None) -> None:
        with self.state_lock:
            if marker_ids is None:
                self.latest_state = {}
                self.last_seen_by_marker = {}
                self.last_change_log_by_marker = {}
                self.invalidated_markers.clear()
                return

            marker_ids = {int(marker_id) for marker_id in marker_ids}
            self.latest_state = {
                int(marker_id): pose
                for marker_id, pose in self.latest_state.items()
                if int(marker_id) not in marker_ids
            }
            for marker_id in marker_ids:
                self.last_seen_by_marker.pop(int(marker_id), None)
                self.last_change_log_by_marker.pop(int(marker_id), None)
                self.invalidated_markers.discard(int(marker_id))

    def start_worker(self, queueL, queueR):
        """
        Starts a background thread that continuously reads from the given DepthAI queues
        and processes frames for ArUco markers.
        """
        self.running = True
        self.worker_thread = threading.Thread(target=self._worker_loop, args=(queueL, queueR), daemon=True)
        self.worker_thread.start()
        if bool(getattr(cfg, "DEBUG_ENABLED", False)):
            info("PERCEPTION", "Started perception worker thread.")

    def _worker_loop(self, queueL, queueR):
        while self.running:
            try:
                # Attempt to get uncompressed frames from DepthAI Left and Right queues
                # Typically we only need one for ArUco, or we can cross-validate. Let's use Left.
                if queueL.has():
                    frame_data = queueL.get()
                    # DepthAI mono frames are CV_8UC1 (grayscale), so we don't strictly need BGR for ArucoDetector,
                    # but our ArucoTracker converts BGR to GRAY. We can reshape it directly to GRAY or fake BGR.
                    imgL = frame_data.getCvFrame() 
                    
                    # Ensure it's 3-channel BGR for our tracker (or we can just skip cvtColor in tracker)
                    if len(imgL.shape) == 2:
                        imgL_bgr = cv2.cvtColor(imgL, cv2.COLOR_GRAY2BGR)
                    else:
                        imgL_bgr = imgL

                    # If intrinsics aren't set yet, wait
                    if self.camera_matrix is not None:
                        self.process_frame(imgL_bgr)
                        
                else:
                    time.sleep(0.01) # Avoid busy waiting
            except Exception as e:
                # Suppress exceptions during shutdown (self.running is False)
                if self.running:
                    warn("PERCEPTION", f"Exception in worker loop: {e}")
                time.sleep(0.5)

    def stop_worker(self):
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=1.0)
            if bool(getattr(cfg, "DEBUG_ENABLED", False)):
                info("PERCEPTION", "Stopped perception worker thread.")

# Singleton instance
engine = PerceptionEngine()
