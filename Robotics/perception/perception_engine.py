import threading
import time
import cv2
from aruco_tracker import ArucoTracker
from logger import info, warn

class PerceptionEngine:
    def __init__(self, camera_matrix=None, dist_coeffs=None):
        self.tracker = ArucoTracker()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.latest_state = {}
        self.state_lock = threading.Lock()
        
        self.running = False
        self.worker_thread = None

    def update_intrinsics(self, camera_matrix, dist_coeffs):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def process_frame(self, frame_bgr):
        if self.camera_matrix is None or self.dist_coeffs is None:
            # Wait until calibration data is provided
            return
            
        poses = self.tracker.compute_poses(frame_bgr, self.camera_matrix, self.dist_coeffs)
        
        with self.state_lock:
            self.latest_state = poses
            
    def get_latest_state(self):
        with self.state_lock:
            return dict(self.latest_state)

    def start_worker(self, queueL, queueR):
        """
        Starts a background thread that continuously reads from the given DepthAI queues
        and processes frames for ArUco markers.
        """
        self.running = True
        self.worker_thread = threading.Thread(target=self._worker_loop, args=(queueL, queueR), daemon=True)
        self.worker_thread.start()
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
                warn("PERCEPTION", f"Exception in worker loop: {e}")
                time.sleep(0.5)

    def stop_worker(self):
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=1.0)
            info("PERCEPTION", "Stopped perception worker thread.")

# Singleton instance
engine = PerceptionEngine()
