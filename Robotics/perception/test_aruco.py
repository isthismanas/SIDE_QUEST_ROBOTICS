import cv2
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from aruco_tracker import ArucoTracker

def main():
    print(f"OpenCV Version: {cv2.__version__}")
    try:
        tracker = ArucoTracker()
        print("ArucoTracker instantiated successfully.")
        
        # Test with a dummy black image
        dummy_img = np.zeros((720, 1280, 3), dtype=np.uint8)
        corners, ids, _ = tracker.detect_markers(dummy_img)
        print(f"Detection on dummy image: {ids} (Expected: None or empty array)")
        
        # Optionally create a synthetic marker
        if tracker.is_v4_7_plus:
            # OpenCV 4.7+
            marker_img = cv2.aruco.generateImageMarker(tracker.dictionary, 24, 200)
        else:
            # Older OpenCV
            marker_img = cv2.aruco.drawMarker(tracker.dictionary, 24, 200)
            
        print("Generated synthetic marker.")

        # Insert marker into a larger image
        test_img = np.ones((480, 640), dtype=np.uint8) * 255
        test_img[100:300, 100:300] = marker_img
        test_img_bgr = cv2.cvtColor(test_img, cv2.COLOR_GRAY2BGR)

        corners, ids, _ = tracker.detect_markers(test_img_bgr)
        print(f"Detection on synthetic image... Found IDs: {ids.flatten() if ids is not None else None}")
        
        # Fake intrinsics for 640x480
        camera_matrix = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)
        dist_coeffs = np.zeros(5)
        
        poses = tracker.compute_poses(test_img_bgr, camera_matrix, dist_coeffs)
        for m_id, pose in poses.items():
            print(f"Marker {m_id} Pose: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}")

        print("Test passed successfully.")
    except Exception as e:
        print(f"Test failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
