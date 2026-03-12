import cv2
import numpy as np


DEFAULT_MARKER_SIZE_M = 0.0375
DEFAULT_MARKER_SIZE_BY_ID_M = {
    0: 0.0375,
    11: 0.0200,
    12: 0.0200,
    13: 0.0200,
    14: 0.0200,
    15: 0.0200,
    16: 0.0200,
    17: 0.0200,
}


class ArucoTracker:
    def __init__(self, dictionary_id=cv2.aruco.DICT_4X4_50, marker_size=DEFAULT_MARKER_SIZE_M):
        """
        :param dictionary_id: The ArUco dictionary to use. Default matches a 4x4 dictionary.
        :param marker_size: The real-world size of the marker in meters, or a dict
            mapping marker_id -> size_m for mixed marker sizes.
        """
        if isinstance(marker_size, dict):
            self.default_marker_size = float(DEFAULT_MARKER_SIZE_M)
            self.marker_size_by_id = {
                int(marker_id): float(size_m)
                for marker_id, size_m in marker_size.items()
            }
        else:
            self.default_marker_size = float(marker_size)
            self.marker_size_by_id = dict(DEFAULT_MARKER_SIZE_BY_ID_M)

        # Determine OpenCV version for API compatibility
        self.is_v4_7_plus = hasattr(cv2.aruco, 'ArucoDetector')
        
        if self.is_v4_7_plus:
            self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
        else:
            # Fallback for older OpenCV versions
            self.dictionary = cv2.aruco.Dictionary_get(dictionary_id)
            self.detector_params = cv2.aruco.DetectorParameters_create()

        # Set specific params if needed (e.g., adaptive thresholding for bad lighting)
        self.detector_params.adaptiveThreshWinSizeMin = 3
        self.detector_params.adaptiveThreshWinSizeMax = 23
        self.detector_params.adaptiveThreshWinSizeStep = 10

    def detect_markers(self, image):
        """
        Detect ArUco markers in the given image.
        :param image: BGR image.
        :return: corners, ids, rejected
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if self.is_v4_7_plus:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)
            
        return corners, ids, rejected

    def _marker_size_for_id(self, marker_id: int) -> float:
        return float(self.marker_size_by_id.get(int(marker_id), self.default_marker_size))

    def _object_points_for_size(self, marker_size_m: float):
        half_size = float(marker_size_m) / 2.0
        return np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

    def estimate_pose(self, corners, ids, camera_matrix, dist_coeffs):
        """
        Estimate pose of the detected markers.
        :param corners: Marker corners detected by detect_markers.
        :param ids: Marker ids corresponding to corners.
        :param camera_matrix: 3x3 camera intrinsics matrix.
        :param dist_coeffs: Distortion coefficients.
        :return: rvecs, tvecs
        """
        if len(corners) == 0 or ids is None or len(ids) == 0:
            return [], []

        if hasattr(cv2, 'solvePnP'):
            # The older cv2.aruco.estimatePoseSingleMarkers has been deprecated/removed in latest cv2.
            # Using cv2.solvePnP manually for each marker is more robust across versions.
            rvecs = []
            tvecs = []

            for corner, marker_id in zip(corners, ids.flatten()):
                # corner shape is (1, 4, 2)
                img_points = corner[0].astype(np.float32)
                obj_points = self._object_points_for_size(self._marker_size_for_id(int(marker_id)))
                success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    rvecs.append(rvec)
                    tvecs.append(tvec)
            
            return np.array(rvecs), np.array(tvecs)
        else:
            # Very old fallback path. Use the default size only; mixed-size support
            # depends on the solvePnP path above.
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.default_marker_size, camera_matrix, dist_coeffs)
            return rvecs, tvecs
            
    def compute_poses(self, image, camera_matrix, dist_coeffs):
        """
        Detect and estimate pose for all markers in the image.
        :return: A dict mapping marker_id -> (x, y, z, roll, pitch, yaw)
        """
        corners, ids, _ = self.detect_markers(image)
        if ids is None or len(ids) == 0:
            return {}

        rvecs, tvecs = self.estimate_pose(corners, ids, camera_matrix, dist_coeffs)
        
        poses = {}
        for i, m_id in enumerate(ids.flatten()):
            rvec = rvecs[i]
            tvec = tvecs[i]
            
            # Convert rotation vector to rotation matrix, then to Euler angles (roll, pitch, yaw)
            rmat, _ = cv2.Rodrigues(rvec)
            # Extrinsic yaw-pitch-roll order depends on convention. Wait, using standard formulas:
            sy = np.sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0])
            singular = sy < 1e-6
            if not singular:
                roll = np.arctan2(rmat[2,1], rmat[2,2])
                pitch = np.arctan2(-rmat[2,0], sy)
                yaw = np.arctan2(rmat[1,0], rmat[0,0])
            else:
                roll = np.arctan2(-rmat[1,2], rmat[1,1])
                pitch = np.arctan2(-rmat[2,0], sy)
                yaw = 0
                
            x, y, z = tvec.flatten()
            poses[int(m_id)] = (x, y, z, roll, pitch, yaw)
            
        return poses
