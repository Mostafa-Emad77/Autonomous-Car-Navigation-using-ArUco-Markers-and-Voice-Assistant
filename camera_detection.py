import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from picamera2 import Picamera2

class CameraDetection:
    def __init__(self, marker_length=0.025):
        self.picam2 = Picamera2()
        self.marker_length = marker_length

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
        self.parameters = cv2.aruco.DetectorParameters()

        self.camera_matrix = np.array([[483.99382395, 0.0, 311.03878107], 
                                       [0.0, 485.569195, 256.835], 
                                       [0.0, 0.0, 1.0]], dtype=np.float64)
        self.dist_coeffs = np.array([0.12606201, -0.23427149, -0.00336546, -0.00672485, 0.0610827], dtype=np.float64)

        self.camera_rvec = [np.radians(-90), 0, np.radians(-90 - 10)]
        self.camera_tvec = [0, 0, 18]
        self.camera_tf = self.to_tf(self.camera_rvec, self.camera_tvec, order="ZYX")

    def to_tf(self, rvec, tvec, order="xyz"):
        tf = np.identity(4, dtype=float)
        r = Rotation.from_euler(order, rvec, degrees=False)
        rot_matrix = r.as_matrix()
        tf[:3, :3] = rot_matrix
        tf[:3, 3] = tvec
        return tf

    def start_camera(self):
        self.picam2.start(show_preview=False)

    def stop_camera(self):
        self.picam2.stop_preview()
        self.picam2.close()

    def capture_frame(self):
        return self.picam2.capture_array()

    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def estimate_pose(self, corners):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
        return rvecs, tvecs

    def compute_world_coordinates(self, rvec, tvec):
        aruco_tf = self.to_tf(rvec.flatten(), tvec.flatten(), order="XYZ")
        aruco_robot_tf = np.dot(self.camera_tf, aruco_tf)
        x = aruco_robot_tf[0, 3]
        y = aruco_robot_tf[1, 3]
        return x, y

    def draw_markers(self, frame, corners, ids):
        return cv2.aruco.drawDetectedMarkers(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR), corners, ids)

    def show_frame(self, frame):
        cv2.imshow('ArUco Markers', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False
        return True
