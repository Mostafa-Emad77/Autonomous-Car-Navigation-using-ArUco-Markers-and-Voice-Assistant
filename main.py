import time
import serial
import cv2
import numpy as np
import tkinter as tk
from tkinter import Canvas
from ekf_slam import EKFSLAM
from camera_detection import CameraDetection
from types import SimpleNamespace

from listener import listen_for_activation_phrase
from recognition import recognize_audio
from commands import handle_command

class RobotController:
    def __init__(self):
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)  # Allow time for Arduino to reset

        self.slam = EKFSLAM()
        self.camera_detection = CameraDetection()

        self.root = tk.Tk()
        self.root.title("ArUco Marker Detection")
        self.canvas = Canvas(self.root, width=600, height=600)
        self.canvas.pack()
        self.marker_positions = {}

    def draw_grid(self):
        for i in range(-2, 3):
            x = i * 100 + 300  # Convert world coordinates to canvas coordinates
            y = i * 100 + 300
            self.canvas.create_line(x, 0, x, 600, fill="gray")
            self.canvas.create_line(0, y, 600, y, fill="gray")
            if i != 0:
                self.canvas.create_text(x, 310, text=f"{i}")
                self.canvas.create_text(290, y, text=f"{-i}")
        self.canvas.create_line(300, 0, 300, 600, fill="black")
        self.canvas.create_line(0, 300, 600, 300, fill="black")
        self.canvas.create_text(310, 310, text="0")

    def update_plot(self):
        self.canvas.delete("all")
        self.draw_grid()
        for marker_id, position in self.marker_positions.items():
            x, y = position
            x_canvas = x * 100 + 300
            y_canvas = -y * 100 + 300
            self.canvas.create_oval(x_canvas-5, y_canvas-5, x_canvas+5, y_canvas+5, fill="red")
            self.canvas.create_text(x_canvas, y_canvas-10, text=str(marker_id))
        self.root.after(100, self.update_plot)

    def run_ekf_slam(self, img):
        line = self.arduino.readline().decode().strip()
        parts = line.split(',')
        angle1, angle2 = None, None

        for part in parts:
            if part.startswith('A1:'):
                angle1 = float(part[3:])
            elif part.startswith('A2:'):
                angle2 = float(part[3:])

        if angle1 is not None and angle2 is not None:
            self.slam.predict(angle1, angle2) 

        corners, ids = self.camera_detection.detect_markers(img)
        if ids is not None and len(ids) > 0:
            rvecs, tvecs = self.camera_detection.estimate_pose(corners)
            for i in range(len(ids)):
                if 0 < ids[i] < 1001:
                    rvec = rvecs[i]
                    tvec = tvecs[i]
                    x, y = self.camera_detection.compute_world_coordinates(rvec, tvec)
                    self.marker_positions[ids[i]] = (x, y)
                    landmark_positions = [(x, y)]
                    if ids[i] not in self.slam.get_landmark_ids():
                        self.slam.add_landmark((x, y), ids[i])
                    else:
                        self.slam.correction([(x, y)], [ids[i]])

        robot_x, robot_y, robot_theta, robot_stdev = self.slam.get_robot_pose()
        data = SimpleNamespace(
            landmark_ids=ids,
            landmark_positions=[self.marker_positions[ids[i]] for i in range(len(ids))],
            robot_position=np.array([robot_x, robot_y]),
            robot_theta=robot_theta,
            robot_stdev=robot_stdev
        )
        return data

    def decide_movement(self, robot_pose):
        if robot_pose[0] > 2:
            self.arduino.write(b'L')
        elif robot_pose[1] > 2:
            self.arduino.write(b'R')
        else:
            self.arduino.write(b'F')

    def run(self):
        self.camera_detection.start_camera()
        self.root.after(100, self.update_plot)
        try:
            while True:
                if listen_for_activation_phrase():
                    print("Car activated. You can now give commands.")
                    while True:
                        audio_data = recognize_audio()
                        if audio_data:
                            handle_command(audio_data['text'])
                            break
                frame = self.camera_detection.capture_frame()
                data = self.run_ekf_slam(frame)
                self.decide_movement(data.robot_position)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                self.root.update_idletasks()
                self.root.update()
        finally:
            self.arduino.write(b'S')
            self.arduino.close()
            self.camera_detection.stop_camera()
            cv2.destroyAllWindows()
            self.root.destroy()

if __name__ == '__main__':
    controller = RobotController()
    controller.run()
