import rclpy
from rclpy.node import Node
import cv2
import os
import tkinter as tk
from PIL import Image as PilImage, ImageTk
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import time

MODES = ['live', 'image', 'lidar', 'gps', 'digits']

class CameraViewer(Node):
    def __init__(self, root):
        super().__init__('camera_viewer')
        self.root = root

        self.camera_annotated_bridge = CvBridge()
        self.camera_annotated_subscription = self.create_subscription(Image, '/target_annotated/image_raw', self.camera_annotated_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, "fix", self.gps_cb, 10)
        self.lidar_sub = self.create_subscription(Float32MultiArray, "obstacle_team10", self.obstacle_cb, 10)
        self.imu_sub = self.create_subscription(Float32MultiArray, "imu_team10", self.imu_cb, 10)
        self.digit_annotated_subscription = self.create_subscription(Image, '/digit_annotated/image_raw', self.digit_annotated_callback, 10)

        self.last_camera_image_time = 0
        self.last_digit_image_time = 0
        self.mode = 0

        self.image_dirs = {
            'Digits': '/home/team10/saved_images/digits',
            'Objects': '/home/team10/saved_images/objects'
        }
        self.current_image_dir_name = 'Digits'
        self.image_dir = self.image_dirs[self.current_image_dir_name]
        self.image_files = []
        self.image_index = 0

        self.build_base_ui()
        self.update_ui()

    def build_base_ui(self):
        for i in range(8):
            self.root.grid_rowconfigure(i, weight=1)
        for i in range(5):
            self.root.grid_columnconfigure(i, weight=1)

        self.button_frame = tk.Frame(self.root)
        self.button_frame.grid(row=0, column=0, columnspan=5, sticky="ew", pady=5)
        self.root.grid_rowconfigure(0, minsize=50)

        for i in range(5):
            self.button_frame.grid_columnconfigure(i, weight=1)
        for i in range(2):
            self.button_frame.grid_rowconfigure(i, weight=1)

        tk.Button(self.button_frame, text="← Prev", command=self.prev_mode).grid(row=0, column=0, rowspan=2, padx=5, sticky="ew")
        tk.Button(self.button_frame, text="Next →", command=self.next_mode).grid(row=0, column=4, rowspan=2, padx=5, sticky="ew")

        self.button_center_frame = tk.Frame(self.button_frame)
        self.button_center_frame.grid(row=0, column=1, columnspan=3, sticky="ew")
        for i in range(3):
            self.button_center_frame.grid_columnconfigure(i, weight=1)
            self.button_center_frame.grid_rowconfigure(i, weight=1)

        self.content_frame = tk.Frame(self.root)
        self.content_frame.grid(row=1, column=0, rowspan=7, columnspan=5, sticky="nsew")
        for i in range(2):
            self.content_frame.grid_rowconfigure(i, weight=1)
            self.content_frame.grid_columnconfigure(i, weight=1)

        self.live_label = tk.Label(self.content_frame)
        self.image_label = tk.Label(self.content_frame)
        self.lidar_frame = tk.Frame(self.content_frame)
        self.gps_frame = tk.Frame(self.content_frame)
        self.digit_frame = tk.Frame(self.content_frame)
        self.digit_label = tk.Label(self.digit_frame)
        self.digit_label.pack(expand=True, fill="both")

    def next_mode(self):
        self.mode = (self.mode + 1) % len(MODES)
        self.update_ui()
        self.get_logger().info(f"Switched to mode: {MODES[self.mode]}")

    def prev_mode(self):
        self.mode = (self.mode - 1) % len(MODES)
        self.update_ui()
        self.get_logger().info(f"Switched to mode: {MODES[self.mode]}")

    def update_ui(self):
        for widget in self.button_center_frame.grid_slaves():
            widget.grid_forget()
        for widget in self.content_frame.grid_slaves():
            widget.grid_forget()
        self.build_base_ui()
        for widget in self.button_center_frame.grid_slaves():
            widget.grid_forget()
        for widget in self.content_frame.grid_slaves():
            widget.grid_forget()

        if self.mode == 0:
            self.ui_live_mode()
        elif self.mode == 1:
            self.ui_image_mode()
        elif self.mode == 2:
            self.ui_lidar_mode()
        elif self.mode == 3:
            self.ui_gps_mode()
        elif self.mode == 4:
            self.ui_digit_mode()
        else:
            self.get_logger().error("Invalid mode selected.")

    def ui_live_mode(self):
        self.live_label.grid(row=0, column=0, rowspan=2, columnspan=2, sticky="nsew")
        tk.Label(self.button_center_frame, text="Live Camera Feed", font=("Arial", 16)).grid(row=0, column=0, columnspan=3, rowspan=3, sticky="ew")

    def ui_image_mode(self):
        self.image_label.grid(row=0, column=0, rowspan=2, columnspan=2, sticky="nsew")
        tk.Label(self.button_center_frame, text="Image Viewer", font=("Arial", 16)).grid(row=0, column=0, columnspan=3, sticky="ew")
        tk.Button(self.button_center_frame, text="← Prev", command=self.prev_image).grid(row=1, column=0, rowspan=2, padx=5, sticky="ew")
        tk.Button(self.button_center_frame, text="Next →", command=self.next_image).grid(row=1, column=3, rowspan=2, padx=5, sticky="ew")
        dir_label = tk.Label(self.button_center_frame, text=f"Folder: {self.current_image_dir_name}", font=("Arial", 12))
        dir_label.grid(row=2, column=0, columnspan=1, sticky="ew")
        switch_button = tk.Button(self.button_center_frame, text="Switch Folder", command=self.switch_image_folder)
        switch_button.grid(row=2, column=1, columnspan=2, sticky="ew")
        self.load_image_files()
        self.display_image()

    def switch_image_folder(self):
        names = list(self.image_dirs.keys())
        current_idx = names.index(self.current_image_dir_name)
        new_idx = (current_idx + 1) % len(names)
        self.current_image_dir_name = names[new_idx]
        self.image_dir = self.image_dirs[self.current_image_dir_name]
        self.image_index = 0
        self.load_image_files()
        self.update_ui()

    def load_image_files(self):
        if os.path.exists(self.image_dir):
            self.image_files = sorted(f for f in os.listdir(self.image_dir) if f.lower().endswith((".jpg", ".jpeg", ".png")))
        else:
            self.image_files = []

    def ui_lidar_mode(self):
        self.lidar_frame.grid(row=0, column=0, rowspan=2, columnspan=2, sticky="nsew")
        for i in range(2): self.lidar_frame.grid_rowconfigure(i, weight=1)
        for i in range(2): self.lidar_frame.grid_columnconfigure(i, weight=1)
        tk.Label(self.button_center_frame, text="LiDAR Data", font=("Arial", 16)).grid(row=0, column=0, rowspan=3, columnspan=3, sticky="ew")

    def ui_gps_mode(self):
        self.gps_frame.grid(row=0, column=0, rowspan=2, columnspan=2, sticky="nsew")
        for i in range(3): self.gps_frame.grid_rowconfigure(i, weight=1)
        for i in range(2): self.gps_frame.grid_columnconfigure(i, weight=1)
        tk.Label(self.button_center_frame, text="GPS and IMU Data", font=("Arial", 16)).grid(row=0, column=0, rowspan=3, columnspan=3, sticky="ew")

    def ui_digit_mode(self):
        self.digit_frame.grid(row=0, column=0, rowspan=2, columnspan=2, sticky="nsew")
        tk.Label(self.button_center_frame, text="Digit Detection Stream", font=("Arial", 16)).grid(row=0, column=0, columnspan=3, rowspan=3, sticky="ew")

    def camera_annotated_callback(self, msg):
        if self.mode != 0:
            return
        now = time.time()
        if now - self.last_camera_image_time < 0.2:
            return
        self.last_camera_image_time = now
        frame = self.camera_annotated_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.root.after(0, lambda: self.update_live_image(frame))

    def update_live_image(self, frame):
        win_width = max(self.live_label.winfo_width(), 100)
        win_height = max(self.live_label.winfo_height(), 100)
        img_pil = PilImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        img_pil = img_pil.resize((win_width, win_height - 60), PilImage.Resampling.BILINEAR)
        img_tk = ImageTk.PhotoImage(img_pil)
        self.live_label.config(image=img_tk)
        self.live_label.image = img_tk

    def digit_annotated_callback(self, msg):
        if self.mode != 4:
            return
        now = time.time()
        if now - self.last_digit_image_time < 0.2:
            return
        self.last_digit_image_time = now
        frame = self.camera_annotated_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.root.after(0, lambda: self.update_digit_image(frame))

    def update_digit_image(self, frame):
        win_width = max(self.digit_label.winfo_width(), 100)
        win_height = max(self.digit_label.winfo_height(), 100)
        img_pil = PilImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        img_pil = img_pil.resize((win_width, win_height - 60), PilImage.Resampling.BILINEAR)
        img_tk = ImageTk.PhotoImage(img_pil)
        self.digit_label.config(image=img_tk)
        self.digit_label.image = img_tk

    def gps_cb(self, msg):
        if self.mode != 3:
            return
        tk.Label(self.gps_frame, text=f'Latitude: {msg.latitude}', font=("Arial", 16)).grid(row=0,column=0, sticky='ew')
        tk.Label(self.content_frame, text=f'Longitude: {msg.longitude}', font=("Arial", 16)).grid(row=1,column=0, sticky='ew')

    def obstacle_cb(self, msg):
        if len(msg.data) != 3:
            return
        obstacle = msg.data[0] != 0.0
        if self.mode != 2:
            return
        tk.Label(self.lidar_frame, text=f"OBSTACLE DETECTED? {'TRUE' if obstacle else 'FALSE'}", font=("Arial", 16)).grid(row=0, column=0, columnspan=2, sticky='nsew')
        tk.Label(self.lidar_frame, text=f"Minimum Angle: {msg.data[2]}", font=("Arial", 16)).grid(row=1, column=0, sticky='nsew')
        tk.Label(self.lidar_frame, text=f"Minimum Range: {msg.data[1]}", font=("Arial", 16)).grid(row=1, column=1, sticky='nsew')

    def imu_cb(self, msg):
        if len(msg.data) != 3 or self.mode != 3:
            return
        pitch, roll, heading = msg.data
        tk.Label(self.gps_frame, text=f'Pitch: {pitch}', font=("Arial", 16)).grid(row=0,column=1, sticky='ew')
        tk.Label(self.gps_frame, text=f'Roll: {roll}', font=("Arial", 16)).grid(row=1,column=1, sticky='ew')
        tk.Label(self.gps_frame, text=f'Heading: {heading}', font=("Arial", 16)).grid(row=2,column=1, sticky='ew')

    def display_image(self):
        if not self.image_files:
            return
        file_path = os.path.join(self.image_dir, self.image_files[self.image_index])
        try:
            img_pil = PilImage.open(file_path)
            win_width = max(self.image_label.winfo_width(), 100)
            win_height = max(self.image_label.winfo_height(), 100)
            img_pil = img_pil.resize((win_width, win_height - 60), PilImage.Resampling.BILINEAR)
            img_tk = ImageTk.PhotoImage(img_pil)
            self.image_label.config(image=img_tk)
            self.image_label.image = img_tk
            self.image_label.update_idletasks()
        except Exception as e:
            print(f"Error opening image: {e}")

    def next_image(self):
        if self.mode == 1 and self.image_files:
            self.image_index = (self.image_index + 1) % len(self.image_files)
            self.display_image()

    def prev_image(self):
        if self.mode == 1 and self.image_files:
            self.image_index = (self.image_index - 1) % len(self.image_files)
            self.display_image()

def main():
    rclpy.init()
    root = tk.Tk()
    root.title("ROS2 Camera Viewer")
    root.geometry("800x600")

    node = CameraViewer(root)
    node.get_logger().info("Camera Viewer Node started")

    import threading
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()