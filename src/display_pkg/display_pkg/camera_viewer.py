import rclpy
from rclpy.node import Node
import cv2
import os
import tkinter as tk
from PIL import Image as PilImage, ImageTk
from sensor_msgs.msg import Image
# GPS
from sensor_msgs.msg import NavSatFix
# Lidar, IMU
from std_msgs.msg import String, Float32, Int32, Float32MultiArray
from cv_bridge import CvBridge

MODES = {'live', 'image', 'lidar', 'gps'}  # Define modes for the application

class CameraViewer(Node):
    def __init__(self, root):
        super().__init__('camera_viewer')
        self.root = root

        self.camera_annotated_bridge = CvBridge()
        self.camera_annotated_subscription = self.create_subscription(Image, '/target_annotated/image_raw', self.camera_annotated_callback, 10)
        # GPS
        self.gps_sub = self.create_subscription(NavSatFix, "fix", self.gps_cb, 10)
        # Lidar
        self.lidar_sub = self.create_subscription(Float32MultiArray, "obstacle_team10", self.obstacle_cb, 10)
        # IMU
        self.imu_sub = self.create_subscription(Float32MultiArray, "imu_team10", self.imu_cb, 10)
        
        self.mode = 0  # Track whether live mode is active
        
        # Image Directory
        self.image_dir = "/home/team10/AUTO4508/center_detected_images"
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)  # Create the directory if it doesn't exist
            self.get_logger().info(f"Directory '{self.image_dir}' created.")

        self.image_files = sorted([f for f in os.listdir(self.image_dir) if f.endswith((".jpg", ".png", ".jpeg"))])
        self.image_index = 0  # Track which image is displayed

        self.build_base_ui()  # Build the base UI layout
        self.update_ui()  # Build the UI layout

    def build_base_ui(self):
        """ Build the Tkinter UI layout """
        # Configure grid layout for root
        for i in range(8):  # 8 rows total
            self.root.grid_rowconfigure(i, weight=1)
        for i in range(5):  # 5 columns total
            self.root.grid_columnconfigure(i, weight=1)
        
        # Buttons frame (Row 0)
        self.button_frame = tk.Frame(self.root)
        self.button_frame.grid(row=0, column=0, columnspan=5, sticky="ew", pady=5)
        self.root.grid_rowconfigure(0, minsize=50)  # Set minimum row height for buttons

        # Force button frame to expand properly
        for i in range(5):
            self.button_frame.grid_columnconfigure(i, weight=1)
        for i in range(2):
            self.button_frame.grid_rowconfigure(i, weight=1)

        # Buttons within the frame
        tk.Button(self.button_frame, text="← Prev", command=self.prev_mode).grid(row=0, column=0, rowspan=2, padx=5, sticky="ew")
        tk.Button(self.button_frame, text="Next →", command=self.next_mode).grid(row=0, column=4, rowspan=2, padx=5, sticky="ew")

        # Frame to hold the title label and any other buttons
        self.button_center_frame = tk.Frame(self.button_frame)
        self.button_center_frame.grid(row=0, column=1, columnspan=3, sticky="ew")
        for i in range(3):
            self.button_center_frame.grid_columnconfigure(i, weight=1)
        for i in range(3):
            self.button_center_frame.grid_rowconfigure(i, weight=1)

        # Frame to hold the content labels
        self.content_frame = tk.Frame(self.root)
        self.content_frame.grid(row=1, column=0, rowspan=7, columnspan=5, sticky="nsew")
        for i in range(2):
            self.content_frame.grid_rowconfigure(i, weight=1)
        for i in range(2):
            self.content_frame.grid_columnconfigure(i, weight=1)
        
        self.live_label = tk.Label(self.content_frame)
        self.image_label = tk.Label(self.content_frame)
        self.lidar_label = tk.Label(self.content_frame)
        self.gps_label = tk.Label(self.content_frame)
        

    def next_mode(self):
        """ Switch to the next mode """
        self.mode = (self.mode + 1) % len(MODES)
        self.update_ui()
        self.get_logger().info(f"Switched to mode: {list(MODES)[self.mode]}")
    
    def prev_mode(self):
        """ Switch to the previous mode """
        self.mode = (self.mode - 1) % len(MODES)
        self.update_ui()
        self.get_logger().info(f"Switched to mode: {list(MODES)[self.mode]}")
    
    def update_ui(self):
        """ Update the UI based on the current mode """
        # Build consistent base UI
        for widget in self.button_center_frame.grid_slaves():
            widget.grid_forget()
        for widget in self.content_frame.grid_slaves():
            widget.grid_forget()
        self.build_base_ui()  # Rebuild the base UI
        
        # Clear Variable content
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
        else:
            self.get_logger().error("Invalid mode selected.")
            return

    def ui_live_mode(self):
        """ Update UI for live mode """
        # Place Label that holds live feed
        self.live_label.grid(row=0, column=0, rowspan=2, columnspan=2, sticky="nsew")  # Ensure full expansion
        # Title
        tk.Label(self.button_center_frame, text="Live Camera Feed", font=("Arial", 16)).grid(row=0, column=0, columnspan=3, rowspan=3, sticky="ew")
        

    def ui_image_mode(self):
        """ Update the UI to show the image viewer """
        # place Label that holds image viewer
        self.image_label.grid(row=0, column=0, rowspan=2, columnspan=2, sticky="nsew")
        # Title
        tk.Label(self.button_center_frame, text="Image Viewer", font=("Arial", 16)).grid(row=0, column=0, columnspan=3, rowspan=1, sticky="ew")
        # Navigation buttons
        tk.Button(self.button_center_frame, text="← Prev", command=self.prev_image).grid(row=1, column=0, rowspan=2, columnspan=1, padx=5, sticky="ew")
        tk.Button(self.button_center_frame, text="Next →", command=self.next_image).grid(row=1, column=3, rowspan=2, columnspan=1,padx=5, sticky="ew")
    

    def ui_lidar_mode(self):
        """ Update the UI to show the LiDAR viewer """
        # Place Label that holds LiDAR viewer
        self.lidar_label.grid(row=0, column=0, rowspan=2, columnspan=2, sticky="nsew")
        # Title
        tk.Label(self.button_center_frame, text="LiDAR Data", font=("Arial", 16)).grid(row=0, column=0, rowspan=3, columnspan=3, sticky="ew")
    

    def ui_gps_mode(self):
        """ Update the UI to show the GPS viewer """
        # Place Label that holds GPS viewer
        self.gps_label.grid(row=1, column=0, rowspan=7, columnspan=3, sticky="nsew")
        # Title
        tk.Label(self.button_center_frame, text="GPS Data", font=("Arial", 16)).grid(row=0, column=0, rowspan=3, columnspan=3, sticky="ew")


    def camera_annotated_callback(self, msg):
        """ Convert ROS2 Image message to OpenCV format and update Tkinter UI """
        if not self.mode == 0:
            return  # Ignore live feed if viewing static images

        frame = self.camera_annotated_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Get window dimensions dynamically
        win_width = self.live_label.winfo_width()
        win_height = self.live_label.winfo_height()

        # Resize the image while keeping aspect ratio
        img_pil = PilImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        img_pil.thumbnail((win_width, win_height - 60), PilImage.Resampling.LANCZOS)  # Preserve aspect ratio

        img_tk = ImageTk.PhotoImage(img_pil)

        self.live_label.config(image=img_tk)
        self.live_label.image = img_tk  # Prevent garbage collection

    def gps_cb(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude

    def obstacle_cb(self, msg):
        if (len(msg.data) != 3):
            print('obstacle_cb Massage not matching')
        if (msg.data[0] == 0.0):
            obstacle = False
        else:
            obstacle = True
        
        min_range = msg.data[1]
        min_angle = msg.data[2]
    
    def imu_cb(self, msg):
        if (len(msg.data) != 3):
            print('imu_cb Massage not matching')
        pitch = msg.data[0]
        roll = msg.data[1]
        heading = msg.data[2]
        
    def display_image(self):
        """ Display the current image based on self.image_index """
        if not self.image_files:
            print("No images available.")
            return
        
        file_path = os.path.join(self.image_dir, self.image_files[self.image_index])
        try:
            img_pil = PilImage.open(file_path)

            # Resize for proper display
            win_width = self.label.winfo_width()
            win_height = self.label.winfo_height()
            img_pil.thumbnail((win_width, win_height - 60), PilImage.Resampling.LANCZOS)

            img_tk = ImageTk.PhotoImage(img_pil)

            self.image_label.config(image=img_tk)
            self.image_label.image = img_tk  # Prevent garbage collection
            self.image_label.update_idletasks()  # Force UI refresh
        except Exception as e:
            print(f"Error opening image: {e}")

    def next_image(self):
        """ Move to the next image in the folder """
        if not self.mode == 1 or not self.image_files:
            return
        
        self.image_index = (self.image_index + 1) % len(self.image_files)
        self.display_image()

    def prev_image(self):
        """ Move to the previous image in the folder """
        if not self.mode == 1 or not self.image_files:
            return
        
        self.image_index = (self.image_index - 1) % len(self.image_files)
        self.display_image()

def main():
    rclpy.init()
    root = tk.Tk()
    root.title("ROS2 Camera Viewer")
    root.geometry("800x600")  # Set window size for better visibility

    node = CameraViewer(root)
    node.get_logger().info("Camera Viewer Node started")

    # Run ROS2 spinning in a separate thread
    def ros_spin():
        rclpy.spin(node)

    import threading
    threading.Thread(target=ros_spin, daemon=True).start()

    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
