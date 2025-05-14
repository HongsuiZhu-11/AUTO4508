import rclpy
from rclpy.node import Node
import cv2
import os
import tkinter as tk
from PIL import Image as PilImage, ImageTk
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraViewer(Node):
    def __init__(self, root):
        super().__init__('camera_viewer')
        self.root = root
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.live_mode = True  # Track whether live mode is active
        
        # Image Directory
        self.image_dir = "/home/team10/AUTO4508/center_detected_images"
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)  # Create the directory if it doesn't exist
            self.get_logger().info(f"Directory '{self.image_dir}' created.")

        self.image_files = sorted([f for f in os.listdir(self.image_dir) if f.endswith((".jpg", ".png", ".jpeg"))])
        self.image_index = 0  # Track which image is displayed

        # Configure grid layout for root
        for i in range(8):  # 8 rows total
            self.root.grid_rowconfigure(i, weight=1)
        for i in range(3):  # 3 columns total
            self.root.grid_columnconfigure(i, weight=1)

        # Buttons frame (Row 0)
        button_frame = tk.Frame(self.root)
        button_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=5)
        self.root.grid_rowconfigure(0, minsize=50)  # Set minimum row height for buttons

        # Force button frame to expand properly
        button_frame.grid_columnconfigure(0, weight=1)
        button_frame.grid_columnconfigure(1, weight=1)
        button_frame.grid_columnconfigure(2, weight=1)
        button_frame.grid_columnconfigure(3, weight=1)

        # Buttons within the frame
        tk.Button(button_frame, text="Load Image", command=self.load_image).grid(row=0, column=0, padx=5, sticky="ew")
        tk.Button(button_frame, text="Return to Live Feed", command=self.return_to_live).grid(row=0, column=1, padx=5, sticky="ew")
        tk.Button(button_frame, text="← Prev", command=self.prev_image).grid(row=0, column=2, padx=5, sticky="ew")
        tk.Button(button_frame, text="Next →", command=self.next_image).grid(row=0, column=3, padx=5, sticky="ew")

        # Expand Image to Fill Space (Spans Rows 1-7)
        self.label = tk.Label(self.root)
        self.label.grid(row=1, column=0, rowspan=7, columnspan=3, sticky="nsew")  # Ensure full expansion

    def image_callback(self, msg):
        """ Convert ROS2 Image message to OpenCV format and update Tkinter UI """
        if not self.live_mode:
            return  # Ignore live feed if viewing static images

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Get window dimensions dynamically
        win_width = self.root.winfo_width()
        win_height = self.root.winfo_height()

        # Resize the image while keeping aspect ratio
        img_pil = PilImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        img_pil.thumbnail((win_width, win_height - 60), PilImage.Resampling.LANCZOS)  # Preserve aspect ratio

        img_tk = ImageTk.PhotoImage(img_pil)

        self.label.config(image=img_tk)
        self.label.image = img_tk  # Prevent garbage collection

    def load_image(self):
        """ Load the first image from the directory and disable live mode """
        if not self.image_files:
            print("No images found in directory.")
            return
        
        self.live_mode = False  # Stop live feed
        self.image_index = 0  # Start at first image
        self.display_image()

    def display_image(self):
        """ Display the current image based on self.image_index """
        if not self.image_files:
            print("No images available.")
            return
        
        file_path = os.path.join(self.image_dir, self.image_files[self.image_index])
        try:
            img_pil = PilImage.open(file_path)

            # Resize for proper display
            win_width = self.root.winfo_width()
            win_height = self.root.winfo_height()
            img_pil.thumbnail((win_width, win_height - 60), PilImage.Resampling.LANCZOS)

            img_tk = ImageTk.PhotoImage(img_pil)

            self.label.config(image=img_tk)
            self.label.image = img_tk  # Prevent garbage collection
            self.label.update_idletasks()  # Force UI refresh
        except Exception as e:
            print(f"Error opening image: {e}")

    def next_image(self):
        """ Move to the next image in the folder """
        if self.live_mode or not self.image_files:
            return
        
        self.image_index = (self.image_index + 1) % len(self.image_files)
        self.display_image()

    def prev_image(self):
        """ Move to the previous image in the folder """
        if self.live_mode or not self.image_files:
            return
        
        self.image_index = (self.image_index - 1) % len(self.image_files)
        self.display_image()

    def return_to_live(self):
        """ Switch back to live camera feed """
        self.live_mode = True

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
