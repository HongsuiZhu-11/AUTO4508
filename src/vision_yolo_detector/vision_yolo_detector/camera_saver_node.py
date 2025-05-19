import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray  # 用于模拟 LiDAR 前方 30° 的距离数据
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import re

class CameraSaver(Node):
    def __init__(self):
        super().__init__('camera_saver')
        self.bridge = CvBridge()

        self.save_dir_digits = '/home/team10/saved_images/digits'
        self.save_dir_objects = '/home/team10/saved_images/objects'
        os.makedirs(self.save_dir_digits, exist_ok=True)
        os.makedirs(self.save_dir_objects, exist_ok=True)

        self.saved_labels = set()
        self.latest_detections = {
            'digit': None,
            'object': None
        }
        self.latest_images = {
            'digit': None,
            'object': None
        }
        self.latest_lidar_ranges = []  # 模拟接收前方30°激光雷达数据

        # Subscriptions
        self.create_subscription(String, '/digit_detected', self.digit_detected_callback, 10)
        self.create_subscription(Image, '/digit_annotated/image_raw', self.digit_image_callback, 10)
        self.create_subscription(String, '/target_detected', self.object_detected_callback, 10)
        self.create_subscription(Image, '/target_annotated/image_raw', self.object_image_callback, 10)
        self.create_subscription(Float32MultiArray, '/front_lidar_range', self.lidar_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/camera_status', 10)
        self.align_pub = self.create_publisher(String, '/camera_request_align', 10)
        self.saved_pub = self.create_publisher(String, '/camera_saved_image', 10)

        self.get_logger().info("📸 Camera Saver Node initialized (watching digits & objects)")

    def lidar_callback(self, msg):
        self.latest_lidar_ranges = msg.data

    def digit_detected_callback(self, msg):
        self.latest_detections['digit'] = msg.data
        self.check_and_save('digit')

    def object_detected_callback(self, msg):
        self.latest_detections['object'] = msg.data
        self.check_and_save('object')

    def digit_image_callback(self, msg):
        self.latest_images['digit'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def object_image_callback(self, msg):
        self.latest_images['object'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def check_and_save(self, mode):
        detection = self.latest_detections[mode]
        frame = self.latest_images[mode]
        if not detection or frame is None:
            return

        if 'No' in detection:
            return  # No detection

        for entry in detection.split('|'):
            parts = re.split(r'[,:= ]+', entry.strip())
            if len(parts) < 6:
                continue
            label = parts[0]
            offset = int(parts[2])
            depth_mm = int(parts[4])

            # Skip if already saved
            if label in self.saved_labels:
                continue

            # Check if too far from center
            if abs(offset) > 50:
                self.align_pub.publish(String(data=f"{mode}:{label}:offset={offset}"))
                self.status_pub.publish(String(data=f"{mode}:{label} not centered"))
                continue

            # Check if too far
            if depth_mm > 1500 or depth_mm <= 0:
                self.status_pub.publish(String(data=f"{mode}:{label} too far: {depth_mm}mm"))
                continue

            # All good: Save image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(
                self.save_dir_digits if mode == 'digit' else self.save_dir_objects,
                f"{label}_{timestamp}.jpg")
            cv2.imwrite(save_path, frame)
            self.saved_labels.add(label)
            self.status_pub.publish(String(data=f"📸 Saved {mode}:{label}"))
            self.saved_pub.publish(String(data=save_path))
            self.get_logger().info(f"✅ Saved {label} at {save_path}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
