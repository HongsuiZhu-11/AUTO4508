import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import re
from collections import defaultdict

class CameraSaver(Node):
    def __init__(self):
        super().__init__('camera_saver')
        self.bridge = CvBridge()

        self.save_dir_digits = '/home/team10/saved_images/digits'
        self.save_dir_objects = '/home/team10/saved_images/objects'
        os.makedirs(self.save_dir_digits, exist_ok=True)
        os.makedirs(self.save_dir_objects, exist_ok=True)

        self.previous_object_label = None
        self.saved_digit_labels = set()

        self.latest_detections = {
            'digit': None,
            'object': None
        }
        self.latest_images = {
            'digit': None,
            'object': None
        }
        self.latest_lidar_ranges = []

        self.detection_counts = defaultdict(int)  # ✅ track repeated detections
        self.required_count = 3  # ✅ require 3 consecutive detections to trigger save

        self.create_subscription(String, '/digit_detected', self.digit_detected_callback, 10)
        self.create_subscription(Image, '/digit_annotated/image_raw', self.digit_image_callback, 10)
        self.create_subscription(String, '/target_detected', self.object_detected_callback, 10)
        self.create_subscription(Image, '/target_annotated/image_raw', self.object_image_callback, 10)
        self.create_subscription(Float32MultiArray, '/front_lidar_range', self.lidar_callback, 10)

        self.status_pub = self.create_publisher(String, '/camera_status', 10)
        self.align_pub = self.create_publisher(String, '/camera_request_align', 10)
        self.saved_pub = self.create_publisher(String, '/camera_saved_image', 10)

        self.get_logger().info("📸 Camera Saver Node ready.")

    def lidar_callback(self, msg):
        self.latest_lidar_ranges = msg.data

    def digit_detected_callback(self, msg):
        self.latest_detections['digit'] = msg.data
        self.check_digit()

    def object_detected_callback(self, msg):
        self.latest_detections['object'] = msg.data
        self.check_object()

    def digit_image_callback(self, msg):
        self.latest_images['digit'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def object_image_callback(self, msg):
        self.latest_images['object'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def check_digit(self):
        detection = self.latest_detections['digit']
        frame = self.latest_images['digit']
        if not detection:
            self.get_logger().info("⏳ No digit detection data.")
            return
        if frame is None:
            self.get_logger().warn("🚫 Frame is None — image not received or too late")
            return

        self.get_logger().info(f"✅ Entered check_digit with detection: {detection}")

        for entry in detection.split('|'):
            entry = entry.strip()
            if not entry or 'offset=' not in entry:
                continue
            try:
                label = entry.split(',')[0].strip()
                offset = int(entry.split('offset=')[-1].strip())
            except Exception as e:
                self.get_logger().warn(f"❌ Failed to parse digit entry: {entry} ({e})")
                continue

            self.get_logger().info(f"🔍 Processing label={label}, offset={offset}")

            if label in self.saved_digit_labels:
                self.get_logger().info(f"⏩ Already saved digit: {label}")
                continue

            if abs(offset) > 50:
                self.get_logger().info(f"📐 Skipping {label}: offset too large ({offset})")
                self.align_pub.publish(String(data=f"digit:{label}:offset={offset}"))
                self.status_pub.publish(String(data=f"digit:{label} not centered"))
                continue

            self.detection_counts[label] += 1
            self.get_logger().info(f"🧮 Detection count for {label}: {self.detection_counts[label]}")

            if self.detection_counts[label] < self.required_count:
                continue

            self.get_logger().info(f"💾 Saving digit {label} at offset {offset}")
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(self.save_dir_digits, f"{label}_{timestamp}.jpg")
            cv2.imwrite(save_path, frame)
            self.saved_digit_labels.add(label)
            self.detection_counts[label] = 0
            self.status_pub.publish(String(data=f"📸 Saved digit:{label}"))
            self.saved_pub.publish(String(data=save_path))
            self.get_logger().info(f"✅ Saved digit {label} at {save_path}")

    def check_object(self):
        detection = self.latest_detections['object']
        frame = self.latest_images['object']
        if frame is None:
            return

        if not detection or 'No' in detection:
            if self.previous_object_label is not None:
                self.previous_object_label = None
            return

        self.get_logger().info(f"✅ Entered check_object with detection: {detection}")

        for entry in detection.split('|'):
            entry = entry.strip()
            if not entry or 'offset=' not in entry:
                continue
            try:
                label = entry.split(',')[0].strip()
                offset = int(entry.split('offset=')[-1].strip())
            except Exception as e:
                self.get_logger().warn(f"❌ Failed to parse object entry: {entry} ({e})")
                continue

            self.get_logger().info(f"🔍 Processing label={label}, offset={offset}")

            if label == self.previous_object_label:
                continue

            if abs(offset) > 50:
                self.get_logger().info(f"📐 Skipping {label}: offset too large ({offset})")
                self.align_pub.publish(String(data=f"object:{label}:offset={offset}"))
                self.status_pub.publish(String(data=f"object:{label} not centered"))
                continue

            self.detection_counts[label] += 1
            self.get_logger().info(f"🧮 Detection count for {label}: {self.detection_counts[label]}")

            if self.detection_counts[label] < self.required_count:
                continue

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(self.save_dir_objects, f"{label}_{timestamp}.jpg")
            cv2.imwrite(save_path, frame)
            self.previous_object_label = label
            self.detection_counts[label] = 0
            self.status_pub.publish(String(data=f"📸 Saved object:{label}"))
            self.saved_pub.publish(String(data=save_path))
            self.get_logger().info(f"✅ Saved object {label} at {save_path}")

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
