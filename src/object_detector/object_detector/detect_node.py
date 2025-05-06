import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime
import os

class ColorShapeDetector(Node):
    def __init__(self):
        super().__init__('color_shape_detector')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        os.makedirs("detected_images", exist_ok=True)
        self.get_logger().info("🎯 Color+Shape detector started.")

    def image_callback(self, msg):
        self.get_logger().info("📥 Received one image. Processing...")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = self.detect_objects_by_color_shape(frame)
        annotated = frame.copy()

        for label, (x1, y1, x2, y2) in detections:
            color = (0, 255, 255) if "cone" in label else (0, 0, 255)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        filename = f"detected_images/detect_{datetime.now().strftime('%H%M%S')}.jpg"
        cv2.imwrite(filename, annotated)
        self.get_logger().info(f"📸 Saved image with {len(detections)} detections → {filename}")

        # ✅ 取消订阅并关闭节点（只处理一次）
        self.destroy_subscription(self.subscription)
        self.get_logger().info("👋 Shutting down after one detection.")
        rclpy.shutdown()


    def detect_objects_by_color_shape(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        COLOR_RANGES = {
            'cone_orange': ([5, 100, 100], [20, 255, 255]),
            'cone_yellow_green': ([25, 80, 80], [70, 255, 255]),
            'bucket_red': ([0, 100, 100], [10, 255, 255])
        }

        for label, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / float(h)

                if label.startswith("cone"):
                    if aspect_ratio < 0.8:
                        detections.append((label, (x, y, x + w, y + h)))
                elif label == "bucket_red":
                    if 0.8 < aspect_ratio < 1.2:
                        detections.append((label, (x, y, x + w, y + h)))

        return detections

def main(args=None):
    rclpy.init(args=args)
    node = ColorShapeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
