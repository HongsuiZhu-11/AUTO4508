import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import os
import pkg_resources

class UnifiedDetector(Node):
    def __init__(self):
        super().__init__('unified_detector_node')
        self.bridge = CvBridge()

        # Publishers
        self.pub_digit_text = self.create_publisher(String, '/digit_detected', 10)
        self.pub_digit_img = self.create_publisher(Image, '/digit_annotated/image_raw', 10)
        self.pub_target_text = self.create_publisher(String, '/target_detected', 10)
        self.pub_target_img = self.create_publisher(Image, '/target_annotated/image_raw', 10)

        # Subscriptions
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Load YOLO models
        model_dir = pkg_resources.resource_filename('vision_yolo_detector', 'model')
        self.model_digit = YOLO(os.path.join(model_dir, 'best_number.pt'))
        self.model_target = YOLO(os.path.join(model_dir, 'best_object.pt'))

        self.get_logger().info("✅ Unified YOLO detector initialized.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape
        center_x = width // 2

        # Clone frame for separate annotation
        digit_annotated = frame.copy()
        target_annotated = frame.copy()

        # Run both detections
        self.process_digits(frame.copy(), digit_annotated, width, center_x)
        self.process_targets(frame.copy(), target_annotated, center_x)

    def process_digits(self, frame, annotated, width, center_x):
        results = self.model_digit(frame)[0]
        detections = []

        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model_digit.names[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            offset = cx - center_x

            # Check for white background
            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                continue
            center_pixel = roi[roi.shape[0] // 2, roi.shape[1] // 2]
            if not np.all(center_pixel > 200):
                continue

            detections.append(f"{label}, offset={offset}")
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"{label}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        msg_out = " | ".join(detections) if detections else "No digits detected"
        self.pub_digit_text.publish(String(data=msg_out))
        self.pub_digit_img.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        self.get_logger().info(f"🔢 {msg_out}")

    def process_targets(self, frame, annotated, center_x):
        results = self.model_target(frame)[0]
        detections = []

        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model_target.names[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            offset = cx - center_x

            detections.append(f"{label}, offset={offset}")
            color = self.get_color_for_class(label)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, f"{label}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        msg_out = " | ".join(detections) if detections else "No valid target"
        self.pub_target_text.publish(String(data=msg_out))
        self.pub_target_img.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        self.get_logger().info(f"🎯 {msg_out}")

    def get_color_for_class(self, label):
        h = hash(label) % 180
        hsv_color = np.uint8([[[h, 255, 255]]])
        bgr_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0][0]
        return tuple(int(c) for c in bgr_color)

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
