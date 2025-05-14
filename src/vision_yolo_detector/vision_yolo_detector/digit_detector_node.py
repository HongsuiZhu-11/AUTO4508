import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import cv2
import numpy as np
import pkg_resources
from datetime import datetime

class DigitDetector(Node):
    def __init__(self):
        super().__init__('digit_detector_node')
        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # Publishers
        self.publisher_ = self.create_publisher(String, '/digit_detected', 10)
        self.annotated_pub = self.create_publisher(Image, '/digit_annotated/image_raw', 10)

        # Load model
        model_path = os.path.join(pkg_resources.resource_filename(
            'vision_yolo_detector', 'model'), 'best_number.pt')
        self.model = YOLO(model_path)

        # State
        self.latest_depth = None
        self.saved_labels = set()
        os.makedirs("digit_detected_images", exist_ok=True)

        self.get_logger().info(" Digit detector initialized (video + save-once per digit).")

    def depth_callback(self, msg):
        # Get raw mono16 depth (in mm)
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def rgb_callback(self, msg):
        if self.latest_depth is None:
            self.get_logger().warn(" Waiting for depth frame...")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]
        height, width, _ = frame.shape
        center_x = width // 2

        detections = []
        annotated = frame.copy()

        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            offset = cx - center_x

            # Read depth at center point
            depth = -1
            if 0 <= cy < self.latest_depth.shape[0] and 0 <= cx < self.latest_depth.shape[1]:
                depth = int(self.latest_depth[cy, cx])

            detections.append(f"{label}, offset={offset}, depth_mm={depth}")

            # Draw annotated image
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"{label}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Save image for new label
            if label not in self.saved_labels:
                self.saved_labels.add(label)
                filename = f"digit_detected_images/{label}_{datetime.now().strftime('%H%M%S')}.jpg"
                cv2.imwrite(filename, annotated)
                self.get_logger().info(f" Saved image for digit '{label}': {filename}")

        # Publish detection message
        if detections:
            msg_out = " | ".join(detections)
        else:
            msg_out = "No digits detected"
        self.publisher_.publish(String(data=msg_out))
        self.get_logger().info(f"message: {msg_out}")

        # Publish annotated image
        msg_img = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.annotated_pub.publish(msg_img)

def main(args=None):
    rclpy.init(args=args)
    node = DigitDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
