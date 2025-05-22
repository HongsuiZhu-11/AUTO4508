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

class DigitDetector(Node):
    def __init__(self):
        super().__init__('digit_detector_node')
        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, 10)

        # Publishers
        self.publisher_ = self.create_publisher(String, '/digit_detected', 10)
        self.annotated_pub = self.create_publisher(Image, '/digit_annotated/image_raw', 10)

        # Load model
        model_path = os.path.join(pkg_resources.resource_filename(
            'vision_yolo_detector', 'model'), 'best_number.pt')
        self.model = YOLO(model_path)

        self.get_logger().info("✅ Digit detector initialized (detect all, filter by white regions only).")

    def is_white_background(self, image, x1, y1, x2, y2):
        region = image[y1:y2, x1:x2]
        if region.size == 0:
            return False
        mean_color = np.mean(region, axis=(0, 1))
        return all(mean_color > 200)

    def rgb_callback(self, msg):
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
            offset = cx - center_x

            # ✅ 仅对白底区域内的数字进行处理
            if not self.is_white_background(frame, x1, y1, x2, y2):
                continue

            detections.append(f"{label}, offset={offset}")

            # Draw annotated image
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"{label}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        msg_out = " | ".join(detections) if detections else "No digits detected"
        self.publisher_.publish(String(data=msg_out))
        self.get_logger().info(f"📢 {msg_out}")

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
