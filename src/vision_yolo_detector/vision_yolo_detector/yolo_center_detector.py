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

class YoloCenterDetector(Node):
    def __init__(self):
        super().__init__('yolo_center_detector')
        self.bridge = CvBridge()

        # Subscriptions
        self.rgb_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.publisher_ = self.create_publisher(String, '/target_detected', 10)
        self.annotated_pub = self.create_publisher(Image, '/target_annotated/image_raw', 10)

        # Load model
        model_path = os.path.join(pkg_resources.resource_filename('vision_yolo_detector', 'model'), 'best_object.pt')
        self.model = YOLO(model_path)
        self.all_classes = set(self.model.names.values())

        self.get_logger().info("✅ YOLO center detector initialized (no depth).")

    def get_color_for_class(self, label):
        h = hash(label) % 180
        hsv_color = np.uint8([[[h, 255, 255]]])
        bgr_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0][0]
        return tuple(int(c) for c in bgr_color)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]
        height, width, _ = frame.shape
        center_x = width // 2

        detections = []
        annotated = frame.copy()

        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            offset = cx - center_x

            detections.append(f"{label}, offset={offset}")

            color = self.get_color_for_class(label)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, f"{label}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        msg_out = " | ".join(detections) if detections else "No valid target"
        self.publisher_.publish(String(data=msg_out))
        self.get_logger().info(f"📢 {msg_out}")

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.annotated_pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloCenterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
