import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import os
from datetime import datetime
import pkg_resources

CLASS_COLORS = {
    'cone-orange': (0, 140, 255),
    'bucket-red': (0, 0, 255),
    'cone-yellow-green': (0, 255, 0)
}

TARGET_CLASSES = set(CLASS_COLORS.keys())
EXTRA_CLASSES = {'bucket-red', 'cone-yellow-green'}

class YoloCenterDetector(Node):
    def __init__(self):
        super().__init__('yolo_center_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, '/target_detected', 10)

        model_path = os.path.join(pkg_resources.resource_filename('vision_yolo_detector', 'model'), 'best.pt')
        self.model = YOLO(model_path)

        os.makedirs("center_detected_images", exist_ok=True)

        self.detected_cone = False
        self.detected_extra = False

        self.get_logger().info("🚀 Dual-target YOLO center detector running.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]

        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2

        closest_label = None
        closest_box = None
        closest_dist = float('inf')

        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            if label not in TARGET_CLASSES:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            obj_cx = (x1 + x2) // 2
            obj_cy = (y1 + y2) // 2
            dist = np.hypot(center_x - obj_cx, center_y - obj_cy)

            if dist < closest_dist:
                closest_dist = dist
                closest_label = label
                closest_box = (x1, y1, x2, y2)

        annotated = frame.copy()

        if closest_label:
            x1, y1, x2, y2 = closest_box
            color = CLASS_COLORS[closest_label]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, f"{closest_label}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            self.publisher_.publish(String(data=closest_label))
            self.get_logger().info(f"📢 Published: {closest_label}")

            if closest_dist < 50:
                if closest_label == 'cone-orange' and not self.detected_cone:
                    self._save_image(annotated, closest_label)
                    self.detected_cone = True
                elif closest_label in EXTRA_CLASSES and not self.detected_extra:
                    self._save_image(annotated, closest_label)
                    self.detected_extra = True
        else:
            self.publisher_.publish(String(data="No-target"))
            self.get_logger().info("❌ No valid target detected.")

    def _save_image(self, image, label):
        filename = f"center_detected_images/{label}_{datetime.now().strftime('%H%M%S')}.jpg"
        cv2.imwrite(filename, image)
        self.get_logger().info(f"📸 Saved: {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloCenterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
