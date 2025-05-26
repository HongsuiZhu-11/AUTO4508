import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import numpy as np
import cv2

class OakCameraPublisher(Node):
    def __init__(self):
        super().__init__('oak_camera_publisher')

        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        self.pipeline = dai.Pipeline()

        cam_rgb = self.pipeline.createColorCamera()
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(10)

        xout_rgb = self.pipeline.createXLinkOut()
        xout_rgb.setStreamName("video")
        cam_rgb.video.link(xout_rgb.input)

        self.device = dai.Device(self.pipeline)
        self.video_queue = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("✅ OAK camera node with RGB publishing at 720p started.")

    def timer_callback(self):
        if self.video_queue.has():
            in_video = self.video_queue.get()
            frame = in_video.getCvFrame()

            # Resize from 1080p to 720p (1920x1080 -> 1280x720)
            frame_resized = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_LINEAR)

            msg_rgb = self.bridge.cv2_to_imgmsg(frame_resized, encoding="bgr8")
            self.rgb_pub.publish(msg_rgb)

    def destroy_node(self):
        self.get_logger().info("Releasing OAK resources.")
        self.device.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OakCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()