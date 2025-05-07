import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import numpy as np

class OakCameraPublisher(Node):
    def __init__(self):
        super().__init__('oak_camera_publisher')

        # ROS Publisher
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # DepthAI Pipeline
        self.pipeline = dai.Pipeline()
        cam_rgb = self.pipeline.createColorCamera()
        xout_rgb = self.pipeline.createXLinkOut()

        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(10)

        xout_rgb.setStreamName("video")
        cam_rgb.video.link(xout_rgb.input)

        # Start device and stream
        self.device = dai.Device(self.pipeline)
        self.video_queue = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

        # ROS Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("OAK camera publisher started.")

    def timer_callback(self):
        in_video = self.video_queue.get()
        frame = in_video.getCvFrame()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

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
