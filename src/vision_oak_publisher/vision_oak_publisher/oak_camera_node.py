import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import numpy as np

class OakCameraPublisher(Node):
    def __init__(self):
        super().__init__('oak_camera_publisher')

        # Bridge for converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Create publishers for RGB and Depth image topics
        self.rgb_pub = self.create_publisher(Image, 'camera/image_raw', 10)            # Publishes RGB images
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)    # Publishes depth images

        # Build DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Configure RGB camera
        cam_rgb = self.pipeline.createColorCamera()
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(10)

        # Configure left and right mono cameras for stereo depth
        mono_left = self.pipeline.createMonoCamera()
        mono_right = self.pipeline.createMonoCamera()
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # Stereo depth configuration
        stereo = self.pipeline.createStereoDepth()
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Output streams
        xout_rgb = self.pipeline.createXLinkOut()
        xout_rgb.setStreamName("video")
        cam_rgb.video.link(xout_rgb.input)

        xout_depth = self.pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # Start DepthAI device
        self.device = dai.Device(self.pipeline)
        self.video_queue = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)
        self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        # Timer to regularly fetch and publish frames
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("✅ OAK camera node with RGB + Depth publishing started.")

    def timer_callback(self):
        # Publish RGB image if available
        if self.video_queue.has():
            in_video = self.video_queue.get()
            frame = in_video.getCvFrame()
            msg_rgb = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")  # Convert OpenCV BGR to ROS Image
            self.rgb_pub.publish(msg_rgb)

        # Publish Depth image if available
        # Publish Depth image in mono16 (raw millimeters)
        if self.depth_queue.has():
            in_depth = self.depth_queue.get()
            depth_frame = in_depth.getFrame()  # uint16, in millimeters
            msg_depth = self.bridge.cv2_to_imgmsg(depth_frame, encoding="mono16")
            self.depth_pub.publish(msg_depth)


    def destroy_node(self):
        # Properly close the device on shutdown
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
