import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
import os

class ROSCameraCapture:
    def __init__(self, image_topic="/usb_cam/image_raw", save_dir="./captures"):
        self.bridge = CvBridge()
        self.image_topic = image_topic
        self.save_dir = save_dir
        self.frame = None

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")

    def get_current_frame(self):
        return self.frame

    def capture_and_save(self):
        if self.frame is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            cv2.imwrite(filepath, self.frame)
            rospy.loginfo(f"Image saving successful：{filepath}")
            return self.frame, filepath
        else:
            rospy.logwarn("NO image, cannot take picture")
            return None, None