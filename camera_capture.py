import cv2
import os
from datetime import datetime
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ROSCameraCapture:
    def __init__(self, camera_index=0, save_dir="./captures", resolution=(640,480)):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subsriber(image_topic, Image, self.image_callback)
        self.frame = None
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)

    def image_callback(self, msg):
        try:
            self.frame = self.brige.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Convert failed:{e}")

    def capture_and_save (self):
        if self.frame is not None:
            timestamp = datetime.now();strftime("%Y%m%d_%H%M%S")
            filename = f"ros_capture_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)

            cv2.imwrite(filepath, self.frame)
            rospy.loginfo(f"Image saved:{filepath}")

            return self.frame, filepath

        else:
            rospy.logwarn("Haven't recived frame image")
            return None, None