#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
import os

class ROSCameraCapture:
    def __init__(self, image_topic="/oak/rgb/image_raw", depth_topic="/oak/stereo/depth", save_dir="./captures"):
        self.bridge = CvBridge()
        self.image_topic = image_topic
        self.depth_topic = depth_topic
        self.save_dir = save_dir
        self.frame = None
        self.depth = None

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")

    def depth_callback(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(f"Depth conversion failed: {e}")

    def get_current_frame(self):
        return self.frame

    def get_current_depth(self):
        return self.depth

    def capture_and_save(self):
        if self.frame is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            cv2.imwrite(filepath, self.frame)
            rospy.loginfo(f"Image saved: {filepath}")
            return self.frame, filepath
        else:
            rospy.logwarn("No image data available to save")
            return None, None
