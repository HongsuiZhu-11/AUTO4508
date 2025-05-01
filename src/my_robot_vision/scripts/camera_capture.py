# camera_capture.py
from depthai_sdk import OakCamera
from datetime import datetime
import os
import cv2

class OakCameraCapture:
    def __init__(self, save_dir="./captures"):
        self.oak = OakCamera()
        self.rgb = self.oak.create_camera('color', resolution='1080p')
        self.depth = self.oak.create_stereo('left', 'right', output='depth')

        self.rgb_stream = self.rgb.out.main
        self.depth_stream = self.depth.out

        self.frame = None
        self.depth_map = None
        self.save_dir = save_dir

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

    def start(self):
        self.oak.callback(self.rgb_stream, self._rgb_callback)
        self.oak.callback(self.depth_stream, self._depth_callback)
        self.oak.start(blocking=False)

    def _rgb_callback(self, packet):
        self.frame = packet.frame

    def _depth_callback(self, packet):
        self.depth_map = packet.frame

    def get_current_frame(self):
        return self.frame

    def get_current_depth(self):
        return self.depth_map

    def capture_and_save(self):
        if self.frame is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            cv2.imwrite(filepath, self.frame)
            print(f"[OakCamera] Image saved: {filepath}")
            return self.frame, filepath
        else:
            print("[OakCamera] No frame available to save.")
            return None, None
