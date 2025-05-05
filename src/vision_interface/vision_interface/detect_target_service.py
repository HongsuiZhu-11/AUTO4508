import rclpy
from rclpy.node import Node
from vision_interface.srv import DetectTarget
from depthai_sdk import OakCamera
import cv2
import numpy as np
import time
import os

# HSV color ranges
COLOR_RANGES = {
    'cone_orange': {'lower': [10, 100, 100], 'upper': [30, 255, 255]},
    'cone_yellowgreen': {'lower': [30, 50, 50], 'upper': [80, 255, 255]},
    'bucket_red': {'lower': [0, 100, 100], 'upper': [10, 255, 255]},
}

CENTER_TOLERANCE = 80  # Pixel distance from center

def detect_target(rgb, depth, spatial):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    h, w, _ = rgb.shape
    cx_frame, cy_frame = w // 2, h // 2

    for label, ranges in COLOR_RANGES.items():
        mask = cv2.inRange(hsv, np.array(ranges['lower']), np.array(ranges['upper']))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < 500:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            cx, cy = x + bw // 2, y + bh // 2
            if abs(cx - cx_frame) < CENTER_TOLERANCE and abs(cy - cy_frame) < CENTER_TOLERANCE:
                coord = spatial[cy, cx]
                distance = coord[2]
                return {
                    'label': label,
                    'coords': coord,
                    'distance': distance,
                    'image': rgb
                }
    return None

class DetectTargetNode(Node):
    def __init__(self):
        super().__init__('detect_target_node')
        self.srv = self.create_service(DetectTarget, 'detect_target', self.callback)
        self.get_logger().info('Service /detect_target ready.')

    def callback(self, request, response):
        result_data = {}

        with OakCamera() as oak:
            color = oak.create_camera('color', resolution='1080p', fps=30)
            stereo = oak.create_stereo(spatial_location=True)
            color.config_color_preview(416, 416)

            def cb(packet):
                rgb = packet['color'].getCvFrame()
                depth = packet['depth'].getFrame()
                spatial = packet['spatial_location'].getSpatialLocationsMap()
                result = detect_target(rgb, depth, spatial)
                if result:
                    ts = int(time.time())
                    img_path = f'/output/{result["label"]}_{ts}.jpg'
                    os.makedirs('/output', exist_ok=True)
                    cv2.imwrite(img_path, result['image'])
                    result_data.update({
                        'label': result['label'],
                        'distance': result['distance'],
                        'coords': result['coords'],
                        'image_path': img_path
                    })
                    oak.stop()

            oak.callback(sync=['color', 'depth', 'spatial_location'], callback=cb)
            oak.start(blocking=True)

        if result_data:
            response.success = True
            response.label = result_data['label']
            response.distance = float(result_data['distance'])
            response.world_x = float(result_data['coords'][0])
            response.world_y = float(result_data['coords'][1])
            response.world_z = float(result_data['coords'][2])
            response.image_path = result_data['image_path']
        else:
            response.success = False
        return response
