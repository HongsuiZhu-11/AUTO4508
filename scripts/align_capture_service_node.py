#!/usr/bin/env python3
import rospy
from camera_capture import ROSCameraCapture
from PTZController import VCC4PTZ
from detector import detect_colored_object
from coordinate_utils import estimate_local_coords, transform_to_world
from srv.CaptureTarget.srv import CaptureTarget, CaptureTargetResponse
import time
import cv2
import os
import sys


def load_params():
    params = {}
    params["cone_height_m"] = rospy.get_param("~cone_height_m", 0.5)
    params["scale_k"] = rospy.get_param("~scale_k", 15.0)
    params["center_threshold_px"] = rospy.get_param("~center_threshold_px", 20)
    params["hsv_lower"] = rospy.get_param("~hsv_lower", [10, 100, 100])
    params["hsv_upper"] = rospy.get_param("~hsv_upper", [25, 255, 255])
    params["ptz_port"] = rospy.get_param("~ptz_port", "/dev/ttyS1")
    params["image_topic"] = rospy.get_param("~image_topic", "/usb_cam/image_raw")
    params["robot_pose"] = rospy.get_param("~robot_pose", {"x": 0.0, "y": 0.0, "theta_deg": 0.0})
    return params


def run_capture(params):
    ptz = VCC4PTZ(params["ptz_port"])
    camera = ROSCameraCapture(params["image_topic"])
    rate = rospy.Rate(5)
    timeout = rospy.Time.now() + rospy.Duration(15)

    while not rospy.is_shutdown():
        if rospy.Time.now() > timeout:
            return False, "timeout", 0.0, 0.0

        frame = camera.get_current_frame()
        if frame is None:
            rate.sleep()
            continue

        (tx, ty), mask = detect_colored_object(frame, params["hsv_lower"], params["hsv_upper"])
        if tx is None:
            rate.sleep()
            continue

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            _, _, _, pixel_height = cv2.boundingRect(largest)
        else:
            pixel_height = 60

        cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
        dx, dy = tx - cx, ty - cy

        if abs(dx) < params["center_threshold_px"] and abs(dy) < params["center_threshold_px"]:
            frame_saved, path = camera.capture_and_save()
            pan_angle = ptz.get_pan_angle()
            x_rel, y_rel = estimate_local_coords(pixel_height, params["cone_height_m"], params["scale_k"])
            x_world, y_world = transform_to_world(params["robot_pose"], x_rel, y_rel, pan_angle)
            return True, path, x_world, y_world
        else:
            if dx > params["center_threshold_px"]:
                ptz.pan_rel_right()
            elif dx < -params["center_threshold_px"]:
                ptz.pan_rel_left()

            if dy > params["center_threshold_px"]:
                ptz.tilt_down()
            elif dy < -params["center_threshold_px"]:
                ptz.tilt_up()

            time.sleep(0.5)
            ptz.stop()
        rate.sleep()


def handle_capture_target(req):
    params = load_params()
    success, path, x, y = run_capture(params)
    return CaptureTargetResponse(success, path, x, y)


if __name__ == "__main__":
    rospy.init_node("align_capture_service_node")
    service = rospy.Service("/capture_target", CaptureTarget, handle_capture_target)
    rospy.loginfo("[AlignCaptureService] Ready to capture target on request.")
    rospy.spin()
