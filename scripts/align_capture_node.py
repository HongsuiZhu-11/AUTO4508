#!/usr/bin/env python3
import rospy
from camera_capture import ROSCameraCapture
from PTZController import VCC4PTZ
from detector import detect_colored_object
from coordinate_utils import estimate_local_coords, transform_to_world
import time
import cv2


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


def align_and_capture_loop(params):
    ptz = VCC4PTZ(params["ptz_port"])
    camera = ROSCameraCapture(params["image_topic"])

    rospy.loginfo("[AlignCapture] Node started. Waiting for image and detecting target...")

    rate = rospy.Rate(5)  # 5 Hz
    while not rospy.is_shutdown():
        frame = camera.get_current_frame()
        if frame is None:
            rate.sleep()
            continue

        (tx, ty), mask = detect_colored_object(frame, params["hsv_lower"], params["hsv_upper"])
        if tx is None:
            rate.sleep()
            continue

        # 获取目标高度像素（使用 bounding box 高度）
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            _, _, _, pixel_height = cv2.boundingRect(largest)
        else:
            pixel_height = 60  # fallback

        cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
        dx, dy = tx - cx, ty - cy

        if abs(dx) < params["center_threshold_px"] and abs(dy) < params["center_threshold_px"]:
            rospy.loginfo("[AlignCapture] Target centered. Capturing image...")
            frame_saved, path = camera.capture_and_save()

            # 使用目标像素高度估算距离
            x_rel, y_rel = estimate_local_coords(pixel_height, params["cone_height_m"], params["scale_k"])
            pan_angle = ptz.get_pan_angle()  # 获取 PTZ 当前 pan 角度（度）
            x_world, y_world = transform_to_world(params["robot_pose"], x_rel, y_rel, pan_angle)


            rospy.loginfo(f"[AlignCapture] Target World Position: X={x_world:.2f}, Y={y_world:.2f}")
            break
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


if __name__ == "__main__":
    rospy.init_node("align_capture_node")
    params = load_params()
    align_and_capture_loop(params)
