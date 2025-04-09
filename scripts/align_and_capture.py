import cv2
import numpy as np
import time
from detector import detect_colored_object
from vcc4_ptz import VCC4PTZ
from camera_capture import ROSCameraCapture
from coordinate_utils import estimate_local_coords, transform_to_world

# # 加载参数（可替换为 YAML 或 argparse 加载）
# params = {
#     "hsv_lower": [10, 100, 100],
#     "hsv_upper": [25, 255, 255],
#     "cone_width_m": 0.3,
#     "scale_k": 15.0,
#     "center_threshold_px": 20,
#     "robot_pose": {"x": 2.0, "y": 3.5, "theta_deg": 45.0}
# }

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

# 初始化 PTZ 和 摄像头
ptz = VCC4PTZ("/dev/ttyS1")
camera = ROSCameraCapture("/usb_cam/image_raw")

print("[INFO] System ready. Detecting target...")


params = load_params()

while True:
    frame = camera.get_current_frame()
    if frame is None:
        continue

    (tx, ty), mask = detect_colored_object(frame, params["hsv_lower"], params["hsv_upper"])
    if tx is None:
        continue

    # 计算与图像中心偏移
    cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
    dx, dy = tx - cx, ty - cy

    if abs(dx) < params["center_threshold_px"] and abs(dy) < params["center_threshold_px"]:
        print("[INFO] Target centered. Capturing image...")
        frame_saved, path = camera.capture_and_save()

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            _, _, _, pixel_height = cv2.boundingRect(largest)
        else:
            pixel_height = 60  # fallback

        pan_angle = ptz.get_pan_angle()  # 获取 PTZ 当前 pan 角度（度）
        x_rel, y_rel = estimate_local_coords(pixel_height, params["cone_height_m"], params["scale_k"])
        x_world, y_world = transform_to_world(params["robot_pose"], x_rel, y_rel, pan_angle)



        x_world, y_world = transform_to_world(
            params["robot_pose"], x_rel, y_rel
        )

        print(f"[RESULT] World Coordinates: X={x_world:.2f}, Y={y_world:.2f}")
        break
    else:
        # 控制 PTZ 进行微调
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
