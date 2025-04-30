#!/usr/bin/env python3
import rospy
import cv2
import time
from camera_capture import ROSCameraCapture
from detector import detect_colored_object, detect_colored_buckets
from coordinate_utils import transform_to_world
from my_robot_vision.srv import CaptureTarget, CaptureTargetResponse
from my_robot_vision.msg import BucketInfo
from utils.image_utils import crop_and_save_object

def load_params():
    params = {}
    params["center_threshold_px"] = rospy.get_param("~center_threshold_px", 20)
    params["image_topic"] = rospy.get_param("~image_topic", "/oak/rgb/image_raw")
    params["depth_topic"] = rospy.get_param("~depth_topic", "/oak/stereo/depth")
    params["hsv_lower"] = rospy.get_param("~hsv_lower", [10, 100, 100])
    params["hsv_upper"] = rospy.get_param("~hsv_upper", [25, 255, 255])
    params["bucket_heights"] = rospy.get_param("~bucket_heights", {})
    params["robot_pose"] = rospy.get_param("~robot_pose", {"x": 0.0, "y": 0.0, "theta_deg": 0.0})
    return params

def run_capture(params):
    camera = ROSCameraCapture(params["image_topic"], params["depth_topic"])
    rate = rospy.Rate(5)
    timeout = rospy.Time.now() + rospy.Duration(15)

    while not rospy.is_shutdown():
        if rospy.Time.now() > timeout:
            return False, "", 0.0, 0.0, []

        frame = camera.get_current_frame()
        depth = camera.get_current_depth()

        if frame is None or depth is None:
            rate.sleep()
            continue

        (tx, ty), mask = detect_colored_object(frame, params["hsv_lower"], params["hsv_upper"])
        if tx is None:
            rate.sleep()
            continue

        cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
        dx, dy = tx - cx, ty - cy

        if abs(dx) < params["center_threshold_px"] and abs(dy) < params["center_threshold_px"]:
            rospy.loginfo("[AlignCapture] Target centered. Capturing image...")
            frame_saved, path = camera.capture_and_save()

            # Get cone depth (convert from mm to meters)
            cone_depth = depth[ty, tx] / 1000.0

            # Assume cone is directly ahead in camera frame (e.g., x=0, z=depth)
            x_rel, y_rel = 0.0, cone_depth
            cone_x_world, cone_y_world = transform_to_world(params["robot_pose"], x_rel, y_rel, 0.0)

            # Detect and process buckets
            bucket_candidates = detect_colored_buckets(frame)
            buckets = []
            for b in bucket_candidates:
                bx, by = b["center"]
                bucket_depth = depth[by, bx] / 1000.0  # convert to meters

                # Skip invalid or zero depth values
                if bucket_depth <= 0.1:
                    continue

                bx_rel, by_rel = 0.0, bucket_depth
                bucket_x_world, bucket_y_world = transform_to_world(params["robot_pose"], bx_rel, by_rel, 0.0)

                distance = ((bucket_x_world - cone_x_world) ** 2 + (bucket_y_world - cone_y_world) ** 2) ** 0.5
                image_path = crop_and_save_object(frame, b["contour"], b["color"])

                info = BucketInfo(
                    color=b["color"],
                    image_path=image_path,
                    distance_to_cone=distance,
                    x=bucket_x_world,
                    y=bucket_y_world
                )
                buckets.append(info)

            return True, path, cone_x_world, cone_y_world, buckets

        rate.sleep()

def handle_capture_target(req):
    params = load_params()
    params["robot_pose"] = {"x": req.robot_x, "y": req.robot_y, "theta_deg": req.robot_theta_deg}
    success, path, x, y, buckets = run_capture(params)
    return CaptureTargetResponse(success, path, x, y, buckets)

if __name__ == "__main__":
    rospy.init_node("align_capture_service_node")
    service = rospy.Service("/capture_target", CaptureTarget, handle_capture_target)
    rospy.loginfo("[AlignCaptureService] Ready to capture target using OAK-D V2.")
    rospy.spin()
