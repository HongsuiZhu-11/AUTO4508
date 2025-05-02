#!/usr/bin/env python3
import rospy
import time
from camera_capture import OakCameraCapture
from detector import detect_colored_object, detect_colored_buckets
from coordinate_utils import transform_to_world
from my_robot_vision.srv import CaptureTarget, CaptureTargetResponse
from my_robot_vision.msg import BucketInfo
from utils.image_utils import save_full_frame_with_label

def load_params():
    params = {}
    params["center_threshold_px"] = rospy.get_param("~center_threshold_px", 20)
    params["hsv_lower"] = rospy.get_param("~hsv_lower", [10, 100, 100])  # cone (orange)
    params["hsv_upper"] = rospy.get_param("~hsv_upper", [25, 255, 255])
    params["robot_pose"] = rospy.get_param("~robot_pose", {"x": 0.0, "y": 0.0, "theta_deg": 0.0})
    params["bucket_color_ranges"] = rospy.get_param("~bucket_color_ranges", {})  # YAML bucket color HSVs
    return params

def run_capture(params):
    camera = OakCameraCapture()
    camera.start()

    rospy.loginfo("[AlignCapture] OAK-D camera started, waiting for target...")
    rate = rospy.Rate(5)
    timeout = rospy.Time.now() + rospy.Duration(15)

    while not rospy.is_shutdown():
        if rospy.Time.now() > timeout:
            return False, "timeout", 0.0, 0.0, []

        frame = camera.get_current_frame()
        depth = camera.get_current_depth()
        if frame is None or depth is None:
            rate.sleep()
            continue

        cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
        cone_center, cone_mask = detect_colored_object(frame, params["hsv_lower"], params["hsv_upper"])
        bucket_candidates = detect_colored_buckets(frame, params["bucket_color_ranges"])

        found_cone = cone_center is not None
        found_bucket = len(bucket_candidates) > 0

        if not found_cone and not found_bucket:
            rate.sleep()
            continue

        rospy.loginfo("[AlignCapture] Target detected. Capturing image...")
        frame_saved, path = camera.capture_and_save()

        x_world = y_world = 0.0
        buckets = []

        # Cone world coordinate
        if found_cone:
            tx, ty = cone_center
            cone_depth = depth[ty, tx] / 1000.0
            x_rel, y_rel = 0.0, cone_depth
            x_world, y_world = transform_to_world(params["robot_pose"], x_rel, y_rel, 0.0)

        # Bucket detection & distance
        for b in bucket_candidates:
            bx, by = b["center"]
            bucket_depth = depth[by, bx] / 1000.0
            if bucket_depth <= 0.1:
                continue
            bx_rel, by_rel = 0.0, bucket_depth
            bx_world, by_world = transform_to_world(params["robot_pose"], bx_rel, by_rel, 0.0)

            distance = ((bx_world - x_world) ** 2 + (by_world - y_world) ** 2) ** 0.5 if found_cone else 0.0
            image_path = save_full_frame_with_label(frame, b["contour"], b["color"])

            buckets.append(BucketInfo(
                color=b["color"],
                image_path=image_path,
                distance_to_cone=distance,
                x=bx_world,
                y=by_world
            ))

        return True, path, x_world, y_world, buckets

        rate.sleep()

def handle_capture_target(req):
    params = load_params()
    params["robot_pose"] = {"x": req.robot_x, "y": req.robot_y, "theta_deg": req.robot_theta_deg}
    success, path, x, y, buckets = run_capture(params)
    return CaptureTargetResponse(success, path, x, y, buckets)

if __name__ == "__main__":
    rospy.init_node("align_capture_service_node")
    service = rospy.Service("/capture_target", CaptureTarget, handle_capture_target)
    rospy.loginfo("[AlignCaptureService] Ready to capture target using OAK-D and return coordinates.")
    rospy.spin()

