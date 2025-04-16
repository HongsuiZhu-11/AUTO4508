import cv2
import numpy as np

def detect_colored_object(frame, hsv_lower, hsv_upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy), mask
    return (None, None), mask


def detect_colored_buckets(frame, color_ranges):
    """
    prams：
        frame: current frame
        color_ranges: dict，eg：
            {
              "red": [[0,100,100],[10,255,255]],
              "blue": [[100,150,50],[130,255,255]],
              "yellow": [[25,100,100],[35,255,255]]
            }
    return：
        List[dict]，every elem contains：center, color, mask, contour
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    results = []
    for color, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue  # 过滤小目标
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                results.append({
                    "center": (cx, cy),
                    "color": color,
                    "mask": mask,
                    "contour": cnt
                })
    return results
