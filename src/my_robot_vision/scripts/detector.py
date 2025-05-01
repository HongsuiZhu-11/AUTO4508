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



def detect_colored_buckets(image, color_ranges):
    """
    Detects colored buckets in an image using provided HSV color ranges.

    Parameters:
        image (np.ndarray): BGR input image
        color_ranges (dict): Dict of {color_name: [lower_HSV, upper_HSV]} from YAML

    Returns:
        List[Dict]: Each with keys: "color", "center", "contour"
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    results = []

    for color, (lower, upper) in color_ranges.items():
        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_np, upper_np)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) < 200:  # filter small blobs
                continue
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            results.append({
                "color": color,
                "center": (cx, cy),
                "contour": contour
            })

    return results
