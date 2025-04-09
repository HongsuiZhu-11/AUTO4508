import cv2
import numpy as np

def detect_colored_object(frame, hsv_lower, hsv_upper, debug=False):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array(hsv_lower, dtype=np.uint8)
    upper = np.array(hsv_upper, dtype=np.uint8)

    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        center_x = x + w // 2
        center_y = y + h // 2

        if debug:
            debug_frame = frame.copy()
            cv2.rectangle(debug_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(debug_frame, (center_x, center_y), 5, (255, 0, 0), -1)
            cv2.imshow("Detection", debug_frame)
            cv2.waitKey(1)

        return (center_x, center_y), mask

    return (None, None), mask
