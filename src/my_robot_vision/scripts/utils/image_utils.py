import cv2
import os
from datetime import datetime

def generate_filename(prefix="bucket", color="unknown", ext=".jpg"):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{color}_{timestamp}{ext}"

def crop_and_save_object(frame, contour, color="unknown", save_dir="./captures"):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    x, y, w, h = cv2.boundingRect(contour)
    cropped = frame[y:y+h, x:x+w]
    filename = generate_filename("bucket", color)
    path = os.path.join(save_dir, filename)
    cv2.imwrite(path, cropped)
    return path