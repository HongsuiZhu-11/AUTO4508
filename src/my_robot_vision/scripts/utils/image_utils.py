import os
import cv2
from datetime import datetime

def generate_filename(prefix="image", label=None, save_dir="./captures", ext=".jpg"):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"{prefix}"
    if label:
        name += f"_{label}"
    name += f"_{timestamp}{ext}"
    return os.path.join(save_dir, name)

def save_full_frame_with_label(frame, label="bucket", save_dir="./captures"):
    """
    保存整张图像，并带有颜色标签用于区分不同桶。
    """
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    filepath = generate_filename(prefix="bucket", label=label, save_dir=save_dir)
    cv2.imwrite(filepath, frame)
    return filepath
