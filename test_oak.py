import depthai as dai
import cv2
from datetime import datetime
import os

# 保存路径
save_dir = "./captures"
os.makedirs(save_dir, exist_ok=True)

# 创建 pipeline
pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam_rgb.video.link(xout.input)

with dai.Device(pipeline) as device:
    print("✅ Connected cameras:", device.getConnectedCameraFeatures())
    print("✅ USB speed:", device.getUsbSpeed())

    video = device.getOutputQueue(name="video", maxSize=4, blocking=False)
    frame_count = 0

    while frame_count < 5:  # 保存5张图
        in_frame = video.get()
        frame = in_frame.getCvFrame()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filepath = os.path.join(save_dir, f"oak_capture_{timestamp}.jpg")
        cv2.imwrite(filepath, frame)
        print(f"📸 Saved frame to {filepath}")
        frame_count += 1
