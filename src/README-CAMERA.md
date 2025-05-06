# 🎯 Vision-Based Detection System (OAK-D + YOLOv8)

This project consists of two ROS 2 Python packages:

- `vision_oak_publisher`: Publishes images from an OAK-D camera.
- `vision_yolo_detector`: Uses a YOLOv8 model to detect centered target objects in a single frame and publish the result.

---

## 📦 Package 1: `vision_oak_publisher`

**Purpose:**  
Publishes image frames from an OAK-D stereo camera to a ROS2 topic.

**Published Topic:**  
- `/camera/image_raw` (`sensor_msgs/msg/Image`)

**Run:**

```bash
ros2 run vision_oak_publisher oak_camera_node
```

This node continuously streams images from the OAK-D device.

---

## 📦 Package 2: `vision_yolo_detector`

**Purpose:**  
Subscribes to `/camera/image_raw`, runs YOLOv8 detection once on the center of the first received frame, and publishes the detection result.

**Model File:**  
- `vision_yolo_detector/model/best.pt`

**Published Topic:**  
- `/target_detected` (`std_msgs/msg/String`)

### 🏷️ Target Classes:

The node only publishes detection results if the centered object is one of the following:

- `cone-orange`
- `bucket-red`
- `cone-yellow-green`

If no valid target is found in the center, the node will publish:

```
No-target
```

**Image Saving:**  
Detected images are saved under the `center_detected_images/` directory.

---

## 🔧 Python Dependencies

Ensure the following Python packages are installed:

```bash
pip install --upgrade --force-reinstall --break-system-packages ultralytics opencv-python depthai numpy
```

> ⚠️ `--break-system-packages` is required if you're using Python installed via your system package manager (e.g., Ubuntu's `apt`).

---

## 🚀 Running the YOLO Detection Node

```bash
ros2 run vision_yolo_detector detect_node
```

- The node runs only once per launch.
- It waits for a single image.
- It processes and publishes the centered detection result.
- The node remains alive until manually stopped, allowing potential future service calls or reuse.

---

## 📡 Listening to Detection Results

You can listen to the result with:

```bash
ros2 topic echo /target_detected
```

---

## 🗂️ Project Structure

```
your_workspace/
├── vision_oak_publisher/
│   ├── vision_oak_publisher/
│   │   ├── __init__.py
│   │   └── oak_camera_node.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
└── vision_yolo_detector/
    ├── vision_yolo_detector/
    │   ├── __init__.py
    │   ├── yolo_center_detector.py
    │   └── model/
    │       └── best.pt
    ├── package.xml
    ├── setup.cfg
    └── setup.py
```

---

## 📬 Contact

For any questions, please contact the maintainer:

**Maintainer:** Your Name  
**Email:** you@example.com
