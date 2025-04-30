# 🟠 OAK-D V2 Vision Module — Cone & Bucket Detection for Robot Navigation

This ROS package provides a vision-based module for detecting orange traffic cones (waypoints) and nearby colored buckets using the **OAK-D V2 stereo camera**. It includes automatic detection, depth-based localization, image saving, and a service interface for integration into an autonomous robot navigation system.

---

## ✅ Features

- 🎯 Detects and centers on orange traffic cones using HSV color filtering  
- 🖼️ Captures images automatically once the target is centered  
- 🌍 Computes world coordinates of both cone and nearby buckets using OAK-D stereo depth  
- 🪣 Identifies nearby colored buckets (red, blue, yellow by default)  
- 📏 Calculates the Euclidean distance between the cone and each bucket  
- 🧾 Provides structured service output including all world positions and image paths  
- ⚙️ Fully configurable via YAML file  
- 🧠 Built as a ROS service for easy integration with your robot’s main control system  

---

## 📦 File Structure

```
my_robot_vision/
├── scripts/
│   ├── align_capture_service_node.py     # Main ROS service node
│   ├── camera_capture.py                 # RGB + depth image subscriber
│   ├── detector.py                       # HSV-based cone and bucket detection
│   └── coordinate_utils.py               # Relative-to-world coordinate conversion
├── config/
│   └── vision_params.yaml                # Parameter configuration
├── msg/
│   └── BucketInfo.msg                    # Custom message for each bucket
├── srv/
│   └── CaptureTarget.srv                 # Service definition for triggering detection
├── launch/
│   └── align_capture_service.launch      # Launches the node and loads parameters
```

---

## ⚙️ Requirements

- ROS Noetic or Melodic  
- [Luxonis OAK-D V2 stereo camera](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK/)  
- [depthai-ros](https://github.com/luxonis/depthai-ros) (publishing RGB + depth topics)  
- `cv_bridge`, `sensor_msgs`, `rospy`, `message_generation`

---

## 🚀 Setup Instructions

### 1. Create workspace and package

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_create_pkg my_robot_vision rospy std_msgs sensor_msgs message_generation cv_bridge
```

### 2. Copy files into your package

```bash
cd ~/catkin_ws/src/my_robot_vision
mkdir scripts config msg srv launch
# Copy all files into respective folders
chmod +x scripts/*.py
```

### 3. Update `CMakeLists.txt` and `package.xml`

Make sure to include `message_generation` and your custom message/service types.

### 4. Build the package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 🚀 Deployment & Launch

### ✅ Step-by-step Deployment Guide

1. Plug in the **OAK-D camera** via USB  
2. Launch the camera using `depthai-ros`:

```bash
roslaunch depthai_ros_driver camera.launch
```

3. Launch the vision module node:

```bash
roslaunch my_robot_vision align_capture_service.launch
```

4. Call the capture service:

```bash
rosservice call /capture_target "{robot_x: 2.0, robot_y: 3.5, robot_theta_deg: 45.0}"
```

---

## 📤 Example Output

```yaml
success: true
cone_image_path: "/path/to/capture.jpg"
cone_x: 2.87
cone_y: 4.12
buckets:
  - color: "red"
    image_path: "/path/to/bucket_red.jpg"
    distance_to_cone: 1.42
    x: 3.55
    y: 4.75
  - ...
```

---

## 🛠 Parameters (`vision_params.yaml`)

```yaml
image_topic: /oak/rgb/image_raw
depth_topic: /oak/stereo/depth

hsv_lower: [10, 100, 100]
hsv_upper: [25, 255, 255]

bucket_color_ranges:
  red: [[0, 100, 100], [10, 255, 255]]
  blue: [[100, 150, 0], [140, 255, 255]]
  yellow: [[20, 100, 100], [35, 255, 255]]

center_threshold_px: 20

robot_pose:
  x: 0.0
  y: 0.0
  theta_deg: 0.0
```

---

## 🔄 Workflow Summary

1. Detect cone using HSV filtering  
2. Wait until cone is centered in image  
3. Capture image from OAK-D RGB camera  
4. Read depth value at cone center from stereo depth map  
5. Convert local camera-relative position to global world coordinates  
6. Detect buckets, repeat steps 4–5  
7. Return results via ROS service  

---

## 🔧 Future Improvements

- Real-time visualization (RViz Markers)  
- Use of deep learning detection models  
- Publish cone/bucket positions to TF tree or map  

---

## 📬 Contact

Developed by: **Your Team**  
Adapted for: Autonomous Navigation using OAK-D V2  
Supervisor: *[Insert name if applicable]*