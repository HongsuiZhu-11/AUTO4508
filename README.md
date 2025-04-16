# my_robot_vision

## Overview
This package implements an automated PTZ camera vision system for detecting traffic cones and nearby color-coded buckets. It supports autonomous alignment, photo capture, distance estimation, and conversion to world coordinates based on current robot pose and camera pan angle.

The system is designed as a ROS service that can be called from a central mission controller or FSM when the robot reaches a waypoint. The vision system will then:

1. Detect the traffic cone.
2. Center the cone using the PTZ camera.
3. Capture an image.
4. Detect nearby buckets.
5. Estimate and return:
   - Cone world coordinates.
   - Bucket world coordinates.
   - Euclidean distances from cone to each bucket.
   - Cropped bucket images.

---

## Features
- PTZ camera control (VC-C4 or VISCA-compatible).
- HSV-based color detection for cones and buckets.
- Pixel-based distance estimation using known object height.
- World coordinate transformation using robot pose and camera orientation.
- Image saving and cropping.
- ROS Service interface: `/capture_target`.

---

## File Structure
```bash
my_robot_vision/
├── scripts/
│   ├── align_capture_service_node.py    # Main service node
│   ├── ptz_controller.py                # VC-C4 PTZ control interface
│   ├── camera_capture.py                # Image subscription and saving
│   ├── detector.py                      # Cone & bucket HSV detection
│   └── coordinate_utils.py              # Distance + world coordinate transform
│
├── msg/
│   └── BucketInfo.msg                   # Custom message: bucket metadata
│
├── srv/
│   └── CaptureTarget.srv                # Service definition
│
├── config/
│   └── vision_params.yaml               # Tunable HSV and size parameters
│
├── launch/
│   └── align_capture_service.launch     # Launch file with parameter loading
```

---

## How to Use

### Step 1: Setup ROS workspace (if not already)
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Step 2: Create the ROS package
```bash
cd ~/catkin_ws/src
catkin_create_pkg my_robot_vision rospy std_msgs sensor_msgs message_generation cv_bridge
cd my_robot_vision
```

### Step 3: Add folders and copy the files
```bash
mkdir scripts msg srv config launch
chmod +x scripts/*.py
```
Place the following files:
- `.py` files → `scripts/`
- `BucketInfo.msg` → `msg/`
- `CaptureTarget.srv` → `srv/`
- `vision_params.yaml` → `config/`
- `align_capture_service.launch` → `launch/`

### Step 4: Update `CMakeLists.txt`
Make sure it includes:
```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

add_message_files(FILES BucketInfo.msg)
add_service_files(FILES CaptureTarget.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime sensor_msgs cv_bridge
)

catkin_install_python(PROGRAMS
  scripts/align_capture_service_node.py
  scripts/ptz_controller.py
  scripts/camera_capture.py
  scripts/detector.py
  scripts/coordinate_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

### Step 5: Update `package.xml`
Ensure the following dependencies are present:
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
```

### Step 6: Build the package
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Step 7: Launch the node
```bash
roslaunch my_robot_vision align_capture_service.launch
```

### Step 8: Call the service
```bash
rosservice call /capture_target "{robot_x: 2.0, robot_y: 3.5, robot_theta_deg: 45.0}"
```

---

## Service Output Example
```yaml
success: true
cone_image_path: "/path/to/image.jpg"
cone_x: 2.58
cone_y: 4.11
buckets:
  - color: "red"
    image_path: "/path/to/red_bucket.jpg"
    distance_to_cone: 0.89
    x: 3.0
    y: 4.7
```

---

## Camera & Hardware Setup

### Supported Camera:
- Canon VC-C4 PTZ camera (VISCA protocol)
- USB webcam (image topic `/usb_cam/image_raw`)

### Requirements:
- PTZ camera connected via `/dev/ttyS1` or another serial port
- ROS driver for USB camera already publishing images

---

## Parameter Configuration (YAML)
`config/vision_params.yaml`:
```yaml
cone_height_m: 0.5
scale_k: 480.0
center_threshold_px: 20
ptz_port: "/dev/ttyS1"
image_topic: "/usb_cam/image_raw"
hsv_lower: [10, 100, 100]
hsv_upper: [25, 255, 255]

bucket_heights:
  red: 0.4
  blue: 0.6
  yellow: 0.45

robot_pose:
  x: 0.0
  y: 0.0
  theta_deg: 0.0
```

---

## Notes
- All `.py` scripts in `scripts/` must be executable (`chmod +x scripts/*.py`).
- Be sure to connect the PTZ camera and USB webcam properly.
- HSV values should be tuned for your lighting conditions.
- If the bucket color is not listed in `bucket_heights`, the system will default to `cone_height_m`.

---

## Contact
Please contact the vision module developer if you need help integrating this system into your control framework, mission planner, or waypoint driver.

