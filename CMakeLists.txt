cmake_minimum_required(VERSION 3.0.2)
project(my_robot_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

## 声明消息与服务文件
add_message_files(
  FILES
  BucketInfo.msg
)

add_service_files(
  FILES
  CaptureTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime sensor_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 安装 Python 脚本节点
catkin_install_python(PROGRAMS
  scripts/align_capture_service_node.py
  scripts/ptz_controller.py
  scripts/camera_capture.py
  scripts/detector.py
  scripts/coordinate_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
