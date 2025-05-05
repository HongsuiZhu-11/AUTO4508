cmake_minimum_required(VERSION 3.5)
project(vision_interface)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(depthai_ros_msgs REQUIRED)  # 如无可去除
find_package(builtin_interfaces REQUIRED)

# 服务接口
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectTarget.srv"
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY vision_interface
  DESTINATION lib/python3.10/site-packages/
)

install(
  PROGRAMS
  vision_interface/detect_target_service.py
  vision_interface/detect_target_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
