cmake_minimum_required(VERSION 3.5)
project(my_robot_vision)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 服务/消息接口
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CaptureTarget.srv"
  "msg/BucketInfo.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
