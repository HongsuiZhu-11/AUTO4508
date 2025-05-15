cmake_minimum_required(VERSION 3.8)
project(aria_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DriveDistance.srv"
  "srv/TurnAngle.srv"
  DEPENDENCIES example_interfaces std_msgs
)

add_executable(ariaNode src/ariaNode.cpp)
ament_target_dependencies(ariaNode
"rclcpp"
"sensor_msgs"
"rclcpp_components"
"example_interfaces"
"std_msgs"
"rosidl_default_runtime"
"aria_pkg"
)

target_include_directories(ariaNode PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_link_libraries(
  ariaNode
  libAria.so
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_dependencies(ariaNode ${cpp_typesupport_target})
message(STATUS "Typesupport target: ${cpp_typesupport_target}")

install(TARGETS
ariaNode
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
