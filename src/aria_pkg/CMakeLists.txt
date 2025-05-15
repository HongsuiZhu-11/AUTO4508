cmake_minimum_required(VERSION 3.8)
project(aria_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED) #remove
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Find Aria
find_library(
  ARIA_LIBRARY
  NAMES Aria
  PATHS /opt/Aria/lib  # Replace with the *ACTUAL* path to Aria's lib directory
  /usr/local/lib
  /usr/lib
)
if(NOT ARIA_LIBRARY)
  message(FATAL_ERROR "Aria library not found.  Please specify the installation path.")
endif()

include_directories(
  include
  /opt/Aria/include  # Replace with the *ACTUAL* path to Aria's include directory
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  #${example_interfaces_INCLUDE_DIRS} #remove
  ${std_msgs_INCLUDE_DIRS}
)

add_executable(ariaNode src/ariaNode.cpp)
ament_target_dependencies(ariaNode
  "rclcpp"
  "sensor_msgs"
  "rclcpp_components"
  #"example_interfaces" #remove
  "std_msgs"
  "rosidl_default_runtime"
  "aria_pkg"
)

target_include_directories(ariaNode PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_link_libraries(
  ariaNode
  ${ARIA_LIBRARY}
)

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