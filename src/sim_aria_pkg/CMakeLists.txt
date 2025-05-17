cmake_minimum_required(VERSION 3.8)
project(sim_aria_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Node executable
add_executable(sim_aria_node
  src/sim_aria_node.cpp
)
ament_target_dependencies(sim_aria_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Install targets
install(TARGETS
  sim_aria_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()