#!/usr/bin/env bash

colcon build --symlink-install --packages-select sim_aria_pkg control_pkg p3at_description

source install/setup.bash

if ! ros2 pkg list | grep -q ublox_gps; then
  sudo apt update
  sudo apt install -y ros-jazzy-ublox-gps ros-jazzy-ublox-msgs
fi

ros2 launch p3at_description display.launch.py
