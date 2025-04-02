FROM osrf/ros:jazzy-desktop-full

RUN apt update && apt upgrade -y && apt install ros-jazzy-joy

