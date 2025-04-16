FROM osrf/ros:jazzy-desktop-full

RUN apt update && apt upgrade -y && apt install ros-jazzy-joy && apt install python3-pip && pip3 install opencv_python

