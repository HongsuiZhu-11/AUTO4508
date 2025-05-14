FROM osrf/ros:jazzy-desktop-full

RUN apt update && apt upgrade -y && apt install ros-jazzy-joy 

RUN apt update && apt upgrade -y && sudo apt install ros-jazzy-ublox-gps ros-jazzy-ublox-msgs -y

RUN sudo apt upgrade -y && sudo apt install ros-jazzy-depthai-ros -y

#RUN sudo apt install python3-pip && pip install --upgrade --force-reinstall --break-system-packages ultralytics opencv-python depthai numpy
