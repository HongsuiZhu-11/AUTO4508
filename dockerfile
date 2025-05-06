FROM osrf/ros:jazzy-desktop-full

RUN apt update && apt upgrade -y && apt install ros-jazzy-joy 

RUN sudo apt install ros-jazzy-ublox-gps ros-jazzy-ublox-msgs -y

RUN apt upgrade -y && apt install ros-jazzy-depthai-ros -y

#RUN apt install python3-pip && pip3 install opencv_python

