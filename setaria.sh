#!/bin/bash -x

source /opt/ros/jazzy/setup.bash
git clone https://github.com/HongsuiZhu-11/AUTO4508
cd ./AriaCoda
cmake CMakeList.txt
make
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/team10/ws/AriaCoda/lib
export PYTHONPATH=$PYTHONPATH:/home/team10/ws/AriaCoda/python
#./gpsExample -robotPort /dev/ttyUSB0 -gpsPort /dev/ttyUSB0

# 1. run aria_node
# in aria_pkg path
# source install/local_setup.sh
# ros2 run aria_pkg ariaNode -robotPort /dev/ttyUSB0

# 2. run gps node
# in ros2GPSx path
# source install/setup.bash
# ros2 run gpsx gps_node --ros-args -p "comm_port:=/dev/ttyACM0"

# 3. run control node
# in control_pkg path
# source install/local_setup.sh
# ros2 run control_pkg control_node
