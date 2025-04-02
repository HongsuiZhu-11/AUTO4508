#!/bin/bash -x

source /opt/ros/jazzy/setup.bash
git clone https://github.com/HongsuiZhu-11/AUTO4508
cd ./AriaCoda
cmake CMakeList.txt
make
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/team10/ws/AriaCoda/lib
export PYTHONPATH=$PYTHONPATH:/home/team10/ws/AriaCoda/python
