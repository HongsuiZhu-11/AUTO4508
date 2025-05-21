#!/bin/bash
# set -e 
export DISPLAY=host.docker.internal:0.0
export LIBGL_ALWAYS_INDIRECT=0

source /opt/ros/humble/setup.bash
source /workspaces/gazebo_ws/install/setup.bash

exec "$@"