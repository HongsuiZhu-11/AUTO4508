# To be run in the terminal
# To be run from within the display_pkg directory

source /opt/ros/jazzy/setup.bash
colcon build
source install/local_setup.sh
ros2 run display_pkg display_node