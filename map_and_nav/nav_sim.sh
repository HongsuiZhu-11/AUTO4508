# recommented improved data distribution service
# NOTE: Shouldn't interfere with other systems by changing dds, if it does, will need to remove
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-slam-toolbox

# NOTE: Uncomment use_sim_time if using a simulation environment:
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Visualize the map in RViz
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz