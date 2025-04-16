mkdir -p ./sick_scan_ws
cd ./sick_scan_ws
mkdir ./src
pushd ./src
git clone https://github.com/SICKAG/libsick_ldmrs.git
git clone -b master https://github.com/SICKAG/sick_scan_xd.git
popd
rm -rf ./build ./build_isolated/ ./devel ./devel_isolated/ ./install ./install_isolated/ ./log/ # remove any files from a previous build

# Should have already been done but if it hasn't then this will ensure
source /opt/ros/jazzy/setup.bash # replace foxy by your ros distro

colcon build --packages-select libsick_ldmrs --event-handlers console_direct+
source ./install/setup.bash
colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
source ./install/setup.bash

# NOTE: Will build with LD-MRS to begin with.
colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2"  --event-handlers console_direct+
cd ./doxygen
doxygen ./docs/Doxyfile