#!/bin/bash

# Navigate to your ROS2 workspace
# cd /home/team10/AUTO4508 || { echo "❌ AUTO4508 workspace not found!"; exit 1; }

# Build only the specified packages
echo "🔨 Building vision_oak_publisher and vision_yolo_detector..."
colcon build --packages-select vision_oak_publisher vision_yolo_detector

# Source the setup file
echo "✅ Sourcing install/setup.bash..."
source install/setup.bash

echo "🎉 Build and environment setup complete."

# ros2 run vision_oak_publisher oak_camera_publisher
# ros2 run vision_yolo_detector detect_node
# Uncomment the above lines to run the nodes after sourcing
# Note: Make sure to run this script in a terminal that has the necessary permissions