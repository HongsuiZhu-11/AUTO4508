#!/bin/bash

# 安装依赖
echo "🔧 Updating system and installing dependencies..."
sudo apt update && sudo apt install -y python3-pip python3-colcon-common-extensions

echo "📦 Installing Python packages..."
# pip install --upgrade pip
pip install ultralytics depthai opencv-python numpy --break-system-packages

# 进入工作区
# WORKSPACE_DIR="/home/team10/AUTO4508"
# cd "$WORKSPACE_DIR" || { echo "❌ Cannot find workspace: $WORKSPACE_DIR"; exit 1; }

# 构建指定包
echo "🔨 Building vision_oak_publisher and vision_yolo_detector..."
colcon build --packages-select vision_oak_publisher vision_yolo_detector

# Source 环境变量
echo "✅ Sourcing setup file..."
source install/setup.bash

SESSION="vision_nodes"
tmux new-session -d -s $SESSION

# 创建四个 pane 并运行节点
tmux send-keys -t $SESSION "source install/setup.bash && ros2 run vision_oak_publisher oak_camera_publisher" C-m
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "source install/setup.bash && ros2 run vision_yolo_detector detect_node" C-m
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "source install/setup.bash && ros2 run vision_yolo_detector digit_node" C-m
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "source install/setup.bash && ros2 run vision_yolo_detector camera_saver_node" C-m

# 布局并附加
tmux select-layout -t $SESSION tiled
tmux attach-session -t $SESSION