#!/bin/bash

# 进入你的工作区路径（可选）
cd /home/team10/project/AUTO4508

# 激活虚拟环境
source /opt/venvs/vision_env/bin/activate

# 设置 ROS 环境（重要）
source install/setup.bash

# 启动 ROS 2 服务节点
ros2 run my_robot_vision align_capture_service_node.py
