#!/usr/bin/env python3
import numpy as np

def transform_to_world(robot_pose, x_rel, y_rel, heading_deg):
    """
    将相对坐标 (x_rel, y_rel) 转换为世界坐标 (X, Y)

    参数:
        robot_pose: {"x": float, "y": float, "theta_deg": float}
        x_rel, y_rel: 相对坐标（单位：米）
        heading_deg: 摄像头朝向（默认填 0.0 即正前方）

    返回:
        x_world, y_world
    """
    # 相机相对机器人方向（例如正前方）
    angle_deg = robot_pose["theta_deg"] + heading_deg
    angle_rad = np.deg2rad(angle_deg)

    R = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad),  np.cos(angle_rad)]
    ])

    rel = np.array([x_rel, y_rel])
    base = np.array([robot_pose["x"], robot_pose["y"]])

    world = base + R @ rel
    return world[0], world[1]

