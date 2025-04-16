import numpy as np

def estimate_local_coords(pixel_height, object_height_m, scale_k):
    # 使用经验公式估算目标在机器人局部坐标系中的距离（基于高度）
    distance_m = scale_k * object_height_m / pixel_height
    return distance_m, 0

def transform_to_world(robot_pose, x_rel, y_rel, ptz_pan_deg=0.0):
    total_theta = np.deg2rad(robot_pose["theta_deg"] + ptz_pan_deg)
    R = np.array([
        [np.cos(total_theta), -np.sin(total_theta)],
        [np.sin(total_theta),  np.cos(total_theta)]
    ])
    rel_pos = np.array([[x_rel], [y_rel]])
    base_pos = np.array([[robot_pose["x"]], [robot_pose["y"]]])
    world_pos = R @ rel_pos + base_pos
    return float(world_pos[0]), float(world_pos[1])

def estimate_pixel_distance(center1, center2, scale_k):
    # 输入为像素坐标 (x, y)，输出为估算的实际距离（米）
    dx = center2[0] - center1[0]
    dy = center2[1] - center1[1]
    pixel_distance = np.sqrt(dx ** 2 + dy ** 2)
    distance_m = pixel_distance / scale_k  # 假设 scale_k 是像素 / 米
    return distance_m
