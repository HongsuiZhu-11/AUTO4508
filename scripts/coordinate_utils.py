import numpy as np

def estimate_local_coords(pixel_height, object_height_m, scale_k):
    # 使用经验公式估算目标在机器人局部坐标系中的距离（基于高度）
    # x' = k * d / i，其中 i 为像素高度，d 为目标真实高度
    distance_m = scale_k * object_height_m / pixel_height
    return distance_m, 0  # 简化版，仅估算前向距离（x方向）

def transform_to_world(robot_pose, x_rel, y_rel, ptz_pan_deg=0.0):
    # robot_pose: dict with x, y, theta_deg
    # ptz_pan_deg: 当前 PTZ 摄像头朝向（相对于机器人正前方，单位度）
    total_theta = np.deg2rad(robot_pose["theta_deg"] + ptz_pan_deg)
    R = np.array([
        [np.cos(total_theta), -np.sin(total_theta)],
        [np.sin(total_theta),  np.cos(total_theta)]
    ])
    rel_pos = np.array([[x_rel], [y_rel]])
    base_pos = np.array([[robot_pose["x"]], [robot_pose["y"]]])
    world_pos = R @ rel_pos + base_pos
    return float(world_pos[0]), float(world_pos[1])