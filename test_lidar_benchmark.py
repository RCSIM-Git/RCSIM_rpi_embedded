import time
import numpy as np

def _update_from_lidar_orig(pose, data, grid_size, map_offset_x, map_offset_y, resolution):
    x_robot, y_robot, yaw_robot_deg = pose
    yaw_rad = np.deg2rad(yaw_robot_deg)

    angles_deg = data[:, 0]
    distances_m = data[:, 1] / 1000.0

    mask = (distances_m > 0.1) & (distances_m < 8.0)
    angles_rad = np.deg2rad(angles_deg[mask])
    dists = distances_m[mask]

    global_angles = angles_rad + yaw_rad
    xs_global = x_robot + dists * np.cos(global_angles)
    ys_global = y_robot + dists * np.sin(global_angles)

    center = grid_size // 2
    gxs = center + (xs_global - map_offset_x) / resolution
    gys = center + (ys_global - map_offset_y) / resolution
    gxs = np.clip(gxs, 0, grid_size - 1).astype(int)
    gys = np.clip(gys, 0, grid_size - 1).astype(int)

    valid_indices = (
        (gxs >= 0) & (gxs < grid_size) & (gys >= 0) & (gys < grid_size)
    )
    gxs = gxs[valid_indices]
    gys = gys[valid_indices]

    # self.costmap[gxs, gys] = self.OBSTACLE_COST

data = np.random.rand(1000, 2) * 10000

t0 = time.time()
for _ in range(1000):
    _update_from_lidar_orig((0.0, 0.0, 0.0), data, 200, 0.0, 0.0, 0.05)
t1 = time.time()

print(f"Lidar Orig: {t1-t0:.4f}s")
