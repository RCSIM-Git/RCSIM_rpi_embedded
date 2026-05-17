# -*- coding: utf-8 -*-
"""
Menadżer Mapy Kosztów (CostmapManager)
Zarządza globalną siatką occupancy grid, aktualizuje ją danymi LiDAR oraz YOLO
oraz oblicza transformatę dystansu (Distance Transform).

Separated from local_planner.py.
"""

import logging
import math
from typing import Any

import cv2
import numpy as np

logger: logging.Logger = logging.getLogger(__name__)


class CostmapManager:
    """
    Zarządza mapą zajętości 2D (Costmap) w systemie lokalnym RPi.
    """

    OBSTACLE_COST: float = 1.0
    ROUGH_TERRAIN_COST: float = 0.5  # Penalty for slopes [PLAN-010]

    def __init__(
        self,
        grid_size: int = 400,
        resolution: float = 0.05,
        inflation_radius_cm: float = 10.0,
        config: dict[str, Any] = None,
    ) -> None:
        self.grid_size: int = grid_size
        self.resolution: float = resolution
        self.inflation_radius_cm: float = inflation_radius_cm
        self.config: dict[str, Any] = config or {}

        self.map_offset_x: float = 0.0
        self.map_offset_y: float = 0.0

        self.costmap: np.ndarray = np.zeros((grid_size, grid_size), dtype=np.float32)

        # Puste pole = max dystans
        free_map = np.ones((grid_size, grid_size), dtype=np.uint8)
        self.dist_transform: np.ndarray = (
            cv2.distanceTransform(free_map, cv2.DIST_L2, 5) * resolution
        )

    def clear_map(self) -> None:
        """Czyści grid costmapy."""
        self.costmap.fill(0)

    def world_to_grid(
        self, x: float | np.ndarray, y: float | np.ndarray
    ) -> tuple[int | np.ndarray, int | np.ndarray]:
        """Konwertuje globalne pozycje świata metrycznego na indeksy siatki."""
        center = self.grid_size // 2
        # [PLAN-004] Safe conversion with clipping BEFORE int casting
        if isinstance(x, np.ndarray):
            gx = center + (x - self.map_offset_x) / self.resolution
            gy = center + (y - self.map_offset_y) / self.resolution
            gx = np.clip(gx, 0, self.grid_size - 1).astype(int)
            gy = np.clip(gy, 0, self.grid_size - 1).astype(int)
        else:
            gx = center + (x - self.map_offset_x) / self.resolution
            gy = center + (y - self.map_offset_y) / self.resolution
            gx = int(max(0, min(self.grid_size - 1, gx)))
            gy = int(max(0, min(self.grid_size - 1, gy)))
        return gx, gy

    def grid_to_world(
        self, gx: int | np.ndarray, gy: int | np.ndarray
    ) -> tuple[float | np.ndarray, float | np.ndarray]:
        """Konwertuje indeksy siatki na pozycje świata metrycznego."""
        center = self.grid_size // 2
        x = (gx - center) * self.resolution + self.map_offset_x
        y = (gy - center) * self.resolution + self.map_offset_y
        return x, y

    def check_and_scroll_map(self, rx: float, ry: float) -> None:
        """
        Centruje mapę na robocie kasując przeterminowane bloki,
        jeśli zbytnio zbliża się do jej granic (1.5 metra ochrony = 30 komórek).
        """
        center = self.grid_size // 2
        gx = int(center + (rx - self.map_offset_x) / self.resolution)
        gy = int(center + (ry - self.map_offset_y) / self.resolution)

        margin = 30
        if (
            gx < margin
            or gx >= self.grid_size - margin
            or gy < margin
            or gy >= self.grid_size - margin
        ):
            logger.debug(
                f"Robot at costmap edge ({gx},{gy}). "
                f"Re-centering map to ({rx:.2f}, {ry:.2f})."
            )
            self.map_offset_x = rx
            self.map_offset_y = ry
            self.clear_map()

    def update_occupancy(
        self,
        pose: tuple[float, float, float],
        lidar_points: list[tuple[float, float]],
        imu_orientation: dict[str, float] = None,  # [PLAN-010] Pitch/Roll
    ) -> None:
        """
        Główna rura aktualizacji costmapy na daną klatkę czasową.
        """
        self._update_from_lidar(pose, lidar_points)

        if imu_orientation:
            self._update_from_terrain(pose, imu_orientation)

        # [PLAN-010] Powolne wygasanie kosztów terenu (Decay)
        self._decay_terrain_costs()

        # Inflate obstacles
        iterations = int(self.inflation_radius_cm / (self.resolution * 100.0))
        iterations = max(1, iterations)
        self._inflate_map(iterations=iterations)

        # Update Distance Transform
        binary_map = (self.costmap < self.OBSTACLE_COST).astype(np.uint8)
        self.dist_transform = (
            cv2.distanceTransform(binary_map, cv2.DIST_L2, 5) * self.resolution
        )

    def _update_from_lidar(
        self, pose: tuple[float, float, float], lidar_points: list[tuple[float, float]]
    ) -> None:
        if not lidar_points:
            return

        x_robot, y_robot, yaw_robot_deg = pose
        yaw_rad = np.deg2rad(yaw_robot_deg)

        self.check_and_scroll_map(x_robot, y_robot)

        data = np.array(lidar_points)
        if data.size == 0:
            return

        angles_deg = data[:, 0]
        distances_m = data[:, 1] / 1000.0

        mask = (distances_m > 0.1) & (distances_m < 8.0)
        angles_rad = np.deg2rad(angles_deg[mask])
        dists = distances_m[mask]

        global_angles = angles_rad + yaw_rad
        xs_global = x_robot + dists * np.cos(global_angles)
        ys_global = y_robot + dists * np.sin(global_angles)

        gxs, gys = self.world_to_grid(xs_global, ys_global)

        valid_indices = (
            (gxs >= 0) & (gxs < self.grid_size) & (gys >= 0) & (gys < self.grid_size)
        )
        gxs = gxs[valid_indices]
        gys = gys[valid_indices]

        self.costmap[gxs, gys] = self.OBSTACLE_COST

    def _update_from_yolo(
        self,
        pose: tuple[float, float, float],
        detections: list[dict[str, Any]],
        camera_extrinsics: dict[str, Any],
    ) -> None:
        robot_x, robot_y, robot_theta_deg = pose

        cam_config = self.config.get("camera", {})
        fx = cam_config.get("fx", 500.0)
        fy = cam_config.get("fy", 500.0)
        cx = cam_config.get("cx", 320.0)
        cy = cam_config.get("cy", 240.0)
        img_w = cam_config.get("width", 640)
        img_h = cam_config.get("height", 480)

        cam_x = camera_extrinsics.get("x", 0.1)
        cam_y = camera_extrinsics.get("y", 0.0)
        cam_z = camera_extrinsics.get("z", 0.15)
        cam_pitch = camera_extrinsics.get("pitch", 0.0)

        for det in detections:
            bbox = det.get("bbox")
            if not bbox:
                continue

            u = (bbox[0] + bbox[2]) / 2.0 * img_w
            v = bbox[3] * img_h

            ray_x = (u - cx) / fx
            ray_y = (v - cy) / fy
            ray_z = 1.0

            angle_v = math.atan2(ray_y, ray_z)
            total_pitch = cam_pitch + angle_v

            # [PLAN-004] ZeroDivision Protection: Ensure total_pitch is safe for math.tan
            safe_pitch = max(0.02, total_pitch)
            dist_ground = cam_z / math.tan(safe_pitch)

            if dist_ground > 8.0 or dist_ground < 0.1:
                continue

            angle_h = math.atan2(ray_x, ray_z)

            obj_x_local = dist_ground * math.cos(angle_h)
            obj_y_local = -dist_ground * math.sin(angle_h)

            obj_x_local += cam_x
            obj_y_local += cam_y

            theta_rad = np.deg2rad(robot_theta_deg)
            wc = math.cos(theta_rad)
            ws = math.sin(theta_rad)

            obj_x_world = obj_x_local * wc - obj_y_local * ws + robot_x
            obj_y_world = obj_x_local * ws + obj_y_local * wc + robot_y

            gx, gy = self.world_to_grid(obj_x_world, obj_y_world)

            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                self.costmap[gx, gy] = self.OBSTACLE_COST

    def _update_from_terrain(
        self, pose: tuple[float, float, float], imu_orientation: dict[str, float]
    ) -> None:
        """
        Nanosi koszty trudu terenu na podstawie nachylenia (Pitch/Roll).
        """
        pitch = imu_orientation.get("pitch", 0.0)
        roll = imu_orientation.get("roll", 0.0)

        # Progi nachylenia (stopnie)
        nav_cfg = self.config.get("navigation", {})
        SLOPE_THRESHOLD = nav_cfg.get("rough_terrain_threshold_deg", 15.0)

        max_slope = max(abs(pitch), abs(roll))

        if max_slope > SLOPE_THRESHOLD:
            # Robot jest na trudnym terenie - oznaczamy komórki pod nim
            rx, ry, _ = pose
            gx, gy = self.world_to_grid(rx, ry)

            # Dodajemy koszt w promieniu robota (np. 5x5 komórek)
            # Im większe nachylenie, tym wyższy koszt (do limitu OBSTACLE_COST)
            slope_factor = min(1.0, (max_slope - SLOPE_THRESHOLD) / 20.0)
            added_cost = self.ROUGH_TERRAIN_COST + (slope_factor * 0.4)

            radius = 3
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        # Nie nadpisuj przeszkód twardych (murów)
                        if self.costmap[nx, ny] < self.OBSTACLE_COST:
                            # Zachowaj wyższy koszt jeśli już tam był
                            self.costmap[nx, ny] = max(self.costmap[nx, ny], added_cost)

    def _decay_terrain_costs(self) -> None:
        """Powolne zmniejszanie kosztów terenu, aby robot 'zapominał' o starych wstrząsach."""
        DECAY_RATE = 0.01  # Co klatkę odejmujemy 1% kosztu

        # Tylko dla komórek które NIE są przeszkodami twardymi
        terrain_mask = (self.costmap > 0) & (self.costmap < self.OBSTACLE_COST)
        self.costmap[terrain_mask] -= DECAY_RATE
        self.costmap[self.costmap < 0] = 0

    def _inflate_map(self, iterations: int = 1) -> None:
        mask = (self.costmap >= self.OBSTACLE_COST).astype(np.uint8)

        if not np.any(mask):
            return

        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(mask, kernel, iterations=iterations)

        self.costmap[dilated > 0] = np.maximum(
            self.costmap[dilated > 0], self.OBSTACLE_COST
        )

    def get_closest_obstacle_dist(
        self, robot_pose: tuple[float, float, float] = (0, 0, 0)
    ) -> float:
        """Odszukuje na Distance Transform Map odległość robota od fizycznej granicy."""
        rx, ry, _ = robot_pose
        gx, gy = self.world_to_grid(rx, ry)

        # Boundary check is already in world_to_grid (clipped),
        # but we add an extra layer here for clarity and safety.
        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            return float(self.dist_transform[gx, gy])

        return 0.0

    def is_line_free(self, x0: int, y0: int, x1: int, y1: int) -> bool:
        """
        Wektoryzowane sprawdzanie pustej widoczności linii między 2 grid blockami.
        """
        num_points = int(np.hypot(x1 - x0, y1 - y0))
        if num_points == 0:
            return True

        height, width = self.costmap.shape
        ts = np.linspace(0, 1, num_points, endpoint=False)
        xs = (x0 + (x1 - x0) * ts).astype(int)
        ys = (y0 + (y1 - y0) * ts).astype(int)

        valid = (xs >= 0) & (xs < height) & (ys >= 0) & (ys < width)
        xs = xs[valid]
        ys = ys[valid]

        return not np.any(self.costmap[xs, ys] >= self.OBSTACLE_COST)
