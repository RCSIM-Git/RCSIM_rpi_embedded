# -*- coding: utf-8 -*-
"""
Reaktywny Planer Dynamic Window Approach (DWA) Gap Follower.
Znajduje wolne luki i omija przeszkody lokalnie korzystając z Mapy Kosztów.
"""

import logging
import math
from typing import Any

import numpy as np

from .costmap_manager import CostmapManager

logger: logging.Logger = logging.getLogger(__name__)


class ReactivePlanner:
    """
    Znajduje wolne strefy przestrzeni (Gap Finding) skanując komórki mapy
    zajętości. Oblicza bezpieczną prędkość redukując gaz przy ścianach.
    """

    def __init__(
        self, costmap_manager: CostmapManager, config: dict[str, Any] = None
    ) -> None:
        self.cm = costmap_manager

        self.wheelbase: float = 0.25
        self.max_steering_angle: float = np.deg2rad(30)
        self.lookahead_dist: float = 1.0
        self.pure_pursuit_gain: float = 0.8
        self.velocity_max: float = 1.5

        self.min_obstacle_dist: float = 0.6
        self.emergency_stop_dist: float = 0.3

        self.update_config(config)

    def update_config(self, config: dict[str, Any]) -> None:
        if not config or "autonomous_navigation" not in config:
            return

        nav_config = config["autonomous_navigation"]

        pp_cfg = nav_config.get("pure_pursuit", {})
        self.lookahead_dist = pp_cfg.get("lookahead_distance_m", self.lookahead_dist)
        self.pure_pursuit_gain = pp_cfg.get("pure_pursuit_gain", self.pure_pursuit_gain)
        self.velocity_max = pp_cfg.get("max_speed_mps", self.velocity_max)

        safety_cfg = nav_config.get("safety", {})
        self.min_obstacle_dist = safety_cfg.get(
            "min_obstacle_distance_m", self.min_obstacle_dist
        )
        self.emergency_stop_dist = safety_cfg.get(
            "emergency_stop_distance_m", self.emergency_stop_dist
        )

    def plan_reactive(
        self,
        robot_pose: tuple[float, float, float] = (0, 0, 0),
        goal_pose: tuple[float, float] = None,
    ) -> tuple[float, float, float]:
        """
        Zwraca tuple (Steering Angle, Throttle, Safety Score <0-1>).
        Skanuje łuk od -60 do +60 stopni szukając szerokich przerw w LiDARowych chmurach.
        """
        height, width = self.cm.costmap.shape
        rx, ry, ryaw_deg = robot_pose
        ryaw = np.deg2rad(ryaw_deg)

        self.cm.check_and_scroll_map(rx, ry)
        rx_grid, ry_grid = self.cm.world_to_grid(rx, ry)

        if not (0 <= rx_grid < height and 0 <= ry_grid < width):
            logger.warning("Robot wypadł poza tablicę NumPy pomimo auto-centrowania!")
            return 0.0, 0.0, 0.0

        target_heading_local = 0.0
        if goal_pose:
            gx, gy = goal_pose
            dx = gx - rx
            dy = gy - ry
            angle_to_goal = math.atan2(dy, dx)
            target_heading_local = angle_to_goal - ryaw
            target_heading_local = (target_heading_local + np.pi) % (2 * np.pi) - np.pi

        angles = np.linspace(-np.pi / 3, np.pi / 3, 40)
        free_angles = []

        for angle in angles:
            global_ray_angle = ryaw + angle
            x_ray = self.lookahead_dist * np.cos(global_ray_angle)
            y_ray = self.lookahead_dist * np.sin(global_ray_angle)

            gx, gy = self.cm.world_to_grid(rx + x_ray, ry + y_ray)

            if 0 <= gx < height and 0 <= gy < width:
                if self.cm.costmap[gx, gy] < self.cm.OBSTACLE_COST:
                    if self.cm.is_line_free(rx_grid, ry_grid, gx, gy):
                        free_angles.append(angle)

        base_throttle = self.velocity_max * 0.5
        closest_obs = self.cm.get_closest_obstacle_dist(robot_pose)
        safety_score = min(1.0, closest_obs / 2.0)

        if closest_obs < self.emergency_stop_dist or not free_angles:
            return 0.0, 0.0, 0.0

        # Grupowanie luk
        clusters = []
        curr = [free_angles[0]]
        for i in range(1, len(free_angles)):
            if free_angles[i] - free_angles[i - 1] < 0.15:
                curr.append(free_angles[i])
            else:
                clusters.append(curr)
                curr = [free_angles[i]]
        clusters.append(curr)

        best_cluster = None
        best_score = -float("inf")

        for cluster in clusters:
            gap_center = np.mean(cluster)
            gap_width = len(cluster)

            width_score = gap_width * 1.0
            alignment_score = -abs(gap_center - target_heading_local) * 10.0
            score = width_score + alignment_score

            if score > best_score:
                best_score = score
                best_cluster = cluster

        target_angle = np.mean(best_cluster)
        gap_width_ratio = len(best_cluster) / len(angles)

        throttle = self.calculate_velocity_scaling(
            closest_obs, base_throttle, gap_width_ratio
        )

        target_x = self.lookahead_dist * np.cos(target_angle)
        target_y = self.lookahead_dist * np.sin(target_angle)

        l2 = target_x**2 + target_y**2
        curvature = 2 * target_y / l2
        steering = math.atan(curvature * self.wheelbase * self.pure_pursuit_gain)

        steering = np.clip(steering, -self.max_steering_angle, self.max_steering_angle)
        throttle = throttle * (1.0 - abs(steering) / self.max_steering_angle * 0.5)

        return float(steering), float(throttle), float(safety_score)

    def calculate_velocity_scaling(
        self, closest_obs_dist: float, base_throttle: float, gap_width_ratio: float
    ) -> float:
        """
        Adaptacyjne zwalnianie predkości wobec wąskich przesmyków (Tunnel Effect).
        Oblicza krzywą przyrostową hamowania do Distance Transformu mapy.
        """
        if closest_obs_dist <= self.emergency_stop_dist:
            return 0.0

        slow_down_start = max(1.0, self.min_obstacle_dist * 2.5)

        if closest_obs_dist >= slow_down_start:
            dist_factor = 1.0
        else:
            d_norm = (closest_obs_dist - self.emergency_stop_dist) / (
                slow_down_start - self.emergency_stop_dist
            )
            d_norm = max(0.0, min(1.0, d_norm))
            dist_factor = d_norm**1.5

        gap_factor = 0.3 + 0.7 * min(1.0, gap_width_ratio / 0.5)
        target_throttle = base_throttle * dist_factor * gap_factor

        if target_throttle < 0.15 and dist_factor > 0.1:
            target_throttle = 0.15

        return max(0.0, target_throttle)
