# -*- coding: utf-8 -*-
"""
Pure Pursuit Planner zintegrowany ze sterowaniem odpychającym
z projektu Kai Nakamury.
"""

import logging
import math
from typing import Any

import numpy as np

from .costmap_manager import CostmapManager

logger: logging.Logger = logging.getLogger(__name__)


class PurePursuitPlanner:
    """
    Poszukuje kąta zgięcia serwa, by pojazd podążał za punktem celu
    (lookahead_point) i implementuje omijanie przeszkód.
    """

    def __init__(
        self, costmap_manager: CostmapManager, config: dict[str, Any] = None
    ) -> None:
        self.cm = costmap_manager
        self.wheelbase: float = 0.25
        self.max_steering_angle: float = np.deg2rad(30)
        self.pure_pursuit_gain: float = 0.8
        self.velocity_max: float = 1.5
        self.lookahead_min: float = 0.6
        self.lookahead_max: float = 4.0
        self.lookahead_gain: float = 0.3
        self.min_obstacle_dist: float = 0.6
        self.emergency_stop_dist: float = 0.3

        self.update_config(config)

    def update_config(self, config: dict[str, Any]) -> None:
        if not config or "autonomous_navigation" not in config:
            return

        nav_config = config["autonomous_navigation"]
        pp_cfg = nav_config.get("pure_pursuit", {})
        self.pure_pursuit_gain = pp_cfg.get("pure_pursuit_gain", self.pure_pursuit_gain)
        self.velocity_max = pp_cfg.get("max_speed_mps", self.velocity_max)
        self.lookahead_min = pp_cfg.get("lookahead_min", self.lookahead_min)
        self.lookahead_max = pp_cfg.get("lookahead_max", self.lookahead_max)
        self.lookahead_gain = pp_cfg.get("lookahead_gain", self.lookahead_gain)

        safety_cfg = nav_config.get("safety", {})
        self.min_obstacle_dist = safety_cfg.get(
            "min_obstacle_distance_m", self.min_obstacle_dist
        )
        self.emergency_stop_dist = safety_cfg.get(
            "emergency_stop_distance_m", self.emergency_stop_dist
        )

    def plan_pure_pursuit(
        self,
        robot_pose: tuple[float, float, float],
        lookahead_point: tuple[float, float],
        current_speed: float = 0.0,
        is_reversed: bool = False,
    ) -> tuple[float, float, float]:
        """
        Zwraca Tuple: (Steering Angle, Throttle, Safety Score 0-1)

        Args:
            robot_pose: (x, y, yaw_deg)
            lookahead_point: (x, y, L) [PLAN-002]
        """
        rx, ry, ryaw_deg = robot_pose
        ryaw = np.deg2rad(ryaw_deg)

        dx = lookahead_point[0] - rx
        dy = lookahead_point[1] - ry
        alpha = math.atan2(dy, dx) - ryaw

        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        # [PLAN-002] Adaptive Lookahead Distance
        # Formula: L = L_min + speed * gain
        if len(lookahead_point) > 2:
            # Overridden by caller if 3rd element exists
            lookahead_distance = lookahead_point[2]
        else:
            lookahead_distance = (
                self.lookahead_min + abs(current_speed) * self.lookahead_gain
            )
            lookahead_distance = np.clip(
                lookahead_distance, self.lookahead_min, self.lookahead_max
            )

        self.logger.debug(
            f"Adaptive Lookahead: Speed={current_speed:.2f}m/s -> L={lookahead_distance:.2f}m"
        )

        if abs(np.sin(alpha)) < 1e-6:
            radius_of_curvature = float("inf")
            base_steering = 0.0
        else:
            radius_of_curvature = lookahead_distance / (2 * np.sin(alpha))
            base_steering = math.atan(
                self.wheelbase * self.pure_pursuit_gain / radius_of_curvature
            )

        steering_adjustment = self._calculate_kai_steering_adjustment(
            robot_pose, is_reversed
        )

        steering = base_steering + steering_adjustment
        steering = float(
            np.clip(steering, -self.max_steering_angle, self.max_steering_angle)
        )

        dist_to_obs = self.cm.get_closest_obstacle_dist(robot_pose)
        safety_score = min(1.0, dist_to_obs / 2.0)

        base_throttle = self.velocity_max * 0.5

        if dist_to_obs < self.emergency_stop_dist:
            throttle = 0.0
        else:
            slow_down_start = max(1.0, self.min_obstacle_dist * 2.0)
            if dist_to_obs >= slow_down_start:
                throttle = base_throttle
            else:
                d_norm = (dist_to_obs - self.emergency_stop_dist) / (
                    slow_down_start - self.emergency_stop_dist
                )
                throttle = base_throttle * max(0.2, d_norm)

        # Hamowanie na ostrych zakrętach:
        throttle = throttle * (1.0 - abs(steering) / self.max_steering_angle * 0.5)

        return float(steering), float(throttle), float(safety_score)

    def _calculate_kai_steering_adjustment(
        self, robot_pose: tuple[float, float, float], is_reversed: bool
    ) -> float:
        """Korekcja wirtualnych magnesów odpychających (Nakamura Force)"""
        rx, ry, ryaw_deg = robot_pose
        ryaw = np.deg2rad(ryaw_deg)

        rgx, rgy = self.cm.world_to_grid(rx, ry)

        FOV_DISTANCE = int(1.25 / self.cm.resolution)
        FOV_RAD = np.deg2rad(200)
        SMALL_FOV_DISTANCE = int(0.5 / self.cm.resolution)
        SMALL_FOV_RAD = np.deg2rad(300)
        FOV_DEADZONE = np.deg2rad(80)

        height, width = self.cm.costmap.shape

        min_x = max(0, rgx - FOV_DISTANCE)
        max_x = min(height, rgx + FOV_DISTANCE + 1)
        min_y = max(0, rgy - FOV_DISTANCE)
        max_y = min(width, rgy + FOV_DISTANCE + 1)

        if min_x >= max_x or min_y >= max_y:
            return 0.0

        local_costmap = self.cm.costmap[min_x:max_x, min_y:max_y]
        walls_gx_local, walls_gy_local = np.where(
            local_costmap >= self.cm.OBSTACLE_COST
        )

        if len(walls_gx_local) == 0:
            return 0.0

        walls_gx = walls_gx_local + min_x
        walls_gy = walls_gy_local + min_y

        dx_grid = walls_gx - rgx
        dy_grid = walls_gy - rgy
        dist_sq = dx_grid**2 + dy_grid**2

        valid = dist_sq <= FOV_DISTANCE**2
        dx_grid = dx_grid[valid]
        dy_grid = dy_grid[valid]
        dist_sq = dist_sq[valid]

        if len(dx_grid) == 0:
            return 0.0

        dist_m = np.sqrt(dist_sq) * self.cm.resolution
        angles = np.arctan2(dy_grid, dx_grid) - ryaw

        if is_reversed:
            angles += np.pi

        angles = (angles + np.pi) % (2 * np.pi) - np.pi

        in_fov = (
            (dist_sq <= FOV_DISTANCE**2)
            & (angles >= -FOV_RAD / 2)
            & (angles <= FOV_RAD / 2)
            & ~(np.abs(angles) < FOV_DEADZONE / 2)
        )
        in_small_fov = (
            (dist_sq <= SMALL_FOV_DISTANCE**2)
            & (angles >= -SMALL_FOV_RAD / 2)
            & (angles <= SMALL_FOV_RAD / 2)
        )
        kept = in_fov | in_small_fov

        dist_m_kept = dist_m[kept]
        angles_kept = angles[kept]

        if len(dist_m_kept) == 0:
            return 0.0

        weights = np.zeros_like(dist_m_kept)
        non_zero = dist_m_kept > 0
        weights[non_zero] = 1.0 / (dist_m_kept[non_zero] ** 2)

        total_weight = np.sum(weights)
        wall_cell_count = len(weights)

        if total_weight == 0 or wall_cell_count == 0:
            return 0.0

        average_angle = np.sum(weights * angles_kept) / total_weight

        OBSTACLE_AVOIDANCE_GAIN = 0.3
        steering_adjustment = -OBSTACLE_AVOIDANCE_GAIN * average_angle / wall_cell_count

        return float(steering_adjustment)
