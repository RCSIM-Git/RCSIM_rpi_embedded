# -*- coding: utf-8 -*-
"""
Global Planner (A*) for Raspberry Pi.

Wyznacza ścieżkę na mapie zajętości (SLAM Grid) od punktu startowego do celu.
Calculates a path on the occupancy grid (SLAM Grid) from start to goal.

[COORDINATE SYSTEM]
- X (meters) -> Map Columns (c)
- Y (meters) -> Map Rows (r)
- Center (0,0) meters is grid center (rows//2, cols//2)
"""

import heapq
import logging
import math

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class GlobalPlanner:
    """
    Planer globalny wykorzystujący algorytm A* na siatce zajętości.
    Global planner using A* algorithm on occupancy grid.
    """

    def __init__(
        self,
        logger: logging.Logger | None = None,
        resolution: float = 0.05,
        inflation_radius: float = 0.2,
    ):
        """
        Args:
            logger (logging.Logger): Logger instance.
            resolution (float): Map resolution [m/pixel].
            inflation_radius (float): Obstacle inflation radius [m].
        """
        self.logger = logger or logging.getLogger(__name__)
        self.resolution = resolution
        self.inflation_radius_px = int(math.ceil(inflation_radius / resolution))

        # Directions for 8-connected grid (x, y, cost)
        self.directions = [
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (1, 1, 1.414),
            (1, -1, 1.414),
            (-1, 1, 1.414),
            (-1, -1, 1.414),
        ]

    def calc_cost_map(self, grid: np.ndarray) -> np.ndarray:
        """
        Wycznacza mapę kosztów (Hallway Costmap) dla A*.
        Omija ściany trzymając robota możliwie na środku pomieszczeń/korytarzy.
        """
        # Free space = 255, Obstacle (and unknown > 127) = 0
        free_mask = np.zeros_like(grid, dtype=np.uint8)
        free_mask[grid < 127] = 255

        # Oblicza dystans komórek od najbliższych przeszkód (0 dla przeszkód)
        dist_from_obs = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)

        max_dist = float(np.max(dist_from_obs))
        if max_dist == 0:
            return np.zeros_like(grid, dtype=np.float32)

        # Proporcjonalna odległość od ścian.
        # Im dalej od ściany, tym koszt bliższy 0.
        cost_map = ((max_dist - dist_from_obs) / max_dist) ** 2
        return cost_map

    def plan_path(
        self,
        grid: np.ndarray,
        start_pose: tuple[float, float],
        goal_pose: tuple[float, float],
        origin: tuple[float, float] = (0.0, 0.0),
    ) -> list[tuple[float, float]] | None:
        """
        Planuje ścieżkę z punktu Start do Celu.
        Plans path from Start to Goal.

        Args:
            grid (np.ndarray): Mapa zajętości (0=wolne, >0=zajęte/nieznane).
            start_pose (x, y): Pozycja startowa [m].
            goal_pose (x, y): Pozycja celu [m].
            origin (x, y): Central map point in meters.
        """
        rows, cols = grid.shape

        center_r, center_c = rows // 2, cols // 2

        start_c = int(center_c + start_pose[0] / self.resolution)
        start_r = int(center_r + start_pose[1] / self.resolution)

        goal_c = int(center_c + goal_pose[0] / self.resolution)
        goal_r = int(center_r + goal_pose[1] / self.resolution)

        # Check bounds
        if not (0 <= start_c < cols and 0 <= start_r < rows):
            self.logger.error("Start position out of map bounds.")
            return None
        if not (0 <= goal_c < cols and 0 <= goal_r < rows):
            self.logger.error("Goal position out of map bounds.")
            return None

        if grid[goal_r, goal_c] > 127:
            self.logger.warning("Goal is inside an obstacle.")
            return None

        # [NEW] Hallway Costmap
        cost_map = self.calc_cost_map(grid)
        cost_map_weight = 10.0
        OBSTACLE_COST: float = 1.0
        ROUGH_TERRAIN_COST: float = 0.5

        # [PLAN-010] Traversability Cost
        terrain_map = getattr(self, "terrain_map", None)
        terrain_weight = 20.0

        # A* Algorithm
        open_set = []
        heapq.heappush(open_set, (0, start_r, start_c))

        came_from = {}
        g_score = np.full((rows, cols), np.inf, dtype=np.float32)
        g_score[start_r, start_c] = 0.0

        def heuristic(r, c):
            return math.hypot(goal_r - r, goal_c - c)

        max_iters = rows * cols // 4
        iters = 0

        while open_set:
            _, current_r, current_c = heapq.heappop(open_set)

            if (current_r, current_c) == (goal_r, goal_c):
                path = []
                curr = (goal_r, goal_c)
                while curr in came_from:
                    r, c = curr
                    x_m = (c - center_c) * self.resolution
                    y_m = (r - center_r) * self.resolution
                    path.append((x_m, y_m))
                    curr = came_from[curr]
                path.reverse()
                return path

            for dr, dc, base_cost_step in self.directions:
                neighbor_r, neighbor_c = current_r + dr, current_c + dc

                if 0 <= neighbor_r < rows and 0 <= neighbor_c < cols:
                    # Check occupancy
                    cell_val = grid[neighbor_r, neighbor_c]

                    # Treat Unknown (0 or 128?) as traversable but expensive?
                    # Let's assume strict collision for now: > 127 = Obstacle
                    if cell_val > 127:
                        continue

                    # [NEW] Hallway Cost Penalty
                    hallway_penalty = cost_map_weight * float(
                        cost_map[neighbor_r, neighbor_c]
                    )

                    # [PLAN-010] Terrain Cost Penalty
                    terrain_penalty = 0.0
                    if terrain_map is not None:
                        terrain_penalty = terrain_weight * float(
                            terrain_map[neighbor_r, neighbor_c]
                        )

                    added_cost = base_cost_step + hallway_penalty + terrain_penalty

                    tentative_g_score = g_score[current_r, current_c] + added_cost

                    if tentative_g_score < g_score[neighbor_r, neighbor_c]:
                        came_from[(neighbor_r, neighbor_c)] = (current_r, current_c)
                        g_score[neighbor_r, neighbor_c] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor_r, neighbor_c)
                        heapq.heappush(open_set, (f_score, neighbor_r, neighbor_c))

            iters += 1
            if iters > max_iters:
                self.logger.error("GlobalPlanner: A* iteration limit exceeded.")
                break

        self.logger.warning("Global Planner: No path found.")
        return None
