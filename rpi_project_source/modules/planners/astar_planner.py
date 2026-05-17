# -*- coding: utf-8 -*-
"""
Algorytm A* Path Planning oparty na lokalnej mapie kosztów.
Wyszukuje optymalną bezkolizyjną ścieżkę w globalnym układzie.
"""

import heapq
import logging
import math

from .costmap_manager import CostmapManager

logger: logging.Logger = logging.getLogger(__name__)


class AStarPlanner:
    """
    Znajduje najkrótszą drogę omijającą przeszkody zebraną z radaru LDS-02/Yolov8
    za pomocą tradycyjnego grafowego algorytmu A-Star.
    """

    A_STAR_PENALTY: float = 10.0

    def __init__(self, costmap_manager: CostmapManager) -> None:
        self.cm = costmap_manager

    def plan_path(
        self,
        start_pose: tuple[float, float],
        goal_global: tuple[float, float],
        world_coordinates: bool = True,
    ) -> list[tuple[float, float]] | list[tuple[int, int]] | None:
        """
        Zwraca listę punktów drogi do celu.

        Args:
            start_pose: Pozycja startowa (x, y) w metrach.
            goal_global: Cel (x, y) w metrach.
            world_coordinates: Jeśli True, zwraca współrzędne w metrach.
                              Jeśli False, zwraca indeksy siatki.
        """
        sx, sy = self.cm.world_to_grid(start_pose[0], start_pose[1])
        gx, gy = self.cm.world_to_grid(goal_global[0], goal_global[1])

        height, width = self.cm.costmap.shape

        if not (0 <= sx < height and 0 <= sy < width):
            self.cm.check_and_scroll_map(start_pose[0], start_pose[1])
            sx, sy = self.cm.world_to_grid(start_pose[0], start_pose[1])
            if not (0 <= sx < height and 0 <= sy < width):
                logger.warning("A* Start poza granicami mapy kosztów po rotacji.")
                return None

        if not (0 <= gx < height and 0 <= gy < width):
            logger.warning("A* Cel (Goal) leży poza widnokręgiem układu.")
            return None

        # Zablokowany cel?
        if self.cm.costmap[gx, gy] >= self.cm.OBSTACLE_COST:
            return None

        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))

        came_from = {}
        g_score = {(sx, sy): 0.0}

        def heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
            return math.hypot(a[0] - b[0], a[1] - b[1])

        neighbors = [
            (0, 1, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (-1, 0, 1.0),
            (1, 1, 1.414),
            (1, -1, 1.414),
            (-1, 1, 1.414),
            (-1, -1, 1.414),
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == (gx, gy):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((sx, sy))
                grid_path = path[::-1]

                if world_coordinates:
                    # Convert to world coordinates (meters)
                    world_path = []
                    for gx, gy in grid_path:
                        wx, wy = self.cm.grid_to_world(gx, gy)
                        world_path.append((float(wx), float(wy)))
                    return world_path

                return grid_path

            cx, cy = current
            for dx, dy, move_cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < height and 0 <= ny < width:
                    cell_cost = self.cm.costmap[nx, ny]
                    if cell_cost >= self.cm.OBSTACLE_COST:
                        continue

                    tentative_g_score = g_score[current] + move_cost * (
                        1.0 + cell_cost * self.A_STAR_PENALTY
                    )
                    neighbor = (nx, ny)

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor, (gx, gy))
                        heapq.heappush(open_set, (f_score, neighbor))

        return None
