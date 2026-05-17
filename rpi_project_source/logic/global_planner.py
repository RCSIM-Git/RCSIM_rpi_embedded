"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""
import heapq
import logging
import math

import numpy as np


class GlobalPlanner:
    """
    Planer globalny ścieżki (Global Path Planner) dla RPi.
    Implementuje algorytm A* na mapie zajętości (grid map).
    Zoptymalizowany pod kątem RPi 5.
    """

    def __init__(self, logger: logging.Logger, resolution: float = 0.1):
        """
        Inicjalizuje globalny planer.
        Args:
            logger (logging.Logger): Instancja loggera.
            resolution (float): Rozdzielczość mapy w metrach (rozmiar komórki).
        """
        self.logger = logger
        self.resolution = resolution
        self.cost_straight = 1.0
        self.cost_diagonal = 1.414

    def plan(
        self,
        start_pose: tuple[float, float],
        goal_pose: tuple[float, float],
        grid_map: np.ndarray,
        origin: tuple[float, float] = (0.0, 0.0),
    ) -> list[tuple[float, float]]:
        """
        Znajduje ścieżkę od startu do celu używając A*.
        Args:
            start_pose: (x, y) w metrach.
            goal_pose: (x, y) w metrach.
            grid_map: 2D array (0=free, 100=occupied).
            origin: (x, y) lewego dolnego rogu mapy.
        """
        start_idx = self._world_to_grid(start_pose, origin)
        goal_idx = self._world_to_grid(goal_pose, origin)

        if not self._is_valid(start_idx, grid_map):
            self.logger.warning(f"Start index {start_idx} is invalid (occupied or OOB)")
            return []

        if not self._is_valid(goal_idx, grid_map):
            self.logger.warning(f"Goal index {goal_idx} is invalid (occupied or OOB)")
            return []

        open_set = []
        heapq.heappush(open_set, (0, start_idx))
        came_from = {}
        g_score = {start_idx: 0}

        rows, cols = grid_map.shape
        max_iters = rows * cols

        iters = 0
        while open_set:
            if iters > max_iters:
                self.logger.error("A* search limit reached.")
                break
            iters += 1

            _, current = heapq.heappop(open_set)

            if current == goal_idx:
                path = self._reconstruct_path(came_from, current, origin)
                return self.smooth_path(path)

            # 8-direction neighbors
            for dx, dy, cost in [
                (0, 1, 1.0),
                (0, -1, 1.0),
                (1, 0, 1.0),
                (-1, 0, 1.0),
                (1, 1, 1.414),
                (1, -1, 1.414),
                (-1, 1, 1.414),
                (-1, -1, 1.414),
            ]:
                neighbor = (current[0] + dx, current[1] + dy)

                if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows:
                    if grid_map[neighbor[1], neighbor[0]] > 50:  # Occupied
                        continue

                    tentative_g = g_score[current] + cost
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f = tentative_g + self._heuristic(neighbor, goal_idx)
                        heapq.heappush(open_set, (f, neighbor))

        return []

    def _world_to_grid(self, pose, origin):
        return (
            int((pose[0] - origin[0]) / self.resolution),
            int((pose[1] - origin[1]) / self.resolution),
        )

    def _grid_to_world(self, idx, origin):
        return (
            idx[0] * self.resolution + origin[0],
            idx[1] * self.resolution + origin[1],
        )

    def _heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _is_valid(self, idx, grid_map):
        cols, rows = grid_map.shape[1], grid_map.shape[0]
        if not (0 <= idx[0] < cols and 0 <= idx[1] < rows):
            return False
        return grid_map[idx[1], idx[0]] <= 50

    def _reconstruct_path(self, came_from, current, origin):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return [self._grid_to_world(idx, origin) for idx in path]

    def smooth_path(self, path: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """Wygładza ścieżkę (Catmull-Rom) i resampluje co 15cm."""
        if len(path) < 3:
            return path

        # Proste wygładzanie i resamplowanie (uproszczone pod kątem RPi)
        resampled = [path[0]]
        step = 0.15  # 15cm

        last_p = np.array(path[0])
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])
            vec = p2 - p1
            dist = np.linalg.norm(vec)
            if dist < 0.001:
                continue

            while True:
                dist_to_start = np.linalg.norm(p1 - last_p)
                needed = step - dist_to_start
                if needed <= dist:
                    new_p = p1 + (vec / dist) * max(0, needed)
                    resampled.append(tuple(new_p))
                    last_p = new_p
                    p1 = new_p
                    vec = p2 - p1
                    dist = np.linalg.norm(vec)
                else:
                    break

        if np.linalg.norm(np.array(path[-1]) - np.array(resampled[-1])) > step / 2:
            resampled.append(path[-1])

        return resampled
