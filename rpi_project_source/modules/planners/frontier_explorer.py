# -*- coding: utf-8 -*-
"""
Frontier Explorer for Raspberry Pi.

Moduł do wykrywania granic między obszarami poznanymi a niepoznanymi
na siatce zajętości (Occupancy Grid / SLAM map).
Służy do automatycznej eksploracji (Auto-Explore).
Na podstawie podejścia: Expanding Wavefront Frontier Detection.
"""

import logging
from collections import deque
from typing import List, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class Frontier:
    """Reprezentacja pojedynczej granicy (Frontier)."""

    def __init__(self, size: int, centroid_m: Tuple[float, float]):
        self.size = size
        self.centroid_m = centroid_m


class FrontierExplorer:
    """
    Wyszukuje granice eksploracji w siatce mapy i wybiera najbardziej obiecujący cel.
    """

    def __init__(
        self,
        resolution: float = 0.05,
        unknown_value: int = 127,  # Zależy od wyjścia mapy SLAM (niektóre dają 127 lub 128 lub 0)
        walkable_threshold: int = 50,
        min_frontier_size: int = 8,
    ):
        self.resolution = resolution
        self.unknown_value = unknown_value
        self.walkable_threshold = walkable_threshold
        self.min_frontier_size = min_frontier_size

        self.directions_4 = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        self.directions_8 = [
            (0, 1),
            (0, -1),
            (1, 0),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]

    def find_frontiers(
        self,
        grid: np.ndarray,
        start_pose: Tuple[float, float],
        origin: Tuple[float, float] = (0.0, 0.0),
    ) -> List[Frontier]:
        """
        Główna metoda BFS znajdująca i grupująca komórki brzegowe.
        Returnuje listę obiektów Frontier.
        """
        rows, cols = grid.shape
        center_r, center_c = rows // 2, cols // 2

        start_c = int(center_c + start_pose[0] / self.resolution)
        start_r = int(center_r + start_pose[1] / self.resolution)

        if not (0 <= start_c < cols and 0 <= start_r < rows):
            logger.warning("Frontier: Start pose out of bounds.")
            return []

        # Wektorowe przygotowanie map (dla szybkiego sprawdzania)
        # Traktujemy wszystko z przedziału (0..walkable_threshold) jako Free,
        # unknown_value jako Unknown. Przeszkody jako > walkable_threshold

        queue = deque([(start_r, start_c)])

        visited = np.zeros_like(grid, dtype=bool)
        is_frontier = np.zeros_like(grid, dtype=bool)

        visited[start_r, start_c] = True

        frontiers = []

        while queue:
            current_r, current_c = queue.popleft()

            for dr, dc in self.directions_4:
                nr, nc = current_r + dr, current_c + dc

                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue

                if visited[nr, nc]:
                    continue

                neighbor_val = grid[nr, nc]

                # Jeśli neighbor jest "Free"
                if (
                    neighbor_val < self.walkable_threshold
                    and neighbor_val != self.unknown_value
                ):
                    visited[nr, nc] = True
                    queue.append((nr, nc))

                # Jeśli neighbor jest kandydatem na boundary (Unknown obok Free)
                elif self._is_new_frontier_cell(grid, nr, nc, is_frontier):
                    is_frontier[nr, nc] = True
                    new_frontier = self._build_new_frontier(
                        grid, nr, nc, is_frontier, rows, cols, center_r, center_c
                    )
                    if new_frontier.size >= self.min_frontier_size:
                        frontiers.append(new_frontier)

        return frontiers

    def _is_new_frontier_cell(
        self, grid: np.ndarray, r: int, c: int, is_frontier: np.ndarray
    ) -> bool:
        """Komórka jest nową granicą jeśli jest Unknown i sąsiaduje z przynajmniej 1 Free."""
        if grid[r, c] != self.unknown_value or is_frontier[r, c]:
            return False

        rows, cols = grid.shape
        for dr, dc in self.directions_4:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                val = grid[nr, nc]
                if val < self.walkable_threshold and val != self.unknown_value:
                    return True
        return False

    def _build_new_frontier(
        self,
        grid: np.ndarray,
        init_r: int,
        init_c: int,
        is_frontier: np.ndarray,
        rows: int,
        cols: int,
        center_r: int,
        center_c: int,
    ) -> Frontier:
        """BFS budujący krawędź granicy."""
        size = 1
        sum_r, sum_c = init_r, init_c

        queue = deque([(init_r, init_c)])

        while queue:
            cr, cc = queue.popleft()

            for dr, dc in self.directions_8:
                nr, nc = cr + dr, cc + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    if self._is_new_frontier_cell(grid, nr, nc, is_frontier):
                        is_frontier[nr, nc] = True
                        size += 1
                        sum_r += nr
                        sum_c += nc
                        queue.append((nr, nc))

        centroid_r = sum_r / size
        centroid_c = sum_c / size

        # Grid -> World m
        centroid_x_m = (centroid_c - center_c) * self.resolution
        centroid_y_m = (centroid_r - center_r) * self.resolution

        return Frontier(size, (centroid_x_m, centroid_y_m))

    def get_best_frontier(
        self, frontiers: List[Frontier], current_pose: Tuple[float, float]
    ) -> Tuple[float, float] | None:
        """Zwraca (x, y) centroidu najlepszej granicy (rozmiar vs. odległość w linii prostej)."""
        if not frontiers:
            return None

        best_frontier = None
        best_score = float("inf")

        # Waga heurystyczna odległości / wielkości krawędzi (im większa tym lepsza = mniejszy koszt)
        distance_weight = 1.0
        size_weight = 2.0

        for f in frontiers:
            dist = np.hypot(
                current_pose[0] - f.centroid_m[0], current_pose[1] - f.centroid_m[1]
            )
            score = (distance_weight * dist) - (size_weight * f.size)

            if score < best_score:
                best_score = score
                best_frontier = f

        if best_frontier:
            # Add some logging
            logger.info(
                f"Selected frontier with size {best_frontier.size} at dist {dist:.2f}m"
            )
            return best_frontier.centroid_m
        return None
