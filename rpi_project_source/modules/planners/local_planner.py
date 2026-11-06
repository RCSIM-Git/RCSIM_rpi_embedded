# -*- coding: utf-8 -*-
"""PL: Udostępnia fasadę lokalnego planowania i fuzji danych o przeszkodach.
EN: Provides a local planning facade and obstacle data fusion.
"""

import logging
import threading
from typing import Any

import numpy as np

from .costmap_manager import CostmapManager
from .global_planner import GlobalPlanner as AStarPlanner
from .pure_pursuit_planner import PurePursuitPlanner
from .reactive_planner import ReactivePlanner

logger: logging.Logger = logging.getLogger(__name__)


class LocalPlanner:
    """PL: Łączy mapę kosztów i algorytmy planowania lokalnego.
    EN: Combines the cost map and local path planning algorithms.
    """

    def __init__(self, config: dict[str, Any] = None) -> None:
        """PL: Tworzy menedżer mapy i podplanery dla przekazanej konfiguracji.
        EN: Creates the map manager and subplanners for the supplied configuration.
        """
        self.config: dict[str, Any] = config or {}

        # Inicjalizacja składowych menedżera
        self.cm = CostmapManager(config=self.config)
        self.pure_pursuit = PurePursuitPlanner(
            costmap_manager=self.cm, config=self.config
        )
        self.reactive = ReactivePlanner(costmap_manager=self.cm, config=self.config)
        self.astar = AStarPlanner(logger=logger)

        self.last_path: list[tuple[float, float]] = []
        self._lock = threading.Lock()

        # Pierwsza konfiguracja
        self.update_config(config)

    @property
    def costmap(self) -> np.ndarray:
        """PL: Zwraca macierz mapy kosztów.
        EN: Returns the cost map matrix.
        """
        return self.cm.costmap

    @property
    def costmap_manager(self) -> CostmapManager:
        """PL: Zwraca menedżer mapy kosztów.
        EN: Returns the cost map manager.
        """
        return self.cm

    def update_occupancy(
        self,
        pose: tuple[float, float, float],
        lidar_points: list[tuple[float, float]],
        imu_orientation: dict[str, float] = None,  # [PLAN-010]
    ) -> None:
        """PL: Aktualizuje mapę kosztów danymi LiDAR i nachyleniem IMU.
        EN: Updates the cost map with LiDAR and IMU slope data.
        """
        # [PLAN-004] Guard against None
        if lidar_points is None:
            lidar_points = []

        with self._lock:
            self.cm.update_occupancy(pose, lidar_points, imu_orientation)

        # Log limited to avoid flooding
        if getattr(self, "loop_count", 0) % 20 == 0:
            logger.info("LocalPlanner: Occupancy updated (fused LiDAR + Terrain)")
        self.loop_count = getattr(self, "loop_count", 0) + 1

    def update_occupancy_from_lidar_and_yolo(
        self,
        pose: tuple[float, float, float],
        lidar_points: list[tuple[float, float]],
        detections: list[dict[str, Any]] = None,
        camera_extrinsics: dict[str, Any] = None,
        imu_orientation: dict[str, float] = None,
    ) -> None:
        """PL: Aktualizuje mapę z wejść LiDAR, detekcji i orientacji IMU.
        EN: Updates the map from LiDAR, detections, and IMU orientation inputs.
        """
        self.update_occupancy(pose, lidar_points, imu_orientation)

    def update_map(
        self, pose: tuple[float, float, float], lidar_points: list[tuple[float, float]]
    ) -> None:
        """PL: Zachowuje zgodny alias aktualizacji mapy LiDAR.
        EN: Preserves a compatible alias for LiDAR map updates.
        """
        self.update_occupancy(pose, lidar_points)

    def clear_map(self) -> None:
        """PL: Czyści mapę kosztów pod blokadą.
        EN: Clears the cost map while holding the lock.
        """
        with self._lock:
            self.cm.clear_map()

    def plan_reactive(
        self,
        robot_pose: tuple[float, float, float],
        goal_pose: tuple[float, float] = None,
    ) -> tuple[float, float, float]:
        """PL: Przekazuje bieżącą pozycję i cel do planera reaktywnego.
        EN: Forwards the current pose and goal to the reactive planner.
        """
        with self._lock:
            if not self.reactive:
                return 0.0, 0.0, 0.0
            return self.reactive.plan_reactive(robot_pose, goal_pose)

    def plan_pure_pursuit(
        self,
        robot_pose: tuple[float, float, float],
        lookahead_point: tuple[float, float],
        current_speed: float = 0.0,
        is_reversed: bool = False,
    ) -> tuple[float, float, float]:
        """PL: Wyznacza sterowanie metodą Pure Pursuit z unikaniem przeszkód.
        EN: Computes Pure Pursuit controls with obstacle avoidance.
        """
        with self._lock:
            if not self.pure_pursuit:
                return 0.0, 0.0, 0.0
            return self.pure_pursuit.plan_pure_pursuit(
                robot_pose, lookahead_point, current_speed, is_reversed
            )

    def plan_path(
        self, start_pose: tuple[float, float], goal_global: tuple[float, float]
    ) -> list[tuple[float, float]] | None:
        """PL: Planuje trasę A* i zapisuje ją jako ostatnią ścieżkę.
        EN: Plans an A* route and stores it as the latest path.
        """
        with self._lock:
            if not self.astar:
                return None

            # [PLAN-010] Pass terrain costmap for traversability analysis
            self.astar.terrain_map = self.cm.costmap

            # [PLAN-004] Ensure returning floats (meters)
            # grid argument is binary slam map, we use costmap for costs
            path = self.astar.plan_path(self.cm.costmap, start_pose, goal_global)
            self.last_path = path or []
            return path

    def update_config(self, config: dict[str, Any]) -> None:
        """PL: Przekazuje konfigurację do mapy i podplanerów.
        EN: Propagates configuration to the map manager and subplanners.
        """
        if config is None:
            return

        with self._lock:
            self.config = config
            self.cm.config = config

            # Dynamiczna re-konfiguracja parametrów siatki
            nav_config = config.get("autonomous_navigation", {})
            grid_cfg = nav_config.get("grid", {})
            inf_cells = grid_cfg.get("inflation_radius_cells", 4)
            new_res = grid_cfg.get("resolution_m", 0.05)

            self.cm.resolution = new_res
            self.cm.inflation_radius_cm = inf_cells * new_res * 100.0

            # Przesłanie do pod-planerów
            if self.pure_pursuit:
                self.pure_pursuit.update_config(config)
            if self.reactive:
                self.reactive.update_config(config)

        logger.info("LocalPlanner (Facade) config updated successfully.")

    def is_healthy(self) -> bool:
        """PL: Sprawdza dostępność wszystkich składowych planera.
        EN: Checks whether all planner components are available.
        """
        return all([self.cm, self.pure_pursuit, self.reactive, self.astar])
