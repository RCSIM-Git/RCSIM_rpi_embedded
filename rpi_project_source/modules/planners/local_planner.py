# -*- coding: utf-8 -*-
"""
Local Planner - Fasada Planowania Lokalnego (RPi).

Ten moduł zarządza fuzją danych z sensorów (LiDAR) oraz udostępnia
interfejsy dla różnych algorytmów sterowania i omijania przeszkód.
Użytkuje CostmapManager do zarządzania mapą zajętości.
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
    """
    Główny punkt dostępowy dla planowania toru jazdy na Raspberry Pi.
    Wzorzec: Fasada (Facade).
    """

    def __init__(self, config: dict[str, Any] = None) -> None:
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
        """Udostępnia surową macierz mapy zajętości."""
        return self.cm.costmap

    @property
    def costmap_manager(self) -> CostmapManager:
        """Dostęp do menedżera mapy."""
        return self.cm

    def update_occupancy(
        self,
        pose: tuple[float, float, float],
        lidar_points: list[tuple[float, float]],
        imu_orientation: dict[str, float] = None,  # [PLAN-010]
    ) -> None:
        """Zaktualizowanie przeszkód z sensorów (LiDAR, Terrain)."""
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
        """
        Zaktualizowanie mapy zajętości z fuzją wielu źródeł.
        Dla modeli E2E, detekcje są ignorowane.
        """
        self.update_occupancy(pose, lidar_points, imu_orientation)

    def update_map(
        self, pose: tuple[float, float, float], lidar_points: list[tuple[float, float]]
    ) -> None:
        """Alias kompatybilności wstecznej dla update_occupancy"""
        self.update_occupancy(pose, lidar_points)

    def clear_map(self) -> None:
        """Czyści globalną siatkę kosztów."""
        with self._lock:
            self.cm.clear_map()

    def plan_reactive(
        self,
        robot_pose: tuple[float, float, float],
        goal_pose: tuple[float, float] = None,
    ) -> tuple[float, float, float]:
        """Przekazuje rozkaz do silnika DWA / Reactive."""
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
        """Praca w trybie Pure Pursuit z asystentem sił odbijających."""
        with self._lock:
            if not self.pure_pursuit:
                return 0.0, 0.0, 0.0
            return self.pure_pursuit.plan_pure_pursuit(
                robot_pose, lookahead_point, current_speed, is_reversed
            )

    def plan_path(
        self, start_pose: tuple[float, float], goal_global: tuple[float, float]
    ) -> list[tuple[float, float]] | None:
        """Pobierz drogę optymalną w grafie (Algorytm A*)."""
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
        """Przesyła re-konfigurację kaskadowo w dół do agentów sterujących (Hot Reloading)."""
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
        """Sprawdza czy wszystkie komponenty planera są gotowe."""
        return all([self.cm, self.pure_pursuit, self.reactive, self.astar])
