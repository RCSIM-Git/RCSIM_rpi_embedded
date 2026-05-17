"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Moduł menedżera nawigacji (Navigation Manager).
Navigation Manager Module.

Ten moduł zawiera logikę odpowiedzialną za autonomiczną nawigację pojazdu
na podstawie danych GPS i IMU.
This module contains the logic responsible for autonomous vehicle navigation
based on GPS and IMU data.
"""

import logging
import time
from typing import Any

import numpy as np
from modules.map_utils import calculate_bearing, haversine_distance
from modules.planners.frontier_explorer import FrontierExplorer
from modules.planners.global_planner import GlobalPlanner


class NavigationManager:
    """
    Zarządza autonomiczną nawigacją, w tym logiką RTH, A* i sterowaniem PID.
    Manages autonomous navigation, including RTH logic, A*, and PID control.
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1) -> None:
        """
        Inicjalizuje menedżera nawigacji.
        Initializes the navigation manager.

        Args:
            kp (float): Współczynnik proporcjonalny regulatora PID. / Proportional gain for PID.
            ki (float): Współczynnik całkujący regulatora PID. / Integral gain for PID.
            kd (float): Współczynnik różniczkujący regulatora PID. / Derivative gain for PID.
        """
        self.kd: float = kd
        self.integral: float = 0.0
        self.previous_error: float = 0.0
        self.integral_max: float = 1.0
        self.integral_min: float = -1.0

        self.config = {}

        # Path Follower state
        self.logger = logging.getLogger("NavigationManager")
        self.global_planner = GlobalPlanner(logger=self.logger)
        self.current_path: list | None = None
        self.current_waypoint_idx: int = 0
        self.goal_tolerance: float = 0.2  # meters

        # Adaptive Lookahead params [PLAN-009]
        self.min_lookahead: float = 0.8
        self.lookahead_gain: float = 0.5
        self.max_lookahead: float = 3.0

        # Auto-Explore
        self.frontier_explorer = FrontierExplorer()
        self.auto_explore_active: bool = False
        self.last_auto_explore_time: float = 0.0

    def calculate_steering(
        self,
        current_lat: float,
        current_lon: float,
        current_heading: float,
        target_lat: float,
        target_lon: float,
        dt: float,
    ) -> float:
        """
        Oblicza sterowanie za pomocą regulatora PID w celu osiągnięcia celu.
        Calculates the steering value using a PID controller to reach a target.

        Args:
            current_lat (float): Aktualna szerokość geograficzna. / Current latitude.
            current_lon (float): Aktualna długość geograficzna. / Current longitude.
            current_heading (float): Aktualny kurs w stopniach. / Current heading in degrees.
            target_lat (float): Docelowa szerokość geograficzna. / Target latitude.
            target_lon (float): Docelowa długość geograficzna. / Target longitude.
            dt (float): Czas od ostatniej aktualizacji w sekundach. / Time since last update in seconds.

        Returns:
            float: Wartość sterowania w zakresie [-1.0, 1.0]. / Steering value in the range [-1.0, 1.0].
        """
        target_bearing = calculate_bearing(
            current_lat, current_lon, target_lat, target_lon
        )
        error = target_bearing - current_heading

        # Normalizacja błędu do zakresu [-180, 180]
        # Normalize error to [-180, 180] range
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Człon proporcjonalny / Proportional term
        p_term = self.kp * error

        # Człon całkujący z ograniczeniem (anti-windup)
        # Integral term with anti-windup
        self.integral = np.clip(
            self.integral + error * dt, self.integral_min, self.integral_max
        )
        i_term = self.ki * self.integral

        # Człon różniczkujący / Derivative term
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative

        self.previous_error = error

        steering = p_term + i_term + d_term
        return float(np.clip(steering, -1.0, 1.0))

    def update_rth(
        self,
        is_rth_active: bool,
        home_position: dict[str, float] | None,
        current_lat: float,
        current_lon: float,
        current_heading: float,
        dt: float,
    ) -> tuple[float, float, bool]:
        """
        Aktualizuje logikę RTH (Return To Home).
        Updates the RTH (Return To Home) logic.

        Args:
            is_rth_active (bool): Czy tryb RTH jest aktywny. / Whether RTH mode is active.
            home_position (dict[str, float] | None): Słownik z pozycją 'lat' i 'lon' domu.
                                                        Dictionary with 'lat' and 'lon' of the home position.
            current_lat (float): Aktualna szerokość geograficzna. / Current latitude.
            current_lon (float): Aktualna długość geograficzna. / Current longitude.
            current_heading (float): Aktualny kurs w stopniach. / Current heading in degrees.
            dt (float): Czas od ostatniej aktualizacji w sekundach. / Time since last update in seconds.

        Returns:
            tuple[float, float, bool]: Krotka (sterowanie, gaz, czy dotarł do domu).
                                       Tuple (steering, throttle, has_reached_home).
        """
        if not is_rth_active or not home_position:
            return 0.0, 0.0, False

        distance_to_home = haversine_distance(
            current_lat, current_lon, home_position["lat"], home_position["lon"]
        )
        if distance_to_home < 2.0:  # Próg w metrach / Threshold in meters
            return 0.0, 0.0, True  # Dotarł do domu / Reached home

        steering = self.calculate_steering(
            current_lat,
            current_lon,
            current_heading,
            home_position["lat"],
            home_position["lon"],
            dt,
        )
        throttle = 0.2  # Stały niski gaz dla RTH / Constant low throttle for RTH

        return steering, throttle, False

    def plan_global_path(
        self, grid: np.ndarray, start: tuple[float, float], goal: tuple[float, float]
    ) -> bool:
        """
        Planuje ścieżkę globalną (A*).
        Plans a global path (A*).
        """
        path = self.global_planner.plan_path(grid, start, goal)
        if path:
            self.current_path = path
            self.current_waypoint_idx = 0
            return True
        return False

    def update_auto_explore(
        self, grid: np.ndarray, current_pose: tuple[float, float]
    ) -> bool:
        """
        Wyszukuje nowe granice za pomocą FrontierExplorer i aktualizuje
        ścieżkę do najlepszego znalezionego punktu.
        """
        current_time = time.time()

        # Ograniczenie częstotliwości planowania eksploracji (co 4s lub utrata ścieżki)
        if not self.current_path or (current_time - self.last_auto_explore_time > 4.0):
            self.last_auto_explore_time = current_time

            frontiers = self.frontier_explorer.find_frontiers(grid, current_pose)
            best_target = self.frontier_explorer.get_best_frontier(
                frontiers, current_pose
            )

            if best_target:
                # Zaplanuj drogę do najbliższej granicy
                success = self.plan_global_path(grid, current_pose, best_target)
                return success
            return False

        return True

    def update_config(self, config: dict[str, Any]) -> None:
        """Aktualizuje parametry nawigacji z pliku konfiguracyjnego."""
        if not config:
            return
        self.config = config
        nav_cfg = config.get("autonomous_navigation", {})
        pp_cfg = nav_cfg.get("pure_pursuit", {})

        # [PLAN-009] Sync adaptive lookahead params
        self.min_lookahead = pp_cfg.get("lookahead_distance_m", self.min_lookahead)
        self.lookahead_gain = pp_cfg.get("lookahead_gain", self.lookahead_gain)
        self.max_lookahead = pp_cfg.get("max_lookahead_m", 3.0)

    def clear_path(self):
        """
        Czyści aktualną ścieżkę.
        Clears the current path.
        """
        self.current_path = None
        self.current_waypoint_idx = 0

    def get_next_waypoint(
        self, current_pose: tuple[float, float]
    ) -> tuple[float, float] | None:
        """
        Zwraca następny punkt nawigacyjny ("carrot") z aktualnej ścieżki.
        Returns the next waypoint ("carrot") from the current path.
        """
        if not self.current_path or self.current_waypoint_idx >= len(self.current_path):
            return None

        # Check distance to current target waypoint
        target = self.current_path[self.current_waypoint_idx]
        dist = np.hypot(target[0] - current_pose[0], target[1] - current_pose[1])

        if dist < self.goal_tolerance:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.current_path):
                return None  # Path finished
            return self.current_path[self.current_waypoint_idx]

        return target

    def get_lookahead_point(
        self, current_pose: tuple[float, float], current_speed: float
    ) -> tuple[float, float, float] | None:
        """
        Oblicza dynamiczny punkt lookahead ("carrot") na ścieżce. [PLAN-002]
        Calculates dynamic lookahead point on path.

        Returns:
            tuple (x, y, L) lub None jeśli brak ścieżki.
        """
        if not self.current_path or len(self.current_path) < 2:
            return None

        # 1. Wylicz odległość lookahead: L = L_min + speed * gain
        # [PLAN-009] Wzór wspierający bazowy dystans minimalny
        L = self.min_lookahead + (current_speed * self.lookahead_gain)
        L = np.clip(L, self.min_lookahead, self.max_lookahead)

        # 2. Znajdź punkt na ścieżce oddalony o L od robota
        # Przeszukujemy od aktualnego waypointa w górę
        best_point = self.current_path[self.current_waypoint_idx]

        for i in range(self.current_waypoint_idx, len(self.current_path) - 1):
            _ = self.current_path[i]  # p1 unused, replaced with _
            p2 = self.current_path[i + 1]

            # Dystans do p2
            dist_to_p2 = np.hypot(p2[0] - current_pose[0], p2[1] - current_pose[1])

            if dist_to_p2 >= L:
                # Punkt lookahead znajduje się na tym segmencie (interpolacja)
                # Dla uproszczenia bierzemy p2 jeśli jest blisko L
                best_point = p2
                break
            else:
                # p2 jest za blisko, sprawdzamy następny segment
                best_point = p2
                # Aktualizujemy indeks waypointa, aby nie wracać do punktów z tyłu
                self.current_waypoint_idx = i

        return (best_point[0], best_point[1], L)
