"""Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

PL: Obsługuje autonomiczną nawigację, prowadzenie ścieżki i powrót do domu.
EN: Handles autonomous navigation, path following, and return-to-home behavior.
"""

import logging
import time
from typing import Any, NamedTuple

import numpy as np
from modules.map_utils import calculate_bearing, haversine_distance
from modules.planners.frontier_explorer import FrontierExplorer
from modules.planners.global_planner import GlobalPlanner


class GPSPoint(NamedTuple):
    """PL: Reprezentuje punkt geograficzny jako szerokość i długość.
    EN: Represents a geographic point as latitude and longitude.
    """
    lat: float
    lon: float


class NavigationManager:
    """PL: Zarządza nawigacją GPS, planowaniem trasy i sterowaniem.
    EN: Manages GPS navigation, route planning, and steering control.
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1) -> None:
        """PL: Konfiguruje regulator PID, planowanie trasy i eksplorację.
        EN: Configures the PID controller, route planner, and exploration.
        """
        self.kp: float = kp
        self.ki: float = ki
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
        current_pos: GPSPoint,
        current_heading: float,
        target_pos: GPSPoint,
        dt: float,
    ) -> float:
        """PL: Wyznacza skręt regulatorem PID na podstawie błędu kursu.
        EN: Computes steering with a PID controller from the heading error.
        """
        target_bearing = calculate_bearing(
            current_pos.lat, current_pos.lon, target_pos.lat, target_pos.lon
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
        """PL: Oblicza sterowanie i niski gaz do punktu domu w trybie RTH.
        EN: Computes steering and low throttle toward home in RTH mode.
        """
        if not is_rth_active or not home_position:
            return 0.0, 0.0, False

        distance_to_home = haversine_distance(
            current_lat, current_lon, home_position["lat"], home_position["lon"]
        )
        if distance_to_home < 2.0:  # Próg w metrach / Threshold in meters
            return 0.0, 0.0, True  # Dotarł do domu / Reached home

        steering = self.calculate_steering(
            GPSPoint(current_lat, current_lon),
            current_heading,
            GPSPoint(home_position["lat"], home_position["lon"]),
            dt,
        )
        # Dynamic throttle deceleration when approaching home (under 10 meters)
        if distance_to_home < 10.0:
            throttle = max(0.05, min(0.2, 0.2 * (distance_to_home / 10.0)))
        else:
            throttle = 0.2  # Stały niski gaz dla RTH / Base low throttle for RTH

        return steering, throttle, False

    def plan_global_path(
        self, grid: np.ndarray, start: tuple[float, float], goal: tuple[float, float]
    ) -> bool:
        """PL: Planuje trasę A* i zapisuje ją jako bieżącą ścieżkę.
        EN: Plans an A* route and stores it as the current path.
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
        """PL: Wyszukuje granicę eksploracji i planuje do niej trasę.
        EN: Finds an exploration frontier and plans a route to it.
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
        """PL: Aktualizuje parametry prowadzenia ścieżki z konfiguracji.
        EN: Updates path-following parameters from the configuration.
        """
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
        """PL: Usuwa aktywną ścieżkę i resetuje indeks punktu trasy.
        EN: Clears the active path and resets its waypoint index.
        """
        self.current_path = None
        self.current_waypoint_idx = 0

    def get_next_waypoint(
        self, current_pose: tuple[float, float]
    ) -> tuple[float, float] | None:
        """PL: Zwraca kolejny punkt ścieżki i pomija osiągnięty cel.
        EN: Returns the next path waypoint and advances past a reached target.
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
        """PL: Wybiera punkt prowadzący w odległości zależnej od prędkości.
        EN: Selects a path lookahead point at a speed-dependent distance.
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

    def _calculate_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """PL: Udostępnia obliczanie kursu dla zgodności ze starszym kodem.
        EN: Exposes bearing calculation for compatibility with older code.
        """
        return calculate_bearing(lat1, lon1, lat2, lon2)

    def _haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """PL: Udostępnia odległość Haversine dla zgodności ze starszym kodem.
        EN: Exposes Haversine distance for compatibility with older code.
        """
        return haversine_distance(lat1, lon1, lat2, lon2)
