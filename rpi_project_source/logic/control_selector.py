"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Moduł selektora sterowania (Control Selector) dla RPi.
Control Selector Module for RPi.

Ten moduł jest odpowiedzialny za arbitraż pomiędzy różnymi źródłami sterowania pojazdem
na Raspberry Pi (Manual, AI, Autopilot, Failsafe).
This module is responsible for arbitration between different vehicle control sources
on the Raspberry Pi (Manual, AI, Autopilot, Failsafe).
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from logic.navigation_manager import NavigationManager
    from modules.ai_manager import AIManager

# Stałe trybów sterowania (powinny być przeniesione do wspólnych stałych)
# Control mode constants (should be moved to shared constants)
CONTROL_MODE_MANUAL = "MANUAL"
CONTROL_MODE_USER = "USER_MODE"
CONTROL_MODE_AI_STEER_ONLY = "AI_STEER_ONLY"
CONTROL_MODE_FULL_AUTOPILOT = "FULL_AUTOPILOT"
CONTROL_MODE_AUTONOMOUS = "AUTONOMOUS"
CONTROL_MODE_RTH = "RTH"
CONTROL_MODE_FAILSAFE = "FAILSAFE"


class ControlSelector:
    """
    Zarządza arbitrażem sterowania, wybierając odpowiednie źródło.
    Manages control arbitration by selecting the appropriate source.
    """

    def __init__(
        self,
        nav_manager: "NavigationManager",
        ai_manager: "AIManager" | None = None,
    ) -> None:
        """
        Inicjalizuje selektor sterowania.
        Initializes the control selector.
        """
        self.nav_manager = nav_manager
        self.ai_manager = ai_manager
        self.last_manual_steering: float = 0.0
        self.last_manual_throttle: float = 0.0

        # Ograniczanie częstości logowania / Log rate limiting
        self._last_mode: str | None = None
        self._last_failsafe_log: float = 0.0
        self._failsafe_interval: float = 5.0

        # Ograniczenia bezpieczeństwa / Safety constraints
        self.max_autopilot_speed_kmh: float = (
            30.0  # Max prędkość dla FULL_AUTOPILOT / Max speed for FULL_AUTOPILOT
        )

        # Śledzenie błędów AI / AI failure tracking
        self._consecutive_ai_failures: int = 0
        self._max_ai_failures: int = (
            10  # Wyłącz AI po tylu błędach z rzędu / Disable AI after this many consecutive failures
        )
        self._ai_disabled: bool = False

    def reactive_override(
        self, planner_cmd: tuple[float, float], safety_score: float
    ) -> tuple[float, float]:
        """
        Modyfikuje komendy planera w oparciu o ocenę bezpieczeństwa (Reactive Control).
        Modifies planner commands based on safety score (Reactive Control).

        Args:
            planner_cmd: (steering, throttle) proponowane przez planer.
            safety_score: Ocena bezpieczeństwa 0.0 (kolizja) - 1.0 (bezpiecznie).

        Returns:
            tuple[float, float]: Zmodyfikowane sterowanie.
        """
        steering, throttle = planner_cmd

        # 1. Emergency Stop
        if safety_score < 0.2:
            logging.warning(
                f"Reactive Override: Emergency Stop! Safety Score: {safety_score:.2f}"
            )
            return steering, 0.0

        # 2. Speed Limiting
        if safety_score < 0.7:
            limited_throttle = throttle * (safety_score / 0.8)  # Reduce throttle
            # logging.debug(f"Reactive Override: Limiting throttle {throttle:.2f} -> {limited_throttle:.2f}")
            return steering, limited_throttle

        return steering, throttle

    def _apply_acc(
        self, lidar_scan: list[tuple[float, float]] | None, throttle_input: float
    ) -> float:
        """
        Implementuje logikę Adaptacyjnego Tempomatu (ACC).
        Implements Adaptive Cruise Control (ACC) logic.
        """
        if lidar_scan is None or throttle_input <= 0:
            return 1.0

        relevant_points = [
            distance for angle, distance in lidar_scan if -10 <= angle <= 10
        ]

        if not relevant_points:
            return 1.0

        min_distance_cm = min(relevant_points)
        min_distance_m = min_distance_cm / 100.0

        if min_distance_m > 2.0:
            return 1.0
        elif 0.5 <= min_distance_m <= 2.0:
            multiplier = (min_distance_m - 0.5) / 1.5
            return multiplier
        else:
            logging.warning(
                f"ACC: Zbyt blisko! Dystans {min_distance_m:.2f}m. Zatrzymanie."
            )
            return 0.0

    def _get_current_speed(self, frame_data: dict[str, Any]) -> float:
        """
        Pobiera aktualną prędkość pojazdu z danych GPS.
        Gets the current vehicle speed from GPS data.
        """
        gps_data = frame_data.get("gps")  # Changed from gps_data
        if gps_data and "speed" in gps_data:
            return float(gps_data.get("speed", 0.0))
        return 0.0

    def get_reliable_pose(
        self, frame_data: dict[str, Any]
    ) -> tuple[float, float, float]:
        """
        Wybiera najbardziej wiarygodne źródło pozycji (GPS RTK vs SLAM).
        Selects the most reliable pose source.
        """
        gps = frame_data.get("gps")
        slam_pose = frame_data.get("pose", (0.0, 0.0, 0.0))

        # RTK Fix (4) is very accurate. RTK Float (5) is usually < 1m.
        if gps and gps.get("fix_quality", 0) in [4, 5]:
            # Convert Lat/Lon to a local metric grid if needed for RTH
            # For now, we assume RTH logic handles Lat/Lon
            return gps["lat"], gps["lon"], gps.get("course", 0.0)
        else:
            # Fallback to SLAM Pose
            # logging.debug("GPS loss or poor quality -> using SLAM Pose")
            return slam_pose

    def process_frame(
        self, mode: str, frame_data: dict[str, Any]
    ) -> tuple[float, float]:
        """
        Główna metoda decyzyjna wybierająca sterowanie na podstawie trybu i danych.
        The main decision-making method selecting control based on mode and data.
        """
        lidar_scan = frame_data.get("lidar_scan")
        frame_data.get("gps_data")
        imu_data = frame_data.get("imu_data")
        manual_controls = frame_data.get(
            "manual_controls", {"steering": 0.0, "throttle": 0.0}
        )
        home_position = frame_data.get("home_position")
        dt = frame_data.get("dt", 1 / 30.0)
        frame_data.get("image")

        # New: Planner Command from frame_data (inserted by Supervisor)
        planner_cmd = frame_data.get(
            "planner_cmd"
        )  # (steering, throttle, safety_score)

        self._get_current_speed(frame_data)
        self.last_manual_steering = manual_controls["steering"]
        self.last_manual_throttle = manual_controls["throttle"]

        if mode == CONTROL_MODE_USER:
            mode = CONTROL_MODE_MANUAL

        if mode == CONTROL_MODE_FAILSAFE:
            current_time = time.time()
            if mode != self._last_mode or (
                current_time - self._last_failsafe_log > self._failsafe_interval
            ):
                logging.critical("Tryb FAILSAFE aktywny. Zatrzymywanie pojazdu.")
                self._last_failsafe_log = current_time
            self._last_mode = mode
            return 0.0, 0.0

        self._last_mode = mode
        steering, throttle = 0.0, 0.0

        # AUTONOMOUS MODE (Planner)
        if mode == CONTROL_MODE_AUTONOMOUS:
            if planner_cmd:
                p_steer, p_thrott, p_safety = planner_cmd
                steering, throttle = self.reactive_override(
                    (p_steer, p_thrott), p_safety
                )
            else:
                logging.warning("AUTONOMOUS mode but no planner command. Stop.")
                steering, throttle = 0.0, 0.0

        # AI MODES (Legacy or specific)
        elif mode in (CONTROL_MODE_AI_STEER_ONLY, CONTROL_MODE_FULL_AUTOPILOT):
            # If we have a planner command, use it (it effectively supersedes legacy AI modes in this architecture)
            if planner_cmd:
                p_steer, p_thrott, p_safety = planner_cmd
                steering, throttle = self.reactive_override(
                    (p_steer, p_thrott), p_safety
                )
                if mode == CONTROL_MODE_AI_STEER_ONLY:
                    throttle = self.last_manual_throttle
            else:
                # Fallback if no planner command (should not happen if main_service is wired correctly)
                logging.warning(f"{mode}: No planner command available. Stopping.")
                steering, throttle = 0.0, 0.0

        elif mode == CONTROL_MODE_MANUAL:
            steering = self.last_manual_steering
            throttle = self.last_manual_throttle

        elif mode == CONTROL_MODE_RTH:
            gps = frame_data.get("gps")
            if gps and gps.get("fix_quality", 0) in [4, 5] and home_position:
                steering, _, arrived = self.nav_manager.update_rth(
                    True,
                    home_position,
                    gps["lat"],
                    gps["lon"],
                    imu_data["heading"],
                    dt,
                )
                throttle = 0.2
                if arrived:
                    steering, throttle = 0.0, 0.0
            elif home_position:
                # Fallback RTH using SLAM Pose (simplified)
                # Note: This assumes home_position and slam_pose are in same local frame.
                frame_data.get("pose", (0.0, 0.0, 0.0))
                # For simplicity, we just stop if no stable GPS for now,
                # or we could implement local RTH.
                logging.warning("RTH: GPS lost. Holding position.")
                steering, throttle = 0.0, 0.0
            else:
                steering, throttle = 0.0, 0.0

        # Apply ACC (Adaptive Cruise Control) only in autonomous modes
        # In MANUAL mode, we allow the driver to override everything for safety/testing
        if mode != CONTROL_MODE_MANUAL:
            acc_multiplier = self._apply_acc(lidar_scan, throttle)
            if acc_multiplier < 1.0 and throttle > 0.1:
                logging.info(f"ACC: Limiting throttle ({throttle:.2f} -> {throttle*acc_multiplier:.2f}) due to obstacle.")
            throttle *= acc_multiplier

        return steering, throttle

    def reset_ai_failure_counter(self) -> None:
        self._consecutive_ai_failures = 0
        self._ai_disabled = False
