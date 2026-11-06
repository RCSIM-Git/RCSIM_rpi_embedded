"""Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

PL: Wybiera źródło sterowania pojazdem dla trybów manualnych i autonomicznych.
EN: Selects the vehicle control source for manual and autonomous modes.
"""

from __future__ import annotations

import logging
import math
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
    """PL: Arbitruje polecenia sterujące i stosuje ograniczenia bezpieczeństwa.
    EN: Arbitrates control commands and applies safety constraints.
    """

    def __init__(
        self,
        nav_manager: "NavigationManager",
        ai_manager: "AIManager" | None = None,
        config: dict[str, Any] | None = None,
    ) -> None:
        """PL: Ustawia menedżery nawigacji i AI oraz stan selektora.
        EN: Sets the navigation and AI managers and selector state.
        """
        self.nav_manager = nav_manager
        self.ai_manager = ai_manager
        self.config = config
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
        """PL: Redukuje gaz lub zatrzymuje pojazd według oceny bezpieczeństwa.
        EN: Reduces throttle or stops the vehicle based on the safety score.
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
        """PL: Zwraca mnożnik gazu wynikający z odległości przeszkód z LiDAR.
        EN: Returns a throttle multiplier based on LiDAR obstacle distance.
        """
        acc_enabled = True
        acc_min_dist = 0.5
        acc_max_dist = 2.0

        if self.config is not None:
            safety_cfg = self.config.get("safety", {})
            acc_enabled = safety_cfg.get("acc_enabled", True)
            acc_min_dist = safety_cfg.get("acc_min_dist_m", 0.5)
            acc_max_dist = safety_cfg.get("acc_max_dist_m", 2.0)

        if not acc_enabled:
            return 1.0

        if acc_max_dist <= acc_min_dist:
            logging.error(
                f"ACC: Niepoprawna konfiguracja zakresów! max ({acc_max_dist}m) <= min ({acc_min_dist}m). Awaryjne zatrzymanie."
            )
            return 0.0

        if lidar_scan is None or throttle_input <= 0:
            return 1.0

        relevant_points = []
        for angle, distance in lidar_scan:
            angle_norm = angle % 360.0
            if angle_norm <= 10 or angle_norm >= 350:
                relevant_points.append(distance)

        if not relevant_points:
            return 1.0

        min_distance_cm = min(relevant_points)
        min_distance_m = min_distance_cm / 100.0

        if min_distance_m > acc_max_dist:
            return 1.0
        elif acc_min_dist <= min_distance_m <= acc_max_dist:
            denom = acc_max_dist - acc_min_dist
            if denom <= 0.001:
                return 0.0
            multiplier = (min_distance_m - acc_min_dist) / denom
            return multiplier
        else:
            logging.warning(
                f"ACC: Zbyt blisko! Dystans {min_distance_m:.2f}m. Zatrzymanie."
            )
            return 0.0

    def _get_current_speed(self, frame_data: dict[str, Any]) -> float:
        """PL: Pobiera prędkość GPS z ramki danych lub zwraca zero.
        EN: Reads GPS speed from frame data or returns zero.
        """
        gps_data = frame_data.get("gps")  # Changed from gps_data
        if gps_data and "speed" in gps_data:
            return float(gps_data.get("speed", 0.0))
        return 0.0

    def get_reliable_pose(
        self, frame_data: dict[str, Any]
    ) -> tuple[float, float, float]:
        """PL: Wybiera pozycję GPS RTK albo zapasową pozycję SLAM.
        EN: Selects the GPS RTK position or the fallback SLAM pose.
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
        """PL: Wyznacza sterowanie dla wybranego trybu na podstawie ramki danych.
        EN: Computes controls for the selected mode from the input frame.
        """
        lidar_scan_raw = frame_data.get("lidar")
        is_mm = True
        if lidar_scan_raw is None:
            lidar_scan_raw = frame_data.get("lidar_scan")
            is_mm = False

        if lidar_scan_raw:
            if is_mm:
                lidar_scan = [(angle, dist / 10.0) for angle, dist in lidar_scan_raw]
            else:
                lidar_scan = lidar_scan_raw
        else:
            lidar_scan = None
        gps_data = frame_data.get("gps_data") or frame_data.get("gps")
        imu_data = frame_data.get("imu") or frame_data.get("imu_data") or {}
        manual_controls = frame_data.get(
            "manual_controls", {"steering": 0.0, "throttle": 0.0}
        )
        home_position = frame_data.get("home_position")
        dt = frame_data.get("dt", 1 / 30.0)
        image = frame_data.get("image")

        # New: Planner Command from frame_data (inserted by Supervisor)
        planner_cmd = frame_data.get(
            "planner_cmd"
        )  # (steering, throttle, safety_score)

        current_speed = self._get_current_speed(frame_data)
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
        elif mode in (CONTROL_MODE_AI_STEER_ONLY, CONTROL_MODE_FULL_AUTOPILOT, "AI_AUTOPILOT"):
            if self.ai_manager and not self._ai_disabled:
                if image is not None:
                    try:
                        if "ai_controls" in frame_data:
                            ai_controls = frame_data["ai_controls"]
                        else:
                            _, ai_controls = self.ai_manager.predict(
                                image, sensor_data=frame_data
                            )
                        if not isinstance(ai_controls, dict):
                            raise ValueError("AI did not provide controls")
                        steering = float(ai_controls["steering"])
                        throttle = float(ai_controls["throttle"])
                        if not math.isfinite(steering) or not math.isfinite(throttle):
                            raise ValueError("AI provided non-finite controls")
                        if mode == CONTROL_MODE_AI_STEER_ONLY:
                            throttle = self.last_manual_throttle
                        if planner_cmd:
                            _, _, safety_score = planner_cmd
                            steering, throttle = self.reactive_override(
                                (steering, throttle), safety_score
                            )
                        self._consecutive_ai_failures = 0
                    except Exception as e:
                        logging.error(f"AI prediction error: {e}")
                        self._consecutive_ai_failures += 1
                        if self._consecutive_ai_failures >= self._max_ai_failures:
                            self._ai_disabled = True
                            logging.critical("AI disabled due to too many consecutive failures.")
                        steering, throttle = 0.0, 0.0
                else:
                    logging.warning("AI mode but no image. Stop.")
                    steering, throttle = 0.0, 0.0
            elif planner_cmd:
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
            gps = gps_data
            if gps and gps.get("fix_quality", 0) in [4, 5] and home_position:
                steering, raw_throttle, arrived = self.nav_manager.update_rth(
                    True,
                    home_position,
                    gps["lat"],
                    gps["lon"],
                    imu_data.get("heading", 0.0) if isinstance(imu_data, dict) else 0.0,
                    dt,
                )
                throttle = raw_throttle
                if arrived:
                    steering, throttle = 0.0, 0.0
                elif planner_cmd:
                    # Pass RTH steering/throttle through reactive planner safety evaluation
                    _, _, p_safety = planner_cmd
                    steering, throttle = self.reactive_override(
                        (steering, throttle), p_safety
                    )
            elif home_position:
                # Fallback RTH using SLAM Pose (simplified)
                slam_pose = frame_data.get("pose", (0.0, 0.0, 0.0))
                logging.warning("RTH: GPS lost. Holding position.")
                steering, throttle = 0.0, 0.0
            else:
                steering, throttle = 0.0, 0.0

        # Apply ACC (Adaptive Cruise Control) across modes for safety/testing
        acc_multiplier = self._apply_acc(lidar_scan, throttle)
        if acc_multiplier < 1.0 and throttle > 0.1:
            logging.info(f"ACC: Limiting throttle ({throttle:.2f} -> {throttle*acc_multiplier:.2f}) due to obstacle.")
        throttle *= acc_multiplier

        return steering, throttle

    def reset_ai_failure_counter(self) -> None:
        """PL: Zeruje licznik błędów AI i ponownie włącza predykcje.
        EN: Resets the AI failure counter and re-enables predictions.
        """
        self._consecutive_ai_failures = 0
        self._ai_disabled = False
