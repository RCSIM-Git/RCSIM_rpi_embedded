"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Safety Supervisor Module for RCSIM.
Implements a Finite State Machine (FSM) to handle safety-critical situations.
"""

import logging
import math
import time
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


class SafetyState(Enum):
    NORMAL = "NORMAL"
    AVOID = "AVOID"
    STOP = "STOP"
    RTH = "RTH"
    CRITICAL = "CRITICAL"
    STORAGE_LOW = "STORAGE_LOW"


class SafetySupervisor:
    """
    Manages vehicle safety by monitoring sensors and overriding controls.
    """

    def __init__(self, config: dict[str, Any] | None = None):
        self.config = config or {}
        self.state = SafetyState.NORMAL

        # Thresholds
        safety_cfg = self.config.get("safety", {})
        self.stop_dist_m = safety_cfg.get("emergency_stop_dist_m", 0.3)
        self.avoid_dist_m = safety_cfg.get("avoid_dist_m", 0.8)
        self.impact_g_threshold = safety_cfg.get("impact_g_threshold", 5.0)
        self.critical_battery_v = safety_cfg.get(
            "critical_battery_v", 6.0
        )  # for 2S Lipo

        # [NEW] [SAFETY-007] Dynamic Speed Scaling Factors
        self.thermal_throttle_mult = 1.0
        self.inference_throttle_mult = 1.0
        
        self.reason = "Normal operation"
        self.impact_detected = False
        self.faults = {
            "force_stop": False,
            "force_impact": False,
            "force_rth": False
        }
        self.last_state_change = time.time()
        self.lidar_sector_deg = safety_cfg.get("lidar_sector_deg", 60.0)

    def update(self, sensor_data: dict[str, Any], ai_ready: bool = True) -> SafetyState:
        """
        Updates the safety state based on current sensor data.

        Returns:
            SafetyState: The updated safety state.
        """
        new_state = SafetyState.NORMAL
        self.thermal_throttle_mult = 1.0
        self.inference_throttle_mult = 1.0

        # 0. Check Fault Injection
        if self.faults["force_stop"]:
            new_state = SafetyState.STOP
            self.reason = "FAULT_INJECTION: FORCE_STOP"
        elif self.faults["force_impact"] or self.impact_detected:
            new_state = SafetyState.STOP
            self.reason = "Impact detected"
        elif self.faults["force_rth"]:
            new_state = SafetyState.RTH
            self.reason = "FAULT_INJECTION: FORCE_RTH"

        # 1. Check for Impacts (IMU)
        imu = sensor_data.get("imu")
        if imu and new_state != SafetyState.STOP:
            ax, ay, az = imu.get("ax", 0.0), imu.get("ay", 0.0), imu.get("az", 0.0)
            magnitude_m_s2 = math.sqrt(ax**2 + ay**2 + az**2)
            magnitude_g = magnitude_m_s2 / 9.81
            impact_g_force = abs(magnitude_g - 1.0)

            if impact_g_force > self.impact_g_threshold:
                if not self.impact_detected:
                    logger.critical(
                        f"IMPACT DETECTED! G-force: {impact_g_force:.2f}g (threshold: {self.impact_g_threshold}g)"
                    )
                    self.impact_detected = True
                new_state = SafetyState.STOP
                self.reason = f"Impact detected: {impact_g_force:.2f}g"

        # 2. Check for Obstacles (LiDAR)
        lidar_points = sensor_data.get("lidar")
        if lidar_points and new_state != SafetyState.STOP:
            min_dist_m = self._get_min_forward_dist(lidar_points)

            if min_dist_m < self.stop_dist_m:
                new_state = SafetyState.STOP
                self.reason = f"Obstacle too close! {min_dist_m:.2f}m"
            elif min_dist_m < self.avoid_dist_m:
                new_state = SafetyState.AVOID
                self.reason = f"Obstacle nearby: {min_dist_m:.2f}m"

        # 3. Check Battery
        battery = sensor_data.get("battery")
        if battery and new_state not in [
            SafetyState.STOP,
            SafetyState.CRITICAL,
            SafetyState.RTH,
        ]:
            voltage = battery.get("voltage", 6.5)
            if voltage < self.critical_battery_v:
                new_state = SafetyState.RTH
                self.reason = f"Low battery: {voltage:.2f}V"

        # 4. Check System Critical & [SAFETY-007] Thermal Throttling
        system = sensor_data.get("system")
        if system and new_state != SafetyState.CRITICAL:
            temp = system.get("cpu_temp", 0.0)
            if temp > 85.0:
                new_state = SafetyState.CRITICAL
                self.reason = f"CRITICAL: CPU Temp {temp:.1f}C"
            elif temp > 75.0:
                # Linear throttle reduction from 100% at 75C down to 20% at 85C
                self.thermal_throttle_mult = max(0.2, 1.0 - (temp - 75.0) / 10.0)
                if self.thermal_throttle_mult < 0.9:
                    self.reason = f"Thermal Throttling: {temp:.1f}C (Max Speed {self.thermal_throttle_mult*100:.0f}%)"

        # 5. Check AI Performance & [SAFETY-007] FPS Throttling
        ai_status = sensor_data.get("ai_status")
        if ai_status and new_state not in [SafetyState.STOP, SafetyState.CRITICAL]:
            fps = ai_status.get("fps", 30.0)
            if fps < 10.0:
                new_state = SafetyState.AVOID
                self.reason = f"AI Lag: {fps:.1f} FPS (Too slow for safe flight)"
            elif fps < 18.0:
                # Reduce speed if AI is stuttering
                self.inference_throttle_mult = max(0.3, (fps - 10.0) / 8.0)
                if self.inference_throttle_mult < 0.9:
                    self.reason = f"AI Load Balancing: {fps:.1f} FPS (Max Speed {self.inference_throttle_mult*100:.0f}%)"

        # 6. Check Link Status (AND Gate Failsafe)
        link_status = sensor_data.get("link_status")
        if link_status:
            webrtc_dead = link_status.get("webrtc_dead", False)
            elrs_dead = link_status.get("elrs_dead", False)
            
            if webrtc_dead and elrs_dead:
                # Both links lost -> STOP (or RTH if AI ready)
                if new_state not in [SafetyState.CRITICAL, SafetyState.STOP]:
                    if ai_ready:
                        new_state = SafetyState.RTH
                        self.reason = "CRITICAL FAILSAFE: All RF/IP links lost! Triggering AI RTH."
                    else:
                        new_state = SafetyState.STOP
                        self.reason = "CRITICAL FAILSAFE: All links lost & AI Hat missing. ABSOLUTE STOP."
            else:
                # At least one link is active -> Can recover from Link Failsafe STOP
                if self.state == SafetyState.STOP and "CRITICAL FAILSAFE" in self.reason:
                    logger.info(f"Safety: Link restored (ELRS Dead: {elrs_dead}, WebRTC Dead: {webrtc_dead}). Recovering from Failsafe.")
                    # new_state remains NORMAL as initialized at start of update
                    self.reason = "Link restored"

        # 7. Check Storage Usage (Stress Test #11)
        storage = system.get("storage") if system else sensor_data.get("system", {}).get("storage")
        if storage and new_state not in [SafetyState.STOP, SafetyState.CRITICAL]:
            used_pct = storage.get("used_pct", 0.0)
            inodes_pct = storage.get("inodes_pct", 0.0)
            
            if used_pct > 95.0 or inodes_pct > 95.0:
                new_state = SafetyState.STORAGE_LOW
                self.reason = f"STORAGE CRITICAL: {max(used_pct, inodes_pct):.1f}%! Stopping Logs."
            elif used_pct > 90.0 or inodes_pct > 90.0:
                # Still NORMAL but we keep a warning in reason if nothing else is priority
                if new_state == SafetyState.NORMAL:
                    self.reason = f"STORAGE LOW: {max(used_pct, inodes_pct):.1f}%"

        # State transition logic
        if new_state != self.state:
            logger.info(
                f"Safety State Transition: {self.state.name} -> {new_state.name} Reason: {self.reason}"
            )
            self.state = new_state
            self.last_state_change = time.time()

        return self.state

    def process_controls(self, steering: float, throttle: float) -> tuple[float, float]:
        """
        Applies safety overrides and [SAFETY-007] Dynamic Scaling to control commands.
        """
        if self.state == SafetyState.STOP or self.state == SafetyState.CRITICAL:
            return steering, 0.0  # Force stop

        # Apply global dynamic multipliers
        combined_mult = self.thermal_throttle_mult * self.inference_throttle_mult
        effective_throttle = throttle * combined_mult

        if self.state == SafetyState.AVOID:
            # In AVOID state, we limit throttle further
            return steering, min(effective_throttle, 0.25)

        return steering, effective_throttle

    def _get_min_forward_dist(self, lidar_points: list) -> float:
        """
        Checks forward sector for closest obstacle.
        Uses binary search for sector boundaries (assuming points are angle-sorted).
        """
        if not lidar_points:
            return 8.0

        half_sector = self.lidar_sector_deg / 2.0
        min_dist_mm = 8000.0
        found = False

        def find_idx(target: float, is_left: bool) -> int:
            low, high = 0, len(lidar_points)
            while low < high:
                mid = (low + high) // 2
                if (
                    lidar_points[mid][0] < target
                    if is_left
                    else lidar_points[mid][0] <= target
                ):
                    low = mid + 1
                else:
                    high = mid
            return low

        idx_right_first = find_idx(half_sector, False)
        idx_left_second = find_idx(360.0 - half_sector, True)

        relevant_points = (
            lidar_points[:idx_right_first] + lidar_points[idx_left_second:]
        )

        for _, dist in relevant_points:
            if 50 < dist < min_dist_mm:  # Ignore very close noise < 5cm
                min_dist_mm = dist
                found = True

        return min_dist_mm / 1000.0 if found else 8.0

    def reset_impact(self) -> None:
        self.impact_detected = False
        self.faults = {k: False for k in self.faults}  # Reset all faults
        if self.state == SafetyState.STOP:
            self.state = SafetyState.NORMAL
            self.reason = "Safety reset by user."
            logger.info("Safety STOP reset by user.")

    def inject_fault(self, fault_type: str, enabled: bool = True) -> None:
        if fault_type in self.faults:
            self.faults[fault_type] = enabled
            logger.warning(f"FAULT INJECTED: {fault_type} = {enabled}")
