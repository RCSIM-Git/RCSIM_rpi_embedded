"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Główny serwis aplikacyjny dla oprogramowania na Raspberry Pi.
Main application service for the Raspberry Pi software.
"""

from __future__ import annotations

import json
import logging
import math
import os
import signal
import sys
import threading
import time
from typing import Any

# Global heartbeat toggle for hardware watchdog
_HEARTBEAT_PIN_STATE = False
_GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO

    GPIO.setmode(GPIO.BCM)
    HEARTBEAT_PIN = 26  # Standardowy pin dla watchdoga na HAT
    OVERRIDE_PIN = 21  # Pin wejściowy informujący o przejęciu kontroli przez RC
    GPIO.setup(HEARTBEAT_PIN, GPIO.OUT)
    GPIO.setup(OVERRIDE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    _GPIO_AVAILABLE = True
except (ImportError, RuntimeError) as e:
    logging.warning(
        f"GPIO Initialization failed: {e}. Heartbeat and Override will be disabled."
    )

# Dodanie ścieżki głównej projektu / Add project root path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from core.actuator_worker import ActuatorWorker
from core.command_dispatcher import CommandDispatcher
from core.mavlink_service import MAVLinkService
from core.safety_supervisor import SafetyState, SafetySupervisor
from core.telemetry_builder import TelemetryBuilder
from core.telemetry_sender import TelemetrySender
from core.udp_service import UDPService

# Importy modułów / Module imports
from core.utils.system_info import get_board_info
from core.webrtc_manager import WebRTCManager
from logic.control_selector import ControlSelector
from logic.mission_manager import MissionManager
from logic.navigation_manager import NavigationManager
from modules.ai_manager import AIManager
from modules.camera_manager import CameraManager
from modules.managers.hardware_manager import HardwareManager

# SLAM i Planowanie [PLAN-002]
from modules.managers.slam_manager import SlamManager
from modules.planners.local_planner import LocalPlanner


class TelemetryWorker(threading.Thread):
    """
    Wątek obsługujący pętlę telemetryczną i sterowania.
    Thread handling the telemetry and control loop.
    """

    FAILSAFE_TIMEOUT: float = (
        5.0  # Time without packet to activate Failsafe (increased from 2.0 for RF stability)
    )
    DEFAULT_LOOP_FREQ: int = 20  # Default loop frequency (Hz)

    def __init__(self, config: dict[str, Any]) -> None:
        """
        Inicjalizuje wątek roboczy telemetrii.
        Initializes the telemetry worker thread.
        """
        super().__init__(daemon=True)
        self.config = config
        self.running = True
        self._stop_event = threading.Event()

        # Konfiguracja pętli / Loop configuration
        self.TARGET_FREQ: int = self.config.get(
            "main_loop_freq_hz", self.DEFAULT_LOOP_FREQ
        )
        self.LOOP_TIME: float = 1.0 / self.TARGET_FREQ

        # Stan systemu / System state
        self.current_mode: str = "MANUAL"
        self.last_control_input: dict[str, Any] = {
            "manual_controls": {"steering": 0.0, "throttle": 0.0}
        }
        self.current_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_packet_time = time.time()
        self.slam_frame_count = 0
        self.home_position: dict[str, float] | None = None

        # --- Zarządzanie Stanem / State Management ---
        self.link_established = (
            False  # Whether connection with PC is established (WebRTC/UDP)
        )
        self.elrs_link_established = False  # Whether RF link is established
        self.last_elrs_packet_time = 0.0

        self.pca_armed = False  # Whether PWM controller is armed (active)
        self.extra_channels_data: dict[int, int] = {}  # Data for channels 2-15

        # Gimbal Stabilization states (Plan B)
        self.manual_pitch_offset = 0.0
        self.manual_roll_offset = 0.0
        self.gimbal_stabilizer = None

        # --- Statystyki Sieciowe / Network Stats Tracking ---
        self.last_pc_timestamp = 0.0  # PC timestamp for RTT
        self.telemetry_packet_idx = 0  # Sequential index

        # Usługi komunikacyjne / Communication services
        self.webrtc_service: WebRTCManager | None = None
        self.udp_service: UDPService | None = None
        self.telemetry_sender: TelemetrySender | None = None
        self.comm_mode: str = self.config.get("comm_mode", "WEBRTC")
        self.comm_protocol: str = self.config.get("comm_protocol", "NATIVE")
        self.mavlink_service = None

        # MAVLink Throttling (Dynamic)
        mav_rate = self.config.get("mavlink_throttle_hz", 10)
        self._mavlink_mod = max(1, int(self.TARGET_FREQ / max(1, mav_rate)))
        # Low frequency for battery/system status (e.g. 1Hz)
        self._mavlink_status_mod = max(1, int(self.TARGET_FREQ / 1.0))
        # Medium frequency for GPS (e.g. 2Hz)
        self._mavlink_gps_mod = max(1, int(self.TARGET_FREQ / 2.0))

        # Actuator Worker
        self.actuator_worker: ActuatorWorker | None = None

        # Menedżery (inicjalizowane w run())
        self.hw_manager: HardwareManager | None = None
        self.nav_manager: NavigationManager | None = None
        self.mission_manager: MissionManager | None = None
        self.control_selector: ControlSelector | None = None
        self.camera_manager: CameraManager | None = None
        self.ai_manager: AIManager | None = None
        self.local_planner: LocalPlanner | None = None
        self.slam_manager: SlamManager | None = None
        self.safety_supervisor = SafetySupervisor(config)

        # --- Stan Auto-Kalibracji / Auto-Calibration State ---
        self.last_calibration_time = time.time()
        self.stationary_start_time: float | None = None
        self.AUTO_CALIB_IDLE_TIME = 5.0
        self.AUTO_CALIB_COOLDOWN = 60.0
        self.telemetry_builder = TelemetryBuilder(self)
        self.command_dispatcher = CommandDispatcher(self)
        self.comm_protocol: str = self.config.get("comm_protocol", "NATIVE")
        self.mavlink_service: MAVLinkService | None = None

    def run(self) -> None:
        """
        Główna metoda pętli wątku z precyzyjnym taktowaniem.
        Main thread loop method with high-precision timing.
        """
        logging.info("Telemetry thread started.")
        try:
            # 1. Inicjalizacja Hardware (Silent Mode)
            hw_config = self.config.get("hardware", {})
            hw_config["ntrip"] = self.config.get("ntrip", {})
            # V39.12: Dynamiczne przekazanie watchdog_timeout_ms do HardwareManager
            safety_cfg = self.config.get("safety", {})
            hw_config["watchdog_timeout_ms"] = safety_cfg.get(
                "watchdog_timeout_ms", 500
            )

            self.hw_manager = HardwareManager(hw_config, init_pca_neutral=False)

            # Rozpoczęcie wątku wysokiego priorytetu dla PWM
            self.actuator_worker = ActuatorWorker(self.hw_manager, freq=50)
            self.actuator_worker.start()

            # 2. Inicjalizacja AI i Logiki
            self.ai_manager = AIManager(logging.getLogger("AIManager"), self.config)

            self.nav_manager = NavigationManager()
            self.nav_manager.update_config(
                self.config
            )  # [PLAN-009] Sync adaptive params
            self.mission_manager = MissionManager(self.nav_manager)
            self.control_selector = ControlSelector(self.nav_manager, self.ai_manager)

            # SLAM i Planowanie - Inicjalizacja instancji
            self.slam_manager = SlamManager(self.config.get("slam", {}))
            self.local_planner = LocalPlanner(config=self.config)
            self.slam_manager.start()
            self.current_pose = (0.0, 0.0, 0.0)

            # Load Extrinsics from calibration
            self.camera_extrinsics = {"x": 0.1, "y": 0.0, "z": 0.15, "pitch": 0.0}

            # 3. Inicjalizacja Komunikacji
            self._init_communication()

        except Exception as e:
            logging.critical(f"Init error: {e}", exc_info=True)
            return

        # [OPTIMIZATION] High-precision timing with perf_counter_ns
        interval_ns = int(self.LOOP_TIME * 1_000_000_000)
        next_tick = time.perf_counter_ns()

        # --- Główna pętla sterowania / Main Control Loop ---
        while self.running:
            try:
                # Wykonaj krok pętli / Execute loop step
                self._loop_step()

                # Precyzyjne czekanie / Precision sleep
                next_tick += interval_ns
                now = time.perf_counter_ns()
                if next_tick > now:
                    sleep_time_s = (next_tick - now) / 1_000_000_000
                    if sleep_time_s > 0.002:
                        time.sleep(sleep_time_s - 0.002)
                    while time.perf_counter_ns() < next_tick:
                        pass
                else:
                    # Spóźnienie - resetuj synchronizację
                    next_tick = time.perf_counter_ns()

            except Exception as e:
                logging.error(f"Loop error: {e}")
                time.sleep(0.01)

        self.cleanup()
        logging.info("Telemetry thread stopped.")

    def _loop_step(self) -> None:
        """
        Pojedynczy krok głównej pętli telemetrii.
        A single step of the main telemetry loop.
        """
        # 0. Safety Watchdog [PLAN-011]
        if self.hw_manager.pca:
            self.hw_manager.pca.check_failsafe()

        # A. Odczyt sensorów
        sensor_data = self.hw_manager.read_sensors()

        # B. SLAM i Odometria
        full_scan = []
        if self.hw_manager.lidar:
            full_scan = self.hw_manager.lidar.read_scan(downsample=False) or []

        throttle = self.last_control_input["manual_controls"]["throttle"]
        steering = self.last_control_input["manual_controls"]["steering"]
        dx, dy, dyaw = self.hw_manager.estimate_motion(
            throttle, steering, self.LOOP_TIME, sensor_data.get("imu")
        )

        if full_scan:
            self.slam_frame_count += 1
            self.slam_manager.update(full_scan, (dx, dy, dyaw, self.LOOP_TIME))

        self.current_pose = self.slam_manager.get_pose()

        # Calculate current speed for Adaptive Lookahead [PLAN-002]
        current_speed = (
            math.hypot(dx, dy) / self.LOOP_TIME if self.LOOP_TIME > 0 else 0.0
        )

        # C. Mission Update
        if self.mission_manager and self.mission_manager.is_running:
            grid = self.slam_manager.get_grid_array()
            self.mission_manager.update(self.current_pose, grid)

        # D. Pobranie obrazu i AI
        ai_frame = None
        detections = []
        ai_controls = None
        if self.camera_manager:
            ai_frame = self.camera_manager.get_ai_frame()
            if ai_frame is not None:
                grid = self.local_planner.costmap if self.local_planner else None

                # Przygotowanie rozszerzonego słownika dla wektora stanu AI
                if sensor_data is not None:
                    sensor_data["speed"] = current_speed
                    sensor_data["pose"] = self.current_pose
                    sensor_data["last_throttle"] = throttle
                    sensor_data["last_steering"] = steering
                    # W przyszłości można tu wyliczyć CTE i Heading Error do przekazania

                can_save = self.safety_supervisor.state != SafetyState.STORAGE_LOW
                detections, ai_controls = self.ai_manager.predict(
                    image=ai_frame,
                    grid=grid,
                    sensor_data=sensor_data,
                    nav_manager=self.nav_manager,
                    local_planner=self.local_planner,
                    can_save=can_save,
                )

        # D2. Aktualizacja Local Planner (Fusion: Lidar + AI + IMU Terrain)
        if full_scan:
            self.local_planner.update_occupancy_from_lidar_and_yolo(
                self.current_pose,
                full_scan,
                detections,
                self.camera_extrinsics,
                sensor_data.get("imu"),  # [PLAN-010] Rough Terrain Info
            )

        self.last_ai_detections = detections
        self.last_ai_controls = ai_controls

        # E. Planowanie Reaktywne / Śledzenie Ścieżki / Auto-Explore
        goal_global = None
        if self.nav_manager:
            if getattr(self.nav_manager, "auto_explore_active", False):
                grid = self.slam_manager.get_grid_array()
                self.nav_manager.update_auto_explore(
                    grid, (self.current_pose[0], self.current_pose[1])
                )

            # [PLAN-002] Use Adaptive Lookahead point instead of static waypoint
            goal_global = self.nav_manager.get_lookahead_point(
                (self.current_pose[0], self.current_pose[1]), current_speed
            )

        if goal_global and getattr(self.nav_manager, "current_path", None) is not None:
            planner_steer, planner_throttle, safety_score = (
                self.local_planner.plan_pure_pursuit(
                    robot_pose=self.current_pose, lookahead_point=goal_global
                )
            )
        else:
            planner_steer, planner_throttle, safety_score = (
                self.local_planner.plan_reactive(
                    robot_pose=self.current_pose, goal_pose=goal_global
                )
            )

        # [AI-E2E] Override planner with AI direct controls if available
        if ai_controls and self.current_mode in [
            "AI_STEER_ONLY",
            "FULL_AUTOPILOT",
            "AUTONOMOUS",
        ]:
            ai_steer = ai_controls.get("steering", 0.0)
            ai_throttle = ai_controls.get("throttle", 0.0)

            # Use AI values but keep safety_score from Geometric Planner for Reactive Stop
            planner_steer = ai_steer
            if self.current_mode in ["FULL_AUTOPILOT", "AUTONOMOUS"]:
                planner_throttle = ai_throttle

        # F. Przygotowanie danych ramki
        sys_info = get_board_info()
        frame_data = {
            **sensor_data,
            **self.last_control_input,
            "pose": self.current_pose,
            "home_position": self.home_position,
            "dt": self.LOOP_TIME,
            "image": ai_frame,
            "ai_detections": detections,
            "ai_controls": ai_controls,
            "ai_status": {"fps": getattr(self.ai_manager, "last_fps", 30.0)},
            "system": sys_info,
            "planner_cmd": (planner_steer, planner_throttle, safety_score),
            "link_status": {
                "webrtc_dead": (time.time() - self.last_packet_time) > 5.0,
                "elrs_dead": (time.time() - getattr(self, "last_elrs_packet_time", 0.0))
                > 0.5,
            },
        }

        # G. Obliczenie sterowania
        if self.pca_armed:
            steering, throttle = self.control_selector.process_frame(
                self.current_mode, frame_data
            )

            # H. Safety Supervisor Override
            ai_ready = self.ai_manager and not self.ai_manager.use_mock
            self.safety_supervisor.update(frame_data, ai_ready=ai_ready)
            steering, throttle = self.safety_supervisor.process_controls(
                steering, throttle
            )

            # --- GIMBAL STABILIZATION & RATE-MODE PROCESSING (PLAN B) ---
            self._update_gimbal(sensor_data)

            # Actuate Hardware via ActuatorWorker (Bufferized)
            # [SAFETY] If ESP32 Watchdog has taken over (Override), we stop sending PWM to avoid I2C collisions
            if self.actuator_worker:
                is_overridden = frame_data.get("ovr", False)
                if is_overridden:
                    # ESP32 is in control, we stay in standby
                    self.actuator_worker.set_commands(0.0, 0.0, {}, False)
                else:
                    self.actuator_worker.set_commands(
                        steering, throttle, self.extra_channels_data, self.pca_armed
                    )
        else:
            # System rozbrojony - zapewnij neutralne wartości w workerze
            if self.actuator_worker:
                self.actuator_worker.set_commands(0.0, 0.0, {}, False)

        # Logika Auto-Kalibracji
        if not self.pca_armed and abs(throttle) < 0.05 and abs(steering) < 0.05:
            if self.stationary_start_time is None:
                self.stationary_start_time = time.time()
            elif (time.time() - self.stationary_start_time) > self.AUTO_CALIB_IDLE_TIME:
                if (
                    time.time() - self.last_calibration_time
                ) > self.AUTO_CALIB_COOLDOWN:
                    if self.hw_manager.calibrate_imu():
                        logging.info("Auto-Calibration: Success.")
                    self.last_calibration_time = time.time()
                    self.stationary_start_time = None
        else:
            self.stationary_start_time = None

        # Logika Failsafe (Hybrydowa)
        webrtc_dead = frame_data["link_status"]["webrtc_dead"]
        udp_dead = (
            (time.time() - self.last_packet_time) > self.FAILSAFE_TIMEOUT
            if self.udp_service
            else True
        )
        network_dead = webrtc_dead and udp_dead

        # MAVLink link status (RF Link)
        rf_dead = not self.mavlink_service.link_active if self.mavlink_service else True

        # Dynamic switching logic for HYBRID mode
        if self.comm_mode == "HYBRID":
            if not rf_dead:
                self.link_established = True  # RF Link is dominant
                self.elrs_link_established = True
            else:
                if self.elrs_link_established:
                    # Reset last processed network timestamp on transitioning to VPN
                    self.command_dispatcher.last_processed_tx_time = 0.0
                self.link_established = not network_dead
                self.elrs_link_established = False
        else:
            # Legacy modes
            if network_dead and rf_dead:
                if self.pca_armed and self.current_mode != "FAILSAFE":
                    logging.warning("CRITICAL FAILSAFE: Both Network and RF Link lost!")
                    self.current_mode = "FAILSAFE"
                self.link_established = False
                self.elrs_link_established = False
            else:
                self.link_established = not network_dead
                self.elrs_link_established = not rf_dead

        # Zabezpieczenie nałożone wewnątrz modułu bramki z SafetySupervisor
        if (
            self.safety_supervisor.state == SafetyState.RTH
            and self.current_mode != "FAILSAFE"
        ):
            logging.warning(
                f"RTH OVERRIDE BY SAFETY SUPERVISOR: {self.safety_supervisor.reason}"
            )
            self.current_mode = "FAILSAFE"

        # I. Wysłanie telemetrii
        self.telemetry_packet_idx += 1
        telemetry_packet = self.telemetry_builder.prepare_telemetry(frame_data)

        # [SAF-003] Hardware Heartbeat toggle
        global _HEARTBEAT_PIN_STATE
        is_overridden = False
        if _GPIO_AVAILABLE:
            _HEARTBEAT_PIN_STATE = not _HEARTBEAT_PIN_STATE
            GPIO.output(HEARTBEAT_PIN, _HEARTBEAT_PIN_STATE)
            is_overridden = not GPIO.input(OVERRIDE_PIN)  # Low = Override active

        telemetry_packet["ovr"] = is_overridden

        if hasattr(self, "last_ai_detections"):
            telemetry_packet["ai_detections"] = self.last_ai_detections

        telemetry_packet["safety"] = {
            "state": self.safety_supervisor.state.name,
            "reason": self.safety_supervisor.reason,
            "impact": self.safety_supervisor.impact_detected,
        }

        if self.telemetry_packet_idx % 5 == 0:
            telemetry_packet["planner"] = {
                "steer": planner_steer,
                "throttle": planner_throttle,
                "safety": safety_score,
            }

        if self.telemetry_sender:
            self.telemetry_sender.send_packet(telemetry_packet)

        # J. MAVLink Telemetry (Individually throttled)
        if self.comm_protocol == "MAVLINK" and self.mavlink_service:
            self._send_mavlink_telemetry(frame_data, telemetry_packet)

        # K. Hot-plug Resilience Check (Throttled)
        if self.telemetry_packet_idx % 200 == 0:  # Every ~10 seconds at 20Hz
            if self.hw_manager:
                self.hw_manager.sensors.check_reconnect_needed()

    def _update_gimbal(self, sensor_data: dict[str, Any]) -> None:
        """
        Zintegrowana stabilizacja i manualne sterowanie gimbalem FPV (Pitch/Roll).
        Integrated stabilization and manual gimbal control (FPV Pitch/Roll).
        """
        gimbal_cfg = self.config.get("gimbal", {})
        if not gimbal_cfg.get("enabled", False):
            return

        # Leniwa inicjalizacja GimbalStabilizer (Lazy Initialization)
        if not hasattr(self, "gimbal_stabilizer") or self.gimbal_stabilizer is None:
            try:
                from modules.gimbal_stabilizer import GimbalStabilizer

                self.gimbal_stabilizer = GimbalStabilizer(
                    gimbal_cfg, logging.getLogger("GimbalStabilizer")
                )
            except Exception as e:
                logging.error(f"Failed to initialize GimbalStabilizer: {e}")
                return

        imu = sensor_data.get("imu", {}) or {}
        pitch_val = imu.get("pitch", 0.0) or 0.0
        roll_val = imu.get("roll", 0.0) or 0.0

        # Kanały sterowania gimbalem
        pitch_ch = gimbal_cfg.get("pitch_channel", 4)
        roll_ch = gimbal_cfg.get("roll_channel", 5)

        # Pobranie manualnych sygnałów z GCS (domyślnie 1500 us)
        manual_pitch_pulse = self.extra_channels_data.get(pitch_ch, 1500)
        manual_roll_pulse = self.extra_channels_data.get(roll_ch, 1500)

        # Maksymalne limity kątów do przeliczeń manualnych
        pitch_max_angle = gimbal_cfg.get("pitch_max_angle", 45.0)
        roll_max_angle = gimbal_cfg.get("roll_max_angle", 45.0)
        pitch_min_angle = gimbal_cfg.get("pitch_min_angle", -45.0)
        roll_min_angle = gimbal_cfg.get("roll_min_angle", -45.0)

        # 1. Pitch manual offset calculation
        pitch_mode = gimbal_cfg.get("pitch_mode", "absolute")
        if pitch_mode == "rate":
            # Wyznaczamy wychylenie drążka (-1.0 do 1.0)
            deflection = (manual_pitch_pulse - 1500) / 500.0
            # Deadband 0.05
            if abs(deflection) < 0.05:
                deflection = 0.0
            speed_scale = gimbal_cfg.get("pitch_speed_scale", 30.0)
            self.manual_pitch_offset += deflection * speed_scale * self.LOOP_TIME
            # Przycięcie do zakresów mechanicznych
            self.manual_pitch_offset = max(
                pitch_min_angle, min(pitch_max_angle, self.manual_pitch_offset)
            )
        else:
            # Tryb absolute
            self.manual_pitch_offset = (
                (manual_pitch_pulse - 1500) / 500.0
            ) * pitch_max_angle

        # 2. Roll manual offset calculation
        roll_mode = gimbal_cfg.get("roll_mode", "absolute")
        if roll_mode == "rate":
            deflection = (manual_roll_pulse - 1500) / 500.0
            if abs(deflection) < 0.05:
                deflection = 0.0
            speed_scale = gimbal_cfg.get("roll_speed_scale", 30.0)
            self.manual_roll_offset += deflection * speed_scale * self.LOOP_TIME
            self.manual_roll_offset = max(
                roll_min_angle, min(roll_max_angle, self.manual_roll_offset)
            )
        else:
            # Tryb absolute
            self.manual_roll_offset = (
                (manual_roll_pulse - 1500) / 500.0
            ) * roll_max_angle

        # 3. Połączenie stabilizacji horyzontu z manualnym przesunięciem (offsetem)
        p_gain = gimbal_cfg.get("p_gain", 1.0)

        # Prawidłowy znak stabilizacji: ujemne sprzężenie zwrotne do kompensacji
        # pochylenia / Correct stabilization sign: negative feedback to compensate pitch
        combined_pitch_angle = -pitch_val * p_gain + self.manual_pitch_offset
        combined_roll_angle = -roll_val * p_gain + self.manual_roll_offset

        # 4. Mapowanie kątów na końcowe impulsy PWM
        final_pitch_pulse = self.gimbal_stabilizer._map_value(
            combined_pitch_angle,
            pitch_min_angle,
            pitch_max_angle,
            gimbal_cfg.get("pitch_min_pulse", 1000),
            gimbal_cfg.get("pitch_max_pulse", 2000),
        )
        final_roll_pulse = self.gimbal_stabilizer._map_value(
            combined_roll_angle,
            roll_min_angle,
            roll_max_angle,
            gimbal_cfg.get("roll_min_pulse", 1000),
            gimbal_cfg.get("roll_max_pulse", 2000),
        )

        # Zapisz gotowe sygnały do extra_channels_data
        self.extra_channels_data[pitch_ch] = int(final_pitch_pulse)
        self.extra_channels_data[roll_ch] = int(final_roll_pulse)

    def _send_mavlink_telemetry(
        self, sensor_data: dict[str, Any], telemetry_packet: dict[str, Any]
    ) -> None:
        """Wysyła telemetrię przez standard MAVLink z różnymi częstotliwościami."""
        if not self.mavlink_service:
            return

        ms = self.mavlink_service
        idx = self.telemetry_packet_idx

        # 1. High Frequency: Attitude (IMU)
        if idx % self._mavlink_mod == 0:
            imu = sensor_data.get("imu", {})
            ms.send_attitude(
                imu.get("roll", 0.0),
                imu.get("pitch", 0.0),
                imu.get("yaw", self.current_pose[2]),
                imu_data=imu,
            )

        # 2. Medium Frequency: Position (GPS)
        if idx % self._mavlink_gps_mod == 0:
            gps = sensor_data.get("gps", {})
            if gps:
                ms.send_position(
                    gps["lat"],
                    gps["lon"],
                    gps.get("altitude", 0.0),
                    gps.get("speed_kmh", 0.0),
                    gps.get("track", 0.0),
                )

        # 3. Low Frequency: Status (Battery/CPU)
        if idx % self._mavlink_status_mod == 0:
            bat = sensor_data.get("battery", {})
            sys_info = sensor_data.get("system", {})
            cpu_load = sys_info.get("cpu_load", 0)
            ms.send_status(
                bat.get("voltage", 0.0), bat.get("current", 0.0), int(cpu_load)
            )

        # Obstacle Distance (LiDAR) - DISABLED for RF link to save bandwidth
        # if self.hw_manager.lidar and self.telemetry_packet_idx % 5 == 0:
        #     lidar_data = telemetry_packet.get("lidar", [])
        #     if lidar_data:
        #         ms.send_obstacle_distance([int(d * 100) for d in lidar_data])

    def _init_communication(self) -> None:
        """
        Inicjalizuje usługi komunikacyjne i wideo obiektu.
        Initializes the object's communication and video services.
        """
        logging.info("Initializing communication modules...")
        self.camera_manager = CameraManager(self.config.get("camera", {}))
        self.camera_manager.start()

        # Cleanup: CRSF/FBW is now handled purely in GCS or via MAVLink.
        # RPi no longer listens to CRSF directly (FBW mode archived).
        self.crsf_parser = None

        if self.comm_protocol == "MAVLINK":
            # Prefer serial connection if string looks like a path, otherwise use UDP
            conn_str = self.config.get("mavlink_connection", "udpin:0.0.0.0:14550")
            logging.info(f"Initializing MAVLink on: {conn_str}")

            self.mavlink_service = MAVLinkService(
                connection_str=conn_str,
                system_id=self.config.get("system_id", 10),
                on_rc_channels=self.command_dispatcher._handle_mavlink_rc,
                on_arm_disarm=lambda armed: self.command_dispatcher.handle_command(
                    {"command": "ARM_PCA" if armed else "DISARM_PCA"}
                ),
            )
            self.mavlink_service.start()
            # Wstrzykujemy instancję do dispatchera by mógł wysyłać ACKi
            self.command_dispatcher.worker.mavlink_service = self.mavlink_service
            logging.info("✅ MAVLink Protocol Service initialized (Dynamic Link).")

        self._start_data_service(self.comm_mode)

    def _on_elrs_channels(self, channels: list[int]) -> None:
        """Deprecated."""

    def _start_data_service(self, mode: str) -> None:
        """
        Uruchamia oba serwisy komunikacyjne (WebRTC i UDP) jednocześnie.
        Starts both communication services (WebRTC and UDP) simultaneously.

        Usługi będą niezależ działać i obsługiwać dane przychodzące przez
        wspólny callback on_data_received().
        """
        logging.info("Starting Data Services (WebRTC + UDP)...")

        # Zawsze startuj WebRTC na port 8080
        self.webrtc_service = WebRTCManager(
            port=8080, on_data_received=self.command_dispatcher.on_data_received
        )
        self.webrtc_service.start()
        logging.info("✅ WebRTC signaling server started on port 8080")

        # Zawsze startuj UDP na port 12346 (z IP z config)
        pc_ip = self.config.get("pc_ip")
        if not pc_ip:
            logging.warning("⚠️ No 'pc_ip' in config. UDP will wait for handshake.")
        self.udp_service = UDPService(
            port=12346,
            on_data_received=self.command_dispatcher.on_data_received,
            target_ip=pc_ip,
            target_port=12347,
        )
        self.udp_service.start()
        logging.info("✅ UDP service started on port 12346")

        self.telemetry_sender = TelemetrySender(
            self.webrtc_service, self.udp_service, self
        )
        self.telemetry_sender.start()
        logging.info("✅ Async TelemetrySender queue started")

    def _switch_comm_mode(self, new_mode: str) -> None:
        """
        Aktualizuje preferowany tryb komunikacji w config.
        Oba serwisy (WebRTC i UDP) są zawsze aktywne niezależnie od tego ustawienia.

        Updates the preferred communication mode in config.
        Both services (WebRTC and UDP) are always active regardless of this setting.
        """
        logging.info(f"Updating preferred comm mode to: {new_mode}")
        logging.info("⚠️ Note: Both WebRTC and UDP are ALWAYS active simultaneously!")
        self.comm_mode = new_mode
        self.config["comm_mode"] = new_mode
        try:
            with open(os.path.join(project_root, "config.json"), "w") as f:
                json.dump(self.config, f, indent=4)
            logging.info(
                "✅ Config updated. Active services: WebRTC (port 8080) + UDP (port 12346)"
            )
        except Exception as e:
            logging.error(f"Config save failed: {e}")

    def stop(self) -> None:
        """
        Zatrzymuje główną pętlę roboczą.
        Stops the main worker loop.
        """
        self.running = False

    def cleanup(self) -> None:
        """
        Rozłącza kanały sprzętowe i zatrzymuje menedżery w bezpieczny sposób.
        Disconnects hardware channels and stops managers safely.
        """
        logging.info("Cleaning up resources...")
        if hasattr(self, "actuator_worker") and self.actuator_worker:
            self.actuator_worker.stop()
            self.actuator_worker.join(timeout=1.0)

        if self.hw_manager:
            try:
                self.hw_manager.disable_all_channels()
                self.hw_manager.cleanup()
            except Exception as e:
                logging.error(f"Hardware cleanup error: {e}")
        if hasattr(self, "telemetry_sender") and self.telemetry_sender:
            self.telemetry_sender.stop()
            self.telemetry_sender.join(timeout=1.0)
        if self.webrtc_service:
            self.webrtc_service.stop()
        if self.udp_service:
            self.udp_service.stop()
        if self.camera_manager:
            self.camera_manager.stop()
        if self.ai_manager:
            self.ai_manager.cleanup()
        if hasattr(self, "slam_manager") and self.slam_manager:
            self.slam_manager.stop()


def main() -> None:
    """
    Punkt wejścia głównego serwisu aplikacji Raspberry Pi.
    Entry point for the Raspberry Pi main application service.
    """
    logging.basicConfig(
        level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    for lib in ["aiortc", "aioice", "aiohttp"]:
        logging.getLogger(lib).setLevel(logging.WARNING)
    config_path = os.path.join(project_root, "config.json")
    try:
        with open(config_path, "r") as f:
            config = json.load(f)
    except FileNotFoundError:
        logging.error("Config file not found! Using defaults.")
        config = {}
    worker = TelemetryWorker(config)

    def signal_handler(signum, frame):
        logging.info("Shutdown signal received.")
        worker.stop()

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    worker.start()
    try:
        while worker.is_alive():
            worker.join(timeout=1.0)
    except KeyboardInterrupt:
        worker.stop()
        worker.join()


if __name__ == "__main__":
    main()
