"""
Menedżer SLAM (Simultaneous Localization and Mapping) dla Raspberry Pi.
SLAM Manager for the Raspberry Pi.

Obsługuje algorytm RMHC_SLAM (BreezySLAM) w osobnym wątku.
Supports RMHC_SLAM algorithm (BreezySLAM) in a separate thread.
"""

import logging
import math
import threading
import time
from collections import deque
from typing import Any

import numpy as np

try:
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import Laser

    RPI_SLAM_AVAILABLE = True
except ImportError:
    RPI_SLAM_AVAILABLE = False
    logging.warning("breezyslam not found. SLAM will run in Mock mode.")
    RMHC_SLAM = object  # Mock for type hinting
    Laser = object  # Mock for type hinting


class SlamManager(threading.Thread):
    """
    Zarządza silnikiem SLAM w osobnym wątku.
    Manages the SLAM engine in a separate thread.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        """
        Inicjalizuje menedżera SLAM.
        Initializes the SLAM manager.

        Args:
            config (dict[str, Any]): Konfiguracja SLAM. / SLAM configuration.
        """
        super().__init__(daemon=True)
        self.config = config
        self.running = False
        self.lock = threading.Lock()

        # Parametry mapy / Map parameters
        self.MAP_SIZE_PIXELS = 800
        self.MAP_SIZE_METERS = 40
        self.SCAN_SIZE = 360

        # Stan SLAM / SLAM State
        self.current_pose: tuple[float, float, float] = (
            0.0,
            0.0,
            0.0,
        )  # x, y, theta (rad)
        self.map_data = bytearray(self.MAP_SIZE_PIXELS * self.MAP_SIZE_PIXELS)
        self.last_update_time = time.time()

        # Charakterystyka sensora (LD08) / Sensor characteristics (LD08)

        if RPI_SLAM_AVAILABLE:
            # Laser(scan_size, scan_rate, detection_angle, distance_no_detection_mm)
            self.laser_model = Laser(self.SCAN_SIZE, 10, 360, 4000)
            self.slam = RMHC_SLAM(
                self.laser_model,
                self.MAP_SIZE_PIXELS,
                self.MAP_SIZE_METERS,
            )
        else:
            self.slam = None

        # Kolejka wejściowa / Input queue
        # list[(scan, odometry)]
        self.input_queue: deque[
            tuple[list[tuple[float, float]], tuple[float, float, float, float]]
        ] = deque(maxlen=10)
        self._new_data_event = threading.Event()

    def run(self) -> None:
        """
        Główna pętla wątku SLAM.
        Main loop of the SLAM thread.
        """
        self.running = True
        logging.info("SLAM thread started.")

        while self.running:
            if not self.input_queue:
                self._new_data_event.wait(timeout=0.1)
                if not self.input_queue:
                    continue

            with self.lock:
                if self.input_queue:
                    scan, (dx, dy, dyaw, dt) = self.input_queue.popleft()
                    self._new_data_event.clear()
                else:
                    continue

            # BreezySLAM oczekuje surowych dystansów w mm w odpowiedniej kolejności
            # BreezySLAM expects raw distances in mm in correct order
            distances_mm = [0] * self.SCAN_SIZE
            for angle, dist in scan:
                idx = int(angle % 360)
                if idx < self.SCAN_SIZE:
                    distances_mm[idx] = int(dist)  # dystans w mm / distance in mm

            if RPI_SLAM_AVAILABLE and self.slam:
                try:
                    # BreezySLAM expect update(scan, pose_change=(dxy_mm, dyaw_deg, dt))
                    dxy_mm = math.sqrt(dx**2 + dy**2) * 1000
                    dyaw_deg = math.degrees(dyaw)

                    self.slam.update(distances_mm, (dxy_mm, dyaw_deg, dt))

                    x_mm, y_mm, theta_deg = self.slam.getpos()
                    with self.lock:
                        self.current_pose = (x_mm / 1000.0, y_mm / 1000.0, theta_deg)
                except Exception as e:
                    logging.error(f"SLAM update error: {e}")
            else:
                # Mock move
                with self.lock:
                    x, y, th = self.current_pose
                    self.current_pose = (x + dx, y + dy, th + dyaw)

            self.last_update_time = time.time()

    def update(
        self,
        scan: list[tuple[float, float]],
        odometry: tuple[float, float, float, float],
    ) -> None:
        """
        Dodaje nowe dane do przetworzenia przez SLAM.
        Adds new data to be processed by SLAM.

        Args:
            scan (list[tuple[float, float]]): Lista krotek (kąt, dystans). / List of tuples (angle, distance).
            odometry (tuple[float, float, float, float]): Zmiana pozycji (dx, dy, dyaw, dt). / Pose change (dx, dy, dyaw, dt).
                                                          Note: dt is packed into the 4th element locally before calling this.
        """
        with self.lock:
            # Utrzymuj krótką kolejkę, aby uniknąć opóźnień
            # (deque with maxlen handles this automatically)
            self.input_queue.append((scan, odometry))
            self._new_data_event.set()

    def get_pose(self) -> tuple[float, float, float]:
        """
        Zwraca aktualną estymowaną pozycję.
        Returns the current estimated pose.

        Returns:
            tuple[float, float, float]: (x, y, theta_deg).
        """
        with self.lock:
            return self.current_pose

    def get_map(self) -> bytearray:
        """
        Pobiera aktualną mapę zajętości.
        Retrieves the current occupancy grid map.

        Returns:
            bytearray: Dane mapy. / Map data.
        """
        if RPI_SLAM_AVAILABLE and self.slam:
            try:
                self.slam.getmap(self.map_data)
            except Exception as e:
                logging.error(f"Failed to get map: {e}")
        return self.map_data

    def get_grid_array(self) -> np.ndarray:
        """
        Zwraca mapę jako tablicę numpy (H, W).
        Returns the map as a numpy array.
        """
        # Ensure we have fresh data
        arr = np.frombuffer(self.get_map(), dtype=np.uint8)
        return arr.reshape((self.MAP_SIZE_PIXELS, self.MAP_SIZE_PIXELS))

    def stop(self) -> None:
        """
        Zatrzymuje wątek SLAM.
        Stops the SLAM thread.
        """
        self.running = False
