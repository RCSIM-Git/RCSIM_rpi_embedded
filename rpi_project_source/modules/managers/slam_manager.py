"""PL: Aktualizuje estymację pozycji i mapę SLAM w osobnym wątku.
EN: Updates the SLAM pose estimate and map in a separate thread.
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
    """PL: Przetwarza skany LiDAR i odometrię przy użyciu silnika SLAM.
    EN: Processes LiDAR scans and odometry with the SLAM engine.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        """PL: Przygotowuje silnik, stan mapy i kolejkę wejściową SLAM.
        EN: Prepares the SLAM engine, map state, and input queue.
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
        """PL: Przetwarza kolejkę skanów i aktualizuje estymowaną pozycję.
        EN: Processes queued scans and updates the estimated pose.
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
        """PL: Dodaje skan i zmianę odometrii do kolejki przetwarzania.
        EN: Adds a scan and odometry delta to the processing queue.
        """
        with self.lock:
            # Utrzymuj krótką kolejkę, aby uniknąć opóźnień
            # (deque with maxlen handles this automatically)
            self.input_queue.append((scan, odometry))
            self._new_data_event.set()

    def get_pose(self) -> tuple[float, float, float]:
        """PL: Zwraca chronioną blokadą aktualną estymację pozycji.
        EN: Returns the current pose estimate while holding the lock.
        """
        with self.lock:
            return self.current_pose

    def get_map(self) -> bytearray:
        """PL: Pobiera aktualne dane mapy zajętości z silnika SLAM.
        EN: Retrieves the current occupancy map data from the SLAM engine.
        """
        if RPI_SLAM_AVAILABLE and self.slam:
            try:
                self.slam.getmap(self.map_data)
            except Exception as e:
                logging.error(f"Failed to get map: {e}")
        return self.map_data

    def get_grid_array(self) -> np.ndarray:
        """PL: Zwraca mapę zajętości jako dwuwymiarową tablicę NumPy.
        EN: Returns the occupancy map as a two-dimensional NumPy array.
        """
        # Ensure we have fresh data
        arr = np.frombuffer(self.get_map(), dtype=np.uint8)
        return arr.reshape((self.MAP_SIZE_PIXELS, self.MAP_SIZE_PIXELS))

    def stop(self) -> None:
        """PL: Sygnalizuje wątkowi SLAM zakończenie pracy.
        EN: Signals the SLAM thread to stop running.
        """
        self.running = False
