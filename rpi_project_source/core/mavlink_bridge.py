"""PL: Udostępnia most MAVLink do przesyłania telemetrii do QGroundControl.

EN: Provides a MAVLink bridge for sending telemetry to QGroundControl.
"""

import logging
import threading
import time
from typing import Any

from pymavlink import mavutil

logger = logging.getLogger(__name__)


class MAVLinkBridge:
    """PL: Wysyła heartbeat i telemetrię pojazdu przez połączenie MAVLink.

    EN: Sends vehicle heartbeats and telemetry over a MAVLink connection.
    """

    def __init__(
        self, connection_str: str = "udpout:127.0.0.1:14550", system_id: int = 1
    ):
        """PL: Konfiguruje połączenie i identyfikator systemu MAVLink.

        Args:
            connection_str: Parametr połączenia przekazywany do pymavlink.
            system_id: Identyfikator systemu źródłowego MAVLink.

        EN: Configures the MAVLink connection and system identifier.

        Args:
            connection_str: Connection string passed to pymavlink.
            system_id: MAVLink source system identifier.
        """
        self.connection_str = connection_str
        self.system_id = system_id
        self.component_id = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        self.master: mavutil.mavlink_connection | None = None
        self.running = False
        self.thread: threading.Thread | None = None
        self.boot_time = time.time()

    def start(self):
        """PL: Otwiera połączenie MAVLink i uruchamia wątek heartbeat.

        EN: Opens the MAVLink connection and starts the heartbeat thread.
        """
        try:
            self.master = mavutil.mavlink_connection(
                self.connection_str, source_system=self.system_id
            )
            self.running = True
            self.thread = threading.Thread(target=self._update_loop, daemon=True)
            self.thread.start()
            logger.info(f"✅ MAVLink Bridge started on {self.connection_str}")
        except Exception as e:
            logger.error(f"❌ Failed to start MAVLink Bridge: {e}")

    def stop(self):
        """PL: Zatrzymuje pętlę i czeka krótko na zakończenie wątku.

        EN: Stops the loop and waits briefly for the thread to finish.
        """
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

    def _update_loop(self):
        """PL: Wysyła heartbeat co sekundę, dopóki most jest uruchomiony.

        EN: Sends a heartbeat once per second while the bridge is running.
        """
        last_heartbeat = 0
        while self.running:
            now = time.time()
            if now - last_heartbeat >= 1.0:
                self.send_heartbeat()
                last_heartbeat = now
            time.sleep(0.1)

    def send_heartbeat(self):
        """PL: Wysyła pakiet MAVLink HEARTBEAT, jeśli połączenie jest dostępne.

        EN: Sends a MAVLink HEARTBEAT packet when a connection is available.
        """
        if not self.master:
            return
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0,
            0,
            0,
        )

    def send_telemetry(self, sensor_data: dict[str, Any]):
        """PL: Wysyła pakiety ATTITUDE i GLOBAL_POSITION_INT z danych czujników.

        Przy braku danych EKF orientacja i prędkość przyjmują wartości domyślne;
        przy braku połączenia metoda niczego nie wysyła.

        Args:
            sensor_data: Słownik zawierający dane IMU, GPS i opcjonalnie EKF.

        EN: Sends ATTITUDE and GLOBAL_POSITION_INT packets from sensor data.

        Missing EKF values use defaults; no packets are sent when disconnected.

        Args:
            sensor_data: Dictionary containing IMU, GPS, and optional EKF data.
        """
        if not self.master:
            return

        imu = sensor_data.get("imu", {})
        gps = sensor_data.get("gps", {})
        ekf = sensor_data.get("ekf", {})  # Using EKF as primary source if available

        # 1. ATTITUDE (Roll, Pitch, Yaw in rad)
        # Using placeholder or EKF orientation
        roll, pitch, yaw = 0.0, 0.0, 0.0
        if "orientation" in ekf:
            roll, pitch, yaw = ekf["orientation"]

        time_boot_ms = int((time.time() - self.boot_time) * 1000)
        self.master.mav.attitude_send(
            time_boot_ms,
            roll,
            pitch,
            yaw,
            imu.get("gx", 0.0),
            imu.get("gy", 0.0),
            imu.get("gz", 0.0),
        )

        # 2. GLOBAL_POSITION_INT
        # Lat/Lon in 1E7, Alt in mm, V in cm/s
        lat = int(gps.get("lat", 0.0) * 1e7)
        lon = int(gps.get("lon", 0.0) * 1e7)
        alt = int(gps.get("alt", 0.0) * 1000)

        vx = int(ekf.get("vx", 0.0) * 100)
        vy = int(ekf.get("vy", 0.0) * 100)
        vz = 0

        hdg = int((math.degrees(yaw) % 360) * 100) if yaw else 0

        self.master.mav.global_position_int_send(
            time_boot_ms, lat, lon, alt, 0, vx, vy, vz, hdg  # relative_alt
        )


import math  # Needed for degrees in telemetry
