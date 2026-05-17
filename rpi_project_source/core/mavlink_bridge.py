import logging
import threading
import time
from typing import Any

from pymavlink import mavutil

logger = logging.getLogger(__name__)


class MAVLinkBridge:
    """
    Most MAVLink umożliwiający integrację z QGroundControl.
    MAVLink bridge for QGroundControl integration.
    """

    def __init__(
        self, connection_str: str = "udpout:127.0.0.1:14550", system_id: int = 1
    ):
        self.connection_str = connection_str
        self.system_id = system_id
        self.component_id = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        self.master: mavutil.mavlink_connection | None = None
        self.running = False
        self.thread: threading.Thread | None = None
        self.boot_time = time.time()

    def start(self):
        """Uruchamia most MAVLink."""
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
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

    def _update_loop(self):
        """Pętla wysyłająca Heartbeat (1Hz)."""
        last_heartbeat = 0
        while self.running:
            now = time.time()
            if now - last_heartbeat >= 1.0:
                self.send_heartbeat()
                last_heartbeat = now
            time.sleep(0.1)

    def send_heartbeat(self):
        """Wysyła pakiet HEARTBEAT (wymagane przez QGC)."""
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
        """Wysyła pakiety telemetryczne (GLOBAL_POSITION_INT, ATTITUDE)."""
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
