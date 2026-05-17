"""
Sensor Aggregator - Menedżer Odczytów (RPi).
Zbiera, chłonie błędy i filtruje surowe dane ze wszystkich szyn I2C, SPI oraz UART przed ich wejściem do systemu.
"""

import logging
from typing import Any

from modules.drivers.base_sensor import IMUBase
from modules.drivers.lidar.ld08 import LD08Driver
from modules.gps import GPS_UART

logger = logging.getLogger(__name__)


class SensorAggregator:
    """
    Sub-moduł (SRP) przejęty z dawnego `HardwareManager`.
    Agreguje stan modułów pobocznych takich jak GPS, Lidar, IMU, UPS chłonąc wszelkie ich wyjątki I2C (m.in. EAGAIN).
    """

    def __init__(self, hardware_config: dict[str, Any], i2c_bus: Any):
        self.config = hardware_config
        self.i2c = i2c_bus
        self.imu: IMUBase | None = None
        self.gps: GPS_UART | None = None
        self.lidar: LD08Driver | None = None
        self.ups = None
        self._notified_failures: set[str] = set()
        self._last_reconnect_attempt: float = 0.0
        self.RECONNECT_INTERVAL: float = 10.0  # Seconds between reconnection attempts

        self.last_throttle: float = 0.0

        self._init_imu()
        self._init_gps()
        self._init_lidar()
        self._init_ups()

    def _init_imu(self) -> None:
        if not self.i2c:
            return

        imu_config = self.config.get("imu", {})
        driver_name = imu_config.get("driver", "auto")

        from modules.drivers.sensor_registry import SensorRegistry

        if driver_name == "auto":
            sensor_class = SensorRegistry.detect(self.i2c)
            if not sensor_class:
                logger.warning("Auto-detection: No known IMU found on I2C bus.")
                return
            logger.info(f"Auto-detected IMU: {sensor_class.DRIVER_NAME}")
        else:
            sensor_class = SensorRegistry.get_by_name(driver_name)
            if not sensor_class:
                logger.error(f"Unknown IMU driver: '{driver_name}'")
                return

        try:
            self.imu = sensor_class(self.i2c)
            logger.info(f"IMU '{sensor_class.DRIVER_NAME}' initialized successfully.")
        except Exception as e:
            logger.error(f"Failed to initialize IMU '{sensor_class.DRIVER_NAME}': {e}")

        if self.imu:
            logger.info("Performing startup IMU calibration... (Keep robot stationary)")
            success = self.imu.calibrate()
            if success:
                logger.info("Startup IMU calibration successful.")
            else:
                logger.warning("Startup IMU calibration failed.")

    def _init_gps(self) -> None:
        gps_config = self.config.get("gps", {})
        ntrip_config = self.config.get("ntrip", {})
        if not gps_config.get("enabled", False):
            logger.info("GPS is disabled in the configuration.")
            return

        port = gps_config.get("port", "/dev/ttyAMA0")
        try:
            self.gps = GPS_UART(
                port=port,
                baudrate=gps_config.get("baudrate", 9600),
                ntrip_cfg=ntrip_config,
            )
            self.gps.start()
            logger.info("GPS module initialized and started.")
        except Exception as e:
            logger.error(f"Failed to initialize GPS: {e}")

    def _init_lidar(self) -> None:
        lidar_config = self.config.get("lidar", {})
        if not lidar_config.get("enabled", False):
            logger.info("LiDAR is disabled in the configuration.")
            return

        port = lidar_config.get("port", "/dev/ttyUSB0")
        try:
            self.lidar = LD08Driver(
                port=port, baudrate=lidar_config.get("baudrate", 230400)
            )
            self.lidar.start()
            logger.info("LD08 LiDAR driver initialized and started.")
        except Exception as e:
            logger.error(f"Failed to initialize LiDAR: {e}")

    def _init_ups(self) -> None:
        try:
            from modules.drivers.native.ups import UPS

            if self.i2c:
                self.ups = UPS(logger, i2c_wrapper=self.i2c, address=0x42)
                logger.info("UPS initialized at 0x42")
            else:
                logger.warning("Cannot initialize UPS: No I2C wrapper available.")
                self.ups = None
        except Exception as e:
            logger.error(f"UPS init failed: {e}")
            self.ups = None

    def read_sensors(self) -> dict[str, Any]:
        """Agregacja i pollowanie słownika statusowego ze wszystkich instrumentów."""
        sensor_data: dict[str, Any] = {
            "imu": None,
            "gps": None,
            "lidar": None,
            "battery": None,
        }

        if self.imu:
            try:
                # Omijamy fallback drivera starych IMU przekazując throttle dla Magnetic Compensation
                if hasattr(self.imu, "read_data"):
                    sensor_data["imu"] = self.imu.read_data(throttle=self.last_throttle)
                else:
                    sensor_data["imu"] = self.imu.read_data()
            except TypeError:
                sensor_data["imu"] = self.imu.read_data()
            except Exception as e:
                if "[Errno 11]" in str(e) or "EAGAIN" in str(e):
                    logger.debug(f"IMU temporarily busy: {e}")
                else:
                    if "imu" not in self._notified_failures:
                        logger.warning(f"Failed to read from IMU: {e}")
                        self._notified_failures.add("imu")

        # Wstrzykiwanie bezpiecznego bloku danych w razie uszkodzenia fizycznego IMU
        if not sensor_data["imu"]:
            sensor_data["imu"] = {
                "ax": 0.0,
                "ay": 0.0,
                "az": 9.81,
                "gx": 0.0,
                "gy": 0.0,
                "gz": 0.0,
                "temperature": 25.0,
            }

        if self.gps:
            try:
                sensor_data["gps"] = self.gps.get_latest_data()
            except Exception as e:
                if "gps" not in self._notified_failures:
                    logger.warning(f"Failed to read from GPS: {e}")
                    self._notified_failures.add("gps")

        # Fallback blokowy z zerami dla GPS
        if not sensor_data["gps"]:
            sensor_data["gps"] = {
                "lat": 0.0,
                "lon": 0.0,
                "altitude": 0.0,
                "speed": 0.0,
                "satellites": 0,
                "fix_quality": 0,
                "course": 0.0,
            }

        if self.lidar:
            try:
                sensor_data["lidar"] = self.lidar.read_scan(downsample=False)
            except Exception as e:
                logger.debug(f"LiDAR data not ready: {e}")

        if self.ups:
            try:
                sensor_data["battery"] = self.ups.read_data()
            except Exception as e:
                if "[Errno 11]" in str(e) or "Resource temporarily unavailable" in str(
                    e
                ):
                    logger.debug(f"UPS/INA219 busy (EAGAIN): {e}")
                else:
                    if "ups" not in self._notified_failures:
                        logger.warning(f"Failed to read from UPS: {e}")
                        self._notified_failures.add("ups")

        return sensor_data

    def calibrate_imu(self) -> bool:
        """Przekazanie instrukcji włączenia re-kalibracji żyroskopu na osi poziomej."""
        if self.imu:
            return self.imu.calibrate()
        return False

    def notify_throttle_change(self, throttle: float) -> None:
        """Logowanie gazu do wywołań wibracji sprzężnej dla IMU."""
        self.last_throttle = throttle

    def check_reconnect_needed(self) -> None:
        """
        Próbuje przywrócić połączenie z brakującymi sensorami (Hot-plug support).
        Attempts to reconnect missing sensors.
        """
        import time
        now = time.time()
        if now - self._last_reconnect_attempt < self.RECONNECT_INTERVAL:
            return
            
        self._last_reconnect_attempt = now
        
        # 1. IMU
        if self.imu is None:
            logger.info("Hot-plug: Attempting to reconnect IMU...")
            self._init_imu()
            
        # 2. GPS
        if self.gps is None:
            gps_config = self.config.get("gps", {})
            if gps_config.get("enabled", False):
                logger.info("Hot-plug: Attempting to reconnect GPS...")
                self._init_gps()
                
        # 3. LiDAR
        if self.lidar is None:
            lidar_config = self.config.get("lidar", {})
            if lidar_config.get("enabled", False):
                logger.info("Hot-plug: Attempting to reconnect LiDAR...")
                self._init_lidar()
                
        # 4. UPS
        if self.ups is None:
            logger.info("Hot-plug: Attempting to reconnect UPS...")
            self._init_ups()

    def cleanup(self) -> None:
        """Pętla dezaktywująca UARTY i porty COM"""
        if self.gps:
            self.gps.stop()
        if self.lidar:
            self.lidar.stop()
