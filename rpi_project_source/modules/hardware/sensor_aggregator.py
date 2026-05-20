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

            # Aplikacja kalibracji pokładowej / Apply onboard calibration
            if sensor_data["imu"]:
                self._apply_imu_calibration(sensor_data["imu"])

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

        # Obliczenie szacowanej orientacji / Calculate estimated orientation
        if sensor_data["imu"]:
            self._update_imu_orientation(sensor_data["imu"])

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

    def _apply_imu_calibration(self, imu_data: dict[str, Any]) -> None:
        """
        Aplikuje pokładowo parametry kalibracji IMU (zsynchronizowane z GCS i zapisane w config.json).
        Applies onboard IMU calibration parameters (synced from GCS and saved in config.json).
        """
        calib = self.config.get("imu_calibration", {})
        if not calib:
            return

        # Gyro Bias Calibration
        if "gx" in imu_data and imu_data["gx"] is not None:
            imu_data["gx"] = float(imu_data["gx"]) - calib.get("gyro_bias_x", 0.0)
        if "gy" in imu_data and imu_data["gy"] is not None:
            imu_data["gy"] = float(imu_data["gy"]) - calib.get("gyro_bias_y", 0.0)
        if "gz" in imu_data and imu_data["gz"] is not None:
            imu_data["gz"] = float(imu_data["gz"]) - calib.get("gyro_bias_z", 0.0)

        # Accelerometer 6-position Calibration
        if "ax" in imu_data and imu_data["ax"] is not None:
            imu_data["ax"] = (
                float(imu_data["ax"]) - calib.get("accel_offset_x", 0.0)
            ) * calib.get("accel_scale_x", 1.0)
        if "ay" in imu_data and imu_data["ay"] is not None:
            imu_data["ay"] = (
                float(imu_data["ay"]) - calib.get("accel_offset_y", 0.0)
            ) * calib.get("accel_scale_y", 1.0)
        if "az" in imu_data and imu_data["az"] is not None:
            imu_data["az"] = (
                float(imu_data["az"]) - calib.get("accel_offset_z", 0.0)
            ) * calib.get("accel_scale_z", 1.0)

        # Magnetometer Hard/Soft Iron Calibration
        if "mx" in imu_data and imu_data["mx"] is not None:
            imu_data["mx"] = (
                float(imu_data["mx"]) - calib.get("mag_offset_x", 0.0)
            ) * calib.get("mag_scale_x", 1.0)
        if "my" in imu_data and imu_data["my"] is not None:
            imu_data["my"] = (
                float(imu_data["my"]) - calib.get("mag_offset_y", 0.0)
            ) * calib.get("mag_scale_y", 1.0)
        if "mz" in imu_data and imu_data["mz"] is not None:
            imu_data["mz"] = (
                float(imu_data["mz"]) - calib.get("mag_offset_z", 0.0)
            ) * calib.get("mag_scale_z", 1.0)

        # Oznacz jako skalibrowany / Mark as calibrated
        imu_data["calibrated"] = True

    def _update_imu_orientation(self, imu_data: dict[str, Any]) -> None:
        """
        Oblicza szacowaną orientację (pitch, roll)
        przy użyciu szybkiego filtra komplementarnego.
        Calculates estimated orientation (pitch, roll)
        using a fast complementary filter.
        """
        import math
        import time

        now = time.time()
        if not hasattr(self, "_last_orient_time"):
            self._last_orient_time = now
            self._pitch_est = 0.0
            self._roll_est = 0.0

        dt = now - self._last_orient_time
        self._last_orient_time = now

        # Zabezpieczenie przed zbyt dużym krokiem czasowym (np. pierwsza iteracja)
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05  # Domyślny krok dla 20Hz

        ax = imu_data.get("ax", 0.0)
        ay = imu_data.get("ay", 0.0)
        az = imu_data.get("az", 9.81)
        gx = imu_data.get("gx", 0.0)
        gy = imu_data.get("gy", 0.0)

        if ax is None:
            ax = 0.0
        if ay is None:
            ay = 0.0
        if az is None:
            az = 9.81
        if gx is None:
            gx = 0.0
        if gy is None:
            gy = 0.0

        # Obliczenie kątów z akcelerometru (w stopniach)
        try:
            pitch_acc = math.atan2(ax, math.hypot(ay, az)) * (180.0 / math.pi)
            roll_acc = math.atan2(-ay, az) * (180.0 / math.pi)
        except Exception:
            pitch_acc = 0.0
            roll_acc = 0.0

        # Przeliczenie żyroskopu z rad/s na stopnie/s
        gx_deg = gx * (180.0 / math.pi)
        gy_deg = gy * (180.0 / math.pi)

        # Filtr komplementarny: 98% zintegrowanego żyroskopu + 2% z akcelerometru
        self._pitch_est = 0.98 * (self._pitch_est + gy_deg * dt) + 0.02 * pitch_acc
        self._roll_est = 0.98 * (self._roll_est + gx_deg * dt) + 0.02 * roll_acc

        # Wstrzyknięcie obliczonych kątów do słownika IMU
        imu_data["pitch"] = self._pitch_est
        imu_data["roll"] = self._roll_est

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
