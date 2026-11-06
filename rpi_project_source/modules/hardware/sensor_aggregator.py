"""PL: Zbiera, normalizuje i udostępnia dane czujników Raspberry Pi.
EN: Collects, normalizes, and provides Raspberry Pi sensor data.
"""

import logging
import hashlib
import math
import platform
import time
from typing import Any

from modules.drivers.base_sensor import IMUBase
from modules.drivers.lidar.ld08 import LD08Driver
from modules.gps import GPS_UART

logger = logging.getLogger(__name__)


class SensorAggregator:
    """PL: Inicjalizuje czujniki i scala ich odczyty oraz obsługę błędów.
    EN: Initializes sensors and combines their readings and error handling.
    """

    def __init__(self, hardware_config: dict[str, Any], i2c_bus: Any):
        """PL: Konfiguruje i uruchamia dostępne urządzenia pomiarowe.
        EN: Configures and starts the available sensor devices.
        """
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
        """PL: Wykrywa lub wybiera skonfigurowany sterownik IMU i kalibruje go.
        EN: Detects or selects the configured IMU driver and calibrates it.
        """
        if not self.i2c:
            return

        imu_config = self.config.get("imu", {})
        if not imu_config.get("enabled", True):
            return
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
        """PL: Tworzy i uruchamia odbiornik GPS, jeśli jest włączony.
        EN: Creates and starts the GPS receiver when it is enabled.
        """
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
        """PL: Tworzy i uruchamia LiDAR LD08, jeśli jest włączony.
        EN: Creates and starts the LD08 LiDAR when it is enabled.
        """
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
        """PL: Inicjalizuje monitor zasilania UPS na magistrali I2C.
        EN: Initializes the UPS power monitor on the I2C bus.
        """
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
        """PL: Odczytuje i scala aktualne dane IMU, GPS, LiDAR i UPS.
        EN: Reads and combines current IMU, GPS, LiDAR, and UPS data.
        """
        sensor_data: dict[str, Any] = {
            "imu": None,
            "gps": None,
            "lidar": None,
            "battery": None,
        }

        if self.imu:
            try:
                # Omijamy fallback drivera starych IMU przekazując throttle dla Magnetic Compensation
                try:
                    sensor_data["imu"] = self.imu.read_data(throttle=self.last_throttle)
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
                calibration = self.config.get("imu_calibration", {})
                explicit_si = (calibration.get("accel_unit") == "m/s2"
                               and calibration.get("gyro_unit") == "rad/s")
                if not explicit_si:
                    # Legacy saved offsets use the native driver's g/deg/s units.
                    self._apply_imu_calibration(sensor_data["imu"])
                sensor_data["imu"] = self._normalize_imu(sensor_data["imu"])
                if sensor_data["imu"] and explicit_si:
                    self._apply_imu_calibration(sensor_data["imu"])
            else:
                sensor_data["imu"] = None

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

    def _normalize_imu(self, readings: dict[str, Any]) -> dict[str, Any] | None:
        """PL: Waliduje próbkę IMU, przelicza jednostki i dodaje metadane.
        EN: Validates an IMU sample, converts units, and adds metadata.
        """
        driver = getattr(self.imu, "DRIVER_NAME", "")
        units = {
            "native_mpu6050": ("g", "deg/s"),
            "native_mpu9250": ("g", "deg/s"),
            "native_gy91": ("g", "deg/s"),
            "native_bmx160": ("m/s2", "deg/s"),
            "native_bmx160_bmp388": ("m/s2", "deg/s"),
            "native_bno08x": ("g", "rad/s"),
        }.get(driver, (readings.get("accel_unit"), readings.get("gyro_unit")))
        accel_unit, gyro_unit = units
        if accel_unit not in ("g", "m/s2") or gyro_unit not in ("deg/s", "rad/s"):
            return None
        data = dict(readings)
        for prefix, scale in (("a", 9.81 if accel_unit == "g" else 1.0),
                              ("g", math.pi / 180 if gyro_unit == "deg/s" else 1.0)):
            for axis in "xyz":
                key = prefix + axis
                value = data.get(key)
                if value is None:
                    data.pop(key, None)
                elif not isinstance(value, (int, float)) or not math.isfinite(value):
                    return None
                else:
                    data[key] = value * scale
        if not all(key in data for key in ("ax", "ay", "az")):
            return None
        self._imu_sample_sequence = getattr(self, "_imu_sample_sequence", 0) + 1
        if not hasattr(self, "_imu_device_uid"):
            identity = self.config.get("device_uid")
            if not identity:
                try:
                    with open("/etc/machine-id", encoding="ascii") as identity_file:
                        identity = identity_file.read().strip()
                except OSError:
                    identity = platform.node()
            self._imu_device_uid = hashlib.sha256(str(identity).encode()).hexdigest()[:24]
        data.update(source="rpi_raw", driver=driver, accel_unit="m/s2",
                    gyro_unit="rad/s", mag_unit="uT", angle_unit="deg",
                    calibration_owner="raspberry_pi", sample_time_us=time.monotonic_ns() // 1000,
                    sample_sequence=self._imu_sample_sequence,
                    device_uid=self._imu_device_uid,
                    sensor_id=getattr(self.imu, "address", getattr(self.imu, "addr", 0)))
        return data

    def _apply_imu_calibration(self, imu_data: dict[str, Any]) -> None:
        """PL: Stosuje zapisane przesunięcia i skale kalibracji IMU.
        EN: Applies the stored IMU calibration offsets and scales.
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
        """PL: Szacuje kąty pochylenia i przechyłu filtrem komplementarnym.
        EN: Estimates pitch and roll angles with a complementary filter.
        """
        import math
        import time

        now = time.monotonic()
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

        if not all(key in imu_data for key in ("gx", "gy", "gz")):
            return  # Missing gyro is not a measured zero angular rate.

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
        """PL: Uruchamia ponowną kalibrację IMU, jeśli czujnik jest dostępny.
        EN: Starts IMU recalibration when the sensor is available.
        """
        if self.imu:
            return self.imu.calibrate()
        return False

    def notify_throttle_change(self, throttle: float) -> None:
        """PL: Zapisuje ostatnią wartość gazu do kompensacji IMU.
        EN: Stores the latest throttle value for IMU compensation.
        """
        self.last_throttle = throttle

    def check_reconnect_needed(self) -> None:
        """PL: Okresowo ponawia inicjalizację niedostępnych czujników.
        EN: Periodically retries initialization of unavailable sensors.
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
        """PL: Zatrzymuje urządzenia GPS i LiDAR przed zamknięciem.
        EN: Stops the GPS and LiDAR devices during shutdown.
        """
        if self.gps:
            self.gps.stop()
        if self.lidar:
            self.lidar.stop()
