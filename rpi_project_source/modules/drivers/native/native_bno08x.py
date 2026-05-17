"""
Natywny sterownik dla IMU BNO08x używający biblioteki Adafruit CircuitPython.
Native driver for BNO08x IMU using Adafruit CircuitPython library.
"""

import logging
import time
from typing import Any

try:
    import board
    import busio
    from adafruit_bno08x import (BNO_REPORT_ACCELEROMETER,
                                 BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER,
                                 BNO_REPORT_ROTATION_VECTOR)
    from adafruit_bno08x.i2c import BNO08X_I2C

    AVAILABLE: bool = True
except ImportError:
    AVAILABLE = False

from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry

logger: logging.Logger = logging.getLogger(__name__)


@SensorRegistry.register
class NativeBNO08x(IMUBase):
    """
    Sterownik dla zaawansowanego sensora orientacji (IMU) BNO08x.
    Driver for the advanced BNO08x orientation sensor (IMU).
    """

    DRIVER_NAME = "native_bno08x"
    I2C_ADDRESSES = [0x4A, 0x4B]
    PRIORITY = 10  # Najwyższy — sprzętowa fuzja / Highest — HW fusion

    def __init__(self, i2c_wrapper: Any | None = None) -> None:
        """
        Inicjalizuje sensor BNO08x i włącza raportowanie danych.
        Initializes the BNO08x sensor and enables data reporting.

        Args:
            i2c_wrapper (Any | None): Ignorowane, używa busio bezpośrednio.
                                         Ignored, uses busio directly.
        """
        if not AVAILABLE:
            raise ImportError("adafruit-circuitpython-bno08x library not found")

        try:
            # Używamy busio niezależnie od i2c_wrapper
            # We use busio independently of i2c_wrapper
            self.i2c: busio.I2C = busio.I2C(board.SCL, board.SDA)
            self.bno: BNO08X_I2C = BNO08X_I2C(self.i2c)

            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

            logger.info("BNO08x initialized successfully")

            # Mechanizm zabezpieczający (Circuit Breaker)
            self.error_count: int = 0
            self.is_disabled: bool = False
            self.disabled_until: float = 0.0

        except Exception as e:
            logger.error(f"Failed to init BNO08x: {e}")
            self.bno = None

    def read_data(self) -> dict[str, Any]:
        """
        Odczytuje dane (akceleracja, kwaternion) z sensora.
        Reads data (acceleration, quaternion) from the sensor.

        Returns:
            dict[str, Any]: Słownik z danymi sensora. / Dictionary with sensor data.
        """
        if self.bno is None:
            return {}

        now: float = time.time()
        if self.is_disabled:
            if now > self.disabled_until:
                self.is_disabled = False
                self.error_count = 0
            else:
                return {}

        try:
            # Odczyty mogą zawieść niezależnie
            # Wrap individual reads as they can fail independently
            acc = self.bno.acceleration  # (x, y, z) m/s^2
            # gyro = self.bno.gyro         # (x, y, z) rad/s
            # mag = self.bno.magnetic      # (x, y, z) uT
            quat = self.bno.quaternion  # (i, j, k, real) -> (x, y, z, w)

            data: dict[str, Any] = {}
            if acc:
                data["ax"] = acc[0] / 9.81
                data["ay"] = acc[1] / 9.81
                data["az"] = acc[2] / 9.81

            if quat:
                data["qx"] = quat[0]
                data["qy"] = quat[1]
                data["qz"] = quat[2]
                data["qw"] = quat[3]

            data["temperature"] = 25.0

            self.error_count = 0
            return data

        except Exception as e:
            self.error_count += 1
            if self.error_count < 5:
                logger.warning(f"BNO08x read error: {e}")

            if self.error_count > 10:
                self.is_disabled = True
                self.disabled_until = now + 5.0
                logger.error("BNO08x: Too many errors. Disabling for 5s.")

            return {}

    def calibrate(self) -> bool:
        """
        BNO08x has internal auto-calibration.
        We just log that it's active.
        """
        logger.info("BNO08x: Using internal auto-calibration.")
        return True


# Standardized driver alias
IMUDriver = NativeBNO08x
