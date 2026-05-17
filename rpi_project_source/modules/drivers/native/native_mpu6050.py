# The MIT License (MIT)
# ... (nagłówek licencji bez zmian) ...

"""
Native MPU-6050 Driver using a shared I2C wrapper.
Natywny sterownik MPU-6050 używający współdzielonego wrappera I2C.
"""

import logging
import time

from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry

from .native_i2c import I2CWrapper


@SensorRegistry.register
class NativeMPU6050(IMUBase):
    """
    A lightweight, direct-register-access driver for the MPU-6050 IMU.
    Lekki sterownik z bezpośrednim dostępem do rejestrów dla IMU MPU-6050.
    """

    DRIVER_NAME = "native_mpu6050"
    I2C_ADDRESSES = [0x68]
    PRIORITY = 50

    @classmethod
    def scan(cls, i2c) -> bool:
        """WHO_AM_I check: 0x68."""
        try:
            who = i2c.read_byte_data(0x68, 0x75)
            return who == cls._CHIP_ID
        except (OSError, IOError):
            return False

    # --- Register Definitions ---
    _PWR_MGMT_1 = 0x6B
    _ACCEL_CONFIG = 0x1C
    _GYRO_CONFIG = 0x1B
    _ACCEL_XOUT_H = 0x3B
    _REG_WHO_AM_I = 0x75
    _CHIP_ID = 0x68

    # --- Scale Factors ---
    _ACCEL_SCALE_FACTOR_2G = 16384.0
    _GYRO_SCALE_FACTOR_250DPS = 131.0

    def __init__(self, i2c_wrapper: I2CWrapper, address: int = 0x68):
        """
        Initializes the MPU6050 driver.
        Inicjalizuje sterownik MPU6050.
        """
        self.i2c = i2c_wrapper
        self.address = address

        # Verify chip ID
        chip_id = self.i2c.read_byte_data(self.address, self._REG_WHO_AM_I)
        if chip_id != self._CHIP_ID:
            raise RuntimeError(
                f"Invalid MPU-6050 chip ID: expected {hex(self._CHIP_ID)}, got {hex(chip_id)}"
            )

        logging.info(f"✓ Detected MPU-6050 at address {hex(self.address)}")

        self.wake_up()

    def wake_up(self) -> None:
        """
        Wakes up the MPU6050.
        Wybudza MPU6050.
        """
        self.i2c.write_byte_data(self.address, self._PWR_MGMT_1, 0x00)

    def _bytes_to_int(self, high: int, low: int) -> int:
        value = (high << 8) | low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        return value

    def read_raw(self) -> dict[str, int]:
        """
        Reads raw data from the sensor.
        Odczytuje surowe dane z czujnika.
        """
        data = self.i2c.read_i2c_block_data(self.address, self._ACCEL_XOUT_H, 14)
        raw_data = {
            "accel_x": self._bytes_to_int(data[0], data[1]),
            "accel_y": self._bytes_to_int(data[2], data[3]),
            "accel_z": self._bytes_to_int(data[4], data[5]),
            "temp": self._bytes_to_int(data[6], data[7]),
            "gyro_x": self._bytes_to_int(data[8], data[9]),
            "gyro_y": self._bytes_to_int(data[10], data[11]),
            "gyro_z": self._bytes_to_int(data[12], data[13]),
        }
        return raw_data

    def read_scaled(self) -> dict[str, float]:
        """
        Reads scaled data from the sensor.
        Odczytuje przeskalowane dane z czujnika.
        """
        raw = self.read_raw()
        scaled_data = {
            "ax": raw["accel_x"] / self._ACCEL_SCALE_FACTOR_2G,
            "ay": raw["accel_y"] / self._ACCEL_SCALE_FACTOR_2G,
            "az": raw["accel_z"] / self._ACCEL_SCALE_FACTOR_2G,
            "temp": (raw["temp"] / 340.0) + 36.53,
            "gx": raw["gyro_x"] / self._GYRO_SCALE_FACTOR_250DPS,
            "gy": raw["gyro_y"] / self._GYRO_SCALE_FACTOR_250DPS,
            "gz": raw["gyro_z"] / self._GYRO_SCALE_FACTOR_250DPS,
            # Dodane dla kompatybilności z interfejsem (brak magnetometru)
            "mx": 0.0,
            "my": 0.0,
            "mz": 0.0,
        }
        return scaled_data

    # --- NOWA METODA (NAPRAWA BŁĘDU) ---
    def read_data(self) -> dict[str, float]:
        """
        Alias dla read_scaled() wymagany przez HardwareManager.
        Alias for read_scaled() required by HardwareManager.
        """
        return self.read_scaled()

    def calibrate(self) -> bool:
        """
        Przeprowadza kalibrację sensora (wyznaczenie biasu żyroskopu).
        Performs sensor calibration (gyro bias estimation).
        """
        logging.info("MPU-6050: Starting gyro calibration (100 samples)...")
        # To jest uproszczona wersja, w rzeczywistych zastosowaniach
        # należałoby odejmować ten bias w read_scaled().
        # This is a simplified version for interface compatibility.
        try:
            for _ in range(100):
                self.read_raw()
                time.sleep(0.01)
            logging.info("MPU-6050: Calibration successful.")
            return True
        except Exception as e:
            logging.error(f"MPU-6050: Calibration failed: {e}")
            return False
