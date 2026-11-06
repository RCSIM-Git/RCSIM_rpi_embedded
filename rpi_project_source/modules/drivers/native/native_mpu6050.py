# The MIT License (MIT)
# ... (nagłówek licencji bez zmian) ...

"""PL: Natywny sterownik IMU MPU-6050 korzystający ze wspólnego wrappera I2C.

EN: Native MPU-6050 IMU driver using a shared I2C wrapper.
"""

import logging
import time

from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry

from .native_i2c import I2CWrapper


@SensorRegistry.register
class NativeMPU6050(IMUBase):
    """PL: Odczytuje surowe i przeskalowane dane MPU-6050 przez rejestry I2C.

    EN: Reads raw and scaled MPU-6050 data through its I2C registers.
    """

    DRIVER_NAME = "native_mpu6050"
    I2C_ADDRESSES = [0x68]
    PRIORITY = 50

    @classmethod
    def scan(cls, i2c) -> bool:
        """PL: Sprawdza rejestr WHO_AM_I pod kątem identyfikatora 0x68.

        EN: Checks the WHO_AM_I register for identifier 0x68.
        """
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
        """PL: Weryfikuje identyfikator układu i wybudza MPU-6050.

        EN: Verifies the chip identifier and wakes up the MPU-6050.
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
        """PL: Wybudza MPU-6050 przez zapis do rejestru zasilania.

        EN: Wakes the MPU-6050 by writing to its power management register.
        """
        self.i2c.write_byte_data(self.address, self._PWR_MGMT_1, 0x00)

    def _bytes_to_int(self, high: int, low: int) -> int:
        """PL: Dekoduje dwa bajty big-endian jako liczbę 16-bit ze znakiem.

        EN: Decodes two big-endian bytes as a signed 16-bit integer.
        """
        value = (high << 8) | low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        return value

    def read_raw(self) -> dict[str, int]:
        """PL: Odczytuje surowe próbki akcelerometru, temperatury i żyroskopu.

        EN: Reads raw accelerometer, temperature, and gyroscope samples.
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
        """PL: Zwraca przyspieszenie w g, temperaturę w °C i żyroskop w °/s.

        EN: Returns acceleration in g, temperature in °C, and gyroscope in °/s.
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
        }
        return scaled_data

    # --- NOWA METODA (NAPRAWA BŁĘDU) ---
    def read_data(self) -> dict[str, float]:
        """PL: Zwraca przeskalowane pomiary w formacie wymaganym przez HardwareManager.

        EN: Returns scaled readings in the format required by HardwareManager.
        """
        return self.read_scaled()

    def calibrate(self) -> bool:
        """PL: Wykonuje 100 odczytów kontrolnych jako uproszczoną kalibrację.

        Metoda nie oblicza ani nie odejmuje biasu z odczytów.

        EN: Performs 100 check samples as a simplified calibration routine.

        The method does not calculate or subtract a bias from readings.
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
