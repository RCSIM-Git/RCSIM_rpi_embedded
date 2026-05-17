"""
Połączony sterownik dla modułu GY-91 (MPU-9250/MPU-9255 + BMP280).
Combined driver for GY-91 module (MPU-9250/MPU-9255 + BMP280).

WAŻNE / IMPORTANT:
- Chipy sprzedawane jako MPU-9250 to często MPU-9255 (ID: 0x73 zamiast 0x71)
- Chips sold as MPU-9250 are often MPU-9255 (ID: 0x73 instead of 0x71)
- Oba są wspierane przez ten driver / Both are supported by this driver
- Bazowane na https://github.com/ricardozago/GY91-MPU9250-BMP280

Łączy NativeMPU9250 i NativeBMP280 w jeden interfejs 10-DoF.
Combines NativeMPU9250 and NativeBMP280 into a single 10-DoF interface.
"""

import logging
from typing import Any

try:
    from .native_bmp280 import NativeBMP280
    from .native_i2c import I2CWrapper
    from .native_mpu9250 import NativeMPU9250
except ImportError:
    # Fallback dla testów jednostkowych / Fallback for relative imports when testing
    from native_mpu9250 import NativeMPU9250
    from native_bmp280 import NativeBMP280
    from native_i2c import I2CWrapper

from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry

logger: logging.Logger = logging.getLogger(__name__)


@SensorRegistry.register
class NativeGY91(IMUBase):
    """
    Sterownik dla modułu GY-91 łączący MPU-9250 (IMU) i BMP280 (Barometr).
    GY-91 Driver combining MPU-9250 (IMU) and BMP280 (Barometer).
    """

    DRIVER_NAME = "native_gy91"
    I2C_ADDRESSES = [0x68]
    PRIORITY = 20

    @classmethod
    def scan(cls, i2c) -> bool:
        """Check for MPU-9250/9255 + BMP280 combo."""
        try:
            who = i2c.read_byte_data(0x68, 0x75)
            if who not in [0x71, 0x73]:
                return False
            # BMP280 present at 0x76 or 0x77?
            for bmp_addr in [0x76, 0x77]:
                try:
                    chip_id = i2c.read_byte_data(bmp_addr, 0xD0)
                    if chip_id == 0x58:  # BMP280
                        return True
                except (OSError, IOError):
                    continue
            return False
        except (OSError, IOError):
            return False

    def __init__(self, i2c_wrapper: I2CWrapper) -> None:
        """
        Inicjalizuje moduł GY-91 poprzez skanowanie typowych adresów sensorów.
        Initializes the GY-91 module by scanning typical sensor addresses.

        Args:
            i2c_wrapper (I2CWrapper): Wrapper magistrali I2C. / I2C bus wrapper.
        """
        # Adresy GY-91:
        # MPU9250: zazwyczaj 0x68 (domyślny) lub 0x69
        # BMP280: zazwyczaj 0x76 (SDO->GND) lub 0x77 (SDO->VCC)

        self.mpu: NativeMPU9250 | None = None
        self.bmp: NativeBMP280 | None = None
        self.i2c: I2CWrapper = i2c_wrapper

        # --- Inicjalizacja MPU-9250/MPU-9255 / Init MPU-9250/MPU-9255 ---
        # 🔧 Zgodność z ricardozago: standardowy adres to 0x68
        # Niektóre moduły mogą używać 0x69 (pin AD0 wysoki)
        try:
            # Spróbuj najpierw 0x68 (standardowy adres GY-91)
            self.mpu = NativeMPU9250(i2c_wrapper, address=0x68)
            logger.info("✓ GY-91: MPU-9250/9255 initialized at 0x68")
        except Exception as e:
            logger.warning(f"GY-91: MPU init at 0x68 failed: {e}. Trying 0x69...")
            try:
                self.mpu = NativeMPU9250(i2c_wrapper, address=0x69)
                logger.info("✓ GY-91: MPU-9250/9255 initialized at 0x69")
            except Exception as e2:
                logger.error(f"✗ GY-91: MPU init failed at both addresses: {e2}")

        # --- Inicjalizacja BMP280 / Init BMP280 ---
        # 🔧 Zgodność z ricardozago: typowe adresy to 0x76 (SDO->GND) lub 0x77 (SDO->VCC)
        try:
            # Spróbuj najpierw 0x76 (standardowy dla większości GY-91)
            self.bmp = NativeBMP280(i2c_wrapper, address=0x76)
            logger.info("✓ GY-91: BMP280 initialized at 0x76")
        except Exception as e:
            logger.warning(f"GY-91: BMP280 init at 0x76 failed: {e}. Trying 0x77...")
            try:
                self.bmp = NativeBMP280(i2c_wrapper, address=0x77)
                logger.info("✓ GY-91: BMP280 initialized at 0x77")
            except Exception as e2:
                logger.error(f"✗ GY-91: BMP280 init failed at both addresses: {e2}")

        if not self.mpu and not self.bmp:
            logger.critical("GY-91: Neither sensor could be initialized!")

    def read_data(self) -> dict[str, Any]:
        """
        Odczytuje dane z obu sensorów i scala je w jeden słownik.
        Reads data from both sensors and merges them.

        Returns:
            dict[str, Any]: Scalone dane (acc, gyro, mag, pressure, temp).
                            Merged data (acc, gyro, mag, pressure, temp).
        """
        data: dict[str, Any] = {}

        # 1. Odczyt danych MPU (zawiera temp, ale BMP jest dokładniejszy)
        # Read MPU data (includes temp, but BMP temp is usually better)
        if self.mpu:
            try:
                mpu_data = self.mpu.read_data()
                data.update(mpu_data)
            except Exception:
                pass

        # 2. Odczyt danych BMP (ciśnienie, temp) - nadpisuje temp z MPU
        # Read BMP data (pressure, temp) - overwrites MPU temp
        if self.bmp:
            try:
                bmp_data = self.bmp.read_data()
                data.update(bmp_data)  # 'pressure', 'temperature'
            except Exception:
                pass

        return data

    def calibrate(self) -> bool:
        """
        Przeprowadza kalibrację IMU (MPU9250/9255).
        Performs calibration of the IMU (MPU9250/9255).
        """
        if self.mpu:
            return self.mpu.calibrate()
        return False

    def get_chip_info(self) -> dict[str, Any]:
        """
        Zwraca informacje o wykrytych chipach w module GY-91.
        Returns information about detected chips in the GY-91 module.

        Returns:
            dict[str, Any]: Informacje o chipach / Chip information
        """
        info: dict[str, Any] = {
            "module": "GY-91 (10-DoF)",
            "mpu_present": self.mpu is not None,
            "bmp_present": self.bmp is not None,
            "magnetometer_present": (
                self.mpu.magnetometer is not None if self.mpu else False
            ),
        }

        if self.mpu:
            try:
                chip_id = self.i2c.bus.read_byte_data(self.mpu.address, 0x75)
                chip_names = {
                    0x71: "MPU-9250",
                    0x73: "MPU-9255",
                    0x70: "MPU-6500",
                }
                info["mpu_chip"] = chip_names.get(chip_id, f"Unknown (0x{chip_id:02X})")
                info["mpu_chip_id"] = hex(chip_id)
                info["mpu_address"] = hex(self.mpu.address)
            except Exception:
                info["mpu_chip"] = "Unknown"

        if self.bmp:
            info["barometer_chip"] = "BMP280"
            info["barometer_address"] = hex(self.bmp.address)

        return info


# Standardized driver alias
IMUDriver = NativeGY91
